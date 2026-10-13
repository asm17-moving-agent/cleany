// Offline robot-only replay. Does not start ROS nodes or issue motion requests.
#include <geometric_shapes/body_operations.h>
#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <geometric_shapes/mesh_operations.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <urdf_parser/urdf_parser.h>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>

namespace
{
template <typename T> T readMessage(const std::string& path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read " + path);
  const std::string bytes((std::istreambuf_iterator<char>(file)), {});
  rclcpp::SerializedMessage serialized(bytes.size());
  auto& buffer = serialized.get_rcl_serialized_message();
  std::memcpy(buffer.buffer, bytes.data(), bytes.size());
  buffer.buffer_length = bytes.size();
  T message;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &message);
  return message;
}

Eigen::Isometry3d transform(double x, double y, double z, double qx, double qy, double qz, double qw)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, z);
  result.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  return result;
}

shapes::ShapeConstPtr shape(const urdf::Geometry& geometry)
{
  switch (geometry.type)
  {
    case urdf::Geometry::MESH:
    {
      const auto& mesh = dynamic_cast<const urdf::Mesh&>(geometry);
      return shapes::ShapeConstPtr(shapes::createMeshFromResource(
          mesh.filename, Eigen::Vector3d(mesh.scale.x, mesh.scale.y, mesh.scale.z)));
    }
    case urdf::Geometry::BOX:
    {
      const auto& box = dynamic_cast<const urdf::Box&>(geometry);
      return std::make_shared<shapes::Box>(box.dim.x, box.dim.y, box.dim.z);
    }
    case urdf::Geometry::CYLINDER:
    {
      const auto& cylinder = dynamic_cast<const urdf::Cylinder&>(geometry);
      return std::make_shared<shapes::Cylinder>(cylinder.radius, cylinder.length);
    }
    case urdf::Geometry::SPHERE:
      return std::make_shared<shapes::Sphere>(dynamic_cast<const urdf::Sphere&>(geometry).radius);
  }
  throw std::runtime_error("Unsupported URDF collision geometry");
}

double milliseconds(const std::chrono::steady_clock::time_point& start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}
}

int main(int argc, char** argv)
{
  try
  {
    std::cout << std::unitbuf;
    const auto preparation_start = std::chrono::steady_clock::now();
    if (argc != 2) throw std::runtime_error("Usage: profile_mask FIXTURE_DIRECTORY");
    const std::string folder = argv[1];
    const auto cloud = readMessage<sensor_msgs::msg::PointCloud2>(folder + "/raw.cdr");
    const auto fk = readMessage<moveit_msgs::srv::GetPositionFK::Response>(folder + "/fk.cdr");
    auto robot = urdf::parseURDFFile(folder + "/robot.urdf");
    if (!robot) throw std::runtime_error("Invalid robot URDF");
    std::map<std::string, Eigen::Isometry3d> poses;
    for (std::size_t i = 0; i < fk.fk_link_names.size(); ++i)
    {
      const auto& pose = fk.pose_stamped.at(i).pose;
      const auto& p = pose.position;
      const auto& q = pose.orientation;
      poses.emplace(fk.fk_link_names[i], transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w));
    }
    std::map<unsigned int, Eigen::Isometry3d> cache;
    point_containment_filter::ShapeMask mask([&](unsigned int handle, Eigen::Isometry3d& pose) {
      pose = cache.at(handle); return true;
    });
    std::vector<std::unique_ptr<cleany_scene_mapping::PartitionedShapeMask>> parallel_masks;
    for (auto count : {2U, 4U})
    {
      auto parallel = std::make_unique<cleany_scene_mapping::PartitionedShapeMask>(count);
      parallel->setTransformCallback([&](unsigned int handle, Eigen::Isometry3d& pose) {
        pose = cache.at(handle); return true;
      });
      parallel_masks.push_back(std::move(parallel));
    }
    std::vector<std::pair<std::string, bodies::BodyPtr>> bodies;
    for (const auto& [name, pose] : poses)
    {
      const auto link = robot->getLink(name);
      std::size_t index = 0;
      for (const auto& collision : link->collision_array)
      {
        auto geometry = shape(*collision->geometry);
        if (!geometry) throw std::runtime_error("Missing mesh on " + name);
        const auto& p = collision->origin.position;
        const auto& q = collision->origin.rotation;
        const auto placed = pose * transform(p.x, p.y, p.z, q.x, q.y, q.z, q.w);
        const auto handle = mask.addShape(geometry, 1.0, 0.015);
        cache[handle] = placed;
        for (auto& parallel : parallel_masks)
          if (parallel->addShape(geometry, 1.0, 0.015) != handle)
            throw std::runtime_error("Unexpected replay handle mapping");
        bodies::BodyPtr body(bodies::createBodyFromShape(geometry.get()));
        body->setPadding(0.015);
        body->setPose(placed);
        bodies.emplace_back(name + "/" + std::to_string(index++), body);
      }
    }
    std::vector<Eigen::Vector3d> points;
    sensor_msgs::PointCloud2ConstIterator<float> xyz(cloud, "x");
    for (; xyz != xyz.end(); ++xyz)
      points.emplace_back(xyz[0], xyz[1], xyz[2]);
    std::cout << "Robot-only fixture: points=" << points.size() << " bodies=" << bodies.size()
              << " all_comparators_setup_ms=" << milliseconds(preparation_start) << '\n';
    std::vector<int> classified;
    for (unsigned int iteration = 0; iteration < 3; ++iteration)
    {
      const auto start = std::chrono::steady_clock::now();
      mask.maskContainment(cloud, Eigen::Vector3d::Zero(), 0.0, 2.0, classified);
      std::cout << "mask_ms=" << milliseconds(start) << '\n';
    }
    unsigned int workers = 2;
    for (auto& parallel : parallel_masks)
    {
      for (unsigned int iteration = 0; iteration < 3; ++iteration)
      {
        std::vector<int> candidate;
        const auto start = std::chrono::steady_clock::now();
        parallel->maskContainment(cloud, Eigen::Vector3d::Zero(), 0.0, 2.0, candidate);
        const auto duration = milliseconds(start);
        if (candidate != classified) throw std::runtime_error("Partitioned mask changed a point classification");
        std::cout << "workers=" << workers << " mask_ms=" << duration << " identical=true\n";
      }
      workers *= 2;
    }
    for (const auto& [name, body] : bodies)
    {
      std::size_t inside = 0;
      const auto start = std::chrono::steady_clock::now();
      for (const auto& point : points)
        if (point.allFinite() && body->containsPoint(point)) ++inside;
      const double contains_ms = milliseconds(start);
      const auto clone_start = std::chrono::steady_clock::now();
      auto clone = body->cloneAt(body->getPose());
      const auto mesh = dynamic_cast<const bodies::ConvexMesh*>(body.get());
      std::cout << name << " contains_ms=" << contains_ms << " inside=" << inside
                << " planes=" << (mesh ? mesh->getPlanes().size() : 0)
                << " clone_ms=" << milliseconds(clone_start) << '\n';
    }
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
