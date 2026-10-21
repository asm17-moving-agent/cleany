// Offline comparison against installed geometric_shapes; no ROS participants or hardware.
#include "cleany_scene_mapping/mesh_merge.hpp"
#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <geometric_shapes/mesh_operations.h>
#include <urdf_parser/urdf_parser.h>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
using Clock = std::chrono::steady_clock;
double seconds(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}
void equal(const shapes::Mesh& a, const shapes::Mesh& b)
{
  const auto differs = [](const auto* left, const auto* right, std::size_t count) {
    return (left != nullptr) != (right != nullptr) ||
        (left && std::memcmp(left, right, count * sizeof(*left)) != 0);
  };
  if (a.vertex_count != b.vertex_count || a.triangle_count != b.triangle_count ||
      differs(a.vertices, b.vertices, 3ULL * a.vertex_count) ||
      differs(a.triangles, b.triangles, 3ULL * a.triangle_count) ||
      differs(a.vertex_normals, b.vertex_normals, 3ULL * a.vertex_count) ||
      differs(a.triangle_normals, b.triangle_normals, 3ULL * a.triangle_count))
    throw std::runtime_error("Accelerated merge differs from geometric_shapes");
}

void dumpBody(std::ostream& output, const bodies::Body& body)
{
  const auto& mesh = dynamic_cast<const bodies::ConvexMesh&>(body);
  const auto value = [&](const auto& number) {
    output.write(reinterpret_cast<const char*>(&number), sizeof(number));
  };
  const auto vectors = [&](const auto& values) {
    value(std::uint64_t(values.size()));
    for (const auto& vector : values)
      for (Eigen::Index axis = 0; axis < vector.size(); ++axis) value(double(vector[axis]));
  };
  vectors(mesh.getVertices()); vectors(mesh.getScaledVertices()); vectors(mesh.getPlanes());
  value(std::uint64_t(mesh.getTriangles().size()));
  for (auto index : mesh.getTriangles()) value(std::uint32_t(index));
  value(body.computeVolume());
  bodies::BoundingSphere sphere; body.computeBoundingSphere(sphere);
  for (int axis = 0; axis < 3; ++axis) value(double(sphere.center[axis]));
  value(sphere.radius);
  bodies::BoundingCylinder cylinder; body.computeBoundingCylinder(cylinder);
  value(cylinder.radius); value(cylinder.length);
  for (int row = 0; row < 4; ++row)
    for (int column = 0; column < 4; ++column) value(double(cylinder.pose.matrix()(row, column)));
  bodies::AABB box; body.computeBoundingBox(box);
  for (int axis = 0; axis < 3; ++axis) { value(double(box.min()[axis])); value(double(box.max()[axis])); }
}
}

int main(int argc, char** argv)
{
  try
  {
    if (argc < 2 || argc > 3) throw std::runtime_error("Usage: profile_mesh_preparation ROBOT_URDF [BODY_DUMP]");
    std::ofstream dump;
    if (argc == 3 && *argv[2])
    {
      dump.open(argv[2], std::ios::binary);
      if (!dump) throw std::runtime_error("Cannot open body dump");
    }
    const auto robot = urdf::parseURDFFile(argv[1]);
    if (!robot) throw std::runtime_error("Invalid robot URDF");
    std::cout << std::unitbuf << std::setprecision(9);
    std::cout << "link\tmesh\tvertices\ttriangles\tmerged_vertices\tload_sec\tupstream_merge_sec\tfast_merge_sec\tbody_sec\texact_equal\n";
    double upstream_total = 0, fast_total = 0, body_total = 0, load_total = 0;
    unsigned int shapes = 0;
    std::size_t vertices = 0, triangles = 0;
    for (const auto& [name, link] : robot->links_)
      for (const auto& collision : link->collision_array)
      {
        if (collision->geometry->type != urdf::Geometry::MESH) continue;
        const auto& description = static_cast<const urdf::Mesh&>(*collision->geometry);
        auto start = Clock::now();
        std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(description.filename,
            Eigen::Vector3d(description.scale.x, description.scale.y, description.scale.z)));
        if (!mesh) throw std::runtime_error("Cannot load " + description.filename);
        const double load_sec = seconds(start);
        std::unique_ptr<shapes::Mesh> reference(mesh->clone());
        shapes::ShapePtr accelerated(mesh->clone());
        auto& fast = static_cast<shapes::Mesh&>(*accelerated);
        start = Clock::now();
        reference->mergeVertices(1e-4);
        const double upstream_sec = seconds(start);
        start = Clock::now();
        cleany_scene_mapping::mergeMeshVertices(fast, 1e-4);
        const double fast_sec = seconds(start);
        equal(*reference, fast);
        start = Clock::now();
        auto prepared = cleany_scene_mapping::PartitionedShapeMask::prepareShape(accelerated, 1., .015);
        const double body_sec = seconds(start);
        if (!prepared.body) throw std::runtime_error("Cannot prepare self-filter geometry");
        if (dump.is_open()) dumpBody(dump, *prepared.body);
        std::cout << name << '\t' << description.filename << '\t' << mesh->vertex_count << '\t'
                  << mesh->triangle_count << '\t' << fast.vertex_count << '\t' << load_sec << '\t'
                  << upstream_sec << '\t' << fast_sec << '\t' << body_sec << "\ttrue\n";
        ++shapes; vertices += mesh->vertex_count; triangles += mesh->triangle_count;
        load_total += load_sec; upstream_total += upstream_sec; fast_total += fast_sec; body_total += body_sec;
      }
    std::cout << "TOTAL\t" << shapes << '\t' << vertices << '\t' << triangles << "\t-\t"
              << load_total << '\t' << upstream_total << '\t' << fast_total << '\t' << body_total << "\ttrue\n";
    return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
