#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <array>
#include <future>
#include <limits>
#include <atomic>
#include <chrono>
#include <mutex>
#include <random>
#include <set>
#include <thread>

namespace
{
using Partitioned = cleany_scene_mapping::PartitionedShapeMask;
using Mask = point_containment_filter::ShapeMask;

// Inject failures/observations into the actual parallel point predicate, rather
// than into transform lookup (which now runs once on the capture thread).
class ObservedSphere : public bodies::Sphere
{
public:
  explicit ObservedSphere(std::function<void()> observer) : observer_(std::move(observer))
  {
    const shapes::Sphere shape(5.0);
    setDimensions(&shape);
  }
  bool containsPoint(const Eigen::Vector3d& point, bool verbose = false) const override
  {
    observer_();
    return bodies::Sphere::containsPoint(point, verbose);
  }
  bodies::BodyPtr cloneAt(const Eigen::Isometry3d& pose, double padding, double scale) const override
  {
    auto result = std::make_shared<ObservedSphere>(observer_);
    result->setPoseDirty(pose);
    result->setPaddingDirty(padding);
    result->setScaleDirty(scale);
    result->updateInternalData();
    return result;
  }
private:
  std::function<void()> observer_;
};

sensor_msgs::msg::PointCloud2 cloud(std::size_t count, bool with_color = true)
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.frame_id = "camera";
  sensor_msgs::PointCloud2Modifier modifier(result);
  if (with_color) modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  else modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(count);
  std::mt19937 random(35);
  std::uniform_real_distribution<float> range(-1.5F, 1.5F);
  sensor_msgs::PointCloud2Iterator<float> xyz(result, "x");
  for (; xyz != xyz.end(); ++xyz)
  { xyz[0] = range(random); xyz[1] = range(random); xyz[2] = range(random); }
  return result;
}

TEST(PartitionedShapeMask, PreservesAllPointLabelsAcrossCountsShapesAndTransforms)
{
  for (const auto workers : {1U, 2U, 4U})
  {
    Mask serial;
    Partitioned parallel(workers);
    auto pose = Eigen::Isometry3d::Identity();
    auto transform = [&](unsigned int handle, Eigen::Isometry3d& output) {
      output = pose;
      output.translation().x() += handle * .3;
      return true;
    };
    serial.setTransformCallback(transform);
    parallel.setTransformCallback(transform);
    std::vector<shapes::ShapeConstPtr> shapes{
      std::make_shared<shapes::Box>(.4, .6, .8), std::make_shared<shapes::Sphere>(.3),
      std::make_shared<shapes::Cylinder>(.2, .7)};
    for (const auto& shape : shapes)
      EXPECT_EQ(serial.addShape(shape, 1.1, .015), parallel.addShape(shape, 1.1, .015));
    for (const auto count : {0U, 1U, 2U, 3U, 7U, 1003U})
    {
      auto input = cloud(count);
      if (count > 1)
      {
        sensor_msgs::PointCloud2Iterator<float> xyz(input, "x");
        xyz[0] = std::numeric_limits<float>::quiet_NaN();
        xyz[1] = std::numeric_limits<float>::infinity();
      }
      for (double yaw : {0.0, .4})
      {
        pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        std::vector<int> expected, actual;
        serial.maskContainment(input, Eigen::Vector3d::Zero(), .1, 1.2, expected);
        parallel.maskContainment(input, Eigen::Vector3d::Zero(), .1, 1.2, actual);
        EXPECT_EQ(actual, expected);
      }
    }
    serial.removeShape(2); parallel.removeShape(2);
    EXPECT_EQ(serial.addShape(shapes[0]), parallel.addShape(shapes[0], 1.0, 0.0));
    auto input = cloud(31);
    std::vector<int> expected, actual;
    serial.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, expected);
    parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, actual);
    EXPECT_EQ(actual, expected);
  }
}

TEST(PartitionedShapeMask, PreservesEmptyGeometryShortcutAndRejectsInvalidWorkers)
{
  EXPECT_THROW(Partitioned(0), std::invalid_argument);
  EXPECT_THROW(Partitioned(5), std::invalid_argument);
  Partitioned parallel(4);
  Mask serial;
  std::vector<int> expected, actual;
  auto input = cloud(19);
  serial.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, .001, expected);
  parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, .001, actual);
  EXPECT_EQ(actual, expected);
  parallel.addShape(std::make_shared<shapes::Box>(1.0, 1.0, 1.0), 1.0, 0.0);
  EXPECT_THROW(parallel.setWorkerCount(2), std::logic_error);
  parallel.removeShape(1);
  EXPECT_NO_THROW(parallel.setWorkerCount(2));
}

TEST(PartitionedShapeMask, JoinsWorkersOnProviderFailureAndRecovers)
{
  Partitioned parallel(4);
  parallel.addShape(std::make_shared<shapes::Sphere>(1.0), 1.0, 0.0);
  parallel.setTransformCallback([](unsigned int, Eigen::Isometry3d&) -> bool {
    throw std::runtime_error("provider failed");
  });
  auto input = cloud(51);
  std::vector<int> actual;
  EXPECT_THROW(parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, actual), std::runtime_error);
  parallel.setTransformCallback([](unsigned int, Eigen::Isometry3d& pose) {
    pose.setIdentity(); return true;
  });
  EXPECT_NO_THROW(parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, actual));
  EXPECT_EQ(actual.size(), 51U);
}

TEST(PartitionedShapeMask, ReusesWorkersAcrossFramesAndRebuildsAfterReconfiguration)
{
  Partitioned parallel(4);
  static std::atomic<unsigned int> next_thread{0};
  std::mutex mutex;
  std::set<unsigned int> threads;
  auto body = std::make_shared<ObservedSphere>([&] {
    // OS thread IDs may be recycled between frames; a TLS token also detects
    // newly created threads that happen to receive an earlier thread's ID.
    thread_local const auto token = next_thread.fetch_add(1);
    std::lock_guard<std::mutex> lock(mutex);
    threads.insert(token);
  });
  parallel.setTransformCallback([](unsigned int, Eigen::Isometry3d& pose) {
    pose.setIdentity();
    return true;
  });
  const Partitioned::PreparedShape shape{body, body->computeVolume()};
  auto handle = parallel.addShape(shape);
  auto input = cloud(101);
  std::vector<int> output;
  for (int frame = 0; frame < 20; ++frame)
    parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, output);
  EXPECT_EQ(threads.size(), 4U);
  for (auto count : {1U, 2U, 4U, 2U})
  {
    parallel.removeShape(handle);
    parallel.setWorkerCount(count);
    threads.clear();
    handle = parallel.addShape(shape);
    for (int frame = 0; frame < 5; ++frame)
      parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, output);
    EXPECT_EQ(threads.size(), count);
  }
}

TEST(PartitionedShapeMask, ReusedBuffersPreserveLabelsWhenFramesShrinkAndLayoutChanges)
{
  Mask serial;
  Partitioned parallel(4);
  auto pose = Eigen::Isometry3d::Identity();
  auto transform = [&](unsigned int, Eigen::Isometry3d& output) {
    output = pose;
    return true;
  };
  serial.setTransformCallback(transform);
  parallel.setTransformCallback(transform);
  const auto shape = std::make_shared<shapes::Box>(.4, .8, 1.0);
  serial.addShape(shape, 1.1, .015);
  parallel.addShape(shape, 1.1, .015);
  for (int frame = 0; frame < 6; ++frame)
    for (auto count : {4097U, 0U, 1U, 2U, 3U, 4U, 31U, 7U, 3001U})
    {
      auto input = cloud(count, frame % 2 == 0);
      pose.translation().x() = frame * .1;
      pose.linear() = Eigen::AngleAxisd(frame * .2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      std::vector<int> expected, actual;
      serial.maskContainment(input, Eigen::Vector3d::Zero(), .1, 1.2, expected);
      parallel.maskContainment(input, Eigen::Vector3d::Zero(), .1, 1.2, actual);
      ASSERT_EQ(actual, expected) << "frame=" << frame << " count=" << count;
    }
}

TEST(PartitionedShapeMask, DrainsAllWorkersBeforePropagatingCallerOrWorkerException)
{
  Partitioned parallel(4);
  auto input = cloud(101);
  const auto caller = std::this_thread::get_id();
  for (bool fail_caller : {false, true})
  {
    std::atomic<int> completed{0};
    const auto body = std::make_shared<ObservedSphere>([&] {
      if ((std::this_thread::get_id() == caller) == fail_caller)
        throw std::runtime_error("provider failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      ++completed;
    });
    const auto handle = parallel.addShape(Partitioned::PreparedShape{body, body->computeVolume()});
    parallel.setTransformCallback([](unsigned int, Eigen::Isometry3d& pose) {
      pose.setIdentity(); return true;
    });
    std::vector<int> actual{42};
    EXPECT_THROW(parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, actual),
                 std::runtime_error);
    const auto drained = completed.load();
    EXPECT_GT(drained, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_EQ(completed.load(), drained);  // No worker may outlive its frame.
    EXPECT_EQ(actual, std::vector<int>{42});
    parallel.removeShape(handle);
    const auto recovered = parallel.addShape(std::make_shared<shapes::Sphere>(.5), 1.0, 0.0);
    EXPECT_NO_THROW(parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, actual));
    EXPECT_EQ(actual.size(), 101U);
    parallel.removeShape(recovered);
  }
}

TEST(PartitionedShapeMask, ResolvesEachTransformOnceAndSnapshotSurvivesHandleReuse)
{
  Partitioned parallel(4);
  const auto old_handle = parallel.addShape(std::make_shared<shapes::Sphere>(1.0), 1.0, .015);
  const auto captured = parallel.snapshot();
  parallel.removeShape(old_handle);
  parallel.addShape(std::make_shared<shapes::Sphere>(.01), 1.0, 0.0);
  EXPECT_EQ(parallel.addShape(std::make_shared<shapes::Sphere>(.02), 1.0, 0.0), old_handle);
  unsigned int calls = 0;
  auto transform = [&](unsigned int handle, Eigen::Isometry3d& pose) {
    EXPECT_GT(handle, 0U);
    ++calls;
    pose.setIdentity();
    return true;
  };
  const auto frame = Partitioned::prepareFrame(captured, transform);
  EXPECT_EQ(calls, 1U);
  auto input = cloud(1, false);
  sensor_msgs::PointCloud2Iterator<float> xyz(input, "x");
  xyz[0] = .5F; xyz[1] = xyz[2] = 0;
  std::vector<int> actual;
  parallel.maskContainment(frame, input, 0.0, 2.0, actual);
  EXPECT_EQ(actual, std::vector<int>{Mask::INSIDE});
  parallel.setTransformCallback(transform);
  parallel.maskContainment(input, Eigen::Vector3d::Zero(), 0.0, 2.0, actual);
  EXPECT_EQ(actual, std::vector<int>{Mask::OUTSIDE});
}

TEST(PartitionedShapeMask, RegistrationDoesNotWaitForCapturedFrameMasking)
{
  Partitioned parallel(4);
  std::promise<void> entered, resume;
  auto resumed = resume.get_future().share();
  std::atomic<bool> first{true};
  const auto body = std::make_shared<ObservedSphere>([&] {
    if (first.exchange(false)) entered.set_value();
    resumed.wait();
  });
  const auto handle = parallel.addShape(Partitioned::PreparedShape{body, body->computeVolume()});
  auto transform = [](unsigned int, Eigen::Isometry3d& pose) { pose.setIdentity(); return true; };
  const auto frame = Partitioned::prepareFrame(parallel.snapshot(), transform);
  auto input = cloud(101);
  auto work = std::async(std::launch::async, [&] {
    std::vector<int> output;
    parallel.maskContainment(frame, input, 0.0, 2.0, output);
    return output;
  });
  const auto ready = entered.get_future().wait_for(std::chrono::seconds(2));
  if (ready != std::future_status::ready)
  {
    resume.set_value();
    work.wait();
    FAIL() << "Mask worker did not enter point classification";
  }
  // The point workers remain blocked while the registered geometry changes.
  parallel.removeShape(handle);
  const auto next = parallel.addShape(std::make_shared<shapes::Sphere>(.1), 1.0, 0.0);
  EXPECT_EQ(parallel.snapshot().size(), 1U);
  EXPECT_NE(next, 0U);
  resume.set_value();
  EXPECT_EQ(work.get().size(), 101U);
}

TEST(PartitionedShapeMask, HierarchyPreservesPaddedRotatedOffsetMeshesAndBoundaryPoints)
{
  // Offset mesh vertices exercise mesh-local bounding boxes, not only meshes
  // centered at their link origin. Include boundaries at FLOAT32 precision.
  const shapes::Box box(.4, .05, .08);
  std::unique_ptr<shapes::Mesh> shifted(shapes::createMeshFromShape(box));
  for (unsigned int i = 0; i < shifted->vertex_count; ++i)
    shifted->vertices[3*i] += .3;
  const shapes::ShapeConstPtr mesh(shifted.release());
  // A non-box hull without the very dense default sphere tessellation. Keep
  // this unit check small; the offline profiler exercises real CAD meshes.
  auto tetrahedron = std::make_shared<shapes::Mesh>(4, 4);
  const std::array<double, 12> vertices{0, 0, 0, .3, 0, 0, 0, .2, 0, 0, 0, .4};
  const std::array<unsigned int, 12> triangles{0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
  std::copy(vertices.begin(), vertices.end(), tetrahedron->vertices);
  std::copy(triangles.begin(), triangles.end(), tetrahedron->triangles);
  const shapes::ShapeConstPtr round_mesh = tetrahedron;
  for (const auto workers : {1U, 2U, 4U})
    for (const auto scale : {.8, 1.0, 1.25})
      for (const auto padding : {0.0, .015})
      {
        Mask serial;
        Partitioned parallel(workers);
        double yaw = 0.0;
        auto transform = [&](unsigned int handle, Eigen::Isometry3d& pose) {
          pose.setIdentity();
          pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
          pose.translation() = Eigen::Vector3d((handle % 4)*.2, (handle / 4)*.15, .1);
          return true;
        };
        serial.setTransformCallback(transform);
        parallel.setTransformCallback(transform);
        for (unsigned int body = 0; body < 12; ++body)
        {
          const auto& shape = body % 2 ? round_mesh : mesh;
          EXPECT_EQ(serial.addShape(shape, scale, padding), parallel.addShape(shape, scale, padding));
        }
        for (const double angle : {0.0, .37, 1.57})
        {
          yaw = angle;
          auto input = cloud(10003, false);
          sensor_msgs::PointCloud2Iterator<float> xyz(input, "x");
          for (double delta : {-1e-7, 0.0, 1e-7})
          {
            xyz[0] = static_cast<float>(.5 + padding + delta);
            xyz[1] = .15F; xyz[2] = .1F;
            ++xyz;
          }
          std::vector<int> expected, actual;
          if (padding == 0.0 && scale == 1.0)
            serial.maskContainment(input, Eigen::Vector3d::Zero(), .1, 2.0, expected);
          else
          {
            // Brute-force volume union checks the hierarchy's widened bounds;
            // padded mesh labels intentionally differ from upstream radial padding.
            const auto frame = Partitioned::prepareFrame(parallel.snapshot(), transform);
            sensor_msgs::PointCloud2ConstIterator<float> point(input, "x");
            for (; point != point.end(); ++point)
            {
              const Eigen::Vector3d p(point[0], point[1], point[2]);
              int label = Mask::OUTSIDE;
              if (p.norm() < .1 || p.norm() > 2.0) label = Mask::CLIP;
              else for (const auto& volume : frame.exclusions)
                if (volume.containsPoint(p)) { label = Mask::INSIDE; break; }
              expected.push_back(label);
            }
          }
          parallel.maskContainment(input, Eigen::Vector3d::Zero(), .1, 2.0, actual);
          EXPECT_EQ(actual, expected) << "workers=" << workers << " scale=" << scale
                                      << " padding=" << padding << " yaw=" << angle;
        }
      }
}

TEST(PartitionedShapeMask, MeshFacePaddingMatchesBoxAndMasksPartialVoxelEndpoint)
{
  const shapes::Box box(.4, .04, .04);
  std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromShape(box));
  const Eigen::Vector3d offset(.3, -.2, .1);
  for (unsigned int i = 0; i < mesh->vertex_count; ++i)
    for (unsigned int axis = 0; axis < 3; ++axis) mesh->vertices[i*3+axis] += offset[axis];
  const shapes::ShapeConstPtr shape(mesh.release());
  for (const auto workers : {1U, 2U, 4U})
    for (const double scale : {.8, 1.0, 1.25})
      for (const double angle : {0.0, .37, 1.57})
      {
        Partitioned mask(workers);
        Mask primitive;
        auto pose = Eigen::Isometry3d::Identity();
        pose.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
        pose.translation() = Eigen::Vector3d(.5, .1, .2);
        mask.addShape(shape, scale, .015);
        primitive.addShape(std::make_shared<shapes::Box>(box), scale, .015);
        mask.setTransformCallback([&](auto, auto& out) { out = pose; return true; });
        primitive.setTransformCallback([&](auto, auto& out) {
          out = pose; out.translation() = pose * offset; return true;
        });
        auto input = cloud(2001, false);
        sensor_msgs::PointCloud2Iterator<float> xyz(input, "x");
        // Side and corner points straddling the exact configured margin. The
        // old radial mesh bound/predicate misses the inside side point.
        for (const double delta : {-.00001, .00001})
          for (const double x : {0.0, .2*scale+.015+delta})
          {
            const Eigen::Vector3d p = pose * (offset + Eigen::Vector3d(x, .02*scale+.015+delta, 0));
            xyz[0]=p.x(); xyz[1]=p.y(); xyz[2]=p.z(); ++xyz;
          }
        std::vector<int> expected, actual;
        primitive.maskContainment(input, Eigen::Vector3d::Zero(), 0., 3., expected);
        mask.maskContainment(input, Eigen::Vector3d::Zero(), 0., 3., actual);
        EXPECT_EQ(actual, expected) << workers << ":" << scale << ":" << angle;
        ASSERT_GE(actual.size(), 4U);
        EXPECT_EQ(actual[0], Mask::INSIDE);
        EXPECT_EQ(actual[2], Mask::OUTSIDE);
      }
}

TEST(PartitionedShapeMask, TetrahedronMatchesAnalyticFacesAfterShrinkAndPadding)
{
  auto mesh=std::make_shared<shapes::Mesh>(4,4);
  const std::array<double,12> vertices{0,0,0, .3,0,0, 0,.2,0, 0,0,.4};
  const std::array<unsigned int,12> triangles{0,2,1, 0,1,3, 0,3,2, 1,2,3};
  std::copy(vertices.begin(),vertices.end(),mesh->vertices);
  std::copy(triangles.begin(),triangles.end(),mesh->triangles);
  const Eigen::Vector3d centroid(.075,.05,.1), normal(1./.3,1./.2,1./.4);
  for (double scale : {.8,1.,1.25})
    for (double padding : {0.,.015})
    {
      Partitioned mask(4);
      mask.addShape(mesh,scale,padding);
      const Eigen::Isometry3d identity=Eigen::Isometry3d::Identity();
      mask.setTransformCallback([&](auto,auto& pose) {pose=identity; return true;});
      auto input=cloud(20003,false);
      sensor_msgs::PointCloud2Iterator<float> sample(input,"x");
      // A point inside the analytically shrunk tetrahedron; old Body containment
      // is not the oracle for nonunit mesh scaling.
      sample[0]=.103469F; sample[1]=.100521F; sample[2]=.025110F;
      std::vector<int> actual;
      mask.maskContainment(input,Eigen::Vector3d::Zero(),0.,3.,actual);
      sensor_msgs::PointCloud2ConstIterator<float> xyz(input,"x");
      for (size_t i=0;i<actual.size();++i,++xyz)
      {
        const Eigen::Vector3d p(xyz[0],xyz[1],xyz[2]);
        const Eigen::Vector3d low=(1.-scale)*centroid-Eigen::Vector3d::Constant(padding);
        const Eigen::Vector3d high=(1.-scale)*centroid+scale*Eigen::Vector3d(.3,.2,.4)+Eigen::Vector3d::Constant(padding);
        const bool inside=(p.array()>=low.array()-1e-9).all() &&
            (p.array()<=high.array()+1e-9).all() &&
            normal.dot(p-(1.-scale)*centroid)<=scale+padding*normal.norm()+1e-9;
        const int expected=p.norm()>3.?Mask::CLIP:(inside?Mask::INSIDE:Mask::OUTSIDE);
        ASSERT_EQ(actual[i],expected) << "scale="<<scale<<" padding="<<padding<<" point="<<p.transpose();
      }
    }
}

TEST(PartitionedShapeMask, BoundingSphereIncludesExactPaddedMeshCorners)
{
  const shapes::Box box(.375,.0625,.0625);
  Partitioned mask(1);
  mask.addShape(shapes::ShapeConstPtr(shapes::createMeshFromShape(box)),1.,.03125);
  mask.setTransformCallback([](auto,auto& pose) {pose.setIdentity(); return true;});
  auto input=cloud(8,false);
  sensor_msgs::PointCloud2Iterator<float> point(input,"x");
  for (unsigned int i=0;i<8;++i,++point)
  {
    point[0]=(i&1U)?.21875F:-.21875F;
    point[1]=(i&2U)?.0625F:-.0625F;
    point[2]=(i&4U)?.0625F:-.0625F;
  }
  std::vector<int> result;
  mask.maskContainment(input,Eigen::Vector3d::Zero(),0.,2.,result);
  EXPECT_EQ(result,std::vector<int>(8,Mask::INSIDE));
}
}  // namespace
