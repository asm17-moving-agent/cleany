#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>
#include <random>

namespace
{
using Partitioned = cleany_scene_mapping::PartitionedShapeMask;
using Mask = point_containment_filter::ShapeMask;

sensor_msgs::msg::PointCloud2 cloud(std::size_t count)
{
  sensor_msgs::msg::PointCloud2 result;
  result.header.frame_id = "camera";
  sensor_msgs::PointCloud2Modifier modifier(result);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
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
}  // namespace
