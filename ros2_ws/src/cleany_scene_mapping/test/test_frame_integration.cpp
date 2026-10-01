#include "cleany_scene_mapping/known_geometry_updater.hpp"
#include <gtest/gtest.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <future>
#include <limits>
#include <thread>

namespace
{
using Monitor = occupancy_map_monitor::OccupancyMapMonitor;
using Cache = occupancy_map_monitor::ShapeTransformCache;
using Updater = cleany_scene_mapping::KnownGeometryOctomapUpdater;
using namespace std::chrono_literals;

class Middleware : public Monitor::MiddlewareHandle
{
public:
  Monitor::Parameters getParameters() const override { return {0.01, "map", {}}; }
  occupancy_map_monitor::OccupancyMapUpdaterPtr loadOccupancyMapUpdater(const std::string&) override
  { return {}; }
  void initializeOccupancyMapUpdater(occupancy_map_monitor::OccupancyMapUpdaterPtr) override {}
  void createSaveMapService(SaveMapServiceCallback) override {}
  void createLoadMapService(LoadMapServiceCallback) override {}
};

class FrameIntegration : public testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
      rclcpp::Parameter("depth_cloud.point_cloud_topic", "/frame_test_cloud"),
      rclcpp::Parameter("depth_cloud.max_range", 2.0),
      rclcpp::Parameter("depth_cloud.padding_offset", 0.0),
      rclcpp::Parameter("depth_cloud.padding_scale", 1.0),
      rclcpp::Parameter("depth_cloud.point_subsample", 1),
      rclcpp::Parameter("depth_cloud.max_update_rate", 0.0),
      rclcpp::Parameter("depth_cloud.filtered_cloud_topic", "/frame_test_filtered"),
    });
    node = std::make_shared<rclcpp::Node>("frame_integration_test", options);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    monitor = std::make_unique<Monitor>(std::make_unique<Middleware>(), tf);
    updater = std::make_unique<Updater>();
    updater->setMonitor(monitor.get());
    ASSERT_TRUE(updater->initialize(node));
    ASSERT_TRUE(updater->setParams("depth_cloud"));
    updater->setTransformCacheCallback([](const auto&, const auto&, Cache&) { return true; });
    monitor->getOcTreePtr()->setUpdateCallback([this] { ++updates; });
  }
  void TearDown() override
  {
    updater.reset();
    monitor.reset();
    tf.reset();
    node.reset();
  }
  sensor_msgs::msg::PointCloud2 cloud(std::initializer_list<Eigen::Vector3f> points)
  {
    sensor_msgs::msg::PointCloud2 result;
    result.header.frame_id = "map";
    result.header.stamp.sec = 7;
    result.header.stamp.nanosec = 123;
    sensor_msgs::PointCloud2Modifier modifier(result);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> iter(result, "x");
    for (const auto& point : points)
    {
      iter[0] = point.x(); iter[1] = point.y(); iter[2] = point.z();
      ++iter;
    }
    return result;
  }
  unsigned int addBox()
  { return updater->excludeShape(std::make_shared<shapes::Box>(0.1, 0.1, 0.1)); }
  Eigen::Isometry3d boxPose()
  {
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0.505, 0.005, 0.005);
    return pose;
  }
  void setBoxCache(unsigned int handle)
  {
    updater->setTransformCacheCallback([this, handle](const auto&, const auto&, Cache& cache) {
      cache[handle] = boxPose(); return true;
    });
  }
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<Monitor> monitor;
  std::unique_ptr<Updater> updater;
  unsigned int updates = 0;
};

TEST_F(FrameIntegration, RejectsMissingCacheWithoutAnyTreeMutation)
{
  addBox();
  auto tree = monitor->getOcTreePtr();
  tree->updateNode(0.505, 0.005, 0.005, true);
  const auto size = tree->size();
  EXPECT_FALSE(updater->processCloud(cloud({{1.005F, 0.005F, 0.005F}})));
  EXPECT_EQ(tree->size(), size);
  EXPECT_TRUE(tree->isNodeOccupied(tree->search(0.505, 0.005, 0.005)));
  EXPECT_EQ(updates, 0U);
}

TEST_F(FrameIntegration, RejectsGeometryChangedDuringProviderEvenWithCompleteCache)
{
  updater->setTransformCacheCallback([this](const auto&, const auto&, Cache& cache) {
    // Would deadlock if the transform provider ran under the geometry mutex.
    const auto handle = addBox();
    cache[handle] = boxPose();
    return true;
  });
  EXPECT_FALSE(updater->processCloud(cloud({{1.005F, 0.005F, 0.005F}})));
  EXPECT_EQ(monitor->getOcTreePtr()->size(), 0U);
  EXPECT_EQ(updates, 0U);
}

TEST_F(FrameIntegration, RejectsRemoveAndReaddDuringProvider)
{
  const auto handle = addBox();
  updater->setTransformCacheCallback([this, handle](const auto&, const auto&, Cache& cache) {
    updater->forgetShape(handle);
    cache[addBox()] = boxPose();
    return true;
  });
  EXPECT_FALSE(updater->processCloud(cloud({{1.005F, 0.005F, 0.005F}})));
  EXPECT_EQ(updates, 0U);
}

TEST_F(FrameIntegration, RejectsNonfiniteAndFailedProvidersThenRecovers)
{
  const auto handle = addBox();
  updater->setTransformCacheCallback([this, handle](const auto&, const auto&, Cache& cache) {
    cache[handle] = boxPose();
    cache[handle].translation().x() = std::numeric_limits<double>::quiet_NaN();
    return true;
  });
  const auto message = cloud({{1.005F, 0.005F, 0.005F}});
  EXPECT_FALSE(updater->processCloud(message));
  updater->setTransformCacheCallback([](const auto&, const auto&, Cache&) { return false; });
  EXPECT_FALSE(updater->processCloud(message));
  EXPECT_EQ(monitor->getOcTreePtr()->size(), 0U);
  setBoxCache(handle);
  EXPECT_TRUE(updater->processCloud(message));
  EXPECT_EQ(updates, 1U);
}

TEST_F(FrameIntegration, NormalFrameMasksRobotAndPreservesUnrelatedObstacle)
{
  setBoxCache(addBox());
  auto tree = monitor->getOcTreePtr();
  tree->updateNode(0.505, 0.005, 0.005, true);
  tree->updateNode(0.005, 0.805, 0.005, true);
  ASSERT_TRUE(updater->processCloud(cloud({{0.505F, 0.005F, 0.005F}, {1.005F, 0.005F, 0.005F}})));
  EXPECT_FALSE(tree->isNodeOccupied(tree->search(0.505, 0.005, 0.005)));
  EXPECT_TRUE(tree->isNodeOccupied(tree->search(1.005, 0.005, 0.005)));
  EXPECT_TRUE(tree->isNodeOccupied(tree->search(0.005, 0.805, 0.005)));
  EXPECT_EQ(updates, 1U);
}

TEST_F(FrameIntegration, RaysStartAtCameraAndRangeUsesCloudCoordinates)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera";
  transform.transform.translation.x = 3.0;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(transform, "test", true));
  auto message = cloud({{0.505F, 0.005F, 0.005F}});
  message.header.frame_id = "camera";
  ASSERT_TRUE(updater->processCloud(message));
  auto tree = monitor->getOcTreePtr();
  ASSERT_NE(tree->search(3.505, 0.005, 0.005), nullptr);
  EXPECT_TRUE(tree->isNodeOccupied(tree->search(3.505, 0.005, 0.005)));
  EXPECT_EQ(tree->search(1.005, 0.005, 0.005), nullptr);
  ASSERT_NE(tree->search(3.105, 0.005, 0.005), nullptr);
  EXPECT_FALSE(tree->isNodeOccupied(tree->search(3.105, 0.005, 0.005)));
}

TEST_F(FrameIntegration, MissingCaptureTransformDoesNotIntegrate)
{
  auto message = cloud({{0.505F, 0.005F, 0.005F}});
  message.header.frame_id = "missing_camera";
  EXPECT_FALSE(updater->processCloud(message));
  EXPECT_EQ(monitor->getOcTreePtr()->size(), 0U);
  EXPECT_EQ(updates, 0U);
}

TEST_F(FrameIntegration, RejectsMalformedLayoutsBeforeCallingProvider)
{
  unsigned int provider_calls = 0;
  updater->setTransformCacheCallback([&](const auto&, const auto&, Cache&) { ++provider_calls; return true; });
  const auto good = cloud({{0.505F, 0.005F, 0.005F}});
  auto bad = good;
  bad.data.pop_back();
  EXPECT_FALSE(updater->processCloud(bad));
  bad = good; bad.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_FALSE(updater->processCloud(bad));
  bad = good; bad.is_bigendian = true;
  EXPECT_FALSE(updater->processCloud(bad));
  bad = good; ++bad.row_step; bad.data.push_back(0);
  EXPECT_FALSE(updater->processCloud(bad));
  EXPECT_EQ(provider_calls, 0U);
  EXPECT_EQ(updates, 0U);
}

TEST_F(FrameIntegration, ClipsDistantReturnsButDoesNotInventOccupiedEndpoints)
{
  ASSERT_TRUE(updater->processCloud(cloud({{3.005F, 0.0F, 0.0F},
      {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}})));
  auto tree = monitor->getOcTreePtr();
  EXPECT_EQ(tree->search(3.005, 0.0, 0.0), nullptr);
  ASSERT_NE(tree->search(1.005, 0.0, 0.0), nullptr);
  EXPECT_FALSE(tree->isNodeOccupied(tree->search(1.005, 0.0, 0.0)));
}

TEST_F(FrameIntegration, PublishesOriginalStampOnlyForAcceptedFramesAndRestarts)
{
  std::vector<sensor_msgs::msg::PointCloud2> received;
  auto subscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/frame_test_filtered", rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { received.push_back(*msg); });
  updater->start();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin = [&] {
    for (unsigned int i = 0; i < 30; ++i) { executor.spin_some(); std::this_thread::sleep_for(10ms); }
  };
  spin();
  const auto message = cloud({{1.005F, 0.005F, 0.005F}});
  ASSERT_TRUE(updater->processCloud(message));
  spin();
  ASSERT_EQ(received.size(), 1U);
  EXPECT_EQ(received[0].header, message.header);
  const auto handle = addBox();
  EXPECT_FALSE(updater->processCloud(message));
  spin();
  EXPECT_EQ(received.size(), 1U);
  updater->stop();
  setBoxCache(handle);
  updater->start();
  spin();
  ASSERT_TRUE(updater->processCloud(message));
  spin();
  EXPECT_EQ(received.size(), 2U);
}

TEST_F(FrameIntegration, SerializesConcurrentFramesIncludingTransformCacheMutation)
{
  std::atomic<unsigned int> in_provider{0};
  std::atomic<bool> overlapped{false};
  updater->setTransformCacheCallback([&](const auto&, const auto&, Cache&) {
    if (in_provider.fetch_add(1) != 0)
      overlapped = true;
    std::this_thread::sleep_for(5ms);
    --in_provider;
    return true;
  });
  const auto message = cloud({{1.005F, 0.005F, 0.005F}});
  std::vector<std::future<bool>> tasks;
  for (unsigned int i = 0; i < 8; ++i)
    tasks.push_back(std::async(std::launch::async, [&] { return updater->processCloud(message); }));
  for (auto& task : tasks)
    EXPECT_TRUE(task.get());
  EXPECT_FALSE(overlapped);
  EXPECT_EQ(updates, 8U);
}

TEST_F(FrameIntegration, TakesOnlyNewestCloudAfterExecutorBacklog)
{
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/frame_test_cloud", rclcpp::SensorDataQoS());
  std::vector<int> processed_stamps;
  updater->setTransformCacheCallback([&](const auto&, const auto& stamp, Cache&) {
    processed_stamps.push_back(static_cast<int>(stamp.nanoseconds() / 1000000000));
    return true;
  });
  updater->start();
  for (unsigned int i = 0; i < 100 && publisher->get_subscription_count() == 0; ++i)
    std::this_thread::sleep_for(10ms);
  ASSERT_EQ(publisher->get_subscription_count(), 1U);

  // DDS receives these while the single callback executor is not running.
  // Once execution resumes, stale queued captures must not delay the newest.
  for (int stamp = 10; stamp < 15; ++stamp)
  {
    auto message = cloud({{1.005F, 0.005F, 0.005F}});
    message.header.stamp.sec = stamp;
    publisher->publish(message);
    std::this_thread::sleep_for(10ms);
  }
  std::this_thread::sleep_for(100ms);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  for (unsigned int i = 0; i < 30; ++i)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(processed_stamps, std::vector<int>({14}));
  EXPECT_EQ(updates, 1U);
}
}  // namespace
