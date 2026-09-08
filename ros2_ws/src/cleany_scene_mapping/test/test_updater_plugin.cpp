#include <gtest/gtest.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(KnownGeometryPlugin, LoadsConfiguresAndRegistersShapes)
{
  rclcpp::init(0, nullptr);
  struct ShutdownContext
  {
    ~ShutdownContext() { rclcpp::shutdown(); }
  } shutdown;
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({
      rclcpp::Parameter("depth_cloud.point_cloud_topic", "/test_cloud"),
      rclcpp::Parameter("depth_cloud.max_range", 2.0),
      rclcpp::Parameter("depth_cloud.padding_offset", 0.015),
      rclcpp::Parameter("depth_cloud.padding_scale", 1.0),
      rclcpp::Parameter("depth_cloud.point_subsample", 1),
      rclcpp::Parameter("depth_cloud.max_update_rate", 2.0),
      rclcpp::Parameter("depth_cloud.filtered_cloud_topic", "/test_filtered"),
      rclcpp::Parameter("depth_cloud.known_geometry_clear_max_leaves", 200000),
    });
    auto node = std::make_shared<rclcpp::Node>("test_known_geometry_plugin", options);
    pluginlib::ClassLoader<occupancy_map_monitor::OccupancyMapUpdater> loader(
        "moveit_ros_occupancy_map_monitor", "occupancy_map_monitor::OccupancyMapUpdater");
    auto updater = loader.createSharedInstance("cleany_scene_mapping/KnownGeometryOctomapUpdater");
    ASSERT_TRUE(updater->initialize(node));
    EXPECT_TRUE(updater->setParams("depth_cloud"));
    const auto handle = updater->excludeShape(std::make_shared<shapes::Box>(0.1, 0.1, 0.1));
    EXPECT_NE(handle, 0U);
    updater->forgetShape(handle);
    node->set_parameter(rclcpp::Parameter("depth_cloud.known_geometry_clear_max_leaves", 0));
    EXPECT_FALSE(updater->setParams("depth_cloud"));
    updater->stop();
  }
}
