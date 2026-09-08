#pragma once

#include "cleany_scene_mapping/known_geometry.hpp"
#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <atomic>
#include <cstdint>
#include <mutex>

namespace cleany_scene_mapping
{
// Own the whole callback: inconsistent geometry rejects the entire frame before
// masking, raycasting, tree writes, and publishing a processed-cloud receipt.
class KnownGeometryOctomapUpdater : public occupancy_map_monitor::OccupancyMapUpdater
{
public:
  KnownGeometryOctomapUpdater();
  ~KnownGeometryOctomapUpdater() override;
  bool initialize(const rclcpp::Node::SharedPtr& node) override;
  bool setParams(const std::string& name_space) override;
  void start() override;
  void stop() override;
  occupancy_map_monitor::ShapeHandle excludeShape(const shapes::ShapeConstPtr& shape) override;
  void forgetShape(occupancy_map_monitor::ShapeHandle handle) override;
  // Same synchronous path as the subscription, exposed for map-level tests.
  bool processCloud(const sensor_msgs::msg::PointCloud2& cloud);

private:
  bool processFrame(const sensor_msgs::msg::PointCloud2& cloud, const char*& stage);
  rclcpp::Node::SharedPtr node_;
  // Diagnostics must remain visible even if this node's ROS clock stops.
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::atomic<bool> active_{false};
  std::mutex frame_mutex_;
  std::mutex geometry_mutex_;
  PartitionedShapeMask shape_mask_;
  KnownBodies bodies_;
  std::uint64_t generation_ = 0;
  std::string cloud_topic_;
  std::string filtered_topic_;
  double scale_ = 1.0;
  double padding_ = 0.0;
  double maximum_range_ = 2.0;
  double maximum_rate_ = 2.0;
  std::int64_t subsample_ = 1;
  std::int64_t mask_workers_ = 4;
  std::int64_t maximum_examined_ = 200000;
  std::int64_t last_update_ns_ = 0;
};
}  // namespace cleany_scene_mapping
