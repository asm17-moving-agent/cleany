#pragma once

#include "cleany_scene_mapping/known_geometry.hpp"
#include "cleany_scene_mapping/raycast.hpp"
#include "cleany_scene_mapping/partitioned_shape_mask.hpp"
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <atomic>
#include <cstdint>
#include <mutex>

namespace cleany_scene_mapping
{
/**
 * @brief 프레임 동안 로봇 형상이 일관될 때만 점군을 통합합니다.
 *
 * @par Subscriptions
 * point_cloud_topic (sensor_msgs/msg/PointCloud2): 필수 plugin 파라미터이며
 * 내장 토픽 기본값이 없습니다. SensorDataQoS, depth 1을 사용합니다.
 *
 * @par Publishers
 * filtered_cloud_topic (sensor_msgs/msg/PointCloud2): 트리 통합 성공 후 원본
 * header로 내보내는 설정 출력입니다. 빈 값이면 출력을 끕니다.
 * SensorDataQoS, depth 1이며 발행은 처리 완료 확인 역할도 합니다.
 *
 * @par TF and constraints
 * occupancy-map monitor가 형상 변환을 제공하며 이 plugin은 TF를 발행하지 않습니다.
 * 점군 구조 오류, 형상 변환 누락 또는 형상 generation 변경 시 트리에 쓰거나
 * 발행하기 전에 프레임을 거부합니다.
 */
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
  /// 구독과 같은 동기 경로이며 지도 수준 테스트용으로 노출합니다.
  bool processCloud(const sensor_msgs::msg::PointCloud2& cloud);

private:
  friend struct KnownGeometryUpdaterTestAccess;
  bool processFrame(const sensor_msgs::msg::PointCloud2& cloud, const char*& stage);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("cleany_scene_mapping")};
  // 노드의 ROS 시계가 멈춰도 진단은 계속 보여야 합니다.
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::atomic<bool> active_{false};
  std::mutex frame_mutex_;
  std::mutex geometry_mutex_;
  PartitionedShapeMask shape_mask_;
  // Empty, private coordinate/ray geometry; never contains occupancy nodes.
  std::unique_ptr<octomap::OcTree> ray_geometry_;
  RaycastWorkspace raycast_;
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
