#pragma once
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
namespace cleany_navigation {
/// 인접 rollout 자세 사이의 중심과 yaw를 선형 보간한 보수적 연속 sweep입니다.
/// 점유 셀은 전체 정사각형 면적을 나타냅니다.
bool sweptFootprintFree(const nav2_costmap_2d::Costmap2D &,
  const std::vector<geometry_msgs::msg::Point> &,
  const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);
}
