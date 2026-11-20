#pragma once
#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
namespace cleany_navigation {
/// Conservative continuous sweep for linearly interpolated center/yaw between
/// adjacent rollout poses. Occupied cells represent their full square area.
bool sweptFootprintFree(const nav2_costmap_2d::Costmap2D &,
  const std::vector<geometry_msgs::msg::Point> &,
  const geometry_msgs::msg::Pose2D &, const geometry_msgs::msg::Pose2D &);
}
