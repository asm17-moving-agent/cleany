#pragma once
#include <cmath>
#include <optional>
#include "geometry_msgs/msg/pose.hpp"
#include "cleany_navigation/mppi/models/path.hpp"

namespace cleany_navigation::mppi {
/// Return the end yaw of a leading equal-XY turn near the robot.
inline std::optional<float> leadingRotationYaw(
  const models::Path & path, const geometry_msgs::msg::Pose & pose, float position_tolerance)
{
  if (path.x.size() < 2 || std::hypot(pose.position.x - path.x(0),
      pose.position.y - path.y(0)) > position_tolerance) {return std::nullopt;}
  size_t end = 0;
  while (end + 1 < path.x.size() && std::hypot(path.x(end + 1) - path.x(0),
      path.y(end + 1) - path.y(0)) < 1e-4f) {++end;}
  const float delta = path.yaws(end) - path.yaws(0);
  if (end == 0 || std::abs(std::atan2(std::sin(delta), std::cos(delta))) <= 1e-4f) {
    return std::nullopt;
  }
  return path.yaws(end);
}
}
