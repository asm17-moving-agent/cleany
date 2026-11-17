#pragma once

#include <moveit/robot_state/robot_state.h>
#include <Eigen/Core>
#include <array>
#include <string>
#include <vector>

namespace cleany
{
/** Resolve the existing CAD pin equations without granting actuator ownership. */
class CadLinkage
{
public:
  explicit CadLinkage(const std::string& json, const moveit::core::RobotModel& model);
  bool resolve(moveit::core::RobotState& state) const;

private:
  struct Finger
  {
    Eigen::Vector2d a, b, c, d, e, p;
    double ad, pd, ac, be, ce;
    int drive_branch, distal_branch;
    std::array<std::string, 4> names;
  };
  struct Gripper
  {
    std::string source;
    double scale, offset, epsilon;
    std::vector<Finger> fingers;
  };
  std::vector<Gripper> groups_;
};
}  // namespace cleany
