#pragma once
#include "dwb_core/trajectory_critic.hpp"
namespace cleany_navigation {
// Body orientation follows the planner; translation direction remains holonomic.
class PathOrientationCritic : public dwb_core::TrajectoryCritic {
public:
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
private:
  double target_{0.};
};
}
