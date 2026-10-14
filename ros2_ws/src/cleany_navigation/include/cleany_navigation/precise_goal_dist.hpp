#pragma once
#include "dwb_critics/goal_dist.hpp"
namespace cleany_navigation {
/// Preserve stock grid guidance and failures, resolving its final-cell plateau.
class PreciseGoalDistCritic : public dwb_critics::GoalDistCritic {
public:
  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
private:
  geometry_msgs::msg::Pose2D goal_;
  double near_goal_distance_{.3}, weight_{0.};
};
}
