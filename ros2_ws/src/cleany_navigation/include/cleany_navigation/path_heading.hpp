#pragma once
#include "dwb_core/trajectory_critic.hpp"
namespace cleany_axis_controller {
/// Continuous heading cost gives in-place rotation a benefit even when XY is unchanged.
class PathHeadingCritic : public dwb_core::TrajectoryCritic {
public:
  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
               const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
private:
  double lookahead_{.3}, tolerance_{.12}, goal_tolerance_{.12}, target_{0.}, initial_error_{0.};
  double reverse_threshold_{2.356194490192345};
  double translation_angle_{0.}, travel_target_{0.}, rotation_position_tolerance_{.025};
  bool respect_path_orientation_{false};
  double release_tolerance_{.18};
  bool enabled_{true}, active_{false}, reverse_{false}, aligned_{false};
};
}
