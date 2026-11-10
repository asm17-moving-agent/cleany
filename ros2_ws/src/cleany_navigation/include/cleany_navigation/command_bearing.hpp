#pragma once
#include <optional>
#include "dwb_core/trajectory_critic.hpp"
namespace cleany_navigation {
/// 이동 방향을 금지하지 않으면서 연속적인 병진 방위를 선호합니다.
class CommandBearingCritic : public dwb_core::TrajectoryCritic {
public:
  void onInit() override;
  void reset() override;
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
  void debrief(const nav_2d_msgs::msg::Twist2D &) override;
private:
  std::optional<double> previous_;
  double near_goal_distance_{.3}, free_goal_distance_{.1}, weight_{1.};
  double switch_cost_{0.}, switch_angle_{.04};
};
}
