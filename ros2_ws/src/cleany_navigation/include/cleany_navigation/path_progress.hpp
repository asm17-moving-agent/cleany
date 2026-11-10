#pragma once
#include "dwb_core/trajectory_critic.hpp"
namespace cleany_navigation {
/// 연속적인 남은 경로 길이로 격자 potential의 평탄 구간을 해소하며
/// 충돌, 속도 또는 도착 수락 조건은 바꾸지 않습니다.
class PathProgressCritic : public dwb_core::TrajectoryCritic {
public:
  bool prepare(const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &,const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
private:
  double project(const geometry_msgs::msg::Pose2D &,double maximum) const;
  nav_2d_msgs::msg::Path2D path_;
  std::vector<double> cumulative_;
  geometry_msgs::msg::Pose2D origin_;
  double origin_progress_{0.};
  bool ready_{false};
};
}
