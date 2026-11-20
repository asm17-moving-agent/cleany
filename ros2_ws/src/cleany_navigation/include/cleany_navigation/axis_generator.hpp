#pragma once
#include <optional>
#include <vector>
#include "dwb_plugins/standard_traj_generator.hpp"
namespace cleany_axis_controller {
/**
 * @brief 측정된 정지 및 정착 판정으로 축이 제한된 DWB 후보를 생성합니다.
 * 계획 reset은 후보를 지우지만 축 변경 전후의 움직임 이력은 유지합니다.
 */
class AxisGenerator : public dwb_plugins::StandardTrajectoryGenerator {
public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr &, const std::string &) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D &) override;
  bool hasMoreTwists() override {return index_ < candidates_.size();}
  nav_2d_msgs::msg::Twist2D nextTwist() override {return candidates_.at(index_++);}
  // 새 계획은 측정된 정지 및 축 이력을 지우지 않습니다.
  void reset() override {candidates_.clear(); index_=0;}
private:
  rclcpp::Clock::SharedPtr clock_;
  bool allow_lateral_{false};
  double translation_angle_{0.};
  double linear_stopped_, angular_stopped_, settle_s_;
  std::optional<double> quiet_since_, last_time_;
  std::vector<nav_2d_msgs::msg::Twist2D> candidates_;
  size_t index_{0};
  int last_axis_{-1};
};
}
