#pragma once
#include <optional>
#include <vector>
#include "dwb_plugins/standard_traj_generator.hpp"
namespace cleany_axis_controller {
class AxisGenerator : public dwb_plugins::StandardTrajectoryGenerator {
public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr &, const std::string &) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D &) override;
  bool hasMoreTwists() override {return index_ < candidates_.size();}
  nav_2d_msgs::msg::Twist2D nextTwist() override {return candidates_.at(index_++);}
  // A new plan does not erase measured stop/axis history.
  void reset() override {candidates_.clear(); index_=0;}
private:
  rclcpp::Clock::SharedPtr clock_;
  bool allow_lateral_{false};
  double linear_stopped_, angular_stopped_, settle_s_;
  std::optional<double> quiet_since_, last_time_;
  std::vector<nav_2d_msgs::msg::Twist2D> candidates_;
  size_t index_{0};
  int last_axis_{-1};
};
}
