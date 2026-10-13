#pragma once
#include <optional>
#include "dwb_core/dwb_local_planner.hpp"
namespace cleany_navigation {
// Preserve DWB's transform, pruning, critic preparation and scoring. Supply
// only synchronous pose/goal context for extra near-goal velocity candidates.
class GoalDirectedController : public dwb_core::DWBLocalPlanner {
public:
  using dwb_core::DWBLocalPlanner::computeVelocityCommands;
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override;
  void setPlan(const nav_msgs::msg::Path &path) override;
  nav_2d_msgs::msg::Twist2DStamped computeVelocityCommands(
    const nav_2d_msgs::msg::Pose2DStamped &pose,
    const nav_2d_msgs::msg::Twist2D &velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> &results) override;
private:
  struct Selected {nav_2d_msgs::msg::Pose2DStamped pose; nav_2d_msgs::msg::Twist2D velocity;};
  std::optional<Selected> selected_;
  double continuity_timeout_{.5};
};
}
