#pragma once
#include "nav2_smac_planner/smac_planner_lattice.hpp"

namespace cleany_navigation {
/// Smac Lattice with a collision-checked same-cell terminal connection.
/// Humble 1.1.20 cannot backtrace a search whose start is already the goal node.
class SmacLatticePlanner : public nav2_smac_planner::SmacPlannerLattice {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
#if CLEANY_NAV2_CANCEL_CHECKER
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &, std::function<bool()>) override;
#else
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &) override;
#endif
private:
  bool validate_swept_path_{false};
  double swept_repair_timeout_{2.0};
  unsigned int body_heading_bins_{32};
  double body_heading_weight_{8.0};
  double cost_penalty_{2.0};
};
}
