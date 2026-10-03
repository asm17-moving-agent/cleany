#pragma once
#include <functional>
#include "nav2_core/global_planner.hpp"
namespace cleany_navigation {
bool pathCollisionFree(const nav_msgs::msg::Path &path, nav2_costmap_2d::Costmap2D &map,
  const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;});
nav_msgs::msg::Path checkedStraightPath(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map, const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;});
class StraightFirstPlanner : public nav2_core::GlobalPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override {fallback_->cleanup(); costmap_.reset();}
  void activate() override {fallback_->activate();}
  void deactivate() override {fallback_->deactivate();}
#if CLEANY_NAV2_CANCEL_CHECKER
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &, std::function<bool()>) override;
#else
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::PoseStamped &) override;
#endif
private:
  nav2_core::GlobalPlanner::Ptr fallback_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
};
}
