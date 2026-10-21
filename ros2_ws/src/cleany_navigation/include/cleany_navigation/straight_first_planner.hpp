#pragma once
#include <functional>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/global_planner.hpp"
namespace cleany_navigation {
/// Certify the entire linear center/yaw connection with conservative sweeps
/// on short angular intervals. The caller holds the costmap mutex.
bool sweptPoseConnectionFree(const nav2_costmap_2d::Costmap2D &,
  const std::vector<geometry_msgs::msg::Point> &,
  const geometry_msgs::msg::Pose2D &,const geometry_msgs::msg::Pose2D &,
  const std::function<bool()> &cancel = [] {return false;});
/// Validate the exact supplied poses, without choosing another yaw strategy.
bool sweptPathCollisionFree(const nav_msgs::msg::Path &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  const std::function<bool()> &cancel = [] {return false;});
/// Search XY at a fixed body yaw. Use an immutable planning map: search holds
/// its mutex. Endpoint rotations, grid connections and every edge are swept.
nav_msgs::msg::Path checkedFixedHeadingGridPath(
  const geometry_msgs::msg::PoseStamped &,const geometry_msgs::msg::PoseStamped &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  double heading,double cost_penalty,
  const std::function<bool()> &cancel = [] {return false;});
/// Search translation and in-place yaw with the full continuous footprint.
/// Weighted A* chooses a feasible route, not a guaranteed minimum-cost route.
nav_msgs::msg::Path checkedBodyHeadingGridPath(
  const geometry_msgs::msg::PoseStamped &,const geometry_msgs::msg::PoseStamped &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  unsigned int bins,double cost_penalty,double heuristic_weight,
  const std::function<bool()> &cancel = [] {return false;});
nav_msgs::msg::Path checkedFixedHeadingPath(const nav_msgs::msg::Path &,
  nav2_costmap_2d::Costmap2D &,const std::vector<geometry_msgs::msg::Point> &,
  const std::function<bool()> &cancel = [] {return false;});
bool pathCollisionFree(const nav_msgs::msg::Path &path, nav2_costmap_2d::Costmap2D &map,
  const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;});
nav_msgs::msg::Path checkedStraightPath(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map, const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;});
nav_msgs::msg::Path checkedHolonomicPath(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map, const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel = [] {return false;});
/**
 * @brief Try footprint-checked direct paths before invoking the fallback planner.
 * Operates inside Nav2's lifecycle and costmap ownership; this plugin does not
 * publish velocity commands or relax downstream collision checks.
 */
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
  bool holonomic_{false};
  nav2_core::GlobalPlanner::Ptr fallback_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
};
}
