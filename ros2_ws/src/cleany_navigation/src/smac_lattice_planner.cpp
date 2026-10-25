#include "cleany_navigation/smac_lattice_planner.hpp"
#include "cleany_navigation/straight_first_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cleany_navigation {
#if CLEANY_NAV2_CANCEL_CHECKER
nav_msgs::msg::Path SmacLatticePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal, std::function<bool()> cancel) {
#else
nav_msgs::msg::Path SmacLatticePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) {
  const std::function<bool()> cancel=[] {return false;};
#endif
  {
    std::lock_guard<std::mutex> guard(_mutex);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    nav_msgs::msg::Path empty;
    empty.header.frame_id=_global_frame; empty.header.stamp=_clock->now();
    unsigned int sx,sy,gx,gy;
    if(cancel() || start.header.frame_id!=_global_frame || goal.header.frame_id!=_global_frame ||
      !_costmap->worldToMap(start.pose.position.x,start.pose.position.y,sx,sy) ||
      !_costmap->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy))return empty;
    // Initializes the native motion table before comparing its angular bins.
    _a_star->setCollisionChecker(&_collision_checker);
    auto &table=nav2_smac_planner::NodeLattice::motion_table;
    if(sx==gx && sy==gy &&
      table.getClosestAngularBin(tf2::getYaw(start.pose.orientation))==
      table.getClosestAngularBin(tf2::getYaw(goal.pose.orientation))) {
      // Preserve the exact requested goal, including sub-cell translation and
      // yaw. Unlike an unchecked singleton, validate the whole swept body.
      auto path=checkedHolonomicPath(start,goal,*_costmap,_costmap_ros->getRobotFootprint(),cancel);
      if(!path.poses.empty() && !sweptPathCollisionFree(
          path,*_costmap,_costmap_ros->getRobotFootprint(),cancel))return empty;
      path.header=empty.header;
      for(auto &pose:path.poses)pose.header=path.header;
      return path;
    }
  }
#if CLEANY_NAV2_CANCEL_CHECKER
  return SmacPlannerLattice::createPlan(start,goal,cancel);
#else
  return SmacPlannerLattice::createPlan(start,goal);
#endif
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::SmacLatticePlanner,nav2_core::GlobalPlanner)
