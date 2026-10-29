#include "cleany_navigation/smac_lattice_planner.hpp"
#include "cleany_navigation/straight_first_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>

namespace cleany_navigation {
void SmacLatticePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) {
  SmacPlannerLattice::configure(parent,name,tf,map);
  auto node=parent.lock();
  const auto parameter=[&](const std::string &key,auto value) {
    if(!node->has_parameter(name+"."+key))node->declare_parameter(name+"."+key,value);
    return node->get_parameter(name+"."+key);
  };
  validate_swept_path_=parameter("validate_swept_path",false).as_bool();
  swept_repair_timeout_=parameter("swept_repair_timeout",2.0).as_double();
  const auto bins=parameter("body_heading_grid_bins",32).as_int();
  body_heading_weight_=parameter("body_heading_grid_heuristic_weight",8.0).as_double();
  cost_penalty_=node->get_parameter(name+".cost_penalty").as_double();
  if(bins<4 || bins>128 || !std::isfinite(swept_repair_timeout_) || swept_repair_timeout_<=0. ||
    !std::isfinite(body_heading_weight_) || body_heading_weight_<1.) {
    throw std::invalid_argument("Invalid continuous-footprint refinement settings");
  }
  body_heading_bins_=static_cast<unsigned int>(bins);
}
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
  const auto began=std::chrono::steady_clock::now();
  bool refine_start_or_goal=false;
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
    if(validate_swept_path_) {
      // A valid continuous pose can become invalid after snapping its center
      // and heading to the native lattice/collision bins. Plan from the real
      // pose in that case instead of reporting a false occupied-start failure.
      const double angle_step=2.*M_PI/_collision_checker.getPrecomputedAngles().size();
      const auto collision=[&](unsigned int x,unsigned int y,const auto &pose) {
        const auto bin=table.getClosestAngularBin(tf2::getYaw(pose.orientation));
        return _collision_checker.inCollision(x,y,table.getAngleFromBin(bin)/angle_step,_allow_unknown);
      };
      refine_start_or_goal=collision(sx,sy,start.pose) || collision(gx,gy,goal.pose);
    }
  }
  nav_msgs::msg::Path path;
  if(!refine_start_or_goal) {
#if CLEANY_NAV2_CANCEL_CHECKER
    path=SmacPlannerLattice::createPlan(start,goal,cancel);
#else
    path=SmacPlannerLattice::createPlan(start,goal);
#endif
    if(!validate_swept_path_ || path.poses.empty())return path;
    // Certify both real endpoint connections, including sub-cell accuracy.
    // Native lattice output terminates at grid centers, which can be outside
    // the tighter arrival tolerance needed by a desk manipulation task.
    path.poses.insert(path.poses.begin(),start);
    path.poses.push_back(goal);
  }
  std::lock_guard<std::mutex> guard(_mutex);
  const auto deadline=std::min(
    began+std::chrono::duration<double>(_max_planning_time),
    std::chrono::steady_clock::now()+std::chrono::duration<double>(swept_repair_timeout_));
  const auto expired=[&] {return cancel() || std::chrono::steady_clock::now()>=deadline;};
  const auto footprint=_costmap_ros->getRobotFootprint();
  if(!refine_start_or_goal && sweptPathCollisionFree(path,*_costmap,footprint,expired))return path;
  // Reuse Cleany's existing continuous SE(2) search when sampled lattice
  // primitives cut a corner between checked poses. Never emit that path.
  auto refined=checkedBodyHeadingGridPath(start,goal,*_costmap,footprint,
    body_heading_bins_,cost_penalty_,body_heading_weight_,expired);
  RCLCPP_INFO(_logger,"Continuous-footprint lattice refinement: %zu poses",refined.poses.size());
  return refined;
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::SmacLatticePlanner,nav2_core::GlobalPlanner)
