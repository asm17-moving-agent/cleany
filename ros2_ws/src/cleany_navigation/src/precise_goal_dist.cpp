#include "cleany_navigation/precise_goal_dist.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace cleany_navigation {
void PreciseGoalDistCritic::onInit() {
  dwb_critics::GoalDistCritic::onInit();
  auto node=node_.lock();const auto key=dwb_plugin_name_+"."+name_+".near_goal_distance";
  if(!node->has_parameter(key))node->declare_parameter(key,.3);
  node->get_parameter(key,near_goal_distance_);
  if(!std::isfinite(near_goal_distance_) || near_goal_distance_<=0.)
    throw std::invalid_argument(key+" must be positive and finite");
}
bool PreciseGoalDistCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
  const nav_2d_msgs::msg::Twist2D &velocity,const geometry_msgs::msg::Pose2D &goal,
  const nav_2d_msgs::msg::Path2D &path) {
  weight_=0.;
  const auto distance=std::hypot(goal.x-pose.x,goal.y-pose.y);
  if(!std::isfinite(distance))return false;
  if(!dwb_critics::GoalDistCritic::prepare(pose,velocity,goal,path))return false;
  goal_=goal;
  // The actual goal, rather than a truncated local-plan endpoint, is used only
  // near arrival. Far from the goal, stock Manhattan guidance is unchanged.
  weight_=std::max(0.,1.-distance/near_goal_distance_);
  return true;
}
double PreciseGoalDistCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) {
  if(trajectory.poses.empty())
    throw dwb_core::IllegalTrajectoryException(name_,"Empty trajectory");
  for(const auto &p:trajectory.poses) {
    if(!std::isfinite(p.x) || !std::isfinite(p.y))
      throw dwb_core::IllegalTrajectoryException(name_,"Non-finite trajectory pose");
  }
  const double grid=dwb_critics::GoalDistCritic::scoreTrajectory(trajectory);
  if(weight_==0.)return grid;
  const auto &end=trajectory.poses.back();
  const double distance=std::hypot(end.x-goal_.x,end.y-goal_.y);
  if(!std::isfinite(distance))
    throw dwb_core::IllegalTrajectoryException(name_,"Non-finite endpoint");
  return grid+weight_*distance/costmap_->getResolution();
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::PreciseGoalDistCritic,dwb_core::TrajectoryCritic)
