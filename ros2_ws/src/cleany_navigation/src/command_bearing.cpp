#include "cleany_navigation/command_bearing.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace cleany_navigation {
void CommandBearingCritic::onInit() {
  auto node=node_.lock();const auto key=dwb_plugin_name_+"."+name_+".near_goal_distance";
  if(!node->has_parameter(key))node->declare_parameter(key,.3);
  node->get_parameter(key,near_goal_distance_);
  const auto free_key=dwb_plugin_name_+"."+name_+".free_goal_distance";
  if(!node->has_parameter(free_key))node->declare_parameter(free_key,.1);
  node->get_parameter(free_key,free_goal_distance_);
  for(auto entry:{std::make_pair("switch_cost",&switch_cost_),std::make_pair("switch_angle",&switch_angle_)}) {
    const auto parameter=dwb_plugin_name_+"."+name_+"."+entry.first;
    if(!node->has_parameter(parameter))node->declare_parameter(parameter,*entry.second);
    node->get_parameter(parameter,*entry.second);
  }
  if(!std::isfinite(switch_cost_) || switch_cost_<0. || !std::isfinite(switch_angle_) || switch_angle_<=0.)
    throw std::invalid_argument("Invalid CommandBearing switching cost");
  if(!std::isfinite(near_goal_distance_) || near_goal_distance_<=0.)
    throw std::invalid_argument("CommandBearing.near_goal_distance must be positive and finite");
  if(!std::isfinite(free_goal_distance_) || free_goal_distance_<0. ||
    free_goal_distance_>=near_goal_distance_)
    throw std::invalid_argument("CommandBearing.free_goal_distance must be finite and in [0, near_goal_distance)");
}
void CommandBearingCritic::reset() {previous_.reset();weight_=1.;}
bool CommandBearingCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
  const nav_2d_msgs::msg::Twist2D &,const geometry_msgs::msg::Pose2D &goal,
  const nav_2d_msgs::msg::Path2D &) {
  const auto distance=std::hypot(goal.x-pose.x,goal.y-pose.y);
  if(!std::isfinite(distance))return false;
  weight_=std::clamp((distance-free_goal_distance_)/
    (near_goal_distance_-free_goal_distance_),0.,1.);return true;
}
double CommandBearingCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) {
  const auto &v=trajectory.velocity;
  if(!std::isfinite(v.x) || !std::isfinite(v.y))
    throw dwb_core::IllegalTrajectoryException(name_,"Non-finite translation command");
  if(!previous_ || std::hypot(v.x,v.y)<=1e-6)return 0.;
  const auto delta=std::atan2(v.y,v.x)-*previous_;
  const auto angle=std::abs(std::atan2(std::sin(delta),std::cos(delta)));
  return weight_*(angle+switch_cost_*std::min(1.,angle/switch_angle_));
}
void CommandBearingCritic::debrief(const nav_2d_msgs::msg::Twist2D &v) {
  // A zero issued on failure leaves the camera at its last requested bearing.
  if(std::isfinite(v.x) && std::isfinite(v.y) && std::hypot(v.x,v.y)>1e-6)
    previous_=std::atan2(v.y,v.x);
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::CommandBearingCritic,dwb_core::TrajectoryCritic)
