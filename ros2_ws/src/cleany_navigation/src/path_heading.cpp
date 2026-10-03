#include "cleany_navigation/path_heading.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace cleany_axis_controller {
namespace {double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}}
void PathHeadingCritic::onInit() {
  auto node=node_.lock();
  auto parameter=[&](const std::string &key, double fallback) {
    const auto full=dwb_plugin_name_+"."+name_+"."+key;
    if (!node->has_parameter(full)) node->declare_parameter(full, fallback);
    auto value=node->get_parameter(full).as_double();
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument(full);
    return value;
  };
  const auto orientation_key=dwb_plugin_name_+"."+name_+".respect_path_orientation";
  if (!node->has_parameter(orientation_key)) node->declare_parameter(orientation_key,false);
  respect_path_orientation_=node->get_parameter(orientation_key).as_bool();
  rotation_position_tolerance_=parameter("rotation_position_tolerance_m", .025);
  lookahead_=parameter("lookahead_m", .3);
  tolerance_=parameter("heading_tolerance", .12);
  release_tolerance_=parameter("heading_release_tolerance", .18);
  if (release_tolerance_ < tolerance_) throw std::invalid_argument("heading release tolerance");
  reverse_threshold_=parameter("reverse_angle_threshold", .75*M_PI);
  if (reverse_threshold_>M_PI) throw std::invalid_argument("reverse_angle_threshold must be <= pi");
  if (node->has_parameter(dwb_plugin_name_+".axis_translation_angle_deg"))
    translation_angle_=node->get_parameter(dwb_plugin_name_+".axis_translation_angle_deg").as_double()*M_PI/180.;
  if (!std::isfinite(translation_angle_) || translation_angle_<0 || translation_angle_>M_PI/18+1e-12)
    throw std::invalid_argument("axis_translation_angle_deg");
  if (node->has_parameter(dwb_plugin_name_+".xy_goal_tolerance"))
    goal_tolerance_=node->get_parameter(dwb_plugin_name_+".xy_goal_tolerance").as_double();
  if (node->has_parameter(dwb_plugin_name_+".axis_allow_lateral"))
    enabled_=!node->get_parameter(dwb_plugin_name_+".axis_allow_lateral").as_bool();
}
bool PathHeadingCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
    const nav_2d_msgs::msg::Twist2D &, const geometry_msgs::msg::Pose2D &goal,
    const nav_2d_msgs::msg::Path2D &path) {
  active_=enabled_ && std::hypot(goal.x-pose.x, goal.y-pose.y)>goal_tolerance_;
  if (!active_) {aligned_=false;return true;}  // RotateToGoal owns the final heading.
  if (path.poses.empty()) return false;
  size_t nearest=0; double distance=std::numeric_limits<double>::infinity();
  for (size_t i=0;i<path.poses.size();++i) {
    const auto &p=path.poses[i];const auto d=std::hypot(p.x-pose.x,p.y-pose.y);
    if (d<distance) {distance=d;nearest=i;}
  }
  auto target=path.poses.back();
  for (size_t i=nearest;i<path.poses.size();++i) {
    const auto &p=path.poses[i];
    if (std::hypot(p.x-pose.x,p.y-pose.y)>=lookahead_) {target=p;break;}
  }
  if (std::hypot(target.x-pose.x,target.y-pose.y)<1e-6) {active_=false;return true;}
  target_=std::atan2(target.y-pose.y,target.x-pose.x);
  reverse_=std::abs(wrap(target_-pose.theta))>reverse_threshold_;
  if (reverse_) target_=wrap(target_+M_PI);
  travel_target_=target_;
  if (respect_path_orientation_) {
    travel_target_=std::atan2(target.y-pose.y,target.x-pose.x);
    target_=path.poses[nearest].theta;
    // Duplicate XY poses encode an explicit in-place rotation. Execute the
    // final heading of that group only after arriving at its position.
    for (size_t i=nearest;i+1<path.poses.size();++i) {
      const auto &p=path.poses[i];
      if (std::hypot(p.x-pose.x,p.y-pose.y)>lookahead_) break;
      size_t end=i;
      while (end+1<path.poses.size() && std::hypot(path.poses[end+1].x-p.x,path.poses[end+1].y-p.y)<1e-6) ++end;
      if (end>i && std::hypot(p.x-pose.x,p.y-pose.y)<=rotation_position_tolerance_) {target_=path.poses[end].theta;break;}
      i=end;
    }
    reverse_=std::abs(wrap(travel_target_-target_))>M_PI/2;
    if (reverse_) travel_target_=wrap(travel_target_+M_PI);
  }
  initial_error_=std::abs(wrap(target_-pose.theta));
  const double cone=respect_path_orientation_ ? 0. : translation_angle_;
  if (initial_error_ > cone+release_tolerance_) aligned_=false;
  else if (initial_error_ <= cone+tolerance_) aligned_=true;
  return true;
}
double PathHeadingCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) {
  if (!active_) return 0.;
  if (trajectory.poses.empty()) throw dwb_core::IllegalTrajectoryException(name_, "Empty trajectory");
  const auto x=trajectory.velocity.x;
  if (std::abs(x)>1e-6 && (!aligned_ || (x<0)!=reverse_))
    throw dwb_core::IllegalTrajectoryException(name_, "Align heading before longitudinal travel");
  // Stay on a longitudinal segment once aligned; rotate again only outside
  // the release band. This prevents tiny heading improvements winning forever.
  if (aligned_ && std::abs(trajectory.velocity.theta)>1e-6)
    throw dwb_core::IllegalTrajectoryException(name_, "Keep aligned segment heading");
  const double offset=std::abs(x)>1e-6 ? std::atan2(trajectory.velocity.y*(x<0?-1:1),std::abs(x)) : 0.;
  const double body_error=std::abs(wrap(target_-trajectory.poses.back().theta));
  if (respect_path_orientation_)
    return body_error+(aligned_ ? std::abs(wrap(travel_target_-trajectory.poses.back().theta-offset)) : 0.);
  return std::abs(wrap(target_-trajectory.poses.back().theta-offset));
}
}
PLUGINLIB_EXPORT_CLASS(cleany_axis_controller::PathHeadingCritic, dwb_core::TrajectoryCritic)
