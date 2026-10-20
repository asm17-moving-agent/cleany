#include "cleany_navigation/path_orientation.hpp"
#include <cmath>
#include <limits>
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace cleany_navigation {
bool PathOrientationCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
  const nav_2d_msgs::msg::Twist2D &, const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &path) {
  if(path.poses.empty())return false;
  size_t nearest=0;double distance=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<path.poses.size();++i) {
    const auto &p=path.poses[i];const double d=std::hypot(p.x-pose.x,p.y-pose.y);
    if(d<distance) {distance=d;nearest=i;}
  }
  // Same-position poses represent a planned turn, not a translation tangent.
  const auto &p=path.poses[nearest];
  while(nearest+1<path.poses.size() &&
    std::hypot(path.poses[nearest+1].x-p.x,path.poses[nearest+1].y-p.y)<1e-6)++nearest;
  target_=path.poses[nearest].theta;
  return std::isfinite(target_);
}
double PathOrientationCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) {
  if(trajectory.poses.empty())throw dwb_core::IllegalTrajectoryException(name_,"Empty trajectory");
  const double delta=trajectory.poses.back().theta-target_;
  return std::abs(std::atan2(std::sin(delta),std::cos(delta)));
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::PathOrientationCritic,dwb_core::TrajectoryCritic)
