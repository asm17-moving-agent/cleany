#include "cleany_navigation/path_progress.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace cleany_navigation {
namespace {
bool finite(const geometry_msgs::msg::Pose2D &p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}
}
bool PathProgressCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
  const nav_2d_msgs::msg::Twist2D &,const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &path) {
  ready_=false;path_.poses.clear();cumulative_.clear();
  if(!finite(pose) || path.poses.empty())return false;
  double length=0.;
  for(size_t i=0;i<path.poses.size();++i) {
    if(!finite(path.poses[i]))return false;
    if(i)length+=std::hypot(path.poses[i].x-path.poses[i-1].x,path.poses[i].y-path.poses[i-1].y);
    if(!std::isfinite(length))return false;
    cumulative_.push_back(length);
  }
  path_=path;origin_=pose;origin_progress_=project(pose,length);ready_=true;return true;
}
double PathProgressCritic::project(const geometry_msgs::msg::Pose2D &pose,double maximum) const {
  double best=std::hypot(pose.x-path_.poses.front().x,pose.y-path_.poses.front().y),progress=0.;
  for(size_t i=1;i<path_.poses.size() && cumulative_[i-1]<=maximum;++i) {
    const auto &a=path_.poses[i-1],&b=path_.poses[i];
    const double length=cumulative_[i]-cumulative_[i-1];
    if(length<=0.)continue; // 계획된 제자리 회전에는 병진 진척이 없습니다.
    const double ux=(b.x-a.x)/length,uy=(b.y-a.y)/length;
    const double along=std::clamp((pose.x-a.x)*ux+(pose.y-a.y)*uy,0.,std::min(length,maximum-cumulative_[i-1]));
    const double distance=std::hypot(pose.x-a.x-along*ux,pose.y-a.y-along*uy);
    // 교차점에서 거리가 같으면 가장 이른 분기를 유지합니다. 같은 위치에서
    // 끝났다는 이유만으로 미래의 한 바퀴 전체를 진행한 것으로 계산하지 않습니다.
    if(distance<best-1e-12) {best=distance;progress=cumulative_[i-1]+along;}
  }
  return progress;
}
double PathProgressCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &trajectory) {
  if(!ready_ || trajectory.poses.empty())throw dwb_core::IllegalTrajectoryException(name_,"No prepared path or trajectory");
  double travel=0.;
  for(size_t i=0;i<trajectory.poses.size();++i) {
    if(!finite(trajectory.poses[i]))throw dwb_core::IllegalTrajectoryException(name_,"Non-finite trajectory pose");
    if(i)travel+=std::hypot(trajectory.poses[i].x-trajectory.poses[i-1].x,trajectory.poses[i].y-trajectory.poses[i-1].y);
  }
  if(!std::isfinite(travel) || std::hypot(trajectory.poses.front().x-origin_.x,trajectory.poses.front().y-origin_.y)>1e-6)
    throw dwb_core::IllegalTrajectoryException(name_,"Invalid trajectory origin or travel");
  const double total=cumulative_.back();
  const double end=project(trajectory.poses.back(),std::min(total,origin_progress_+travel));
  return std::max(0.,total-end); // 양자화된 costmap 셀 거리가 아닌 미터 단위입니다.
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::PathProgressCritic,dwb_core::TrajectoryCritic)
