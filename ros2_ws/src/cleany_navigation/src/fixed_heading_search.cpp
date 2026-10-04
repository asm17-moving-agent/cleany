#include "cleany_navigation/straight_first_planner.hpp"
#include "cleany_navigation/swept_footprint.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>

namespace cleany_navigation {
namespace {
double poseYaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
double wrapped(double angle) {return std::atan2(std::sin(angle),std::cos(angle));}
bool validPose(const geometry_msgs::msg::PoseStamped &p) {
  const auto &q=p.pose.orientation;
  const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
  return std::isfinite(p.pose.position.x) && std::isfinite(p.pose.position.y) &&
    std::isfinite(norm) && std::abs(norm-1.)<=1e-3;
}
}
nav_msgs::msg::Path checkedFixedHeadingGridPath(
  const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map,const std::vector<geometry_msgs::msg::Point> &footprint,
  double heading,double cost_penalty,const std::function<bool()> &cancel) {
  nav_msgs::msg::Path empty;empty.header=start.header;
  if(cancel() || start.header.frame_id.empty() || start.header.frame_id!=goal.header.frame_id ||
    !validPose(start) || !validPose(goal) || footprint.size()<3 ||
    !std::isfinite(heading) || !std::isfinite(cost_penalty) || cost_penalty<0.)return empty;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  const size_t width=map.getSizeInCellsX(),height=map.getSizeInCellsY();
  if(!width || !height || !std::isfinite(map.getResolution()) || map.getResolution()<=0.)return empty;
  unsigned int sx,sy,gx,gy;
  if(!map.worldToMap(start.pose.position.x,start.pose.position.y,sx,sy) ||
    !map.worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy))return empty;
  heading=poseYaw(start.pose.orientation)+wrapped(heading-poseYaw(start.pose.orientation));
  const size_t count=width*height,first=size_t(sy)*width+sx,last=size_t(gy)*width+gx;
  const auto center=[&](size_t index) {
    geometry_msgs::msg::Pose2D p;map.mapToWorld(index%width,index/width,p.x,p.y);p.theta=heading;return p;
  };
  geometry_msgs::msg::Pose2D real_start,real_goal;
  real_start.x=start.pose.position.x;real_start.y=start.pose.position.y;real_start.theta=poseYaw(start.pose.orientation);
  real_goal.x=goal.pose.position.x;real_goal.y=goal.pose.position.y;real_goal.theta=heading+wrapped(poseYaw(goal.pose.orientation)-heading);
  auto fixed_start=real_start;fixed_start.theta=real_start.theta+wrapped(heading-real_start.theta);
  auto fixed_goal=real_goal;fixed_goal.theta=heading;
  if(!sweptFootprintFree(map,footprint,real_start,fixed_start) ||
    !sweptFootprintFree(map,footprint,fixed_start,center(first)) ||
    !sweptFootprintFree(map,footprint,center(last),fixed_goal) ||
    !sweptFootprintFree(map,footprint,fixed_goal,real_goal) || cancel())return empty;
  std::vector<int8_t> valid(count,-1);
  const auto traversable=[&](size_t index) {
    if(valid[index]<0) {
      const auto cost=map.getCost(index%width,index/width);
      const auto p=center(index);
      valid[index]=cost<253 && sweptFootprintFree(map,footprint,p,p);
    }
    return valid[index]==1;
  };
  if(!traversable(first) || !traversable(last))return empty;
  struct Entry {double estimate,cost;size_t index;};
  const auto later=[](const Entry &a,const Entry &b) {
    return a.estimate==b.estimate?a.index>b.index:a.estimate>b.estimate;
  };
  std::priority_queue<Entry,std::vector<Entry>,decltype(later)> open(later);
  std::vector<double> distance(count,std::numeric_limits<double>::infinity());
  std::vector<size_t> parent(count,count);
  const auto heuristic=[&](size_t index) {
    return std::hypot(double(index%width)-gx,double(index/width)-gy)*map.getResolution();
  };
  distance[first]=0.;open.push({heuristic(first),0.,first});
  bool found=false;
  while(!open.empty()) {
    if(cancel())return empty;
    const auto current=open.top();open.pop();
    if(current.cost>distance[current.index])continue;
    if(current.index==last) {found=true;break;}
    const auto from=center(current.index);
    const int64_t x=current.index%width,y=current.index/width;
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      if(cancel())return empty;
      const int64_t nx=x+dx,ny=y+dy;
      if((dx==0 && dy==0) || nx<0 || ny<0 || nx>=int64_t(width) || ny>=int64_t(height))continue;
      const size_t next=size_t(ny)*width+nx;
      const double step=std::hypot(dx,dy)*map.getResolution();
      const double candidate=current.cost+step*(1.+cost_penalty*map.getCost(nx,ny)/252.);
      if(candidate>=distance[next] || !traversable(next))continue;
      // Endpoint checks alone do not certify diagonal edges or corner contact.
      if(!sweptFootprintFree(map,footprint,from,center(next)))continue;
      distance[next]=candidate;parent[next]=current.index;
      open.push({candidate+heuristic(next),candidate,next});
    }
  }
  if(!found || cancel())return empty;
  std::vector<size_t> indices;
  for(size_t index=last;;index=parent[index]) {
    if(cancel() || index>=count)return empty;
    indices.push_back(index);if(index==first)break;
  }
  std::reverse(indices.begin(),indices.end());
  auto path=empty;path.poses.push_back(start);
  const auto append=[&](const geometry_msgs::msg::Pose2D &v) {
    auto p=start;p.pose.position.x=v.x;p.pose.position.y=v.y;
    p.pose.orientation=geometry_msgs::msg::Quaternion();
    p.pose.orientation.z=std::sin(heading/2);p.pose.orientation.w=std::cos(heading/2);
    path.poses.push_back(p);
  };
  append(fixed_start);
  for(auto index:indices)append(center(index));
  append(fixed_goal);path.poses.push_back(goal);
  return cancel()?empty:path;
}
}
