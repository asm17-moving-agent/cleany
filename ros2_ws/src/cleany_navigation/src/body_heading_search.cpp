#include "cleany_navigation/straight_first_planner.hpp"
#include "cleany_navigation/swept_footprint.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <optional>
#include <unordered_map>
namespace cleany_navigation {
nav_msgs::msg::Path checkedBodyHeadingGridPath(
  const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map,const std::vector<geometry_msgs::msg::Point> &footprint,
  unsigned int bins,double cost_penalty,double heuristic_weight,const std::function<bool()> &cancel) {
  nav_msgs::msg::Path empty;empty.header=start.header;
  if(cancel() || bins<4 || bins>128 || !std::isfinite(cost_penalty) || cost_penalty<0. ||
     !std::isfinite(heuristic_weight) || heuristic_weight<1. || start.header.frame_id.empty() ||
     start.header.frame_id!=goal.header.frame_id || footprint.size()<3)return empty;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  const size_t width=map.getSizeInCellsX(),height=map.getSizeInCellsY();
  if(width==0 || height==0 || width>std::numeric_limits<size_t>::max()/height ||
     width*height>std::numeric_limits<size_t>::max()/bins || !std::isfinite(map.getResolution()) || map.getResolution()<=0.)return empty;
  const size_t cells=width*height,none=std::numeric_limits<size_t>::max();
  const double angle_step=2.*M_PI/bins;
  double radius=0.;for(const auto &p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
  const auto yaw=[](const auto &q){return std::atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z));};
  const auto body=[&](const auto &p){geometry_msgs::msg::Pose2D v;v.x=p.pose.position.x;v.y=p.pose.position.y;v.theta=yaw(p.pose.orientation);return v;};
  const auto first=body(start),last=body(goal);
  // Full input and endpoint certification also rejects invalid quaternions.
  auto endpoints=empty;endpoints.poses={start};
  if(!sweptPathCollisionFree(endpoints,map,footprint,cancel))return empty;
  endpoints.poses={goal};if(!sweptPathCollisionFree(endpoints,map,footprint,cancel))return empty;
  unsigned int sx,sy,gx,gy;
  if(!map.worldToMap(first.x,first.y,sx,sy) || !map.worldToMap(last.x,last.y,gx,gy))return empty;
  const auto index=[&](size_t x,size_t y,size_t a){return a*cells+y*width+x;};
  const auto center=[&](size_t n){geometry_msgs::msg::Pose2D p;map.mapToWorld(n%width,(n/width)%height,p.x,p.y);p.theta=(n/cells)*angle_step;return p;};
  const auto connection=[&](const auto &a,auto b){b.theta=a.theta+std::atan2(std::sin(b.theta-a.theta),std::cos(b.theta-a.theta));return sweptPoseConnectionFree(map,footprint,a,b,cancel);};
  // Explicit intermediate headings expose an endpoint turn to consumers
  // which follow the nearest positional path sample. Retain both certified
  // pieces instead of hiding the intended yaw in a moving grid connector.
  const auto split_connection=[&](const auto &a,auto b)->std::optional<geometry_msgs::msg::Pose2D> {
    b.theta=a.theta+std::atan2(std::sin(b.theta-a.theta),std::cos(b.theta-a.theta));
    auto middle=a;middle.theta=b.theta;
    if(connection(a,middle) && connection(middle,b))return middle;
    middle=a;middle.x=b.x;middle.y=b.y;
    if(connection(a,middle) && connection(middle,b))return middle;
    return std::nullopt;
  };
  std::unordered_map<size_t,geometry_msgs::msg::Pose2D> seed_middle;
  struct State {double distance=std::numeric_limits<double>::infinity();size_t parent=std::numeric_limits<size_t>::max();int8_t valid=-1;bool closed=false;};
  // Store explored states only; facility dimensions do not allocate bins full maps.
  std::unordered_map<size_t,State> states;
  struct Entry {double estimate,cost;size_t index;};
  const auto later=[](const Entry&a,const Entry&b){return a.estimate==b.estimate?a.index>b.index:a.estimate>b.estimate;};
  std::priority_queue<Entry,std::vector<Entry>,decltype(later)> open(later);
  const auto heuristic=[&](const auto &p){return heuristic_weight*std::hypot(p.x-last.x,p.y-last.y);};
  const auto safe=[&](size_t n){auto &s=states[n];if(s.valid<0){const auto p=center(n);s.valid=map.getCost(n%width,(n/width)%height)<253 && sweptFootprintFree(map,footprint,p,p);}return bool(s.valid);};
  for(unsigned int a=0;a<bins;++a)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){
    if(cancel())return empty;
    const int64_t x=int64_t(sx)+dx,y=int64_t(sy)+dy;
    if(x<0 || y<0 || x>=int64_t(width) || y>=int64_t(height))continue;
    const auto n=index(x,y,a);const auto p=center(n);
    if(!safe(n))continue;
    const auto middle=split_connection(first,p);if(!middle)continue;seed_middle[n]=*middle;
    const double cost=std::hypot(first.x-p.x,first.y-p.y)+radius*std::abs(std::atan2(std::sin(p.theta-first.theta),std::cos(p.theta-first.theta)));
    states[n].distance=cost;open.push({cost+heuristic(p),cost,n});
  }
  size_t found=none;geometry_msgs::msg::Pose2D goal_middle;
  while(!open.empty()){
    if(cancel())return empty;
    const auto entry=open.top();open.pop();auto &current=states[entry.index];
    if(current.closed || entry.cost>current.distance)continue;
    current.closed=true;
    // Copy before insertions can rehash the sparse state table.
    const double distance=current.distance;const auto p=center(entry.index);
    const int64_t x=entry.index%width,y=(entry.index/width)%height,a=entry.index/cells;
    if(std::abs(x-int64_t(gx))<=1 && std::abs(y-int64_t(gy))<=1){
      const auto middle=split_connection(p,last);if(middle){found=entry.index;goal_middle=*middle;break;}
    }
    const auto consider=[&](int64_t xx,int64_t yy,int64_t aa,double step){
      if(cancel() || xx<0 || yy<0 || xx>=int64_t(width) || yy>=int64_t(height))return;
      aa=(aa+bins)%bins;const auto n=index(xx,yy,aa);
      const double candidate=distance+step*(1.+cost_penalty*map.getCost(xx,yy)/252.);
      if(states[n].closed || candidate>=states[n].distance || !safe(n))return;
      const auto q=center(n);if(!connection(p,q))return;
      states[n].distance=candidate;states[n].parent=entry.index;open.push({candidate+heuristic(q),candidate,n});
    };
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx)if(dx || dy)consider(x+dx,y+dy,a,map.getResolution()*std::hypot(dx,dy));
    consider(x,y,a-1,radius*angle_step);consider(x,y,a+1,radius*angle_step);
  }
  if(found==none || cancel())return empty;
  std::vector<size_t> route;
  for(auto n=found;n!=none;n=states.at(n).parent){if(cancel())return empty;route.push_back(n);}
  std::reverse(route.begin(),route.end());auto path=empty;path.poses.push_back(start);
  const auto append=[&](const auto &p){auto z=start;z.pose.position.x=p.x;z.pose.position.y=p.y;z.pose.orientation={};z.pose.orientation.z=std::sin(p.theta/2.);z.pose.orientation.w=std::cos(p.theta/2.);path.poses.push_back(z);};
  append(seed_middle.at(route.front()));
  for(auto n:route){if(cancel())return empty;append(center(n));}
  append(goal_middle);path.poses.push_back(goal);
  return sweptPathCollisionFree(path,map,footprint,cancel)?path:empty;
}
}
