#include "cleany_navigation/swept_footprint.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace cleany_navigation {
namespace {
struct Point {double x,y;};
double cross(const Point &a,const Point &b,const Point &c) {
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}
std::vector<Point> hull(std::vector<Point> points) {
  std::sort(points.begin(),points.end(),[](const auto &a,const auto &b) {
    return a.x<b.x || (a.x==b.x && a.y<b.y);
  });
  points.erase(std::unique(points.begin(),points.end(),[](const auto &a,const auto &b) {
    return a.x==b.x && a.y==b.y;
  }),points.end());
  if(points.size()<3)return {};
  std::vector<Point> result;
  for(const auto &p:points) {
    while(result.size()>1 && cross(result[result.size()-2],result.back(),p)<=0.)result.pop_back();
    result.push_back(p);
  }
  const size_t lower=result.size();
  for(auto i=points.rbegin()+1;i!=points.rend();++i) {
    while(result.size()>lower && cross(result[result.size()-2],result.back(),*i)<=0.)result.pop_back();
    result.push_back(*i);
  }
  result.pop_back();return result;
}
}
bool sweptFootprintFree(const nav2_costmap_2d::Costmap2D &map,
  const std::vector<geometry_msgs::msg::Point> &footprint,
  const geometry_msgs::msg::Pose2D &a,const geometry_msgs::msg::Pose2D &b) {
  if(footprint.size()<3 || !std::isfinite(a.x) || !std::isfinite(a.y) ||
    !std::isfinite(a.theta) || !std::isfinite(b.x) || !std::isfinite(b.y) ||
    !std::isfinite(b.theta) || !std::isfinite(map.getResolution()) ||
    map.getResolution()<=0. || !std::isfinite(map.getOriginX()) ||
    !std::isfinite(map.getOriginY()))return false;
  double radius=0.,area=0.;
  double min_x=std::numeric_limits<double>::infinity(),max_x=-min_x,min_y=min_x,max_y=-min_x;
  const double ca=std::cos(a.theta),sa=std::sin(a.theta),cb=std::cos(b.theta),sb=std::sin(b.theta);
  for(size_t i=0;i<footprint.size();++i) {
    const auto &p=footprint[i],&next=footprint[(i+1)%footprint.size()];
    if(!std::isfinite(p.x) || !std::isfinite(p.y))return false;
    radius=std::max(radius,std::hypot(p.x,p.y));
    area+=p.x*next.y-p.y*next.x;
    for(const auto &v:std::initializer_list<Point>{{a.x+ca*p.x-sa*p.y,a.y+sa*p.x+ca*p.y},
      {b.x+cb*p.x-sb*p.y,b.y+sb*p.x+cb*p.y}}) {
      min_x=std::min(min_x,v.x);max_x=std::max(max_x,v.x);
      min_y=std::min(min_y,v.y);max_y=std::max(max_y,v.y);
    }
  }
  if(!std::isfinite(area) || std::abs(area)<1e-12)return false;
  // 회전 정점 각각의 선형 보간 오차는
  // max|p''(t)|/8 = radius * delta_yaw^2/8로 제한됩니다. 끝점의 볼록 껍질을
  // 이 한계만큼 확장하면 표본뿐 아니라 모든 중간 footprint를 포함합니다.
  const double angle=b.theta-a.theta;
  // body 좌표계 twist를 내보내면 yaw 변화 중 중심도 곡선을 그립니다.
  // 해당 구간에서 twist가 일정할 때 Euler 중심 선분과의 편차는
  // segment_length * |delta_yaw|/2로 제한됩니다.
  const double padding=radius*angle*angle/8.+
    std::hypot(b.x-a.x,b.y-a.y)*std::abs(angle)/2.+
    1e-10*(1.+std::max({std::abs(min_x),std::abs(max_x),std::abs(min_y),std::abs(max_y)}));
  if(!std::isfinite(padding) || !std::isfinite(min_x-padding) ||
    !std::isfinite(max_x+padding) || !std::isfinite(min_y-padding) ||
    !std::isfinite(max_y+padding))return false;
  unsigned int x0,y0,x1,y1;
  if(min_x-padding<map.getOriginX() || min_y-padding<map.getOriginY() ||
    max_x+padding>=map.getOriginX()+map.getSizeInCellsX()*map.getResolution() ||
    max_y+padding>=map.getOriginY()+map.getSizeInCellsY()*map.getResolution())return false;
  if(!map.worldToMap(min_x-padding,min_y-padding,x0,y0) ||
    !map.worldToMap(max_x+padding,max_y+padding,x1,y1))return false;
  std::vector<Point> points;points.reserve(footprint.size()*2);
  for(const auto &p:footprint) {
    points.push_back({a.x+ca*p.x-sa*p.y,a.y+sa*p.x+ca*p.y});
    points.push_back({b.x+cb*p.x-sb*p.y,b.y+sb*p.x+cb*p.y});
  }
  auto polygon=hull(std::move(points));if(polygon.size()<3)return false;
  struct Axis {double x,y,low,high;};std::vector<Axis> axes;
  for(size_t i=0;i<polygon.size();++i) {
    const auto &p=polygon[i],&n=polygon[(i+1)%polygon.size()];
    const double nx=p.y-n.y,ny=n.x-p.x;
    double low=std::numeric_limits<double>::infinity(),high=-low;
    for(const auto &v:polygon) {const double dot=v.x*nx+v.y*ny;low=std::min(low,dot);high=std::max(high,dot);}
    const double expansion=padding*std::hypot(nx,ny);
    axes.push_back({nx,ny,low-expansion,high+expansion});
  }
  const double half=map.getResolution()/2.;
  for(unsigned int y=y0;y<=y1;++y)for(unsigned int x=x0;x<=x1;++x) {
    const auto cost=map.getCost(x,y);
    if(cost!=nav2_costmap_2d::LETHAL_OBSTACLE && cost!=nav2_costmap_2d::NO_INFORMATION)continue;
    double cx,cy;map.mapToWorld(x,y,cx,cy);
    if(cx+half<min_x-padding || cx-half>max_x+padding ||
      cy+half<min_y-padding || cy-half>max_y+padding)continue;
    bool separated=false;
    for(const auto &axis:axes) {
      const double center=cx*axis.x+cy*axis.y;
      const double extent=half*(std::abs(axis.x)+std::abs(axis.y));
      if(center+extent<axis.low || center-extent>axis.high) {separated=true;break;}
    }
    if(!separated)return false;
  }
  return true;
}
}
