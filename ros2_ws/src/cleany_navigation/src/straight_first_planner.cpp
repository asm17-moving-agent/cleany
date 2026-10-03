#include "cleany_navigation/straight_first_planner.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
namespace cleany_navigation {
namespace {
double yaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}
bool inside(double x, double y, const std::vector<geometry_msgs::msg::Point> &f) {
  bool result=false;
  for (size_t i=0,j=f.size()-1;i<f.size();j=i++)
    if ((f[i].y>y)!=(f[j].y>y) &&
      x<(f[j].x-f[i].x)*(y-f[i].y)/(f[j].y-f[i].y)+f[i].x) result=!result;
  return result;
}
}
bool pathCollisionFree(const nav_msgs::msg::Path &path, nav2_costmap_2d::Costmap2D &map,
  const std::vector<geometry_msgs::msg::Point> &footprint, const std::function<bool()> &cancel) {
  if (path.poses.empty() || footprint.size()<3) return false;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&map);
  double radius=0.;for(const auto &p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
  auto valid=[&](double x,double y,double theta) {
    unsigned int mx,my;
    if(cancel() || !std::isfinite(x+y+theta) || !map.worldToMap(x,y,mx,my) || map.getCost(mx,my)>=253) return false;
    const auto cost=checker.footprintCostAtPose(x,y,theta,footprint);
    if(cost<0 || cost>=254)return false;
    std::vector<geometry_msgs::msg::Point> oriented;nav2_costmap_2d::transformFootprint(x,y,theta,footprint,oriented);
    unsigned int minx=map.getSizeInCellsX(),miny=map.getSizeInCellsY(),maxx=0,maxy=0;
    for(const auto &p:oriented) {
      if(!map.worldToMap(p.x,p.y,mx,my))return false;
      minx=std::min(minx,mx);maxx=std::max(maxx,mx);miny=std::min(miny,my);maxy=std::max(maxy,my);
    }
    for(unsigned int i=minx;i<=maxx;++i)for(unsigned int j=miny;j<=maxy;++j) {
      double wx,wy;map.mapToWorld(i,j,wx,wy);
      if(inside(wx,wy,oriented) && map.getCost(i,j)>=254)return false;
    }
    return true;
  };
  auto previous=path.poses.front().pose;
  if(!valid(previous.position.x,previous.position.y,yaw(previous.orientation)))return false;
  for(const auto &stamped:path.poses) {
    const auto &next=stamped.pose;const double angle=wrap(yaw(next.orientation)-yaw(previous.orientation));
    const double distance=std::hypot(next.position.x-previous.position.x,next.position.y-previous.position.y);
    const int count=std::max(1,int(std::ceil((distance+radius*std::abs(angle))/(map.getResolution()/2))));
    for(int i=1;i<=count;++i) {
      const double f=double(i)/count;
      if(!valid(previous.position.x+f*(next.position.x-previous.position.x),previous.position.y+f*(next.position.y-previous.position.y),yaw(previous.orientation)+f*angle))return false;
    }
    previous=next;
  }
  return true;
}

// Reuse Nav2's lattice search while keeping a private conservative footprint.
// The public costmap and the executor's footprint remain unchanged.
class ConservativeLatticePlanner : public nav2_smac_planner::SmacPlannerLattice {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    SmacPlannerLattice::configure(parent,name,tf,map);
    auto node=parent.lock();const auto key=name+".lattice_collision_padding";
    if(!node->has_parameter(key))node->declare_parameter(key,.05);
    padding_=node->get_parameter(key).as_double();
    if(!std::isfinite(padding_) || padding_<0)throw std::invalid_argument(key);
#if CLEANY_NAV2_CANCEL_CHECKER
    _collision_checker=nav2_smac_planner::GridCollisionChecker(map,720u,node);
#else
    _collision_checker=nav2_smac_planner::GridCollisionChecker(_costmap,720u,node);
#endif
  }
#if CLEANY_NAV2_CANCEL_CHECKER
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,std::function<bool()> cancel) override {
#else
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) override {
    const std::function<bool()> cancel=[] {return false;};
#endif
    std::lock_guard<std::mutex> guard(_mutex);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    nav_msgs::msg::Path plan;plan.header=start.header;plan.header.stamp=_clock->now();
    const auto footprint=_costmap_ros->getRobotFootprint();
    unsigned int sx,sy,gx,gy;
    if(cancel() || !_costmap->worldToMap(start.pose.position.x,start.pose.position.y,sx,sy) ||
       !_costmap->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy))return plan;
    // Additional search clearance is optional. The robot footprint and its
    // configured safety padding are required for every returned path.
    std::vector<double> margins{padding_,std::max(0.,padding_-_costmap->getResolution()/2),0.};
    margins.erase(std::unique(margins.begin(),margins.end()),margins.end());
    for(const double margin:margins) {
      if(cancel())return plan;
      auto conservative=footprint;nav2_costmap_2d::padFootprint(conservative,margin);
      _collision_checker.setFootprint(conservative,false,0.0);
      _a_star->setCollisionChecker(&_collision_checker);
      _a_star->setStart(sx,sy,nav2_smac_planner::NodeLattice::motion_table.getClosestAngularBin(yaw(start.pose.orientation)));
      _a_star->setGoal(gx,gy,nav2_smac_planner::NodeLattice::motion_table.getClosestAngularBin(yaw(goal.pose.orientation)));
      nav2_smac_planner::NodeLattice::CoordinateVector coordinates;int iterations=0;bool found=false;
      try {
#if CLEANY_NAV2_CANCEL_CHECKER
        found=_a_star->createPath(coordinates,iterations,_tolerance/_costmap->getResolution(),cancel);
#else
        found=_a_star->createPath(coordinates,iterations,_tolerance/_costmap->getResolution());
#endif
      } catch(const std::runtime_error &e) {
        RCLCPP_WARN(_logger,"Lattice search with additional %.3f m clearance failed: %s",margin,e.what());
        continue;
      }
      if(!found || cancel())continue;
      plan.poses.clear();
      for(auto i=coordinates.rbegin();i!=coordinates.rend();++i) {
        geometry_msgs::msg::PoseStamped p;p.header=plan.header;
        p.pose=nav2_smac_planner::getWorldCoords(i->x,i->y,_costmap);
        p.pose.orientation=nav2_smac_planner::getWorldOrientation(i->theta);plan.poses.push_back(p);
      }
      plan.poses.insert(plan.poses.begin(),start);
      if(pathCollisionFree(plan,*_costmap,footprint,cancel))return plan;
      RCLCPP_WARN(_logger,"Rejecting lattice path with an invalid continuous footprint sweep (additional clearance %.3f m)",margin);
      plan.poses.clear();
    }
    return plan;
  }
private:
  double padding_{.05};
};

nav_msgs::msg::Path checkedStraightPath(
  const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map, const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel) {
  nav_msgs::msg::Path path;path.header=start.header;
  if (start.header.frame_id!=goal.header.frame_id || footprint.size()<3) return path;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(&map);
  double radius=map.getResolution();
  for (const auto &p:footprint) radius=std::max(radius,std::hypot(p.x,p.y));
  const double step=map.getResolution()/2;
  auto append=[&](double x,double y,double theta) {
    if (cancel() || !std::isfinite(x+y+theta)) return false;
    unsigned int mx,my;
    if (!map.worldToMap(x,y,mx,my) || map.getCost(mx,my)>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false;
    auto cost=checker.footprintCostAtPose(x,y,theta,footprint);
    if (cost<0 || cost>=nav2_costmap_2d::LETHAL_OBSTACLE) return false;
    // Nav2's boundary checker alone can miss a small obstacle inside the body.
    std::vector<geometry_msgs::msg::Point> oriented;
    nav2_costmap_2d::transformFootprint(x,y,theta,footprint,oriented);
    unsigned int minx=map.getSizeInCellsX(),miny=map.getSizeInCellsY(),maxx=0,maxy=0;
    for (const auto &p:oriented) {
      if (!map.worldToMap(p.x,p.y,mx,my)) return false;
      minx=std::min(minx,mx);maxx=std::max(maxx,mx);miny=std::min(miny,my);maxy=std::max(maxy,my);
    }
    for (unsigned int i=minx;i<=maxx;++i) for (unsigned int j=miny;j<=maxy;++j) {
      double wx,wy;map.mapToWorld(i,j,wx,wy);
      if (inside(wx,wy,oriented) && map.getCost(i,j)>=nav2_costmap_2d::LETHAL_OBSTACLE) return false;
    }
    auto p=start;p.pose.position.x=x;p.pose.position.y=y;
    p.pose.orientation=geometry_msgs::msg::Quaternion();p.pose.orientation.z=std::sin(theta/2);p.pose.orientation.w=std::cos(theta/2);
    path.poses.push_back(p);return true;
  };
  const auto &a=start.pose.position;const auto &b=goal.pose.position;
  const double distance=std::hypot(b.x-a.x,b.y-a.y);
  const double heading=distance>1e-6?std::atan2(b.y-a.y,b.x-a.x):yaw(start.pose.orientation);
  // The heading critic may follow a path behind the robot in reverse. Leave
  // those goals to the lattice so its body orientation remains authoritative.
  if (distance>1e-6 && std::abs(wrap(heading-yaw(start.pose.orientation)))>M_PI/2) return path;
  auto rotate=[&](double x,double y,double from,double to) {
    const double delta=wrap(to-from);
    const int count=std::max(1,int(std::ceil(std::abs(delta)*radius/step)));
    for (int i=0;i<=count;++i) if (!append(x,y,from+delta*i/count)) return false;
    return true;
  };
  if (!rotate(a.x,a.y,yaw(start.pose.orientation),heading)) {path.poses.clear();return path;}
  const int count=std::max(1,int(std::ceil(distance/step)));
  for (int i=1;i<=count;++i) if (!append(a.x+(b.x-a.x)*i/count,a.y+(b.y-a.y)*i/count,heading)) {
    path.poses.clear();return path;
  }
  if (!rotate(b.x,b.y,heading,yaw(goal.pose.orientation))) path.poses.clear();
  return path;
}
void StraightFirstPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
  std::string name,std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
  costmap_=costmap;
  fallback_=std::make_shared<ConservativeLatticePlanner>();
  fallback_->configure(parent,name,tf,costmap);
}
#if CLEANY_NAV2_CANCEL_CHECKER
nav_msgs::msg::Path StraightFirstPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal,std::function<bool()> cancel) {
#else
nav_msgs::msg::Path StraightFirstPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
  const geometry_msgs::msg::PoseStamped &goal) {
  const std::function<bool()> cancel=[] {return false;};
#endif
  auto direct=checkedStraightPath(start,goal,*costmap_->getCostmap(),costmap_->getRobotFootprint(),cancel);
  if (!direct.poses.empty() || cancel()) return direct;
#if CLEANY_NAV2_CANCEL_CHECKER
  return fallback_->createPlan(start,goal,cancel);
#else
  return fallback_->createPlan(start,goal);
#endif
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::StraightFirstPlanner,nav2_core::GlobalPlanner)
