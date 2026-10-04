#include "cleany_navigation/straight_first_planner.hpp"
#include "cleany_navigation/swept_footprint.hpp"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <cstdint>
#include <numeric>
#include <fstream>
#include <nlohmann/json.hpp>
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
bool validSweptInput(const nav_msgs::msg::Path &route,
  const std::vector<geometry_msgs::msg::Point> &footprint,const std::function<bool()> &cancel) {
  if(route.header.frame_id.empty() || route.poses.empty() || footprint.size()<3 || cancel())return false;
  for(const auto &p:route.poses) {
    const auto &q=p.pose.orientation;
    const auto norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(p.header.frame_id!=route.header.frame_id || !std::isfinite(norm) ||
      std::abs(norm-1.)>1e-3 || !std::isfinite(p.pose.position.x) ||
      !std::isfinite(p.pose.position.y))return false;
  }
  return true;
}
}
bool sweptPathCollisionFree(const nav_msgs::msg::Path &route,
  nav2_costmap_2d::Costmap2D &map,const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel) {
  if(!validSweptInput(route,footprint,cancel))return false;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  const auto &path=route;
  auto previous=path.poses.front().pose;
  geometry_msgs::msg::Pose2D a;a.x=previous.position.x;a.y=previous.position.y;
  a.theta=yaw(previous.orientation);
  if(!sweptFootprintFree(map,footprint,a,a))return false;
  for(const auto &p:path.poses) {
    if(cancel())return false;
    geometry_msgs::msg::Pose2D b;b.x=p.pose.position.x;b.y=p.pose.position.y;
    b.theta=a.theta+wrap(yaw(p.pose.orientation)-a.theta);
    if(!sweptFootprintFree(map,footprint,a,b))return false;
    a=b;
  }
  return !cancel();
}
nav_msgs::msg::Path checkedFixedHeadingPath(const nav_msgs::msg::Path &route,
  nav2_costmap_2d::Costmap2D &map,const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel) {
  nav_msgs::msg::Path empty;empty.header=route.header;
  if(!validSweptInput(route,footprint,cancel))return empty;
  // This is an alternative-path constructor. Live-map verification instead
  // calls sweptPathCollisionFree so it cannot rewrite an accepted heading.
  for(const double heading:{yaw(route.poses.front().pose.orientation),
                            yaw(route.poses.back().pose.orientation)}) {
    if(cancel())return empty;
    auto candidate=empty;candidate.poses.push_back(route.poses.front());
    for(auto p:route.poses) {
      p.pose.orientation=geometry_msgs::msg::Quaternion();
      p.pose.orientation.z=std::sin(heading/2);p.pose.orientation.w=std::cos(heading/2);
      candidate.poses.push_back(p);
    }
    candidate.poses.push_back(route.poses.back());
    if(sweptPathCollisionFree(candidate,map,footprint,cancel))return candidate;
  }
  return empty;
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
    const auto exact_key=name+".require_exact_goal";
    if(!node->has_parameter(exact_key))node->declare_parameter(exact_key,false);
    exact_goal_=node->get_parameter(exact_key).as_bool();
    const auto holonomic_key=name+".holonomic_direct_path";
    if(!node->has_parameter(holonomic_key))node->declare_parameter(holonomic_key,false);
    holonomic_=node->get_parameter(holonomic_key).as_bool();
    fixed_heading_cost_penalty_=node->get_parameter(name+".cost_penalty").as_double();
    const auto evidence_key=name+".failure_snapshot_directory";
    if(!node->has_parameter(evidence_key))node->declare_parameter(evidence_key,std::string{});
    evidence_directory_=node->get_parameter(evidence_key).as_string();
    evidence_node_=parent;
    if(!std::isfinite(padding_) || padding_<0)throw std::invalid_argument(key);
#if CLEANY_NAV2_CANCEL_CHECKER
    _collision_checker=nav2_smac_planner::GridCollisionChecker(map,720u,node);
#else
    _collision_checker=nav2_smac_planner::GridCollisionChecker(_costmap,720u,node);
#endif
    live_costmap_=_costmap;
    snapshot_=std::make_unique<nav2_costmap_2d::Costmap2D>();
    _costmap=snapshot_.get();
    _collision_checker.setCostmap(_costmap);
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
    if(!std::isfinite(_max_planning_time) || _max_planning_time<=0.)
      throw std::invalid_argument("max_planning_time must be finite and positive");
    const auto deadline=std::chrono::steady_clock::now()+
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(_max_planning_time));
    const auto expired=[&] {return cancel() || std::chrono::steady_clock::now()>=deadline;};
    // Copy bytes and geometry together, then release the live map for updates,
    // IsPathValid and heartbeat callbacks throughout the expensive search.
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live_costmap_->getMutex());
      *snapshot_=*live_costmap_;
    }
    nav_msgs::msg::Path plan;plan.header=start.header;plan.header.stamp=_clock->now();
    const auto footprint=_costmap_ros->getRobotFootprint();
    nlohmann::json attempts=nlohmann::json::array();
    const auto finish=[&](const std::string &reason) {
      const auto now=std::chrono::steady_clock::now();
      const double total_s=_max_planning_time-std::chrono::duration<double>(deadline-now).count();
      RCLCPP_WARN(_logger,"Lattice plan rejected: %s, attempts=%zu elapsed=%.3f budget=%.3f",reason.c_str(),attempts.size(),total_s,_max_planning_time);
      if(!evidence_directory_.empty()) {
        try {
          nav2_costmap_2d::Costmap2D live;
          {std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live_costmap_->getMutex());live=*live_costmap_;}
          nlohmann::json record={{"reason",reason},{"attempts",attempts},
            {"elapsed_s",total_s},{"budget_s",_max_planning_time},{"budget_expired",now>=deadline},
            {"frame_id",start.header.frame_id},{"start",{start.pose.position.x,start.pose.position.y,yaw(start.pose.orientation)}},
            {"goal",{goal.pose.position.x,goal.pose.position.y,yaw(goal.pose.orientation)}},
            {"stamp_ns",_clock->now().nanoseconds()},{"footprint",nlohmann::json::array()}};
          for(const auto &p:footprint)record["footprint"].push_back({p.x,p.y});
          if(auto node=evidence_node_.lock())for(const auto &key:node->list_parameters({},100).names) {
            const auto value=node->get_parameter(key);
            record["parameters"][key]={{"type",value.get_type_name()},{"value",value.value_to_string()}};
          }
          const std::filesystem::path directory(evidence_directory_);std::filesystem::create_directories(directory);
          const auto write=[&](const std::string &name,const char *data,size_t size) {
            auto temp=directory/(name+".tmp");std::ofstream file(temp,std::ios::binary);
            file.exceptions(std::ios::badbit|std::ios::failbit);file.write(data,size);file.close();
            std::filesystem::rename(temp,directory/name);
          };
          const auto save_map=[&](const std::string &name,nav2_costmap_2d::Costmap2D &map) {
            const auto size=size_t(map.getSizeInCellsX())*map.getSizeInCellsY();const auto bytes=map.getCharMap();
            uint64_t hash=14695981039346656037ull;for(size_t i=0;i<size;++i){hash^=bytes[i];hash*=1099511628211ull;}
            record[name]={{"size",{map.getSizeInCellsX(),map.getSizeInCellsY()}},{"resolution",map.getResolution()},
              {"origin",{map.getOriginX(),map.getOriginY()}},{"fnv1a64",std::to_string(hash)},{"file",name+".bin"}};
            write(name+".bin",reinterpret_cast<const char*>(bytes),size);
          };
          save_map("search",*_costmap);save_map("live",live);
          const auto metadata=record.dump(2);write("failure.json",metadata.data(),metadata.size());
        } catch(const std::exception &e) {RCLCPP_WARN(_logger,"Unable to record planner failure: %s",e.what());}
      }
      return plan;
    };
    unsigned int sx,sy,gx,gy;
    if(cancel() || !_costmap->worldToMap(start.pose.position.x,start.pose.position.y,sx,sy) ||
       !_costmap->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy))return finish("cancelled_or_outside_map");
    if(holonomic_) {
      // A tangent-oriented lattice route can be too close to a corner to
      // reorient safely. Search XY for the desired body heading directly.
      // Reserve roughly half the shared budget for the rotating fallback.
      for(const double heading:{yaw(goal.pose.orientation),yaw(start.pose.orientation)}) {
        const auto heading_deadline=std::min(deadline,std::chrono::steady_clock::now()+
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(_max_planning_time/4.)));
        const auto heading_expired=[&] {return expired() || std::chrono::steady_clock::now()>=heading_deadline;};
        auto candidate=checkedFixedHeadingGridPath(start,goal,*_costmap,footprint,
          heading,fixed_heading_cost_penalty_,heading_expired);
        if(candidate.poses.empty())continue;
        if(sweptPathCollisionFree(candidate,*live_costmap_,footprint,expired)) {
          RCLCPP_INFO(_logger,"Using collision-checked fixed-heading XY search (%zu poses)",candidate.poses.size());
          return candidate;
        }
      }
    }
    // Additional search clearance is optional. The robot footprint and its
    // configured safety padding are required for every returned path.
    std::vector<double> margins{padding_,std::max(0.,padding_-_costmap->getResolution()/2),0.};
    margins.erase(std::unique(margins.begin(),margins.end()),margins.end());
    for(size_t attempt=0;attempt<margins.size();++attempt) {
      if(expired())return finish("cancelled_or_budget_expired");
      const double margin=margins[attempt];
      // Reserve a fair share for smaller clearance attempts rather than spend
      // the entire budget on the most conservative optional margin.
      const double remaining=std::chrono::duration<double>(deadline-std::chrono::steady_clock::now()).count();
      if(remaining<=0.)return finish("budget_expired");
#if !CLEANY_NAV2_CANCEL_CHECKER
      _a_star->initialize(_allow_unknown,_max_iterations,_max_on_approach_iterations,
        remaining/(margins.size()-attempt),_lookup_table_size,_metadata.number_of_headings);
#endif
      auto conservative=footprint;nav2_costmap_2d::padFootprint(conservative,margin);
      _collision_checker.setFootprint(conservative,false,0.0);
      _a_star->setCollisionChecker(&_collision_checker);
      // Snapping a valid continuous pose to the closest lattice angle can
      // rotate its corners into an obstacle. Try other heading seeds only
      // through a collision-checked rotation/translation from the real pose.
      std::vector<unsigned int> headings(_metadata.heading_angles.size());
      std::iota(headings.begin(),headings.end(),0u);
      std::stable_sort(headings.begin(),headings.end(),[&](unsigned int a,unsigned int b) {
        return std::abs(wrap(_metadata.heading_angles[a]-yaw(start.pose.orientation)))<
               std::abs(wrap(_metadata.heading_angles[b]-yaw(start.pose.orientation)));
      });
      nav_msgs::msg::Path connection;connection.header=plan.header;
      unsigned int start_heading=0;bool seed_found=false;
      double seed_x,seed_y;_costmap->mapToWorld(sx,sy,seed_x,seed_y);
      for(const auto heading:headings) {
        if(expired())return finish("cancelled_or_budget_expired_selecting_start");
        const double theta=_metadata.heading_angles[heading];
        const double bin=theta/(2*M_PI/_collision_checker.getPrecomputedAngles().size());
        if(_collision_checker.inCollision(sx,sy,bin,_allow_unknown))continue;
        auto rotated=start;rotated.pose.orientation=geometry_msgs::msg::Quaternion();
        rotated.pose.orientation.z=std::sin(theta/2);rotated.pose.orientation.w=std::cos(theta/2);
        auto seed=rotated;seed.pose.position.x=seed_x;seed.pose.position.y=seed_y;
        connection.poses={start,rotated,seed};
        bool connected=pathCollisionFree(connection,*_costmap,footprint,expired);
        if(!connected) {
          auto shifted=start;shifted.pose.position.x=seed_x;shifted.pose.position.y=seed_y;
          connection.poses={start,shifted,seed};
          connected=pathCollisionFree(connection,*_costmap,footprint,expired);
        }
        if(!connected) {
          connection.poses={start,seed};
          connected=pathCollisionFree(connection,*_costmap,footprint,expired);
        }
        if(connected) {
          start_heading=heading;seed_found=true;break;
        }
      }
      if(!seed_found) {
        attempts.push_back({{"margin",margin},{"iterations",0},{"reason","no_safe_start_connection"}});
        RCLCPP_WARN(_logger,"No collision-free connection from continuous start to a lattice heading (clearance %.3f)",margin);
        continue;
      }
      _a_star->setStart(sx,sy,start_heading);
      _a_star->setGoal(gx,gy,nav2_smac_planner::NodeLattice::motion_table.getClosestAngularBin(yaw(goal.pose.orientation)));
      nav2_smac_planner::NodeLattice::CoordinateVector coordinates;int iterations=0;bool found=false;
      const auto attempt_start=std::chrono::steady_clock::now();
      try {
#if CLEANY_NAV2_CANCEL_CHECKER
        const auto attempt_deadline=std::chrono::steady_clock::now()+
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(remaining/(margins.size()-attempt)));
        found=_a_star->createPath(coordinates,iterations,_tolerance/_costmap->getResolution(),
          [&] {return expired() || std::chrono::steady_clock::now()>=attempt_deadline;});
#else
        found=_a_star->createPath(coordinates,iterations,_tolerance/_costmap->getResolution());
#endif
      } catch(const std::runtime_error &e) {
        RCLCPP_WARN(_logger,"Lattice search with additional %.3f m clearance failed: %s",margin,e.what());
        attempts.push_back({{"margin",margin},{"iterations",iterations},{"exception",e.what()}});
        continue;
      }
      const double search_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-attempt_start).count();
      attempts.push_back({{"margin",margin},{"iterations",iterations},{"found",found},{"search_s",search_s},
        {"start_heading",_metadata.heading_angles[start_heading]}});
      RCLCPP_INFO(_logger,"Lattice attempt clearance=%.3f found=%d iterations=%d search_s=%.3f",margin,found,iterations,search_s);
      if(!found || expired())continue;
      plan.poses.clear();
      for(auto i=coordinates.rbegin();i!=coordinates.rend();++i) {
        geometry_msgs::msg::PoseStamped p;p.header=plan.header;
        p.pose=nav2_smac_planner::getWorldCoords(i->x,i->y,_costmap);
        p.pose.orientation=nav2_smac_planner::getWorldOrientation(i->theta);plan.poses.push_back(p);
      }
      plan.poses.insert(plan.poses.begin(),connection.poses.begin(),connection.poses.end());
      // A tolerance endpoint is useful during search, but desk arrival needs
      // the original requested pose. Validate its connecting sweep as well.
      if(exact_goal_)plan.poses.push_back(goal);
      if(pathCollisionFree(plan,*_costmap,footprint,expired)) {
        if(holonomic_) {
          // Extra lattice clearance guides search; it is not the acceptance
          // footprint. Apply the same configured body/padding as the original
          // route, otherwise a safe fixed-yaw route can be discarded while
          // the rotating route below is accepted with less padding.
          auto fixed=checkedFixedHeadingPath(plan,*_costmap,footprint,expired);
          if(!fixed.poses.empty()) {
            // Independently check the same route on the latest live map. Its
            // endpoint orientations still encode any necessary final rotation.
            if(sweptPathCollisionFree(fixed,*live_costmap_,footprint,expired)) {
              RCLCPP_INFO(_logger,"Using fixed body heading on collision-checked lattice XY route (%zu poses)",fixed.poses.size());
              return fixed;
            }
          }
        }
        // A snapshot is for search only. Newly observed live obstacles must
        // still veto the entire continuous body sweep before returning it.
        if(pathCollisionFree(plan,*live_costmap_,footprint,expired))return plan;
        plan.poses.clear();
        return finish("live_sweep_rejected_or_budget_expired");
      }
      RCLCPP_WARN(_logger,"Rejecting lattice path with an invalid continuous footprint sweep (additional clearance %.3f m)",margin);
      plan.poses.clear();
    }
    return finish("no_valid_lattice_path");
  }
private:
  std::string evidence_directory_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr evidence_node_;
  double padding_{.05};
  bool exact_goal_{false};
  bool holonomic_{false};
  double fixed_heading_cost_penalty_{5.};
  nav2_costmap_2d::Costmap2D *live_costmap_{nullptr};
  std::unique_ptr<nav2_costmap_2d::Costmap2D> snapshot_;
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
nav_msgs::msg::Path checkedHolonomicPath(
  const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal,
  nav2_costmap_2d::Costmap2D &map, const std::vector<geometry_msgs::msg::Point> &footprint,
  const std::function<bool()> &cancel) {
  nav_msgs::msg::Path empty;empty.header=start.header;
  if(start.header.frame_id!=goal.header.frame_id || footprint.size()<3)return empty;
  const auto &a=start.pose.position;const auto &b=goal.pose.position;
  const double from=yaw(start.pose.orientation),to=yaw(goal.pose.orientation);
  if(!std::isfinite(a.x+a.y+b.x+b.y+from+to))return empty;
  // A holonomic base need not turn toward its translation vector. Prefer
  // keeping its current body orientation; if the destination lacks turning
  // room, also try turning at the start. Both full sweeps are mandatory.
  for(bool turn_first:{false,true}) {
    nav_msgs::msg::Path path;path.header=start.header;
    auto append=[&](double x,double y,double theta) {
      auto p=start;p.pose.position.x=x;p.pose.position.y=y;
      p.pose.orientation=geometry_msgs::msg::Quaternion();
      p.pose.orientation.z=std::sin(theta/2);p.pose.orientation.w=std::cos(theta/2);
      path.poses.push_back(p);
    };
    const auto rotate=[&](double x,double y) {
      double radius=map.getResolution();
      for(const auto &p:footprint)radius=std::max(radius,std::hypot(p.x,p.y));
      const int count=std::max(1,int(std::ceil(radius*std::abs(wrap(to-from))/(map.getResolution()/2))));
      for(int i=0;i<=count;++i)append(x,y,from+wrap(to-from)*i/count);
    };
    if(turn_first)rotate(a.x,a.y);else append(a.x,a.y,from);
    const int count=std::max(1,int(std::ceil(std::hypot(b.x-a.x,b.y-a.y)/(map.getResolution()/2))));
    for(int i=1;i<=count;++i)append(a.x+(b.x-a.x)*i/count,a.y+(b.y-a.y)*i/count,turn_first?to:from);
    if(!turn_first)rotate(b.x,b.y);
    if(pathCollisionFree(path,map,footprint,cancel))return path;
    if(cancel())return empty;
  }
  return empty;
}
void StraightFirstPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
  std::string name,std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) {
  costmap_=costmap;
  auto node=parent.lock();const auto key=name+".holonomic_direct_path";
  if(!node->has_parameter(key))node->declare_parameter(key,false);
  holonomic_=node->get_parameter(key).as_bool();
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
  auto direct=holonomic_?
    checkedHolonomicPath(start,goal,*costmap_->getCostmap(),costmap_->getRobotFootprint(),cancel):
    checkedStraightPath(start,goal,*costmap_->getCostmap(),costmap_->getRobotFootprint(),cancel);
  if (!direct.poses.empty() || cancel()) return direct;
#if CLEANY_NAV2_CANCEL_CHECKER
  return fallback_->createPlan(start,goal,cancel);
#else
  return fallback_->createPlan(start,goal);
#endif
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::StraightFirstPlanner,nav2_core::GlobalPlanner)
