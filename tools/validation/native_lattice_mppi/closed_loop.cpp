#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <iomanip>
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "cleany_navigation/mppi/optimizer.hpp"
#include "cleany_navigation/mppi/wheel_projection.hpp"
#include "cleany_navigation/straight_first_planner.hpp"

class Goal final:public nav2_core::GoalChecker {
 public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,const std::string&,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override{}
  void reset()override{}
  bool isGoalReached(const geometry_msgs::msg::Pose&,const geometry_msgs::msg::Pose&,
    const geometry_msgs::msg::Twist&)override{return false;}
  bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist&)override{
    p.position.x=p.position.y=.03;p.orientation.z=sin(.1/2);p.orientation.w=cos(.1/2);return true;
  }
};
double angle(const geometry_msgs::msg::Quaternion&q){return atan2(2*q.w*q.z,1-2*q.z*q.z);}
double wrap(double a){return atan2(sin(a),cos(a));}
int main(int argc,char**argv){
 if(argc<3){std::cerr<<"Usage: closed_loop FIXTURE SPEED_RATIO --ros-args --params-file CONFIG\n";return 64;}
 std::cout<<std::setprecision(10);
 rclcpp::init(argc,argv);const std::string root=argv[1];
 const double slowdown=std::stod(argv[2]);
 auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server");
 auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
 cm->on_configure(rclcpp_lifecycle::State());
 unsigned nx,ny;double res,ox,oy;std::ifstream(root+"/map.txt")>>nx>>ny>>res>>ox>>oy;
 cm->getLayeredCostmap()->resizeMap(nx,ny,res,ox,oy,true);auto map=cm->getCostmap();
 std::ifstream(root+"/map.bin",std::ios::binary).read(reinterpret_cast<char*>(map->getCharMap()),nx*ny);
 std::ifstream in(root+"/poses.txt");
 auto read=[&](){double x,y,a;in>>x>>y>>a;geometry_msgs::msg::PoseStamped p;p.header.frame_id="odom";
   p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=sin(a/2);p.pose.orientation.w=cos(a/2);return p;};
 auto pose=read(),goal=read();
 const auto requested_goal=goal;
 nav_msgs::msg::Path goal_check;goal_check.poses={goal};
 std::cout<<"GOAL_VALID "<<cleany_navigation::pathCollisionFree(goal_check,*map,cm->getRobotFootprint())<<'\n';
 auto planner_node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server");
 nav2_smac_planner::SmacPlannerLattice planner;
 planner.configure(planner_node,"GridBased",nullptr,cm);planner.activate();
 double planning_ms=0,maximum_planning_ms=0;size_t planning_calls=0;const double initial_yaw=angle(pose.pose.orientation);
 double total_rotation=0,max_deviation=0;int collisions=0;bool success=false;
 cleany_navigation::mppi::ParametersHandler params(parent);cleany_navigation::mppi::Optimizer optimizer;
 optimizer.initialize(parent,"FollowPath",cm,&params);Goal checker;geometry_msgs::msg::Twist speed;
 nav_msgs::msg::Path path;size_t steps=0;
 std::cout<<"step,x,y,yaw,vx,vy,wz\n";
 for(;steps<2400;++steps){
   const double dx=goal.pose.position.x-pose.pose.position.x,dy=goal.pose.position.y-pose.pose.position.y;
   // ControllerServer checks the global path endpoint, which Smac quantizes.
   // Report the separate requested-goal error rather than hiding quantization.
   if(!path.poses.empty()) {
     const auto &end=path.poses.back().pose;
     if(std::hypot(end.position.x-pose.pose.position.x,end.position.y-pose.pose.position.y)<.03 &&
       std::abs(wrap(angle(end.orientation)-angle(pose.pose.orientation)))<.1){success=true;break;}
   }
   if(steps%20==0){
     const auto begin=std::chrono::steady_clock::now();
     path=planner.createPlan(pose,goal);
     if(steps==0 && std::getenv("SNAP_PREVIEW_GOAL") && !path.poses.empty()){
       goal=path.poses.back();
       std::cout<<"SELECTED_GOAL "<<goal.pose.position.x<<' '<<goal.pose.position.y<<' '<<angle(goal.pose.orientation)<<'\n';
     }
     const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
     planning_ms+=elapsed;maximum_planning_ms=std::max(maximum_planning_ms,elapsed);++planning_calls;
     if(steps==0){std::ofstream planned(root+"/plan.csv");planned<<"x,y,yaw\n";
       for(const auto &p:path.poses)planned<<p.pose.position.x<<','<<p.pose.position.y<<','<<angle(p.pose.orientation)<<'\n';}
     if(path.poses.empty())break;
   }
   auto nearest=std::min_element(path.poses.begin(),path.poses.end(),[&](const auto&a,const auto&b){
     return std::hypot(a.pose.position.x-pose.pose.position.x,a.pose.position.y-pose.pose.position.y)<
       std::hypot(b.pose.position.x-pose.pose.position.x,b.pose.position.y-pose.pose.position.y);});
   path.poses.erase(path.poses.begin(),nearest);
   nav_msgs::msg::Path local=path;double distance=0;
   for(size_t i=1;i<local.poses.size();++i){distance+=std::hypot(local.poses[i].pose.position.x-local.poses[i-1].pose.position.x,local.poses[i].pose.position.y-local.poses[i-1].pose.position.y);if(distance>1.5){local.poses.resize(i);break;}}
   pose.header.stamp=rclcpp::Time(static_cast<int64_t>(steps)*50000000LL);local.header.stamp=pose.header.stamp;
   auto command=optimizer.evalControl(pose,speed,local,&checker).twist;
   float x=command.linear.x,y=command.linear.y,z=command.angular.z;
   cleany_navigation::mppi::projectWheelTwist(x,y,z,.0635f,.47f,4.f);
   cleany_navigation::mppi::projectWheelTwist(x,y,z,.0635f,.47f,4.f*slowdown);
   speed.linear.x=x;speed.linear.y=y;speed.angular.z=z;
   const double a=angle(pose.pose.orientation),next=a+.05*speed.angular.z;
   pose.pose.position.x+=.05*(cos(a)*speed.linear.x-sin(a)*speed.linear.y);
   pose.pose.position.y+=.05*(sin(a)*speed.linear.x+cos(a)*speed.linear.y);
   pose.pose.orientation.z=sin(next/2);pose.pose.orientation.w=cos(next/2);
   total_rotation+=std::abs(.05*speed.angular.z);max_deviation=std::max(max_deviation,std::abs(wrap(next-initial_yaw)));
   nav_msgs::msg::Path check;check.header=pose.header;check.poses={pose};
   if(!cleany_navigation::pathCollisionFree(check,*map,cm->getRobotFootprint())){++collisions;break;}
   std::cout<<steps<<','<<pose.pose.position.x<<','<<pose.pose.position.y<<','<<next<<','<<speed.linear.x<<','<<speed.linear.y<<','<<speed.angular.z<<'\n';
 }
 std::cout<<"RESULT "<<success<<' '<<steps<<' '<<total_rotation<<' '<<max_deviation<<' '<<collisions<<'\n';
 std::cout<<"REQUEST_ERROR "<<std::hypot(requested_goal.pose.position.x-pose.pose.position.x,
   requested_goal.pose.position.y-pose.pose.position.y)<<' '<<std::abs(wrap(angle(requested_goal.pose.orientation)-angle(pose.pose.orientation)))<<'\n';
 std::cout<<"GOAL_ERROR "<<std::hypot(goal.pose.position.x-pose.pose.position.x,
   goal.pose.position.y-pose.pose.position.y)<<' '<<std::abs(wrap(angle(goal.pose.orientation)-angle(pose.pose.orientation)))<<'\n';
 std::cout<<"PLANNING "<<planning_calls<<' '<<planning_ms<<' '<<maximum_planning_ms<<'\n';
 planner.deactivate();planner.cleanup();
 optimizer.shutdown();cm->on_cleanup(rclcpp_lifecycle::State());rclcpp::shutdown();
 return success&&collisions==0?0:2;
}
