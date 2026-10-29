#include <fstream>
#include <iostream>
#include <cstdlib>
#include "cleany_navigation/mppi/optimizer.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
class Goal final:public nav2_core::GoalChecker {
public:
void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr&,const std::string&,const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)override{}
void reset()override{}
bool isGoalReached(const geometry_msgs::msg::Pose&,const geometry_msgs::msg::Pose&,const geometry_msgs::msg::Twist&)override{return false;}
bool getTolerances(geometry_msgs::msg::Pose &p,geometry_msgs::msg::Twist&)override{p.position.x=p.position.y=.3;return true;}
};
int main(int argc,char**argv){
 rclcpp::init(argc,argv);std::string root=argv[1];
 auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server");
 auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap");
 cm->on_configure(rclcpp_lifecycle::State());
 unsigned nx,ny;double res,ox,oy;std::ifstream(root+"/map.txt")>>nx>>ny>>res>>ox>>oy;
 cm->getLayeredCostmap()->resizeMap(nx,ny,res,ox,oy,true);
 auto map=cm->getCostmap();std::ifstream raw(root+"/map.bin",std::ios::binary);raw.read(reinterpret_cast<char*>(map->getCharMap()),nx*ny);
 std::ifstream in(root+"/poses.txt");double x,y,yaw;in>>x>>y>>yaw;
 geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="odom";pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.z=sin(yaw/2);pose.pose.orientation.w=cos(yaw/2);
 nav_msgs::msg::Path path;path.header.frame_id="odom";
 while(in>>x>>y>>yaw){auto p=pose;p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.z=sin(yaw/2);p.pose.orientation.w=cos(yaw/2);path.poses.push_back(p);}
 nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(map);
 for(size_t i=0;i<path.poses.size();++i){
  const auto &p=path.poses[i].pose;
  std::cout<<"PATH_CHECK,"<<i<<','<<p.position.x<<','<<p.position.y<<','<<tf2::getYaw(p.orientation)<<','<<checker.footprintCostAtPose(p.position.x,p.position.y,tf2::getYaw(p.orientation),cm->getRobotFootprint())<<'\n';
 }
 if(std::getenv("MPPI_REPLAY_PATH_ONLY")){cm->on_cleanup(rclcpp_lifecycle::State());rclcpp::shutdown();return 0;}
 cleany_navigation::mppi::ParametersHandler params(parent);cleany_navigation::mppi::Optimizer optimizer;optimizer.initialize(parent,"FollowPath",cm,&params);Goal goal;geometry_msgs::msg::Twist speed;
 std::cout<<"n,vx,vy,wz,optimized_lethal\n";
 for(int n=0;n<240;n++){
  pose.header.stamp=rclcpp::Time(static_cast<int64_t>(n)*100000000LL);path.header.stamp=pose.header.stamp;
  auto u=optimizer.evalControl(pose,speed,path,&goal).twist;auto traj=optimizer.getOptimizedTrajectory();int hits=0;
  for(size_t k=0;k<traj.shape(0);k++)hits+=checker.footprintCostAtPose(traj(k,0),traj(k,1),traj(k,2),cm->getRobotFootprint())==254;
  std::cout<<n<<','<<u.linear.x<<','<<u.linear.y<<','<<u.angular.z<<','<<hits<<'\n';
 }
 optimizer.shutdown();cm->on_cleanup(rclcpp_lifecycle::State());rclcpp::shutdown();
}
