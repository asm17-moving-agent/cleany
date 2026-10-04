#include <gtest/gtest.h>
#include <fstream>
#include <limits>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include "cleany_navigation/swept_footprint.hpp"
#include "cleany_navigation/holonomic_generator.hpp"
#include "dwb_core/exceptions.hpp"
using cleany_navigation::sweptFootprintFree;
TEST(SweptFootprint, TranslationRotationInteriorUnknownAndBoundaries) {
  nav2_costmap_2d::Costmap2D map(40,40,.05,-1,-1,0);
  std::vector<geometry_msgs::msg::Point> footprint(4);
  const double xs[]={-.2,-.2,.2,.2},ys[]={-.1,.1,.1,-.1};
  for(int i=0;i<4;++i){footprint[i].x=xs[i];footprint[i].y=ys[i];}
  geometry_msgs::msg::Pose2D a,b;a.x=-.5;b.x=.5;
  EXPECT_TRUE(sweptFootprintFree(map,footprint,a,b));
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(0,0,x,y));map.setCost(x,y,254);
  EXPECT_TRUE(sweptFootprintFree(map,footprint,a,a));
  EXPECT_TRUE(sweptFootprintFree(map,footprint,b,b));
  EXPECT_FALSE(sweptFootprintFree(map,footprint,a,b));
  map.setCost(x,y,253);EXPECT_TRUE(sweptFootprintFree(map,footprint,a,b));
  map.setCost(x,y,255);a.x=0;b=a;EXPECT_FALSE(sweptFootprintFree(map,footprint,a,b));
  map.setCost(x,y,0);a.x=.9;b=a;EXPECT_FALSE(sweptFootprintFree(map,footprint,a,b));
  EXPECT_FALSE(sweptFootprintFree(map,{},a,b));
  auto invalid=a;invalid.theta=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sweptFootprintFree(map,footprint,invalid,b));
  for(int i=0;i<4;++i)footprint[i].x=xs[i]*2.5;
  a.x=0;a.theta=0;b=a;b.theta=M_PI/2;
  ASSERT_TRUE(map.worldToMap(.35,.35,x,y));map.setCost(x,y,254);
  EXPECT_TRUE(sweptFootprintFree(map,footprint,a,a));
  EXPECT_TRUE(sweptFootprintFree(map,footprint,b,b));
  EXPECT_FALSE(sweptFootprintFree(map,footprint,a,b));
}
TEST(SweptFootprint, RotationArcOutsideEndpointHullIsIncluded) {
  nav2_costmap_2d::Costmap2D map(80,80,.02,-.9905,-.81,0);
  std::vector<geometry_msgs::msg::Point> footprint(4);
  const double xs[]={-.5,-.5,.5,.5},ys[]={-.1,.1,.1,-.1};
  for(int i=0;i<4;++i){footprint[i].x=xs[i];footprint[i].y=ys[i];}
  geometry_msgs::msg::Pose2D a,b,mid;
  mid.theta=-std::atan2(.1,.5);a=mid;b=mid;a.theta-=.05;b.theta+=.05;
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(.51,0,x,y));map.setCost(x,y,254);
  EXPECT_TRUE(sweptFootprintFree(map,footprint,a,a));
  EXPECT_TRUE(sweptFootprintFree(map,footprint,b,b));
  EXPECT_FALSE(sweptFootprintFree(map,footprint,mid,mid));
  EXPECT_FALSE(sweptFootprintFree(map,footprint,a,b));
}
static bool recordedCommandRejectedUnderSweptCellModel() {
  rclcpp::init(0,nullptr);nlohmann::json data;std::ifstream(CLEANY_SWEPT_FIXTURE)>>data;
  auto node=std::make_shared<nav2_util::LifecycleNode>("swept_regression");
  for(const auto &[key,value]:std::vector<std::pair<std::string,double>>{
    {"min_vel_x",-.15},{"max_vel_x",.15},{"min_vel_y",-.15},{"max_vel_y",.15},
    {"max_vel_theta",.3},{"max_speed_xy",.15},{"min_speed_xy",0.},{"min_speed_theta",0.},
    {"acc_lim_x",.4},{"acc_lim_y",.4},{"acc_lim_theta",.6},
    {"decel_lim_x",-.4},{"decel_lim_y",-.4},{"decel_lim_theta",-.6},
    {"sim_time",2.},{"time_granularity",.05}})node->declare_parameter("FollowPath."+key,value);
  node->declare_parameter("FollowPath.discretize_by_time",true);
  node->declare_parameter("FollowPath.limit_vel_cmd_in_traj",true);
  node->declare_parameter("FollowPath.use_elapsed_command_time",false); // recorded legacy rollout
  node->declare_parameter("FollowPath.preserve_translation_direction",false);
  const auto options=rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
    rclcpp::Parameter("plugins",std::vector<std::string>{}),rclcpp::Parameter("global_frame","odom"),
    rclcpp::Parameter("robot_base_frame","base_link"),rclcpp::Parameter("footprint",data["footprint"].dump()),
    rclcpp::Parameter("footprint_padding",0.0)});
  auto ros=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);ros->configure();
  auto map=ros->getCostmap();map->resizeMap(data["size"][0],data["size"][1],data["resolution"],data["origin"][0],data["origin"][1]);
  std::fill(map->getCharMap(),map->getCharMap()+map->getSizeInCellsX()*map->getSizeInCellsY(),0);
  for(const auto &cell:data["occupied"])map->setCost(cell[0],cell[1],cell[2]);
  auto pose=[](const auto &p){geometry_msgs::msg::Pose2D v;v.x=p[0];v.y=p[1];v.theta=p[2];return v;};
  auto twist=[](const auto &p){nav_2d_msgs::msg::Twist2D v;v.x=p[0];v.y=p[1];v.theta=p[2];return v;};
  auto position=pose(data["pose"]),goal=position;auto velocity=twist(data["velocity"]);
  nav_2d_msgs::msg::Path2D path;path.poses={position};
  dwb_critics::ObstacleFootprintCritic stock;cleany_navigation::SweptFootprintCritic swept;
  stock.initialize(node,"Stock","FollowPath",ros);swept.initialize(node,"Swept","FollowPath",ros);
  stock.prepare(position,velocity,goal,path);swept.prepare(position,velocity,goal,path);
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  const auto trajectory=generator.generateTrajectory(position,velocity,twist(data["target"]));
  bool ok=true;
  try {stock.scorePose(position);stock.scoreTrajectory(trajectory);}catch(...) {ok=false;}
  try {swept.scoreTrajectory(trajectory);ok=false;}catch(const dwb_core::IllegalTrajectoryException &) {}
  // Free map keeps stock scores; a stronger collision model must not penalize
  // or reject a genuinely open rollout merely because it was subdivided.
  std::fill(map->getCharMap(),map->getCharMap()+map->getSizeInCellsX()*map->getSizeInCellsY(),0);
  stock.prepare(position,velocity,goal,path);swept.prepare(position,velocity,goal,path);
  try {ok &= stock.scoreTrajectory(trajectory)==swept.scoreTrajectory(trajectory);}catch(...) {ok=false;}
  unsigned int x,y;map->worldToMap(position.x,position.y,x,y);map->setCost(x,y,255);
  swept.prepare(position,velocity,goal,path); // rebuild certificate for changed map
  try {swept.scoreTrajectory(trajectory);ok=false;}catch(const dwb_core::IllegalTrajectoryException &) {}
  return ok;
}
TEST(SweptFootprint, RecordedReturnCommandMissedByStockSamples) {
  ASSERT_EXIT({const bool ok=recordedCommandRejectedUnderSweptCellModel();::_exit(ok?0:1);},
    testing::ExitedWithCode(0),"");
}
