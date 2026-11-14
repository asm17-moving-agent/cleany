#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <limits>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include "cleany_navigation/swept_footprint.hpp"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
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
  node->declare_parameter("FollowPath.use_elapsed_command_time",false); // 기록된 이전 rollout입니다.
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
  dwb_critics::ObstacleFootprintCritic stock;
  stock.initialize(node,"Stock","FollowPath",ros);
  stock.prepare(position,velocity,goal,path);
  dwb_plugins::StandardTrajectoryGenerator generator;generator.initialize(node,"FollowPath");
  const auto trajectory=generator.generateTrajectory(position,velocity,twist(data["target"]));
  const auto footprint=ros->getRobotFootprint();
  auto free=[&]() {
    for(size_t i=1;i<trajectory.poses.size();++i)
      if(!sweptFootprintFree(*map,footprint,trajectory.poses[i-1],trajectory.poses[i]))return false;
    return sweptFootprintFree(*map,footprint,position,position);
  };
  bool ok=true;
  try {stock.scorePose(position);stock.scoreTrajectory(trajectory);}catch(...) {ok=false;}
  ok &= !free();
  // 원본 점유 셀 회귀 사례를 유지하고, 빈 지도에서는 같은 구간을 허용합니다.
  std::fill(map->getCharMap(),map->getCharMap()+map->getSizeInCellsX()*map->getSizeInCellsY(),0);
  ok &= free();
  unsigned int x,y;map->worldToMap(position.x,position.y,x,y);map->setCost(x,y,255);
  ok &= !free();
  return ok;
}
TEST(SweptFootprint, RecordedReturnCommandMissedByStockSamples) {
  ASSERT_EXIT({const bool ok=recordedCommandRejectedUnderSweptCellModel();::_exit(ok?0:1);},
    testing::ExitedWithCode(0),"");
}

TEST(SweptFootprint, RecordedFirstCommandMustRemainSafeDuringHold) {
  nlohmann::json data;std::ifstream(CLEANY_HOLD_FIXTURE)>>data;
  nav2_costmap_2d::Costmap2D map(data["size"][0],data["size"][1],data["resolution"],
    data["origin"][0],data["origin"][1],0);
  for(const auto &cell:data["occupied"])map.setCost(cell[0],cell[1],cell[2]);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(const auto &xy:data["footprint"]) {
    geometry_msgs::msg::Point p;p.x=xy[0];p.y=xy[1];footprint.push_back(p);
  }
  geometry_msgs::msg::Pose2D start;
  start.x=data["pose"][0];start.y=data["pose"][1];start.theta=data["pose"][2];
  const double vx=data["issued_command"][0],vy=data["issued_command"][1],wz=data["issued_command"][2];
  // 폐기한 controller를 재실행하지 않고 당시 실제 발행 명령의 0.2초 유지 구간을 검증합니다.
  const double duration=.2,half=wz*duration/2.,mid=start.theta+half;
  const double distance=std::abs(half)<1e-9 ? duration : duration*std::sin(half)/half;
  auto end=start;end.theta+=wz*duration;
  end.x+=distance*(vx*std::cos(mid)-vy*std::sin(mid));
  end.y+=distance*(vx*std::sin(mid)+vy*std::cos(mid));
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
  auto held_free=[&]() {
    if(!sweptFootprintFree(map,footprint,start,end))return false;
    // 원래 회귀 조건에는 연속 기하뿐 아니라 Nav2의 보수적 래스터 검사도 포함됩니다.
    for(int i=0;i<=20;++i) {
      const double t=duration*i/20.,h=wz*t/2.,heading=start.theta+h;
      const double d=std::abs(h)<1e-9 ? t : t*std::sin(h)/h;
      const double x=start.x+d*(vx*std::cos(heading)-vy*std::sin(heading));
      const double y=start.y+d*(vx*std::sin(heading)+vy*std::cos(heading));
      if(checker.footprintCostAtPose(x,y,start.theta+wz*t,footprint)>=254.)return false;
    }
    return true;
  };
  EXPECT_TRUE(sweptFootprintFree(map,footprint,start,start));
  EXPECT_FALSE(held_free());
  std::fill(map.getCharMap(),map.getCharMap()+map.getSizeInCellsX()*map.getSizeInCellsY(),0);
  EXPECT_TRUE(held_free());
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(start.x,start.y,x,y));map.setCost(x,y,255);
  EXPECT_FALSE(sweptFootprintFree(map,footprint,start,start));
}
