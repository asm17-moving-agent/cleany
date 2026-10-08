#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <sys/resource.h>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "cleany_navigation/straight_first_planner.hpp"
TEST(StraightFirstPlanner, ArbitraryHeadingAndThinObstacleRespectEntireBody) {
  nav2_costmap_2d::Costmap2D map(200,200,.05,-5,-5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (auto xy:std::vector<std::pair<double,double>>{{-.3,-.4},{.3,-.4},{.3,.4},{-.3,.4}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  geometry_msgs::msg::PoseStamped start,goal;start.header.frame_id=goal.header.frame_id="map";
  start.pose.orientation.w=goal.pose.orientation.w=1;goal.pose.position.x=3;goal.pose.position.y=-.4;
  auto path=cleany_navigation::checkedStraightPath(start,goal,map,footprint);
  ASSERT_FALSE(path.poses.empty());
  for (const auto &p:path.poses) EXPECT_NEAR(p.pose.position.y,-.4*p.pose.position.x/3,1e-9);
  auto behind=goal;behind.pose.position.x=-3;
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(start,behind,map,footprint).poses.empty());
  unsigned int x,y;map.worldToMap(1.5,0,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(start,goal,map,footprint).poses.empty());
  map.setCost(x,y,255);
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(start,goal,map,footprint).poses.empty());
}
TEST(StraightFirstPlanner, InitialRotationAndCancellationAreChecked) {
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for (auto xy:std::vector<std::pair<double,double>>{{-.1,-.5},{.1,-.5},{.1,.5},{-.1,.5}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
  a.pose.orientation.w=b.pose.orientation.w=1;b.pose.position.y=1;
  unsigned int x,y;map.worldToMap(.32,.32,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(a,b,map,footprint).poses.empty());
  EXPECT_TRUE(cleany_navigation::checkedStraightPath(a,b,map,footprint,[] {return true;}).poses.empty());
}
TEST(StraightFirstPlanner, HolonomicRearAndSideMotionPreserveOrientationAndFullBodySafety) {
  nav2_costmap_2d::Costmap2D map(200,200,.05,-5,-5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(auto xy:std::vector<std::pair<double,double>>{{-.3,-.4},{.3,-.4},{.3,.4},{-.3,.4}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
  a.pose.orientation.w=b.pose.orientation.w=1.;b.pose.position.x=-2.;b.pose.position.y=1.;
  auto path=cleany_navigation::checkedHolonomicPath(a,b,map,footprint);
  ASSERT_FALSE(path.poses.empty());
  for(const auto &p:path.poses)EXPECT_NEAR(p.pose.orientation.z,0.,1e-9);
  EXPECT_TRUE(cleany_navigation::pathCollisionFree(path,map,footprint));
  unsigned int x,y;map.worldToMap(-1.,.5,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedHolonomicPath(a,b,map,footprint).poses.empty());
  map.setCost(x,y,0);
  EXPECT_TRUE(cleany_navigation::checkedHolonomicPath(a,b,map,footprint,[]{return true;}).poses.empty());
  // Both endpoint rotation locations are blocked, even though center-line
  // translation is clear. The full swept body must veto both strategies.
  footprint.clear();
  for(auto xy:std::vector<std::pair<double,double>>{{-.1,-.5},{.1,-.5},{.1,.5},{-.1,.5}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  b.pose.position.x=2.;b.pose.position.y=0.;b.pose.orientation.w=std::cos(M_PI/4);b.pose.orientation.z=std::sin(M_PI/4);
  map.worldToMap(-.32,.32,x,y);map.setCost(x,y,254);
  map.worldToMap(1.68,.32,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedHolonomicPath(a,b,map,footprint).poses.empty());
}
TEST(StraightFirstPlanner, SparseLatticePathChecksBetweenTranslationAndRotationSamples) {
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(auto xy:std::vector<std::pair<double,double>>{{-.1,-.5},{.1,-.5},{.1,.5},{-.1,.5}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  geometry_msgs::msg::PoseStamped a,b;a.pose.orientation.w=b.pose.orientation.w=1.;b.pose.position.x=1.;
  nav_msgs::msg::Path path;path.poses={a,b};
  EXPECT_TRUE(cleany_navigation::pathCollisionFree(path,map,footprint));
  unsigned int x,y;map.worldToMap(.5,0.,x,y);map.setCost(x,y,254);
  EXPECT_FALSE(cleany_navigation::pathCollisionFree(path,map,footprint));map.setCost(x,y,0);
  b=a;b.pose.orientation.z=std::sin(M_PI/4);b.pose.orientation.w=std::cos(M_PI/4);path.poses={a,b};
  map.worldToMap(-.32,.32,x,y);map.setCost(x,y,254);
  EXPECT_FALSE(cleany_navigation::pathCollisionFree(path,map,footprint));map.setCost(x,y,0);
  EXPECT_TRUE(cleany_navigation::pathCollisionFree(path,map,footprint));
  EXPECT_FALSE(cleany_navigation::pathCollisionFree(path,map,footprint,[] {return true;}));
}
TEST(StraightFirstPlanner, FixedHeadingDetourPreservesPositionsAndRemovesBodyTurns) {
  nav2_costmap_2d::Costmap2D map(200,200,.05,-5,-5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(auto xy:std::vector<std::pair<double,double>>{{-.3,-.4},{.3,-.4},{.3,.4},{-.3,.4}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(auto xy:std::vector<std::pair<double,double>>{{0,0},{1,0},{1,1}}) {
    geometry_msgs::msg::PoseStamped p;p.header=route.header;
    p.pose.position.x=xy.first;p.pose.position.y=xy.second;p.pose.orientation.w=1.;route.poses.push_back(p);
  }
  route.poses[1].pose.orientation.z=std::sin(M_PI/4);route.poses[1].pose.orientation.w=std::cos(M_PI/4);
  auto fixed=cleany_navigation::checkedFixedHeadingPath(route,map,footprint);
  ASSERT_FALSE(fixed.poses.empty());
  EXPECT_EQ(fixed.poses.front(),route.poses.front());EXPECT_EQ(fixed.poses.back(),route.poses.back());
  for(size_t i=0;i<route.poses.size();++i) {
    EXPECT_EQ(fixed.poses[i+1].pose.position,route.poses[i].pose.position);
    EXPECT_DOUBLE_EQ(fixed.poses[i+1].pose.orientation.z,0.);
  }
  unsigned int x,y;map.worldToMap(.5,0,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,footprint).poses.empty());
  map.setCost(x,y,255);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,footprint).poses.empty());
  map.setCost(x,y,0);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,footprint,[]{return true;}).poses.empty());
  route.poses[1].header.frame_id="odom";
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,footprint).poses.empty());
}
TEST(StraightFirstPlanner, FixedHeadingDetourRefusesCorridorThatNeedsRotation) {
  nav2_costmap_2d::Costmap2D map(200,200,.05,-5,-5,0);
  std::vector<geometry_msgs::msg::Point> footprint;
  for(auto xy:std::vector<std::pair<double,double>>{{-.4,-.1},{.4,-.1},{.4,.1},{-.4,.1}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(auto xy:std::vector<std::pair<double,double>>{{-1,0},{0,0},{0,2},{1,2}}) {
    geometry_msgs::msg::PoseStamped p;p.header=route.header;
    p.pose.position.x=xy.first;p.pose.position.y=xy.second;p.pose.orientation.w=1.;route.poses.push_back(p);
  }
  for(size_t i:{1u,2u}) {
    route.poses[i].pose.orientation.z=std::sin(M_PI/4);route.poses[i].pose.orientation.w=std::cos(M_PI/4);
  }
  for(double x:{-.25,.25})for(double y=.8;y<1.2;y+=.05) {
    unsigned int mx,my;map.worldToMap(x,y,mx,my);map.setCost(mx,my,254);
  }
  EXPECT_TRUE(cleany_navigation::pathCollisionFree(route,map,footprint));
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,footprint).poses.empty());
  // Extend the side obstacles to the map boundary: a fixed body yaw cannot
  // detour around them, while the original rotated body fits the passage.
  for(unsigned int x=0;x<map.getSizeInCellsX();++x)for(unsigned int y=116;y<124;++y) {
    double wx,wy;map.mapToWorld(x,y,wx,wy);
    if(std::abs(wx)>=.25)map.setCost(x,y,254);
  }
  EXPECT_TRUE(cleany_navigation::pathCollisionFree(route,map,footprint));
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(
    route.poses.front(),route.poses.back(),map,footprint,0.,5.).poses.empty());
}

TEST(StraightFirstPlanner, FixedHeadingAcceptanceRetainsRequiredPaddingWithoutSearchMargin) {
  nav2_costmap_2d::Costmap2D map(200,200,.05,-5,-5,0);
  std::vector<geometry_msgs::msg::Point> body;
  for(auto xy:std::vector<std::pair<double,double>>{{-.3,-.4},{.3,-.4},{.3,.4},{-.3,.4}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;body.push_back(p);
  }
  auto required=body;nav2_costmap_2d::padFootprint(required,.02);
  auto searched=required;nav2_costmap_2d::padFootprint(searched,.05);
  nav_msgs::msg::Path route;route.header.frame_id="map";
  for(auto xy:std::vector<std::pair<double,double>>{{0,0},{.99,0},{.99,1}}) {
    geometry_msgs::msg::PoseStamped p;p.header=route.header;p.pose.orientation.w=1.;
    p.pose.position.x=xy.first;p.pose.position.y=xy.second;route.poses.push_back(p);
  }
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(1.375,.5,x,y));map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,searched).poses.empty());
  EXPECT_FALSE(cleany_navigation::checkedFixedHeadingPath(route,map,required).poses.empty());
  // A full occupied cell touched only by required padding must still veto.
  ASSERT_TRUE(map.worldToMap(1.325,.5,x,y));map.setCost(x,y,254);
  EXPECT_FALSE(cleany_navigation::checkedFixedHeadingPath(route,map,body).poses.empty());
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,required).poses.empty());
  map.setCost(x,y,255);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(route,map,required).poses.empty());
}

TEST(StraightFirstPlanner, FixedHeadingGridSearchFindsBodySafeDetourAndChecksEndpoints) {
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  std::vector<geometry_msgs::msg::Point> body;
  for(auto xy:std::vector<std::pair<double,double>>{{-.2,-.3},{.2,-.3},{.2,.3},{-.2,.3}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;body.push_back(p);
  }
  geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
  a.pose.orientation.w=b.pose.orientation.w=1.;a.pose.position.x=-1.;b.pose.position.x=1.;
  for(unsigned int y=40;y<60;++y)map.setCost(50,y,254);
  EXPECT_TRUE(cleany_navigation::checkedHolonomicPath(a,b,map,body).poses.empty());
  auto path=cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,0.,5.);
  ASSERT_FALSE(path.poses.empty());EXPECT_EQ(path.poses.front(),a);EXPECT_EQ(path.poses.back(),b);
  double max_y=0.;
  for(const auto &p:path.poses) {
    EXPECT_DOUBLE_EQ(p.pose.orientation.z,0.);max_y=std::max(max_y,std::abs(p.pose.position.y));
  }
  EXPECT_GT(max_y,.75);EXPECT_TRUE(cleany_navigation::pathCollisionFree(path,map,body));
  EXPECT_FALSE(cleany_navigation::checkedFixedHeadingPath(path,map,body).poses.empty());
  int polls=0;
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,0.,5.,[&]{return ++polls>30;}).poses.empty());
  for(unsigned int y=0;y<100;++y)map.setCost(50,y,255);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,0.,5.).poses.empty());
  // Closed endpoint rotation: safe unrotated body but its corner sweeps an occupied cell.
  map.resetMap(0,0,100,100);unsigned int x,y;map.worldToMap(-.75,.2,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,M_PI/2,5.).poses.empty());
}

TEST(StraightFirstPlanner, HalfTurnUsesContinuousSubintervalsAndStillRejectsCornerCollision) {
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  std::vector<geometry_msgs::msg::Point> body;
  for(auto xy:std::vector<std::pair<double,double>>{{-.2,-.4},{.2,-.4},{.2,.4},{-.2,.4}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;body.push_back(p);
  }
  geometry_msgs::msg::PoseStamped start,goal;start.header.frame_id=goal.header.frame_id="map";
  start.pose.orientation.w=1.;goal.pose.orientation.z=1.;goal.pose.orientation.w=0.;goal.pose.position.x=1.;
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(.675,.675,x,y));map.setCost(x,y,254);
  auto path=cleany_navigation::checkedFixedHeadingGridPath(start,goal,map,body,M_PI,5.);
  ASSERT_FALSE(path.poses.empty());
  EXPECT_TRUE(cleany_navigation::sweptPathCollisionFree(path,map,body));
  int polls=0;
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(start,goal,map,body,M_PI,5.,
    [&]{return ++polls>4;}).poses.empty());
  // Both endpoint orientations are free, but the rotating corner contacts
  // this cell between them. Continuous subintervals must retain that veto.
  ASSERT_TRUE(map.worldToMap(.375,.125,x,y));map.setCost(x,y,254);
  nav_msgs::msg::Path turn;turn.header=start.header;turn.poses={start,start};
  turn.poses.back().pose.orientation.z=1.;turn.poses.back().pose.orientation.w=0.;
  EXPECT_FALSE(cleany_navigation::sweptPathCollisionFree(turn,map,body));
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(start,goal,map,body,M_PI,5.).poses.empty());
}

TEST(StraightFirstPlanner, FixedHeadingGridSearchHandlesWrappedYawAndInvalidInputs) {
  nav2_costmap_2d::Costmap2D map(80,80,.05,-2,-2,0);
  std::vector<geometry_msgs::msg::Point> body;
  for(auto xy:std::vector<std::pair<double,double>>{{-.1,-.2},{.1,-.2},{.1,.2},{-.1,.2}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;body.push_back(p);
  }
  geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
  a.pose.orientation.z=sin(3.13/2);a.pose.orientation.w=cos(3.13/2);
  b.pose.orientation.z=sin(-3.13/2);b.pose.orientation.w=cos(-3.13/2);b.pose.position.x=.5;
  auto path=cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,-3.13,5.);
  ASSERT_FALSE(path.poses.empty());EXPECT_EQ(path.poses.front(),a);EXPECT_EQ(path.poses.back(),b);
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,0.,-1.).poses.empty());
  b.header.frame_id="odom";
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,0.,5.).poses.empty());
  b.header.frame_id="map";b.pose.orientation.w=2.;
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingGridPath(a,b,map,body,0.,5.).poses.empty());
}

TEST(StraightFirstPlanner, LiveSweepValidatesChosenHeadingWithoutReorientingTheRoute) {
  nav2_costmap_2d::Costmap2D map(160,160,.05,-4,-4,0);
  std::vector<geometry_msgs::msg::Point> body;
  for(auto xy:std::vector<std::pair<double,double>>{{-.1,-.4},{.1,-.4},{.1,.4},{-.1,.4}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;body.push_back(p);
  }
  geometry_msgs::msg::PoseStamped start,goal;start.header.frame_id=goal.header.frame_id="map";
  start.pose.orientation.z=sin(M_PI/4);start.pose.orientation.w=cos(M_PI/4);
  goal.pose.orientation.w=1.;goal.pose.position.x=2.;
  auto chosen=cleany_navigation::checkedFixedHeadingGridPath(start,goal,map,body,0.,5.);
  ASSERT_FALSE(chosen.poses.empty());const auto original=chosen;
  EXPECT_TRUE(cleany_navigation::sweptPathCollisionFree(chosen,map,body));
  EXPECT_EQ(chosen,original);
  unsigned int x,y;ASSERT_TRUE(map.worldToMap(1.,.325,x,y));map.setCost(x,y,254);
  // The chosen goal-yaw route is newly occupied. A different yaw would fit,
  // but validation must reject this route rather than silently choose it.
  EXPECT_FALSE(cleany_navigation::sweptPathCollisionFree(chosen,map,body));
  EXPECT_FALSE(cleany_navigation::checkedFixedHeadingPath(chosen,map,body).poses.empty());
  EXPECT_EQ(chosen,original);
  auto malformed=original;malformed.poses[2].pose.orientation.w=2.;
  EXPECT_TRUE(cleany_navigation::checkedFixedHeadingPath(malformed,map,body).poses.empty());
}

// A child process also verifies full teardown and process-exit static
// destructors. _Exit would hide duplicate upstream heuristic-table teardown.
static bool blockedSearchReleasesLiveMapAndSharesBudget() {
  rclcpp::init(0,nullptr);
  auto parent=std::make_shared<rclcpp_lifecycle::LifecycleNode>("snapshot_planner_test");
  parent->declare_parameter("GridBased.lattice_filepath",std::string(CLEANY_TEST_LATTICE));
  parent->declare_parameter("GridBased.max_planning_time",.4);
  const auto evidence=std::filesystem::temp_directory_path()/ ("cleany-planner-evidence-"+std::to_string(getpid()));
  parent->declare_parameter("GridBased.failure_snapshot_directory",evidence.string());
  parent->declare_parameter("GridBased.require_exact_goal",true);
  parent->declare_parameter("GridBased.analytic_expansion_max_length",0.0);
  auto options=rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("global_frame","map"),rclcpp::Parameter("plugins",std::vector<std::string>{}),
    rclcpp::Parameter("width",10),rclcpp::Parameter("height",10),
    rclcpp::Parameter("resolution",.05),rclcpp::Parameter("origin_x",-5.),rclcpp::Parameter("origin_y",-5.),
    rclcpp::Parameter("footprint","[[-0.3,-0.4],[0.3,-0.4],[0.3,0.4],[-0.3,0.4]]"),
    rclcpp::Parameter("footprint_padding",.02)});
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  map->configure();
  auto live=map->getCostmap();
  for(unsigned int x=0;x<live->getSizeInCellsX();++x)
    for(unsigned int y=0;y<live->getSizeInCellsY();++y)live->setCost(x,y,x==100?254:0);
  auto tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
  auto planner=std::make_unique<cleany_navigation::StraightFirstPlanner>();
  planner->configure(parent,"GridBased",tf,map);
  geometry_msgs::msg::PoseStamped a,b;a.header.frame_id=b.header.frame_id="map";
  a.pose.orientation.w=b.pose.orientation.w=1.;a.pose.position.x=-2.5;b.pose.position.x=2.5;
  double wait_s=0.;
  std::thread writer([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    auto start=std::chrono::steady_clock::now();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
    wait_s=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    live->setCost(10,10,254);
  });
  auto start=std::chrono::steady_clock::now();
#if CLEANY_NAV2_CANCEL_CHECKER
  const auto path=planner->createPlan(a,b,[]{return false;});
#else
  const auto path=planner->createPlan(a,b);
#endif
  const auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  writer.join();
  if(!(path.poses.empty() && elapsed>.05 && elapsed<.9 && wait_s<.1 && live->getCost(10,10)==254))return false;
  // Failure evidence must retain the searched bytes, not replace them with
  // the live update that happened during exploration.
  std::ifstream metadata(evidence/"failure.json");nlohmann::json record;metadata>>record;
  if(record["attempts"].empty() || record["search"]["fnv1a64"]==record["live"]["fnv1a64"])return false;
  const auto read=[&](const std::string &name) {
    std::ifstream stream(evidence/name,std::ios::binary);
    return std::vector<unsigned char>(std::istreambuf_iterator<char>(stream),{});
  };
  const auto searched=read("search.bin"),updated=read("live.bin");
  const auto index=10*live->getSizeInCellsX()+10;
  if(searched.size()!=live->getSizeInCellsX()*live->getSizeInCellsY() ||
     updated.size()!=searched.size() || searched[index]!=0 || updated[index]!=254)return false;
  // A path that exists in a snapshot must still be vetoed when an observer
  // occupies the live map during search. A behind-robot goal forces lattice.
  live->resetMap(0,0,live->getSizeInCellsX(),live->getSizeInCellsY());
  for(unsigned int x=0;x<live->getSizeInCellsX();++x)
    for(unsigned int y=0;y<live->getSizeInCellsY();++y)live->setCost(x,y,0);
  a.pose.position.x=2.5;a.pose.position.y=-2.5;
  b.pose.position.x=-2.487;b.pose.position.y=2.487;b.pose.orientation.w=0.;b.pose.orientation.z=1.;
#if CLEANY_NAV2_CANCEL_CHECKER
  auto baseline=planner->createPlan(a,b,[]{return false;});
#else
  auto baseline=planner->createPlan(a,b);
#endif
  if(baseline.poses.empty())return false;
  std::ifstream after_success(evidence/"failure.json");nlohmann::json same;after_success>>same;
  if(same!=record)return false;
  if(std::hypot(baseline.poses.back().pose.position.x-b.pose.position.x,
                baseline.poses.back().pose.position.y-b.pose.position.y)>1e-12)return false;
  std::thread observer([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
    for(unsigned int x=0;x<live->getSizeInCellsX();++x)
      for(unsigned int y=0;y<live->getSizeInCellsY();++y)live->setCost(x,y,254);
  });
#if CLEANY_NAV2_CANCEL_CHECKER
  auto stale=planner->createPlan(a,b,[]{return false;});
#else
  auto stale=planner->createPlan(a,b);
#endif
  observer.join();
  if(!stale.poses.empty())return false;
  // A continuous pose can be safe while its nearest angle/grid seed collides.
  // A behind-robot goal forces lattice search; the returned plan
  // must retain the real pose and safely rotate before reaching another seed.
  for(unsigned int x=0;x<live->getSizeInCellsX();++x)
    for(unsigned int y=0;y<live->getSizeInCellsY();++y)live->setCost(x,y,0);
  unsigned int mx,my;live->worldToMap(.125,.525,mx,my);live->setCost(mx,my,254);
  a.pose.position.x=a.pose.position.y=0.;a.pose.orientation.z=std::sin(.24/2);a.pose.orientation.w=std::cos(.24/2);
  b.pose.position.x=-2.;b.pose.position.y=-1.;b.pose.orientation.z=0.;b.pose.orientation.w=1.;
  nav_msgs::msg::Path stationary;stationary.poses={a};
  if(!cleany_navigation::pathCollisionFree(stationary,*live,map->getRobotFootprint()))return false;
  auto snapped=a;snapped.pose.position.x=snapped.pose.position.y=.025;
  snapped.pose.orientation.z=std::sin(.463647609/2);snapped.pose.orientation.w=std::cos(.463647609/2);
  nav_msgs::msg::Path nearest;nearest.poses={snapped};
  if(cleany_navigation::pathCollisionFree(nearest,*live,map->getRobotFootprint()))return false;
#if CLEANY_NAV2_CANCEL_CHECKER
  const auto alternate=planner->createPlan(a,b,[]{return false;});
#else
  const auto alternate=planner->createPlan(a,b);
#endif
  if(alternate.poses.empty() || !cleany_navigation::pathCollisionFree(alternate,*live,map->getRobotFootprint()))return false;
  if(std::hypot(alternate.poses.front().pose.position.x-a.pose.position.x,
                alternate.poses.front().pose.position.y-a.pose.position.y)>1e-12)return false;
  // An occupied real start must never be escaped by snapping somewhere free.
  live->worldToMap(.375,-.225,mx,my);live->setCost(mx,my,254);
  if(cleany_navigation::pathCollisionFree(stationary,*live,map->getRobotFootprint()))return false;
#if CLEANY_NAV2_CANCEL_CHECKER
  const auto unsafe=planner->createPlan(a,b,[]{return false;});
#else
  const auto unsafe=planner->createPlan(a,b);
#endif
  std::filesystem::remove_all(evidence);
  const bool passed=unsafe.poses.empty();
  planner->cleanup();planner.reset();map->cleanup();map.reset();
  rclcpp::shutdown();
  return passed;
}

TEST(StraightFirstPlanner, BlockedSearchSharesDeadlineAndDoesNotBlockLiveMapUpdates) {
  ASSERT_EXIT({
    // Exit status/stderr are sufficient for this regression. A deliberately
    // reproduced crash must not start a host crash analyzer during live tests.
    const rlimit no_core{};
    if(setrlimit(RLIMIT_CORE,&no_core)!=0)std::exit(2);
    std::exit(blockedSearchReleasesLiveMapAndSharesBudget()?0:1);
  },
    ::testing::ExitedWithCode(0),"");
}
