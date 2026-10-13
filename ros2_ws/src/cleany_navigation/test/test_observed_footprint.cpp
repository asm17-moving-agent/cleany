#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include "cleany_navigation/observed_footprint.hpp"
#include "dwb_core/exceptions.hpp"
static bool coherentFailurePreservesScoring() {
  rclcpp::init(0,nullptr);
  const auto directory=std::filesystem::temp_directory_path()/ ("cleany-controller-evidence-"+std::to_string(getpid()));
  auto node=std::make_shared<nav2_util::LifecycleNode>("controller_evidence_test");
  node->declare_parameter("FollowPath.ObstacleFootprint.failure_snapshot_directory",directory.string());
  auto options=rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
    rclcpp::Parameter("plugins",std::vector<std::string>{}),
    rclcpp::Parameter("global_frame","odom"),rclcpp::Parameter("robot_base_frame","base_link"),
    rclcpp::Parameter("footprint","[[-0.2,-0.2],[-0.2,0.2],[0.2,0.2],[0.2,-0.2]]"),
    rclcpp::Parameter("footprint_padding",0.0)});
  auto ros=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);ros->configure();
  auto map=ros->getCostmap();map->resizeMap(40,40,.05,-1,-1);
  std::fill(map->getCharMap(),map->getCharMap()+1600,0);
  cleany_navigation::ObservedFootprintCritic observed;
  dwb_critics::ObstacleFootprintCritic original;
  observed.initialize(node,"ObstacleFootprint","FollowPath",ros);
  original.initialize(node,"Reference","FollowPath",ros);
  geometry_msgs::msg::Pose2D pose,goal;goal.x=.5;
  nav_2d_msgs::msg::Twist2D velocity;
  nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  dwb_msgs::msg::Trajectory2D trajectory;trajectory.poses={pose};
  bool ok=true;
  auto prepare=[&] {return observed.prepare(pose,velocity,goal,path)&&original.prepare(pose,velocity,goal,path);};
  ok &= prepare();ok &= observed.scoreTrajectory(trajectory)==original.scoreTrajectory(trajectory);
  observed.debrief(velocity);ok &= !std::filesystem::exists(directory/"failure.json");
  ok &= prepare();auto outside=trajectory;outside.poses.front().x=2.;
  try {observed.scoreTrajectory(outside);ok=false;}catch(const dwb_core::IllegalTrajectoryException &) {}
  observed.scoreTrajectory(trajectory);observed.debrief(velocity);
  ok &= !std::filesystem::exists(directory/"failure.json");
  unsigned int x,y;map->worldToMap(.2,0,x,y);
  std::fill(map->getCharMap(),map->getCharMap()+1600,254);
  ok &= prepare();
  auto rejects=[&](dwb_core::TrajectoryCritic &critic) {
    try {critic.scoreTrajectory(trajectory);return false;}
    catch(const dwb_core::IllegalTrajectoryException &) {return true;}
  };
  ok &= rejects(original)&&rejects(observed);observed.debrief(velocity);
  const auto bytes=[&] {std::ifstream f(directory/"failure.json");return std::string(std::istreambuf_iterator<char>(f),{});};
  const auto first=bytes();auto evidence=nlohmann::json::parse(first);
  ok &= evidence["candidates"]==1 && evidence["rejected"]==1 && evidence["costs"].size()==1600;
  ok &= evidence["costs"][y*40+x]==254 && evidence["pose"][0]==0 && evidence["frame"]=="odom";
  ok &= evidence["previous_cycle"]["costs"][y*40+x]==0;
  ok &= evidence["previous_cycle"]["candidates"]==2 &&
    evidence["previous_cycle"]["rejected"]==1;
  ok &= evidence["previous_cycle"]["stamp_ns"]<=evidence["stamp_ns"];
  // Repeated failed cycles cannot overwrite the onset with a later map.
  map->setCost(0,0,253);ok &= prepare()&&rejects(observed);observed.debrief(velocity);ok &= bytes()==first;
  // A valid cycle ends the episode; the next failure records its new map.
  std::fill(map->getCharMap(),map->getCharMap()+1600,0);
  ok &= prepare();observed.scoreTrajectory(trajectory);observed.debrief(velocity);
  std::fill(map->getCharMap(),map->getCharMap()+1600,254);map->setCost(0,0,253);
  ok &= prepare()&&rejects(observed);observed.debrief(velocity);
  ok &= nlohmann::json::parse(bytes())["costs"][0]==253;
  // A new plan must not inherit a previous cycle or a latched failure episode.
  observed.reset();ok &= prepare()&&rejects(observed);observed.debrief(velocity);
  ok &= !nlohmann::json::parse(bytes()).contains("previous_cycle");
  // Capture errors must not change collision rejection or propagate from debrief.
  std::fill(map->getCharMap(),map->getCharMap()+1600,0);
  ok &= prepare();observed.scoreTrajectory(trajectory);observed.debrief(velocity);
  std::filesystem::remove_all(directory);std::ofstream(directory)<<"not a directory";
  std::fill(map->getCharMap(),map->getCharMap()+1600,254);
  ok &= prepare()&&rejects(observed);observed.debrief(velocity);
  std::filesystem::remove(directory);return ok;
}
TEST(ObservedFootprint, CoherentFirstFailureAndStockScoring) {
  ASSERT_EXIT({const bool ok=coherentFailurePreservesScoring();::_exit(ok?0:1);},testing::ExitedWithCode(0),"");
}
