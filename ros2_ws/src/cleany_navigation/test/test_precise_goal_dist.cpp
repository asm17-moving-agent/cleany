#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unistd.h>
#include "cleany_navigation/precise_goal_dist.hpp"
#include "cleany_navigation/command_bearing.hpp"
#include "dwb_core/exceptions.hpp"
static bool closingFinalCellPreservesStockFailures() {
  rclcpp::init(0,nullptr);
  auto node=std::make_shared<nav2_util::LifecycleNode>("precise_goal_test");
  auto options=rclcpp::NodeOptions().use_global_arguments(false).parameter_overrides({
    rclcpp::Parameter("plugins",std::vector<std::string>{}),
    rclcpp::Parameter("global_frame","odom"),rclcpp::Parameter("robot_base_frame","base_link"),
    rclcpp::Parameter("footprint","[[-0.2,-0.2],[-0.2,0.2],[0.2,0.2],[0.2,-0.2]]"),
    rclcpp::Parameter("footprint_padding",0.0)});
  auto ros=std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);ros->configure();
  auto map=ros->getCostmap();map->resizeMap(40,40,.05,-1,-1);
  std::fill(map->getCharMap(),map->getCharMap()+1600,0);
  cleany_navigation::PreciseGoalDistCritic precise;
  dwb_critics::GoalDistCritic stock;
  cleany_navigation::CommandBearingCritic bearing;
  precise.initialize(node,"Precise","FollowPath",ros);
  stock.initialize(node,"Stock","FollowPath",ros);
  bearing.initialize(node,"Bearing","FollowPath",ros);
  geometry_msgs::msg::Pose2D pose,goal;pose.x=.041;pose.y=.041;goal.x=.001;goal.y=.001;
  nav_2d_msgs::msg::Twist2D velocity;nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  dwb_msgs::msg::Trajectory2D stop,closing;stop.poses={pose};closing.poses={pose};
  closing.poses.back().x=.021;closing.poses.back().y=.021;
  closing.velocity.x=-.0125;closing.velocity.y=-.0125;
  auto prepare=[&] {return stock.prepare(pose,velocity,goal,path)&&
    precise.prepare(pose,velocity,goal,path)&&bearing.prepare(pose,velocity,goal,path);};
  bool ok=prepare();
  // Outside the 5cm goal gate, but both endpoints lie in the zero-cost goal
  // cell. Previously any bearing change penalized closing more than stopping.
  ok &= std::hypot(pose.x-goal.x,pose.y-goal.y)>.05;
  ok &= stock.scoreTrajectory(stop)==0. && stock.scoreTrajectory(closing)==0.;
  nav_2d_msgs::msg::Twist2D previous;previous.x=.02;previous.y=.02;bearing.debrief(previous);
  ok &= bearing.scoreTrajectory(closing)==0.;
  ok &= precise.scoreTrajectory(closing)<precise.scoreTrajectory(stop);
  // Continuous distance only augments near-goal scoring. Far guidance and
  // trajectory rejection remain stock behavior; this is no collision escape.
  pose.x=.6;path.poses={pose,goal};ok &= prepare();
  ok &= precise.scoreTrajectory(stop)==stock.scoreTrajectory(stop);
  auto outside=stop;outside.poses.back().x=2.;
  auto rejects=[&](dwb_core::TrajectoryCritic &critic) {
    try {critic.scoreTrajectory(outside);return false;}
    catch(const dwb_core::IllegalTrajectoryException &) {return true;}
  };
  ok &= rejects(stock)&&rejects(precise);
  dwb_msgs::msg::Trajectory2D empty;
  try {precise.scoreTrajectory(empty);ok=false;}
  catch(const dwb_core::IllegalTrajectoryException &) {}
  outside.poses.back().x=std::numeric_limits<double>::quiet_NaN();
  ok &= rejects(precise);
  goal.x=std::numeric_limits<double>::quiet_NaN();
  ok &= !precise.prepare(pose,velocity,goal,path);
  return ok;
}
TEST(PreciseGoalDist, FinalCellProgressAndStockRejection) {
  ASSERT_EXIT({const bool ok=closingFinalCellPreservesStockFailures();::_exit(ok?0:1);},
    testing::ExitedWithCode(0),"");
}
