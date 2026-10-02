#include <gtest/gtest.h>
#include <set>
#include "rcl/time.h"
#include "cleany_navigation/axis_generator.hpp"
#include "cleany_navigation/path_heading.hpp"
#include "dwb_core/exceptions.hpp"
class AxisTest : public testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) rclcpp::init(0,nullptr);
    node=std::make_shared<nav2_util::LifecycleNode>("axis_test", "", rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("use_sim_time",true),
      rclcpp::Parameter("FollowPath.min_speed_xy",0.0),rclcpp::Parameter("FollowPath.min_speed_theta",0.0),
      rclcpp::Parameter("FollowPath.max_speed_xy",.15),rclcpp::Parameter("FollowPath.discretize_by_time",true),
      rclcpp::Parameter("FollowPath.time_granularity",.05),rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",false),rclcpp::Parameter("FollowPath.min_vel_x",-.15),
      rclcpp::Parameter("FollowPath.max_vel_x",.15),rclcpp::Parameter("FollowPath.min_vel_y",-.15),
      rclcpp::Parameter("FollowPath.max_vel_y",.15),rclcpp::Parameter("FollowPath.max_vel_theta",.3),
      rclcpp::Parameter("FollowPath.acc_lim_x",.4),rclcpp::Parameter("FollowPath.acc_lim_y",.4),
      rclcpp::Parameter("FollowPath.acc_lim_theta",.6),rclcpp::Parameter("FollowPath.decel_lim_x",-.4),
      rclcpp::Parameter("FollowPath.decel_lim_y",-.4),rclcpp::Parameter("FollowPath.decel_lim_theta",-.6)}));
    gen.initialize(node,"FollowPath");
  }
  void time(double s) {ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), int64_t(s*1e9)), RCL_RET_OK);}
  nav2_util::LifecycleNode::SharedPtr node;
  cleany_axis_controller::AxisGenerator gen;
};
TEST_F(AxisTest, DefaultCandidatesOnlyTranslateLongitudinallyOrRotate) {
  time(1); gen.getTwists(nav_2d_msgs::msg::Twist2D());time(1.4);
  auto twists=gen.getTwists(nav_2d_msgs::msg::Twist2D());std::set<int> axes;
  for (auto t:twists) {int count=(t.x!=0)+(t.y!=0)+(t.theta!=0);EXPECT_LE(count,1);if(t.x)axes.insert(0);if(t.y)axes.insert(1);if(t.theta)axes.insert(2);}
  EXPECT_EQ(axes, (std::set<int>{0,2}));
  bool forward=false, reverse=false;
  for (auto t:twists) {EXPECT_EQ(t.y,0.);forward |= t.x>0;reverse |= t.x<0;}
  EXPECT_TRUE(forward);EXPECT_TRUE(reverse);
}
TEST_F(AxisTest, AxisSwitchRequiresContinuousSettlingAndMixedMotionOnlyBrakes) {
  nav_2d_msgs::msg::Twist2D v;v.x=.08;time(1);
  for(auto t:gen.getTwists(v)){EXPECT_EQ(t.y,0);EXPECT_EQ(t.theta,0);}
  time(1.1);EXPECT_EQ(gen.getTwists(nav_2d_msgs::msg::Twist2D()).size(),1u);
  time(1.2);gen.getTwists(v);time(1.3);EXPECT_EQ(gen.getTwists(nav_2d_msgs::msg::Twist2D()).size(),1u);
  time(1.5);EXPECT_EQ(gen.getTwists(nav_2d_msgs::msg::Twist2D()).size(),1u);
  time(1.7);EXPECT_GT(gen.getTwists(nav_2d_msgs::msg::Twist2D()).size(),1u);
  v.y=.08;EXPECT_EQ(gen.getTwists(v).size(),1u);
  v.x=NAN;EXPECT_EQ(gen.getTwists(v).size(),1u);
}
TEST_F(AxisTest, BrakingTrajectoryDoesNotPretendMovingRobotIsStationary) {
  nav_2d_msgs::msg::Twist2D v;v.x=.08;
  auto trajectory=gen.generateTrajectory(geometry_msgs::msg::Pose2D(),v,nav_2d_msgs::msg::Twist2D());
  ASSERT_FALSE(trajectory.poses.empty());EXPECT_GT(trajectory.poses.back().x,0.);
}

TEST_F(AxisTest, LateralExtensionIsExplicitAndResidualLateralMotionBrakes) {
  nav_2d_msgs::msg::Twist2D v;v.y=.08;time(1);
  EXPECT_EQ(gen.getTwists(v).size(),1u);
  node->set_parameter(rclcpp::Parameter("FollowPath.axis_allow_lateral",true));
  gen.initialize(node,"FollowPath");
  time(2);gen.getTwists(nav_2d_msgs::msg::Twist2D());time(2.4);
  bool lateral=false;
  for (auto t:gen.getTwists(nav_2d_msgs::msg::Twist2D())) lateral |= t.y!=0.;
  EXPECT_TRUE(lateral);
}

TEST_F(AxisTest, HeadingCriticRewardsRotationAndBlocksMisalignedTranslation) {
  cleany_axis_controller::PathHeadingCritic critic;
  critic.initialize(node,"PathHeading","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose, goal;goal.y=1.;
  nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  ASSERT_TRUE(critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path));
  dwb_msgs::msg::Trajectory2D stopped;stopped.poses={pose};
  auto rotating=stopped;rotating.velocity.theta=.3;rotating.poses.back().theta=.5;
  EXPECT_LT(critic.scoreTrajectory(rotating),critic.scoreTrajectory(stopped));
  auto moving=stopped;moving.velocity.x=.1;
  EXPECT_THROW(critic.scoreTrajectory(moving),dwb_core::IllegalTrajectoryException);
  pose.theta=M_PI/2;critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  moving.poses={pose};EXPECT_DOUBLE_EQ(critic.scoreTrajectory(moving),0.);
}
TEST_F(AxisTest, HeadingCriticPreservesReverseAndReleasesFinalGoalHeading) {
  cleany_axis_controller::PathHeadingCritic critic;
  critic.initialize(node,"PathHeading","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose, goal;goal.x=-1.;
  nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  dwb_msgs::msg::Trajectory2D reverse;reverse.poses={pose};reverse.velocity.x=-.1;
  EXPECT_NEAR(critic.scoreTrajectory(reverse),0.,1e-9);
  reverse.velocity.x=.1;
  EXPECT_THROW(critic.scoreTrajectory(reverse),dwb_core::IllegalTrajectoryException);
  goal.x=.05;goal.theta=M_PI/2;
  critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(reverse),0.);
}
TEST_F(AxisTest, SideGoalPrefersTurningForwardDespiteSmallHeadingError) {
  cleany_axis_controller::PathHeadingCritic critic;
  critic.initialize(node,"PathHeading","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose, goal;pose.theta=-.1;goal.y=1.;
  nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  dwb_msgs::msg::Trajectory2D turning;turning.poses={pose};turning.velocity.theta=.3;
  turning.poses.back().theta=.5;const auto forward=critic.scoreTrajectory(turning);
  turning.poses.back().theta=-.5;EXPECT_LT(forward,critic.scoreTrajectory(turning));
}
