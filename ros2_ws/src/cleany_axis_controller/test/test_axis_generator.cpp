#include <gtest/gtest.h>
#include <set>
#include "rcl/time.h"
#include "cleany_axis_controller/axis_generator.hpp"
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
TEST_F(AxisTest, SettledCandidatesContainAllAxesButNeverMix) {
  time(1); gen.getTwists(nav_2d_msgs::msg::Twist2D());time(1.4);
  auto twists=gen.getTwists(nav_2d_msgs::msg::Twist2D());std::set<int> axes;
  for (auto t:twists) {int count=(t.x!=0)+(t.y!=0)+(t.theta!=0);EXPECT_LE(count,1);if(t.x)axes.insert(0);if(t.y)axes.insert(1);if(t.theta)axes.insert(2);}
  EXPECT_EQ(axes.size(),3u);
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
