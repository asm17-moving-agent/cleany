#include <gtest/gtest.h>
#include <set>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include "rcl/time.h"
#include "cleany_navigation/axis_generator.hpp"
#include "cleany_navigation/path_heading.hpp"
#include "cleany_navigation/path_orientation.hpp"
#include "cleany_navigation/holonomic_generator.hpp"
#include "cleany_navigation/command_bearing.hpp"
#include "dwb_core/exceptions.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
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
TEST_F(AxisTest, FineHolonomicSamplesCloseQuantizedGoalWithoutMultiplyingYawGrid) {
  node->set_parameter(rclcpp::Parameter("FollowPath.vx_samples",7));
  node->set_parameter(rclcpp::Parameter("FollowPath.vy_samples",7));
  node->set_parameter(rclcpp::Parameter("FollowPath.vtheta_samples",11));
  node->set_parameter(rclcpp::Parameter("FollowPath.sim_time",2.0));
  dwb_plugins::StandardTrajectoryGenerator baseline;baseline.initialize(node,"FollowPath");
  cleany_navigation::HolonomicGenerator precise;precise.initialize(node,"FollowPath");
  auto old=baseline.getTwists(nav_2d_msgs::msg::Twist2D());
  auto improved=precise.getTwists(nav_2d_msgs::msg::Twist2D());
  EXPECT_LE(improved.size(),old.size()+49u);
  geometry_msgs::msg::Pose2D pose;bool closes=false;
  for(const auto &cmd:improved) {
    if(cmd.theta!=0. || cmd.x<=0. || cmd.y<=0.)continue;
    auto trajectory=precise.generateTrajectory(pose,nav_2d_msgs::msg::Twist2D(),cmd);
    auto end=trajectory.poses.back();closes|=std::hypot(end.x-.05,end.y-.05)<.01;
  }
  EXPECT_TRUE(closes);
  nav_2d_msgs::msg::Twist2D measured;measured.x=.15;
  auto braking=precise.generateTrajectory(pose,measured,nav_2d_msgs::msg::Twist2D());
  EXPECT_GT(braking.poses.back().x,0.);  // Zero command must not imply instant stop.
}
TEST_F(AxisTest, GoalDirectedSamplesUseBodyFrameAndRetainAccelerationRollout) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  node->set_parameter(rclcpp::Parameter("FollowPath.goal_directed_radius",.75));
  generator.initialize(node,"FollowPath");
  geometry_msgs::msg::Pose2D pose,goal;
  pose.x=-1.844914078342891;pose.y=-4.595498134780864;pose.theta=1.5646469010993471;
  goal.x=-1.865;goal.y=-4.705;goal.theta=1.5708;
  time(10.);const auto baseline=generator.getTwists(nav_2d_msgs::msg::Twist2D());
  generator.setGoalContext(pose,goal);time(10.02);
  auto twists=generator.getTwists(nav_2d_msgs::msg::Twist2D());
  ASSERT_EQ(twists.size(),baseline.size()+3u);
  const double dx=goal.x-pose.x,dy=goal.y-pose.y;
  const double bx=std::cos(pose.theta)*dx+std::sin(pose.theta)*dy;
  const double by=-std::sin(pose.theta)*dx+std::cos(pose.theta)*dy;
  for(size_t i=baseline.size();i<twists.size();++i) {
    const auto &cmd=twists[i];EXPECT_DOUBLE_EQ(cmd.theta,0.);
    EXPECT_NEAR(cmd.x*by-cmd.y*bx,0.,1e-12);
    EXPECT_GT(cmd.x*bx+cmd.y*by,0.);
    EXPECT_LE(std::hypot(cmd.x,cmd.y),.15);
    const auto trajectory=generator.generateTrajectory(pose,nav_2d_msgs::msg::Twist2D(),cmd);
    EXPECT_LE(std::abs(trajectory.velocity.x),.4*.02+1e-12);
    EXPECT_LE(std::abs(trajectory.velocity.y),.4*.02+1e-12);
    EXPECT_NEAR(trajectory.velocity.x*by-trajectory.velocity.y*bx,0.,1e-12);
    EXPECT_NEAR(rclcpp::Duration(trajectory.time_offsets.back()).seconds(),
      node->get_parameter("FollowPath.sim_time").as_double(),1e-9);
  }
  // Context expires after this cycle. No asynchronous retained goal.
  EXPECT_EQ(generator.getTwists(nav_2d_msgs::msg::Twist2D()).size(),baseline.size());
  generator.setGoalContext(pose,goal);generator.reset();
  EXPECT_EQ(generator.getTwists(nav_2d_msgs::msg::Twist2D()).size(),baseline.size());
}
TEST_F(AxisTest, GoalDirectedSamplesRespectDisabledRadiusBoundsAndInvalidContext) {
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  geometry_msgs::msg::Pose2D pose,goal;goal.x=.13;goal.y=.037;
  const auto baseline=generator.getTwists(nav_2d_msgs::msg::Twist2D());
  generator.setGoalContext(pose,goal);
  EXPECT_EQ(generator.getTwists(nav_2d_msgs::msg::Twist2D()).size(),baseline.size());
  node->set_parameter(rclcpp::Parameter("FollowPath.goal_directed_radius",.75));
  generator.initialize(node,"FollowPath");
  for(double distance:{0.,1.,std::numeric_limits<double>::quiet_NaN()}) {
    goal.x=distance;goal.y=0.;generator.setGoalContext(pose,goal);
    EXPECT_EQ(generator.getTwists(nav_2d_msgs::msg::Twist2D()).size(),baseline.size());
  }
  // A forward-only X profile must not acquire reverse candidates.
  node->set_parameter(rclcpp::Parameter("FollowPath.min_vel_x",0.));
  generator.initialize(node,"FollowPath");
  const auto forward=generator.getTwists(nav_2d_msgs::msg::Twist2D());
  goal.x=-.1;generator.setGoalContext(pose,goal);
  const auto reverse=generator.getTwists(nav_2d_msgs::msg::Twist2D());
  EXPECT_EQ(reverse.size(),forward.size());
  for(const auto &cmd:reverse)EXPECT_GE(cmd.x,0.);
}
TEST_F(AxisTest, GoalDirectedMaximumSpeedCandidateSurvivesFloatingPointBoundary) {
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  node->set_parameter(rclcpp::Parameter("FollowPath.goal_directed_radius",.75));
  generator.initialize(node,"FollowPath");
  const auto baseline=generator.getTwists(nav_2d_msgs::msg::Twist2D());
  geometry_msgs::msg::Pose2D pose,goal;
  // The live near-43 evaluation contained just two directed candidates. The
  // .15 m/s candidate can round one ULP above max_speed_xy and disappear.
  for(double angle:{.13,.37,.83,1.07,1.417530671655,1.8,2.9,-2.9}) {
    goal.x=.65*std::cos(angle);goal.y=.65*std::sin(angle);
    generator.setGoalContext(pose,goal);
    const auto commands=generator.getTwists(nav_2d_msgs::msg::Twist2D());
    ASSERT_EQ(commands.size(),baseline.size()+3u)<<angle;
    EXPECT_LE(std::hypot(commands.back().x,commands.back().y),.15);
    EXPECT_NEAR(std::hypot(commands.back().x,commands.back().y),.15,1e-12);
  }
}
TEST_F(AxisTest, GoalDirectedControllerIsLoadableAsNav2Controller) {
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
  EXPECT_TRUE(loader.isClassAvailable("cleany_navigation::GoalDirectedController"));
  EXPECT_NE(loader.createSharedInstance("cleany_navigation::GoalDirectedController"),nullptr);
}
TEST_F(AxisTest, PublishedHolonomicCommandMatchesFirstAcceleratingAndBrakingStep) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  geometry_msgs::msg::Pose2D pose;
  nav_2d_msgs::msg::Twist2D target;target.x=.15;target.y=.1;target.theta=.3;
  time(1.);generator.getTwists(nav_2d_msgs::msg::Twist2D());
  time(1.05);generator.getTwists(nav_2d_msgs::msg::Twist2D());
  auto acceleration=generator.generateTrajectory(pose,nav_2d_msgs::msg::Twist2D(),target);
  ASSERT_GE(acceleration.poses.size(),2u);
  EXPECT_NEAR(acceleration.velocity.x,.4*.05,1e-9);
  EXPECT_NEAR(acceleration.velocity.y,target.y*(.4*.05/target.x),1e-9);
  EXPECT_NEAR(acceleration.velocity.theta,.6*.05,1e-9);
  EXPECT_NEAR(acceleration.poses[1].x,acceleration.velocity.x*.05,1e-9);
  EXPECT_NEAR(acceleration.poses[1].theta,acceleration.velocity.theta*.05,1e-9);
  auto braking=generator.generateTrajectory(pose,target,nav_2d_msgs::msg::Twist2D());
  EXPECT_NEAR(braking.velocity.x,.15-.4*.05,1e-9);
  EXPECT_NEAR(braking.velocity.theta,.3-.6*.05,1e-9);
  EXPECT_GT(braking.poses.back().x,braking.poses[1].x);
}
TEST_F(AxisTest, CommandBudgetUsesElapsedRosTimeIncludingPauseResetAndBackwardJump) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  node->set_parameter(rclcpp::Parameter("FollowPath.sim_time",2.));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  geometry_msgs::msg::Pose2D pose;nav_2d_msgs::msg::Twist2D target;
  target.x=.15;target.y=.1;target.theta=.3;
  time(10.);generator.getTwists(nav_2d_msgs::msg::Twist2D());
  EXPECT_EQ(generator.generateTrajectory(pose,nav_2d_msgs::msg::Twist2D(),target).velocity.x,0.);
  time(10.02);generator.getTwists(nav_2d_msgs::msg::Twist2D());
  const auto result=generator.generateTrajectory(pose,nav_2d_msgs::msg::Twist2D(),target);
  EXPECT_NEAR(result.velocity.x,.008,1e-9);EXPECT_NEAR(result.velocity.theta,.012,1e-9);
  EXPECT_NEAR(result.poses[1].x,result.velocity.x*.02,1e-9);
  EXPECT_NEAR(rclcpp::Duration(result.time_offsets.back()).seconds(),2.,1e-9);
  EXPECT_NEAR(generator.generateTrajectory(pose,nav_2d_msgs::msg::Twist2D(),target).velocity.x,.008,1e-9);
  nav_2d_msgs::msg::Twist2D moving;moving.x=.1;
  generator.getTwists(moving); // frozen clock: no additional acceleration budget
  EXPECT_NEAR(generator.generateTrajectory(pose,moving,nav_2d_msgs::msg::Twist2D()).velocity.x,.1,1e-9);
  time(9.);generator.getTwists(moving);
  EXPECT_NEAR(generator.generateTrajectory(pose,moving,nav_2d_msgs::msg::Twist2D()).velocity.x,.1,1e-9);
  time(9.02);generator.getTwists(moving);
  EXPECT_NEAR(generator.generateTrajectory(pose,moving,nav_2d_msgs::msg::Twist2D()).velocity.x,.092,1e-9);
  target.y=0.; // isolate the clock cap from the shared X/Y progress fraction
  generator.reset();time(10.);generator.getTwists(moving);
  EXPECT_NEAR(generator.generateTrajectory(pose,moving,target).velocity.x,.1,1e-9);
  time(11.);generator.getTwists(moving); // long gap is capped at the old .05s budget
  EXPECT_NEAR(generator.generateTrajectory(pose,moving,target).velocity.x,.12,1e-9);
}
TEST_F(AxisTest, TimingTraceSeparatesResetAndFrozenClockWithoutChangingRollout) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator normal;normal.initialize(node,"FollowPath");
  const auto trace=std::filesystem::temp_directory_path()/
    ("cleany-generator-timing-"+std::to_string(reinterpret_cast<uintptr_t>(this))+".json");
  std::filesystem::remove(trace);
  node->set_parameter(rclcpp::Parameter("FollowPath.timing_trace_file",trace.string()));
  {
    cleany_navigation::HolonomicGenerator traced;traced.initialize(node,"FollowPath");
    geometry_msgs::msg::Pose2D pose;nav_2d_msgs::msg::Twist2D target,current;
    target.x=.15;target.y=.1;target.theta=.3;
    for(double now:{10.,10.02,10.02,9.,9.02}) {
      time(now);normal.getTwists(current);traced.getTwists(current);
      const auto a=normal.generateTrajectory(pose,current,target);
      const auto b=traced.generateTrajectory(pose,current,target);
      EXPECT_EQ(a.velocity,b.velocity);EXPECT_EQ(a.poses,b.poses);EXPECT_EQ(a.time_offsets,b.time_offsets);
    }
    normal.reset();traced.reset();time(11.);
    normal.getTwists(current);traced.getTwists(current);
    EXPECT_EQ(normal.generateTrajectory(pose,current,target),traced.generateTrajectory(pose,current,target));
    EXPECT_FALSE(std::filesystem::exists(trace)); // No I/O in the control loop.
  }
  std::ifstream input(trace);nlohmann::json data;input>>data;
  const auto &rows=data.at("records");ASSERT_EQ(rows.size(),6u);
  EXPECT_EQ(rows[0]["reason"],"after_reset");EXPECT_EQ(rows[2]["reason"],"clock_unchanged");
  EXPECT_EQ(rows[3]["reason"],"clock_backward");EXPECT_EQ(rows[5]["reason"],"after_reset");
  EXPECT_EQ(rows[5]["reset_generation"].get<uint64_t>(),rows[0]["reset_generation"].get<uint64_t>()+1);
  EXPECT_DOUBLE_EQ(rows[2]["first_dt_min"].get<double>(),0.);
  EXPECT_DOUBLE_EQ(rows[2]["first_dt_max"].get<double>(),0.);
  EXPECT_EQ(rows[2]["nonzero_targets"],1);EXPECT_EQ(rows[2]["zero_first_commands"],1);
  EXPECT_NEAR(rows[1]["first_dt_min"].get<double>(),.02,1e-9);
  EXPECT_EQ(data["dropped"],0);std::filesystem::remove(trace);
}
TEST_F(AxisTest, TranslationRampPreservesBearingAndRespectsBothComponentLimits) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  geometry_msgs::msg::Pose2D pose;nav_2d_msgs::msg::Twist2D target,current;
  target.x=-.05;target.y=.1;
  time(20.);generator.getTwists(current);
  const double bearing=std::atan2(target.y,target.x);
  for(int i=1;i<=20;++i) {
    time(20.+i*.02);generator.getTwists(current);
    auto next=generator.generateTrajectory(pose,current,target).velocity;
    EXPECT_NEAR(std::atan2(next.y,next.x),bearing,1e-9);
    EXPECT_LE(std::abs(next.x-current.x),.008+1e-9);
    EXPECT_LE(std::abs(next.y-current.y),.008+1e-9);
    current=next;
  }
  EXPECT_NEAR(current.x,target.x,1e-9);EXPECT_NEAR(current.y,target.y,1e-9);
  for(int i=21;i<=40;++i) {
    time(20.+i*.02);generator.getTwists(current);
    auto next=generator.generateTrajectory(pose,current,nav_2d_msgs::msg::Twist2D()).velocity;
    if(std::hypot(next.x,next.y)>1e-9)EXPECT_NEAR(std::atan2(next.y,next.x),bearing,1e-9);
    EXPECT_LE(std::abs(next.x-current.x),.008+1e-9);
    EXPECT_LE(std::abs(next.y-current.y),.008+1e-9);
    current=next;
  }
  EXPECT_NEAR(current.x,0.,1e-9);EXPECT_NEAR(current.y,0.,1e-9);
  // Reversal also follows the X/Y segment within each component's budget.
  current.x=.1;current.y=-.05;target.x=-.1;target.y=.1;
  time(21.);generator.getTwists(current);time(21.02);generator.getTwists(current);
  auto next=generator.generateTrajectory(pose,current,target).velocity;
  EXPECT_NEAR((next.x-current.x)/(target.x-current.x),
    (next.y-current.y)/(target.y-current.y),1e-9);
  EXPECT_LE(std::abs(next.x-current.x),.008+1e-9);
  EXPECT_LE(std::abs(next.y-current.y),.008+1e-9);
}
TEST_F(AxisTest, BearingContinuityRetainsStopHistoryAndAllowsFineGoalCorrection) {
  cleany_navigation::CommandBearingCritic critic;critic.initialize(node,"CommandBearing","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose,goal;goal.x=2.;nav_2d_msgs::msg::Path2D path;
  ASSERT_TRUE(critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path));
  dwb_msgs::msg::Trajectory2D straight,diagonal,lateral,reverse;
  straight.velocity.x=.02;diagonal.velocity.x=.02;diagonal.velocity.y=-.02;
  lateral.velocity.y=.02;reverse.velocity.x=-.02;
  critic.debrief(straight.velocity);
  EXPECT_NEAR(critic.scoreTrajectory(straight),0.,1e-9);
  EXPECT_GT(critic.scoreTrajectory(diagonal),critic.scoreTrajectory(straight));
  EXPECT_TRUE(std::isfinite(critic.scoreTrajectory(lateral)));
  EXPECT_TRUE(std::isfinite(critic.scoreTrajectory(reverse)));
  const auto far=critic.scoreTrajectory(diagonal);
  critic.debrief(nav_2d_msgs::msg::Twist2D());EXPECT_NEAR(critic.scoreTrajectory(diagonal),far,1e-9);
  goal.x=.03;ASSERT_TRUE(critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path));
  EXPECT_NEAR(critic.scoreTrajectory(diagonal),0.,1e-9);
  EXPECT_NEAR(critic.scoreTrajectory(reverse),0.,1e-9);
  goal.x=.2;ASSERT_TRUE(critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path));
  EXPECT_NEAR(critic.scoreTrajectory(diagonal),far*.5,1e-9);
  critic.reset();EXPECT_NEAR(critic.scoreTrajectory(reverse),0.,1e-9);
}
TEST_F(AxisTest, HolonomicBodyYawDoesNotDependOnTravelDirection) {
  cleany_navigation::PathOrientationCritic critic;critic.initialize(node,"PathOrientation","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose,goal;pose.theta=M_PI/2;goal.x=-2.;goal.y=1.;goal.theta=M_PI/2;
  nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  ASSERT_TRUE(critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path));
  dwb_msgs::msg::Trajectory2D lateral;lateral.velocity.y=.1;lateral.poses={pose};
  EXPECT_NEAR(critic.scoreTrajectory(lateral),0.,1e-9);
  auto needless_turn=lateral;needless_turn.poses.back().theta+=.2;
  EXPECT_GT(critic.scoreTrajectory(needless_turn),critic.scoreTrajectory(lateral));
  auto rotated=pose;rotated.theta=-M_PI/2;path.poses={pose,rotated,goal};
  ASSERT_TRUE(critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path));
  auto turning=lateral;turning.poses.back().theta=-M_PI/2;
  EXPECT_LT(critic.scoreTrajectory(turning),critic.scoreTrajectory(lateral));
  path.poses.clear();EXPECT_FALSE(critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path));
}
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
  time(1.1);for (auto t:gen.getTwists(nav_2d_msgs::msg::Twist2D())) {EXPECT_EQ(t.y,0);EXPECT_EQ(t.theta,0);}
  time(1.2);gen.getTwists(v);time(1.3);for (auto t:gen.getTwists(nav_2d_msgs::msg::Twist2D())) EXPECT_EQ(t.theta,0);
  time(1.5);for (auto t:gen.getTwists(nav_2d_msgs::msg::Twist2D())) EXPECT_EQ(t.theta,0);
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

TEST_F(AxisTest, PlanUpdatesPreserveStopHistoryAndClockRewindInvalidatesIt) {
  nav_2d_msgs::msg::Twist2D v;v.x=.08;time(1);gen.getTwists(v);
  time(1.1);gen.getTwists(nav_2d_msgs::msg::Twist2D());gen.reset();
  time(1.2);auto same=gen.getTwists(nav_2d_msgs::msg::Twist2D());
  ASSERT_GT(same.size(),1u);for (auto t:same) EXPECT_EQ(t.theta,0);
  time(1.5);auto switched=gen.getTwists(nav_2d_msgs::msg::Twist2D());
  bool rotate=false;for (auto t:switched) rotate|=t.theta!=0;EXPECT_TRUE(rotate);
  time(.5);EXPECT_EQ(gen.getTwists(nav_2d_msgs::msg::Twist2D()).size(),1u);
}
TEST_F(AxisTest, HeadingHysteresisKeepsAlignedTranslationWithoutRotationChatter) {
  cleany_axis_controller::PathHeadingCritic critic;critic.initialize(node,"PathHeading","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose,goal;goal.x=2;nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  dwb_msgs::msg::Trajectory2D move;move.velocity.x=.1;move.poses={pose};
  auto turn=move;turn.velocity.x=0;turn.velocity.theta=.06;
  critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  EXPECT_NO_THROW(critic.scoreTrajectory(move));EXPECT_THROW(critic.scoreTrajectory(turn),dwb_core::IllegalTrajectoryException);
  pose.theta=.15;critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  EXPECT_NO_THROW(critic.scoreTrajectory(move));
  pose.theta=.2;critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  EXPECT_THROW(critic.scoreTrajectory(move),dwb_core::IllegalTrajectoryException);
}

TEST_F(AxisTest, FineHeadingAlignmentHasAUsefulRotationCandidate) {
  node->set_parameter(rclcpp::Parameter("FollowPath.vtheta_samples",31));
  gen.initialize(node,"FollowPath");
  node->declare_parameter("FollowPath.PathHeading.heading_tolerance",.025);
  node->declare_parameter("FollowPath.PathHeading.heading_release_tolerance",.06);
  cleany_axis_controller::PathHeadingCritic critic;critic.initialize(node,"PathHeading","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose,goal;pose.theta=.04;goal.x=2;
  nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};
  critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  time(1);gen.getTwists(nav_2d_msgs::msg::Twist2D());time(1.4);
  dwb_msgs::msg::Trajectory2D stopped;stopped.poses={pose};
  double best=critic.scoreTrajectory(stopped);bool improving=false;
  for (auto cmd:gen.getTwists(nav_2d_msgs::msg::Twist2D())) {
    if (!cmd.theta) continue;
    auto trajectory=gen.generateTrajectory(pose,nav_2d_msgs::msg::Twist2D(),cmd);
    improving |= critic.scoreTrajectory(trajectory)<best;
  }
  EXPECT_TRUE(improving);
}

TEST_F(AxisTest, BoundedDiagonalCandidatesAndMeasuredTranslationPreserveRotationInterlock) {
  node->set_parameter(rclcpp::Parameter("FollowPath.axis_translation_angle_deg",10.));
  gen.initialize(node,"FollowPath");time(1);gen.getTwists(nav_2d_msgs::msg::Twist2D());time(1.4);
  bool forward=false,reverse=false;
  for(auto t:gen.getTwists(nav_2d_msgs::msg::Twist2D())) {
    if(t.y) {EXPECT_NE(t.x,0);EXPECT_EQ(t.theta,0);EXPECT_LE(std::abs(t.y),std::abs(t.x)*std::tan(M_PI/18)+1e-9);forward|=t.x>0;reverse|=t.x<0;}
  }
  EXPECT_TRUE(forward);EXPECT_TRUE(reverse);
  nav_2d_msgs::msg::Twist2D v;v.x=.08;v.y=.01;time(2);
  EXPECT_GT(gen.getTwists(v).size(),1u);for(auto t:gen.getTwists(v))EXPECT_EQ(t.theta,0);
  v.theta=.1;EXPECT_EQ(gen.getTwists(v).size(),1u);
  v.theta=0;v.y=.08;EXPECT_EQ(gen.getTwists(v).size(),1u);
}
TEST_F(AxisTest, BoundedDiagonalHeadingScoresTravelDirectionInForwardAndReverse) {
  node->set_parameter(rclcpp::Parameter("FollowPath.axis_translation_angle_deg",10.));
  cleany_axis_controller::PathHeadingCritic critic;critic.initialize(node,"PathHeading","FollowPath",nullptr);
  for(double sign : {-1.,1.}) {
    geometry_msgs::msg::Pose2D pose,goal;goal.x=sign*2;goal.y=sign*2*std::tan(.1);
    nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
    dwb_msgs::msg::Trajectory2D t;t.poses={pose};t.velocity.x=sign*.1;
    const auto straight=critic.scoreTrajectory(t);t.velocity.y=sign*.1*std::tan(.1);
    EXPECT_LT(critic.scoreTrajectory(t),straight);EXPECT_NEAR(critic.scoreTrajectory(t),0,1e-9);
  }
}

TEST_F(AxisTest, PlannedBodyHeadingIsRequiredDespiteDiagonalTravelAllowance) {
  node->declare_parameter("FollowPath.PathHeading.respect_path_orientation",true);
  node->declare_parameter("FollowPath.PathHeading.heading_tolerance",.025);
  node->declare_parameter("FollowPath.PathHeading.heading_release_tolerance",.06);
  node->set_parameter(rclcpp::Parameter("FollowPath.axis_translation_angle_deg",10.));
  cleany_axis_controller::PathHeadingCritic critic;critic.initialize(node,"PathHeading","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose,goal;pose.theta=-.14;goal.x=2.;
  auto reference=pose;reference.theta=0.;nav_2d_msgs::msg::Path2D path;path.poses={reference,goal};
  critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  dwb_msgs::msg::Trajectory2D moving;moving.poses={pose};moving.velocity.x=.1;
  EXPECT_THROW(critic.scoreTrajectory(moving),dwb_core::IllegalTrajectoryException);
  auto turning=moving;turning.velocity.x=0.;turning.velocity.theta=.1;turning.poses.back().theta=0.;
  auto stopped=turning;stopped.poses.back()=pose;
  EXPECT_LT(critic.scoreTrajectory(turning),critic.scoreTrajectory(stopped));
  pose.theta=0.;critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);moving.poses={pose};
  EXPECT_NO_THROW(critic.scoreTrajectory(moving));
  // At an explicit rotation waypoint, use its final body orientation.
  reference=pose;auto rotated=reference;rotated.theta=-M_PI/2;goal.y=-2.;goal.x=0.;goal.theta=-M_PI/2;
  path.poses={reference,rotated,goal};critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  EXPECT_THROW(critic.scoreTrajectory(moving),dwb_core::IllegalTrajectoryException);
}

TEST_F(AxisTest, PlannedHeadingScoresStoppedTravelErrorAlongsideDiagonalCorrection) {
  node->declare_parameter("FollowPath.PathHeading.respect_path_orientation",true);
  node->set_parameter(rclcpp::Parameter("FollowPath.axis_translation_angle_deg",10.));
  cleany_axis_controller::PathHeadingCritic critic;critic.initialize(node,"PathHeading","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose,goal;goal.x=2.;goal.y=.2;
  nav_2d_msgs::msg::Path2D path;path.poses={pose,goal};critic.prepare(pose,nav_2d_msgs::msg::Twist2D(),goal,path);
  dwb_msgs::msg::Trajectory2D stopped;stopped.poses={pose};auto moving=stopped;moving.velocity.x=.1;moving.velocity.y=.01;
  EXPECT_LT(critic.scoreTrajectory(moving),critic.scoreTrajectory(stopped));
}
TEST_F(AxisTest, ContinuityCandidatesRetainSelectedBearingButExpireAndRespectLimits) {
  node->declare_parameter("FollowPath.continuity_samples",true);
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  nav_2d_msgs::msg::Twist2D zero,selected;selected.x=.1;selected.y=.047;
  const auto baseline=generator.getTwists(zero);
  generator.setContinuityContext(selected);
  const auto candidates=generator.getTwists(zero);
  ASSERT_EQ(candidates.size(),baseline.size()+3u);
  for(size_t i=baseline.size();i<candidates.size();++i) {
    const auto &v=candidates[i];
    EXPECT_NEAR(v.x*selected.y-v.y*selected.x,0.,1e-12);
    EXPECT_LE(std::hypot(v.x,v.y),.15);
    EXPECT_GT(v.x,0.);EXPECT_DOUBLE_EQ(v.theta,0.);
    const auto rollout=generator.generateTrajectory(geometry_msgs::msg::Pose2D(),zero,v);
    EXPECT_GT(rollout.poses.size(),2u);
    EXPECT_GT(rollout.poses.back().x,0.);
  }
  EXPECT_EQ(generator.getTwists(zero).size(),baseline.size());
  generator.setContinuityContext(selected);generator.reset();
  EXPECT_EQ(generator.getTwists(zero).size(),baseline.size());
  selected.x=std::numeric_limits<double>::quiet_NaN();generator.setContinuityContext(selected);
  EXPECT_EQ(generator.getTwists(zero).size(),baseline.size());
}
TEST_F(AxisTest, SwitchingCostIsSoftAndStillDisappearsForGoalCorrection) {
  node->declare_parameter("FollowPath.CommandBearing.switch_cost",.5);
  cleany_navigation::CommandBearingCritic critic;critic.initialize(node,"CommandBearing","FollowPath",nullptr);
  geometry_msgs::msg::Pose2D pose,goal;goal.x=2.;nav_2d_msgs::msg::Path2D path;
  nav_2d_msgs::msg::Twist2D selected;selected.x=.1;critic.debrief(selected);
  dwb_msgs::msg::Trajectory2D candidate;candidate.velocity.x=.1;candidate.velocity.y=.01;
  ASSERT_TRUE(critic.prepare(pose,selected,goal,path));
  EXPECT_NEAR(critic.scoreTrajectory(candidate),std::atan2(.01,.1)+.5,1e-9);
  candidate.velocity.x=-.1;EXPECT_TRUE(std::isfinite(critic.scoreTrajectory(candidate)));
  goal.x=.05;ASSERT_TRUE(critic.prepare(pose,selected,goal,path));
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(candidate),0.);
}

TEST_F(AxisTest, HolonomicFirstCommandStaysInBoxWhenMeasuredVelocityOvershoots) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  nav_2d_msgs::msg::Twist2D measured,target;measured.x=.18;measured.y=-.17;measured.theta=.4;
  target.x=.15;target.y=-.15;target.theta=.3;
  geometry_msgs::msg::Pose2D pose;
  time(10.);generator.startNewIteration(measured); // reset: zero acceleration budget
  auto trajectory=generator.generateTrajectory(pose,measured,target);
  EXPECT_LE(trajectory.velocity.x,.15);EXPECT_GE(trajectory.velocity.y,-.15);
  EXPECT_LE(trajectory.velocity.theta,.3);
  time(10.001);generator.startNewIteration(measured);
  trajectory=generator.generateTrajectory(pose,measured,target);
  EXPECT_LE(trajectory.velocity.x,.15);EXPECT_GE(trajectory.velocity.y,-.15);
  EXPECT_LE(trajectory.velocity.theta,.3);
  measured.x=-.18;measured.y=.17;measured.theta=-.4;target.x=-.15;target.y=.15;target.theta=-.3;
  time(10.002);generator.startNewIteration(measured);
  trajectory=generator.generateTrajectory(pose,measured,target);
  EXPECT_GE(trajectory.velocity.x,-.15);EXPECT_LE(trajectory.velocity.y,.15);
  EXPECT_GE(trajectory.velocity.theta,-.3);
}

TEST_F(AxisTest, HolonomicBoxClampAlsoAppliesWithoutDirectionPreservation) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  node->set_parameter(rclcpp::Parameter("FollowPath.preserve_translation_direction",false));
  generator.initialize(node,"FollowPath");
  nav_2d_msgs::msg::Twist2D measured,target;
  measured.x=-.18;measured.y=.17;measured.theta=-.4;
  target.x=-.15;target.y=.15;target.theta=-.3;
  geometry_msgs::msg::Pose2D pose;
  time(10.);generator.startNewIteration(measured);
  auto trajectory=generator.generateTrajectory(pose,measured,target);
  EXPECT_GE(trajectory.velocity.x,-.15);EXPECT_LE(trajectory.velocity.y,.15);
  EXPECT_GE(trajectory.velocity.theta,-.3);
}

TEST_F(AxisTest, TargetRayCorrectsSmallCrossTrackVelocityWithinInheritedBudgets) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  node->set_parameter(rclcpp::Parameter("FollowPath.prefer_target_translation_direction",true));
  generator.initialize(node,"FollowPath");
  nav_2d_msgs::msg::Twist2D measured,target;measured.x=.033;measured.y=.002;
  target.x=.15;target.y=-.001;geometry_msgs::msg::Pose2D pose;
  time(10.);generator.startNewIteration(measured);
  time(10.05);generator.startNewIteration(measured);
  auto trajectory=generator.generateTrajectory(pose,measured,target);
  const auto &v=trajectory.velocity;
  EXPECT_NEAR(v.x*target.y-v.y*target.x,0.,1e-12);
  EXPECT_GT(v.x,measured.x);EXPECT_LT(v.y,0.);
  EXPECT_LE(std::abs(v.x-measured.x),.4*.05+1e-12);
  EXPECT_LE(std::abs(v.y-measured.y),.4*.05+1e-12);
}
TEST_F(AxisTest, InfeasibleTargetRayAndZeroTargetRetainSafeSegmentBraking) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  node->set_parameter(rclcpp::Parameter("FollowPath.prefer_target_translation_direction",true));
  generator.initialize(node,"FollowPath");
  nav_2d_msgs::msg::Twist2D measured,target;measured.x=.033;measured.y=-.1;
  target.x=.15;target.y=-.001;geometry_msgs::msg::Pose2D pose;
  time(10.);generator.startNewIteration(measured);
  time(10.05);generator.startNewIteration(measured);
  const auto trajectory=generator.generateTrajectory(pose,measured,target);
  EXPECT_LE(std::abs(trajectory.velocity.x-measured.x),.4*.05+1e-12);
  EXPECT_LE(std::abs(trajectory.velocity.y-measured.y),.4*.05+1e-12);
  EXPECT_LT(trajectory.velocity.y,-.07); // Do not snap to an unreachable ray.
  const auto braking=generator.generateTrajectory(pose,measured,nav_2d_msgs::msg::Twist2D());
  EXPECT_NEAR(braking.velocity.x*measured.y-braking.velocity.y*measured.x,0.,1e-12);
  EXPECT_GT(braking.poses.back().x,0.); // A stop still models finite braking.
}

TEST_F(AxisTest, CoupledMecanumCapScalesFullTwistAndAllWheelSigns) {
  node->set_parameter(rclcpp::Parameter("FollowPath.min_vel_x",-.4));
  node->set_parameter(rclcpp::Parameter("FollowPath.min_vel_y",-.4));
  node->set_parameter(rclcpp::Parameter("FollowPath.max_vel_x",.4));
  node->set_parameter(rclcpp::Parameter("FollowPath.max_vel_y",.4));
  node->set_parameter(rclcpp::Parameter("FollowPath.max_vel_theta",1.));
  node->set_parameter(rclcpp::Parameter("FollowPath.max_speed_xy",.6));
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  node->set_parameter(rclcpp::Parameter("FollowPath.max_wheel_speed_rad_s",5.));
  generator.initialize(node,"FollowPath");
  for(double sign:{1.,-1.}) {
    nav_2d_msgs::msg::Twist2D measured;measured.x=sign*.30;measured.y=sign*.25;measured.theta=sign*.50;
    geometry_msgs::msg::Pose2D pose;generator.reset();time(10.);generator.startNewIteration(measured);
    const auto v=generator.generateTrajectory(pose,measured,measured).velocity;
    const double k=(.35+.6038)/2.;
    for(double rate:{(v.x-v.y-k*v.theta)/.0635,(v.x+v.y+k*v.theta)/.0635,
        (v.x+v.y-k*v.theta)/.0635,(v.x-v.y+k*v.theta)/.0635})EXPECT_LE(std::abs(rate),5.+1e-12);
    EXPECT_NEAR(v.x/measured.x,v.y/measured.y,1e-12);
    EXPECT_NEAR(v.x/measured.x,v.theta/measured.theta,1e-12);
    EXPECT_NEAR((std::abs(v.x)+std::abs(v.y)+k*std::abs(v.theta))/.0635,5.,1e-12);
  }
}
TEST_F(AxisTest, DisabledWheelCapRetainsIndependentBoxAndRejectsBadGeometry) {
  node->set_parameter(rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj",true));
  cleany_navigation::HolonomicGenerator generator;generator.initialize(node,"FollowPath");
  nav_2d_msgs::msg::Twist2D measured;measured.x=.15;measured.y=.15;measured.theta=.3;
  geometry_msgs::msg::Pose2D pose;time(10.);generator.startNewIteration(measured);
  const auto v=generator.generateTrajectory(pose,measured,measured).velocity;
  EXPECT_DOUBLE_EQ(v.x,measured.x);EXPECT_DOUBLE_EQ(v.y,measured.y);EXPECT_DOUBLE_EQ(v.theta,measured.theta);
  node->set_parameter(rclcpp::Parameter("FollowPath.mecanum_wheel_radius_m",0.));
  EXPECT_THROW(generator.initialize(node,"FollowPath"),std::invalid_argument);
}
