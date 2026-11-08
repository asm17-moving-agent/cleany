// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cleany_navigation/mppi/optimizer.hpp"
#include "cleany_navigation/mppi/wheel_projection.hpp"

// Tests main optimizer functions

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace cleany_navigation::mppi;  // NOLINT
using namespace cleany_navigation::mppi::critics;  // NOLINT
using namespace cleany_navigation::mppi::utils;  // NOLINT
using xt::evaluation_strategy::immediate;

class OptimizerTester : public Optimizer
{
public:
  OptimizerTester()
  : Optimizer() {}

  void verifyProximityLimit()
  {
    costmap_ros_=std::make_shared<nav2_costmap_2d::Costmap2DROS>("continuity_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->getLayeredCostmap()->resizeMap(400,400,.01,-2.,-2.,true);
    costmap_=costmap_ros_->getCostmap();costmap_->resetMap(0,0,400,400);
    state_.pose.pose.orientation.w=1.;wheel_max_rad_s_=4.;wheel_acceleration_=3.;
    wheel_radius_m_=.0635;proximity_wheel_limit_=1.5;proximity_clearance_=.11;
    EXPECT_NEAR(proximityWheelLimit(),4.,1e-5);
    double edge=0.;for(const auto&p:costmap_ros_->getRobotFootprint())edge=std::max(edge,p.x);
    const auto limit=[&](double distance,unsigned char value) {
      costmap_->resetMap(0,0,400,400);unsigned int x,y;
      EXPECT_TRUE(costmap_->worldToMap(edge+distance,0.,x,y));costmap_->setCost(x,y,value);
      return proximityWheelLimit();
    };
    const auto ahead=limit(.20,254);EXPECT_GT(ahead,1.5);EXPECT_LT(ahead,4.);
    EXPECT_NEAR(limit(.10,254),1.5,1e-5);
    EXPECT_NEAR(limit(.10,255),1.5,1e-5);
    EXPECT_NEAR(limit(.4,254),4.,1e-5);
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());costmap_=nullptr;
  }

  void verifyMotionContinuity()
  {
    acceleration_limits_=true; ax_max_=.2;ay_max_=.15;az_max_=.4;wheel_acceleration_=3.;
    wheel_radius_m_=.0635;wheelbase_m_=.35;wheel_separation_m_=.59;effective_wheel_limit_=4.;
    settings_.model_dt=.05;settings_.time_steps=80;settings_.constraints={.254,-.254,.254,.5404255};
    setMotionModel("Omni");acceleration_origin_={0,0,0};
    control_sequence_.vx=xt::ones<float>({80})*.254;
    control_sequence_.vy=xt::ones<float>({80})*.254;
    control_sequence_.wz=xt::ones<float>({80})*.54;
    applyControlSequenceConstraints();
    PlanarVelocity previous{};
    for(size_t i=0;i<80;++i) {
      const float dx=control_sequence_.vx(i)-previous.x,dy=control_sequence_.vy(i)-previous.y,dz=control_sequence_.wz(i)-previous.yaw;
      EXPECT_LE(std::abs(dx),.010001);EXPECT_LE(std::abs(dy),.007501);EXPECT_LE(std::abs(dz),.020001);
      EXPECT_LE((std::abs(dx)+std::abs(dy)+.47*std::abs(dz))/.0635,.15001);
      EXPECT_LE((std::abs(control_sequence_.vx(i))+std::abs(control_sequence_.vy(i))+.47*std::abs(control_sequence_.wz(i)))/.0635,4.00001);
      previous={control_sequence_.vx(i),control_sequence_.vy(i),control_sequence_.wz(i)};
    }
    // Switching to a turn brakes translation in the scored and emitted sequence.
    acceleration_origin_={.15,0,0};path_rotation_active_=true;
    applyControlSequenceConstraints();
    EXPECT_GT(control_sequence_.vx(0),.13);EXPECT_NEAR(control_sequence_.vx(79),0.,1e-6);

  }

  void verifyBodyEnvelope()
  {
    costmap_ros_=std::make_shared<nav2_costmap_2d::Costmap2DROS>("body_limit_costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());
    costmap_ros_->getLayeredCostmap()->resizeMap(600,600,.01,-3.,-3.,true);
    costmap_=costmap_ros_->getCostmap();costmap_->resetMap(0,0,600,600);
    state_.pose.pose.orientation.w=1.;translation_max_=.5;proximity_translation_max_=.16;
    proximity_yaw_max_=.3;goal_translation_max_=.06;goal_yaw_max_=.15;goal_slow_radius_=.2;
    acceleration_limits_=true;wheel_max_rad_s_=effective_wheel_limit_=10.;
    wheel_radius_m_=.0635;wheelbase_m_=.35;wheel_separation_m_=.59;
    settings_.constraints={.5,-.5,.5,.7};settings_.model_dt=.05;settings_.time_steps=120;
    setMotionModel("Omni");
    updateBodyLimits();EXPECT_NEAR(effective_translation_limit_,.5,1e-6);
    EXPECT_NEAR(effective_yaw_limit_,.7,1e-6);
    control_sequence_.vx=xt::ones<float>({120})*.5;
    control_sequence_.vy=xt::zeros<float>({120});control_sequence_.wz=xt::zeros<float>({120});
    acceleration_origin_={0,0,0};applyControlSequenceConstraints();
    EXPECT_NEAR(control_sequence_.vx(119),.5,1e-5);  // Not the previous 4 rad/s cap.
    control_sequence_.vx.fill(.5);control_sequence_.vy.fill(.5);control_sequence_.wz.fill(.7);
    applyControlSequenceConstraints();
    for(size_t t=0;t<120;++t) {
      const float x=control_sequence_.vx(t),y=control_sequence_.vy(t),w=control_sequence_.wz(t);
      EXPECT_LE(std::hypot(x,y),.50001);EXPECT_LE(std::abs(w),.70001);
      EXPECT_LE((std::abs(x)+std::abs(y)+.47*std::abs(w))/.0635,10.0001);
    }
    double edge=0;for(const auto&p:costmap_ros_->getRobotFootprint())edge=std::max(edge,p.x);
    unsigned int mx,my;ASSERT_TRUE(costmap_->worldToMap(edge+.10,0.,mx,my));
    costmap_->setCost(mx,my,254);updateBodyLimits();
    EXPECT_NEAR(effective_translation_limit_,.16,1e-6);EXPECT_NEAR(effective_yaw_limit_,.3,1e-6);
    costmap_->resetMap(0,0,600,600);
    ASSERT_TRUE(costmap_->worldToMap(edge+.50,0.,mx,my));costmap_->setCost(mx,my,254);
    updateBodyLimits();EXPECT_GT(effective_translation_limit_,.16);EXPECT_LT(effective_translation_limit_,.5);
    costmap_->resetMap(0,0,600,600);
    path_.x=xt::xtensor<float,1>{.1f};path_.y=xt::xtensor<float,1>{0.f};
    updateBodyLimits();EXPECT_NEAR(effective_translation_limit_,.06,1e-6);EXPECT_NEAR(effective_yaw_limit_,.15,1e-6);
    PlanarVelocity prior{.4,0,0};float x=.5,y=0,w=0;constrainMotion(x,y,w,prior);
    EXPECT_GT(x,.38);EXPECT_LT(x,.4);  // A newly smaller target produces deceleration, not a jump.
    settings_.constraints.vx_max=settings_.constraints.vy=.03;
    settings_.constraints.wz=.08;updateBodyLimits();
    EXPECT_LE(effective_translation_limit_,.030001);EXPECT_LE(effective_yaw_limit_,.080001);
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());costmap_=nullptr;
  }
  void verifyCrossTrackCost()
  {
    settings_.batch_size=3;settings_.time_steps=10;settings_.model_dt=.05;
    state_.reset(3,10);generated_trajectories_.reset(3,10);costs_=xt::zeros<float>({3});
    path_.x=xt::zeros<float>({3});path_.y=xt::xtensor<float,1>{0.f,.25f,.5f};path_.yaws=xt::zeros<float>({3});
    path_velocity_weight_=30;control_variation_weight_=0;
    state_.cvy.fill(.1f);state_.cvx.fill(0.f);state_.cwz.fill(0.f);generated_trajectories_.yaws.fill(0.f);
    for(size_t t=0;t<10;++t){state_.cvx(1,t)=.03f;state_.cvx(2,t)=-.03f;}
    scoreMotionContinuity();EXPECT_LT(costs_(0),costs_(1));EXPECT_LT(costs_(0),costs_(2));
    // Off-path motion back toward the line remains preferred over rigid vx=0.
    state_.pose.pose.position.x=.03;costs_.fill(0);scoreMotionContinuity();
    EXPECT_LT(costs_(2),costs_(0));
  }

  void prepareArrival(double x, nav2_core::GoalChecker * checker, double exit_radius = 0.13)
  {
    goal_rotate_enter_radius_ = 0.10f;
    goal_rotate_exit_radius_ = exit_radius;
    settings_.batch_size = 8;
    settings_.time_steps = 12;
    costs_ = xt::ones<float>({8});
    if (!motion_model_) {setMotionModel("Omni");}
    control_sequence_.vx = xt::ones<float>({12});
    control_sequence_.vy = xt::ones<float>({12});
    control_sequence_.wz = xt::zeros<float>({12});
    nav_msgs::msg::Path path;
    path.poses.resize(2);
    path.poses.back().pose.orientation.w = 1.0;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.orientation.w = 1.0;
    prepare(pose, geometry_msgs::msg::Twist(), path, checker);
  }
  bool rotating() const {return goal_rotation_active_;}
  void preparePathTurn(double heading)
  {
    path_rotate_enabled_ = true;
    settings_.batch_size = 8;
    settings_.time_steps = 12;
    costs_ = xt::ones<float>({8});
    if (!motion_model_) {setMotionModel("DiffDrive");}
    control_sequence_.vx = xt::ones<float>({12});
    control_sequence_.vy = xt::ones<float>({12});
    control_sequence_.wz = xt::zeros<float>({12});
    nav_msgs::msg::Path path;
    path.poses.resize(3);
    path.poses[0].pose.orientation.w = 1.0;
    for (size_t i = 1; i < 3; ++i) {
      path.poses[i].pose.orientation.z = std::sin(.5);
      path.poses[i].pose.orientation.w = std::cos(.5);
    }
    path.poses.back().pose.position.y = 2.0;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.z = std::sin(heading / 2);
    pose.pose.orientation.w = std::cos(heading / 2);
    prepare(pose, geometry_msgs::msg::Twist(), path, nullptr);
  }
  bool pathRotating() const {return path_rotation_active_;}
  void verifyArrivalRetry()
  {
    reset();
    EXPECT_TRUE(goal_rotation_active_);
  }
  void verifyArrivalHistoryZero()
  {
    EXPECT_EQ(xt::sum(xt::abs(control_sequence_.vx))(), 0.0f);
    EXPECT_EQ(xt::sum(xt::abs(control_sequence_.vy))(), 0.0f);
    for (const auto & control : control_history_) {
      EXPECT_EQ(control.vx, 0.0f);
      EXPECT_EQ(control.vy, 0.0f);
    }
  }

  void verifyFilteredBounds()
  {
    settings_.time_steps = 80;
    settings_.constraints.vx_min = -0.15f;
    settings_.constraints.vx_max = 0.15f;
    settings_.constraints.vy = 0.10f;
    settings_.constraints.wz = 0.14f;
    if (!motion_model_) {setMotionModel("Omni");}
    control_sequence_.vx = 0.15f * xt::ones<float>({80});
    control_sequence_.vy = 0.10f * xt::ones<float>({80});
    control_sequence_.wz = 0.14f * xt::ones<float>({80});
    // The negative filter taps multiply opposing, but still bounded, values.
    control_sequence_.vx(4) = -0.15f;
    control_sequence_.vy(4) = -0.10f;
    control_sequence_.wz(4) = -0.14f;
    for (auto & control : control_history_) {control = {0.15f, 0.10f, 0.14f};}
    control_history_[0] = {-0.15f, -0.10f, -0.14f};
    utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
    EXPECT_GT(control_sequence_.vx(0), 0.15f);
    EXPECT_GT(control_sequence_.vy(0), 0.10f);
    EXPECT_GT(control_sequence_.wz(0), 0.14f);
    applyControlSequenceConstraints();
    EXPECT_LE(xt::amax(xt::abs(control_sequence_.vx))(), 0.15f);
    EXPECT_LE(xt::amax(xt::abs(control_sequence_.vy))(), 0.10f);
    EXPECT_LE(xt::amax(xt::abs(control_sequence_.wz))(), 0.14f);
  }

  void verifyArrivalRollout()
  {
    state_.reset(settings_.batch_size, settings_.time_steps);
    generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);
    settings_.model_dt = 0.05f;
    settings_.sampling_std.vx = settings_.sampling_std.vy = 0.1f;
    settings_.sampling_std.wz = 0.2f;
    state_.pose.pose.orientation.w = 1.0;
    state_.speed.linear.x = 0.03;
    state_.speed.linear.y = -0.02;
    noise_generator_.reset(settings_, true);
    generateNoisedTrajectories();
    EXPECT_EQ(xt::sum(xt::abs(state_.cvx))(), 0.0f);
    EXPECT_EQ(xt::sum(xt::abs(state_.cvy))(), 0.0f);
    EXPECT_GT(xt::sum(xt::abs(state_.cwz))(), 0.0f);
    for (size_t batch = 0; batch < settings_.batch_size; ++batch) {
      EXPECT_NEAR(state_.vx(batch, 0), 0.03, 1e-6);
      EXPECT_NEAR(state_.vy(batch, 0), -0.02, 1e-6);
      // Scoring includes the measured initial translation, not an idealized stop.
      EXPECT_NEAR(generated_trajectories_.x(batch, 0), state_.pose.pose.position.x + 0.03 * 0.05, 1e-6);
      EXPECT_NEAR(generated_trajectories_.y(batch, 0), -0.02 * 0.05, 1e-6);
      for (size_t t = 1; t < settings_.time_steps; ++t) {
        EXPECT_EQ(state_.vx(batch, t), 0.0f);
        EXPECT_EQ(state_.vy(batch, t), 0.0f);
      }
    }
  }

  void testSetDiffModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("DiffDrive"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<DiffDriveMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetOmniModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("Omni"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<OmniMotionModel *>(motion_model_.get()));
    EXPECT_TRUE(isHolonomic());
  }

  void testSetAckModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    EXPECT_NO_THROW(setMotionModel("Ackermann"));
    EXPECT_NE(motion_model_.get(), nullptr);
    EXPECT_TRUE(dynamic_cast<AckermannMotionModel *>(motion_model_.get()));
    EXPECT_FALSE(isHolonomic());
  }

  void testSetRandModel()
  {
    EXPECT_EQ(motion_model_.get(), nullptr);
    try {
      setMotionModel("Random");
      FAIL();
    } catch (...) {
      SUCCEED();
    }
    EXPECT_EQ(motion_model_.get(), nullptr);
  }

  void resetMotionModel()
  {
    motion_model_.reset();
  }

  void setOffsetWrapper(const double freq)
  {
    return setOffset(freq);
  }

  bool getShiftControlSequence()
  {
    return settings_.shift_control_sequence;
  }

  void fillOptimizerWithGarbage()
  {
    state_.vx = 0.43432 * xt::ones<float>({1000, 10});
    control_sequence_.vx = 342.0 * xt::ones<float>({30});
    control_history_[0] = {43, 5646, 32432};
    costs_ = 5.32 * xt::ones<float>({56453});
    generated_trajectories_.x = 432.234 * xt::ones<float>({7865, 1});
  }

  void testReset()
  {
    reset();

    EXPECT_EQ(state_.vx, xt::zeros<float>({1000, 50}));
    EXPECT_EQ(control_sequence_.vx, xt::zeros<float>({50}));
    EXPECT_EQ(control_history_[0].vx, 0.0);
    EXPECT_EQ(control_history_[0].vy, 0.0);
    EXPECT_NEAR(xt::sum(costs_, immediate)(), 0, 1e-6);
    EXPECT_EQ(generated_trajectories_.x, xt::zeros<float>({1000, 50}));
  }

  bool fallbackWrapper(bool fail)
  {
    return fallback(fail);
  }

  void testPrepare(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed,
    const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
  {
    prepare(robot_pose, robot_speed, plan, goal_checker);

    EXPECT_EQ(critics_data_.goal_checker, nullptr);
    EXPECT_NEAR(xt::sum(costs_, immediate)(), 0, 1e-6);  // should be reset
    EXPECT_FALSE(critics_data_.fail_flag);  // should be reset
    EXPECT_FALSE(critics_data_.motion_model->isHolonomic());  // object is valid + diff drive
    EXPECT_FALSE(critics_data_.furthest_reached_path_point.has_value());  // val is not set
    EXPECT_FALSE(critics_data_.path_pts_valid.has_value());  // val is not set
    EXPECT_EQ(state_.pose.pose.position.x, 999);
    EXPECT_EQ(state_.speed.linear.y, 4.0);
    EXPECT_EQ(path_.x.shape(0), 17u);
  }

  void shiftControlSequenceWrapper()
  {
    return shiftControlSequence();
  }

  std::pair<double, double> getVelLimits()
  {
    auto & s = settings_;
    return {s.constraints.vx_min, s.constraints.vx_max};
  }

  void applyControlSequenceConstraintsWrapper()
  {
    return applyControlSequenceConstraints();
  }

  models::ControlSequence & grabControlSequence()
  {
    return control_sequence_;
  }

  void testupdateStateVels()
  {
    // updateInitialStateVelocities
    models::State state;
    state.reset(1000, 50);
    state.speed.linear.x = 5.0;
    state.speed.linear.y = 1.0;
    state.speed.angular.z = 6.0;
    state.cvx = 0.75 * xt::ones<float>({1000, 50});
    state.cvy = 0.5 * xt::ones<float>({1000, 50});
    state.cwz = 0.1 * xt::ones<float>({1000, 50});
    updateInitialStateVelocities(state);
    EXPECT_NEAR(state.vx(0, 0), 5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), 6.0, 1e-6);

    // propagateStateVelocitiesFromInitials
    propagateStateVelocitiesFromInitials(state);
    EXPECT_NEAR(state.vx(0, 0), 5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), 1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), 6.0, 1e-6);
    EXPECT_NEAR(state.vx(0, 1), 0.75, 1e-6);
    EXPECT_NEAR(state.vy(0, 1), 0.5, 1e-6);
    EXPECT_NEAR(state.wz(0, 1), 0.1, 1e-6);

    // Putting them together: updateStateVelocities
    state.reset(1000, 50);
    state.speed.linear.x = -5.0;
    state.speed.linear.y = -1.0;
    state.speed.angular.z = -6.0;
    state.cvx = -0.75 * xt::ones<float>({1000, 50});
    state.cvy = -0.5 * xt::ones<float>({1000, 50});
    state.cwz = -0.1 * xt::ones<float>({1000, 50});
    updateStateVelocities(state);
    EXPECT_NEAR(state.vx(0, 0), -5.0, 1e-6);
    EXPECT_NEAR(state.vy(0, 0), -1.0, 1e-6);
    EXPECT_NEAR(state.wz(0, 0), -6.0, 1e-6);
    EXPECT_NEAR(state.vx(0, 1), -0.75, 1e-6);
    EXPECT_NEAR(state.vy(0, 1), -0.5, 1e-6);
    EXPECT_NEAR(state.wz(0, 1), -0.1, 1e-6);
  }

  geometry_msgs::msg::TwistStamped getControlFromSequenceAsTwistWrapper()
  {
    builtin_interfaces::msg::Time stamp;
    return getControlFromSequenceAsTwist(stamp);
  }

  void integrateStateVelocitiesWrapper(
    models::Trajectories & traj,
    const models::State & state)
  {
    return integrateStateVelocities(traj, state);
  }
};

TEST(OptimizerTests, BasicInitializedFunctions)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Should be empty of size batches x time steps
  // and tests getting set params: time_steps, batch_size, controller_frequency
  auto trajs = optimizer_tester.getGeneratedTrajectories();
  EXPECT_EQ(trajs.x.shape(0), 1000u);
  EXPECT_EQ(trajs.x.shape(1), 50u);
  EXPECT_EQ(trajs.x, xt::zeros<float>({1000, 50}));

  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto traj = optimizer_tester.getOptimizedTrajectory();
  EXPECT_EQ(traj(5, 0), 0.0);  // x
  EXPECT_EQ(traj(5, 1), 0.0);  // y
  EXPECT_EQ(traj(5, 2), 0.0);  // yaw
  EXPECT_EQ(traj.shape(0), 50u);
  EXPECT_EQ(traj.shape(1), 3u);

  optimizer_tester.reset();
  optimizer_tester.shutdown();
}

TEST(OptimizerTests, TestOptimizerMotionModels)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Diff Drive should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetDiffModel();

  // Omni Drive should be holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // // Ackermann should be non-holonomic
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetAckModel();

  // // Rand should fail
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetRandModel();
}

TEST(OptimizerTests, setOffsetTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test offsets are properly set based on relationship of model_dt and controller frequency
  // Also tests getting set model_dt parameter.
  EXPECT_THROW(optimizer_tester.setOffsetWrapper(1.0), std::runtime_error);
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(30.0));
  EXPECT_FALSE(optimizer_tester.getShiftControlSequence());
  EXPECT_NO_THROW(optimizer_tester.setOffsetWrapper(10.0));
  EXPECT_TRUE(optimizer_tester.getShiftControlSequence());
}

TEST(OptimizerTests, resetTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Tests resetting the full state of all the functions after filling with garbage
  optimizer_tester.fillOptimizerWithGarbage();
  optimizer_tester.testReset();
}

TEST(OptimizerTests, FallbackTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test fallback logic, also tests getting set param retry_attempt_limit
  // Because retry set to 2, it should attempt soft resets 2x before throwing exception
  // for hard reset
  EXPECT_FALSE(optimizer_tester.fallbackWrapper(false));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_TRUE(optimizer_tester.fallbackWrapper(true));
  EXPECT_THROW(optimizer_tester.fallbackWrapper(true), std::runtime_error);
}

TEST(OptimizerTests, PrepareTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test Prepare function to set the state of the robot pose/speed on new cycle
  // Populate the contents with things easily identifiable if correct
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 999;
  geometry_msgs::msg::Twist speed;
  speed.linear.y = 4.0;
  nav_msgs::msg::Path path;
  path.poses.resize(17);

  optimizer_tester.testPrepare(pose, speed, path, nullptr);
}

TEST(OptimizerTests, shiftControlSequenceTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test shiftControlSequence by setting the 2nd value to something unique to neighbors
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.reset(100);
  sequence.vx(0) = 9999;
  sequence.vx(1) = 6;
  sequence.vx(2) = 888;
  sequence.vy(0) = 9999;
  sequence.vy(1) = 6;
  sequence.vy(2) = 888;
  sequence.wz(0) = 9999;
  sequence.wz(1) = 6;
  sequence.wz(2) = 888;

  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  optimizer_tester.shiftControlSequenceWrapper();

  EXPECT_EQ(sequence.vx(0), 6);
  EXPECT_EQ(sequence.vy(0), 6);
  EXPECT_EQ(sequence.wz(0), 6);
  EXPECT_EQ(sequence.vx(1), 888);
  EXPECT_EQ(sequence.vy(1), 888);
  EXPECT_EQ(sequence.wz(1), 888);
  EXPECT_EQ(sequence.vx(2), 0);
  EXPECT_EQ(sequence.vy(2), 0);
  EXPECT_EQ(sequence.wz(2), 0);
}

TEST(OptimizerTests, SpeedLimitTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.retry_attempt_limit", rclcpp::ParameterValue(2));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test Speed limits API
  auto [v_min, v_max] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max, 0.5f);
  EXPECT_EQ(v_min, -0.35f);
  optimizer_tester.setSpeedLimit(0, false);
  auto [v_min2, v_max2] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max2, 0.5f);
  EXPECT_EQ(v_min2, -0.35f);
  optimizer_tester.setSpeedLimit(50.0, true);
  auto [v_min3, v_max3] = optimizer_tester.getVelLimits();
  EXPECT_NEAR(v_max3, 0.5 / 2.0, 1e-3);
  EXPECT_NEAR(v_min3, -0.35 / 2.0, 1e-3);
  optimizer_tester.setSpeedLimit(0, true);
  auto [v_min4, v_max4] = optimizer_tester.getVelLimits();
  EXPECT_EQ(v_max4, 0.5f);
  EXPECT_EQ(v_min4, -0.35f);
  optimizer_tester.setSpeedLimit(0.75, false);
  auto [v_min5, v_max5] = optimizer_tester.getVelLimits();
  EXPECT_NEAR(v_max5, 0.75, 1e-3);
  EXPECT_NEAR(v_min5, -0.5249, 1e-2);
}

TEST(OptimizerTests, applyControlSequenceConstraintsTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.75));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test constraints being applied to ensure feasibility of trajectories
  // Also tests param get of set vx/vy/wz min/maxes

  // Set model to omni to consider holonomic vy elements
  // Ack is not tested here because `applyConstraints` is covered in detail
  // in motion_models_test.cpp
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto & sequence = optimizer_tester.grabControlSequence();

  // Test boundary of limits
  sequence.vx = xt::ones<float>({50});
  sequence.vy = 0.75 * xt::ones<float>({50});
  sequence.wz = 2.0 * xt::ones<float>({50});
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_EQ(sequence.vx, xt::ones<float>({50}));
  EXPECT_EQ(sequence.vy, 0.75 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.wz, 2.0 * xt::ones<float>({50}));

  // Test breaking limits sets to maximum
  sequence.vx = 5.0 * xt::ones<float>({50});
  sequence.vy = 5.0 * xt::ones<float>({50});
  sequence.wz = 5.0 * xt::ones<float>({50});
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_EQ(sequence.vx, xt::ones<float>({50}));
  EXPECT_EQ(sequence.vy, 0.75 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.wz, 2.0 * xt::ones<float>({50}));

  // Test breaking limits sets to minimum
  sequence.vx = -5.0 * xt::ones<float>({50});
  sequence.vy = -5.0 * xt::ones<float>({50});
  sequence.wz = -5.0 * xt::ones<float>({50});
  optimizer_tester.applyControlSequenceConstraintsWrapper();
  EXPECT_EQ(sequence.vx, -1.0 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.vy, -0.75 * xt::ones<float>({50}));
  EXPECT_EQ(sequence.wz, -2.0 * xt::ones<float>({50}));
}

TEST(OptimizerTests, updateStateVelocitiesTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.60));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test settings of the state to the initial robot speed to start rollout
  // Set model to omni to consider holonomic vy elements
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  optimizer_tester.testupdateStateVels();
}

TEST(OptimizerTests, getControlFromSequenceAsTwistTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  node->declare_parameter("mppic.vx_max", rclcpp::ParameterValue(1.0));
  node->declare_parameter("mppic.vx_min", rclcpp::ParameterValue(-1.0));
  node->declare_parameter("mppic.vy_max", rclcpp::ParameterValue(0.60));
  node->declare_parameter("mppic.wz_max", rclcpp::ParameterValue(2.0));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);

  // Test conversion of control sequence into a Twist command to execute
  auto & sequence = optimizer_tester.grabControlSequence();
  sequence.vx = 0.25 * xt::ones<float>({10});
  sequence.vy = 0.5 * xt::ones<float>({10});
  sequence.wz = 0.1 * xt::ones<float>({10});

  auto diff_t = optimizer_tester.getControlFromSequenceAsTwistWrapper();
  EXPECT_NEAR(diff_t.twist.linear.x, 0.25, 1e-6);
  EXPECT_NEAR(diff_t.twist.linear.y, 0.0, 1e-6);  // Y should not be populated
  EXPECT_NEAR(diff_t.twist.angular.z, 0.1, 1e-6);

  // Set model to omni to consider holonomic vy elements
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();
  auto omni_t = optimizer_tester.getControlFromSequenceAsTwistWrapper();
  EXPECT_NEAR(omni_t.twist.linear.x, 0.25, 1e-6);
  EXPECT_NEAR(omni_t.twist.linear.y, 0.5, 1e-6);  // Now it should be
  EXPECT_NEAR(omni_t.twist.angular.z, 0.1, 1e-6);
}

TEST(OptimizerTests, integrateStateVelocitiesTests)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  OptimizerTester optimizer_tester;
  node->declare_parameter("controller_frequency", rclcpp::ParameterValue(30.0));
  node->declare_parameter("mppic.batch_size", rclcpp::ParameterValue(1000));
  node->declare_parameter("mppic.model_dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter("mppic.time_steps", rclcpp::ParameterValue(50));
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);
  optimizer_tester.initialize(node, "mppic", costmap_ros, &param_handler);
  optimizer_tester.resetMotionModel();
  optimizer_tester.testSetOmniModel();

  // Test integration of velocities for trajectory rollout poses

  // Give it a couple of easy const traj and check rollout, start from 0
  models::State state;
  state.reset(1000, 50);
  models::Trajectories traj;
  state.vx = 0.1 * xt::ones<float>({1000, 50});
  xt::view(state.vx, xt::all(), 0) = xt::zeros<float>({1000});
  state.vy = xt::zeros<float>({1000, 50});
  state.wz = xt::zeros<float>({1000, 50});

  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);
  EXPECT_EQ(traj.y, xt::zeros<float>({1000, 50}));
  EXPECT_EQ(traj.yaws, xt::zeros<float>({1000, 50}));
  for (unsigned int i = 0; i != traj.x.shape(1); i++) {
    EXPECT_NEAR(traj.x(1, i), i * 0.1 /*vel*/ * 0.1 /*dt*/, 1e-3);
  }

  // Give it a bit of a more complex trajectory to crunch
  state.vy = 0.2 * xt::ones<float>({1000, 50});
  xt::view(state.vy, xt::all(), 0) = xt::zeros<float>({1000});
  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);

  EXPECT_EQ(traj.yaws, xt::zeros<float>({1000, 50}));
  for (unsigned int i = 0; i != traj.x.shape(1); i++) {
    EXPECT_NEAR(traj.x(1, i), i * 0.1 /*vel*/ * 0.1 /*dt*/, 1e-3);
    EXPECT_NEAR(traj.y(1, i), i * 0.2 /*vel*/ * 0.1 /*dt*/, 1e-3);
  }

  // Lets add some angular motion to the mix
  state.vy = xt::zeros<float>({1000, 50});
  state.wz = 0.2 * xt::ones<float>({1000, 50});
  xt::view(state.wz, xt::all(), 0) = xt::zeros<float>({1000});
  optimizer_tester.integrateStateVelocitiesWrapper(traj, state);

  float x = 0;
  float y = 0;
  for (unsigned int i = 1; i != traj.x.shape(1); i++) {
    std::cout << i << std::endl;
    x += (0.1 /*vx*/ * cos(0.2 /*wz*/ * 0.1 /*model_dt*/ * (i - 1))) * 0.1 /*model_dt*/;
    y += (0.1 /*vx*/ * sin(0.2 /*wz*/ * 0.1 /*model_dt*/ * (i - 1))) * 0.1 /*model_dt*/;

    EXPECT_NEAR(traj.x(1, i), x, 1e-6);
    EXPECT_NEAR(traj.y(1, i), y, 1e-6);
  }
}

class ArrivalGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist &) override
  {pose.position.x = pose.position.y = 0.15; return true;}
};

TEST(OptimizerTests, ArrivalHysteresisAndToleranceBoundary)
{
  ArrivalGoalChecker checker;
  OptimizerTester optimizer;
  optimizer.prepareArrival(0.11, &checker);
  EXPECT_FALSE(optimizer.rotating());
  optimizer.prepareArrival(0.09, &checker);
  EXPECT_TRUE(optimizer.rotating());
  optimizer.verifyArrivalHistoryZero();
  optimizer.prepareArrival(0.12, &checker);
  EXPECT_TRUE(optimizer.rotating());
  optimizer.prepareArrival(0.14, &checker);
  EXPECT_FALSE(optimizer.rotating());
  optimizer.prepareArrival(0.12, &checker);
  EXPECT_FALSE(optimizer.rotating());
  optimizer.prepareArrival(0.09, &checker, 0.16);
  EXPECT_FALSE(optimizer.rotating());
  optimizer.prepareArrival(0.09, nullptr);
  EXPECT_FALSE(optimizer.rotating());
}

TEST(OptimizerTests, PlannedTurnClearsTranslationUntilHeadingIsReached)
{
  OptimizerTester optimizer;
  optimizer.preparePathTurn(0.0);
  EXPECT_TRUE(optimizer.pathRotating());
  optimizer.verifyArrivalHistoryZero();
  optimizer.preparePathTurn(.95);
  EXPECT_FALSE(optimizer.pathRotating());
}

TEST(OptimizerTests, FilterOvershootIsBounded)
{
  OptimizerTester optimizer;
  optimizer.verifyFilteredBounds();
}

TEST(OptimizerTests, ArrivalRolloutPreservesMeasuredVelocityAndAngularExploration)
{
  ArrivalGoalChecker checker;
  OptimizerTester optimizer;
  optimizer.prepareArrival(0.09, &checker);
  ASSERT_TRUE(optimizer.rotating());
  optimizer.verifyArrivalRollout();
  optimizer.verifyArrivalRetry();
}

TEST(OptimizerTests, CoupledWheelProjectionPreservesRatiosInAllOctants)
{
  const float radius = 0.0635f, lever = 0.4769f;
  for (float sx : {-1.0f, 1.0f}) {
    for (float sy : {-1.0f, 1.0f}) {
      for (float sw : {-1.0f, 1.0f}) {
        float x = sx * 0.3175f, y = sy * 0.2f, w = sw * 0.6f;
        const float ratio_y = y / x, ratio_w = w / x;
        cleany_navigation::mppi::projectWheelTwist(x, y, w, radius, lever, 5.0f);
        EXPECT_NEAR(y / x, ratio_y, 1e-6);
        EXPECT_NEAR(w / x, ratio_w, 1e-6);
        for (float sign_y : {-1.0f, 1.0f}) {
          for (float sign_w : {-1.0f, 1.0f}) {
            EXPECT_LE(std::abs((x + sign_y * y + sign_w * lever * w) / radius), 5.0f + 1e-6f);
          }
        }
      }
    }
  }
  float x = 0.3175f, y = 0.0f, w = 0.0f;
  cleany_navigation::mppi::projectWheelTwist(x, y, w, radius, lever, 5.0f);
  EXPECT_NEAR(x, 0.3175f, 1e-7);
  x = 0.02f; y = -0.01f; w = 0.04f;
  cleany_navigation::mppi::projectWheelTwist(x, y, w, radius, lever, 5.0f);
  EXPECT_EQ(x, 0.02f); EXPECT_EQ(y, -0.01f); EXPECT_EQ(w, 0.04f);
}

TEST(OptimizerTests, MotionBoundsAndTurnBraking) {OptimizerTester tester;tester.verifyMotionContinuity();}
TEST(OptimizerTests, BodyEnvelopeAnticipatesObstaclesAndArrival) {OptimizerTester tester;tester.verifyBodyEnvelope();}
TEST(OptimizerTests, CrossTrackVelocityAllowsNecessaryCorrection) {OptimizerTester tester;tester.verifyCrossTrackCost();}

TEST(OptimizerTests, ProximityBrakingStartsBeforeHardSlowZone) {OptimizerTester tester;tester.verifyProximityLimit();}
