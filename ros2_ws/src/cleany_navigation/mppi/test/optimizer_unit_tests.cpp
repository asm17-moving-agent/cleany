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
#include "cleany_navigation/mppi/path_seed.hpp"

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

  void verifyValidatedStandstill(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & map_ros)
  {
    parent_ = node; costmap_ros_ = map_ros; costmap_ = map_ros->getCostmap();
    costmap_->resizeMap(100, 100, .05, 0., 0.);
    costmap_->resetMap(0, 0, 100, 100);
    for (unsigned y = 0; y < 100; ++y) {costmap_->setCost(26, y, 254);}
    settings_.time_steps = 40; settings_.model_dt = .05f;
    settings_.shift_control_sequence = false;
    settings_.constraints.vx_min = -.5f; settings_.constraints.vx_max = .5f;
    settings_.constraints.vy = .5f; settings_.constraints.wz = .5f;
    setMotionModel("Omni");
    state_.reset(1, 40); state_.pose.pose.position.x = 1.;
    state_.pose.pose.position.y = 1.; state_.pose.pose.orientation.w = 1.;
    state_.cvx.fill(.25f); state_.cvy.fill(0.f); state_.cwz.fill(0.f);
    costs_ = xt::ones<float>({1});
    control_sequence_.reset(40); control_sequence_.vx.fill(.25f);
    validated_stop_fallback_ = false;
    EXPECT_THROW(validateOutput(), std::runtime_error);
    validated_stop_fallback_ = true;
    EXPECT_NO_THROW(validateOutput());
    EXPECT_EQ(xt::sum(xt::abs(control_sequence_.vx))(), 0.f);
    EXPECT_EQ(xt::sum(xt::abs(control_sequence_.vy))(), 0.f);
    EXPECT_EQ(xt::sum(xt::abs(control_sequence_.wz))(), 0.f);
    // The discarded sample must not supply braking which is never emitted.
    models::ControlSequence braking;
    braking.reset(40); braking.vx(0) = -.5f; braking.vx(1) = .5f;
    state_.pose.pose.position.x = 1.18;
    EXPECT_TRUE(sequenceCollisionFree(braking));
    settings_.shift_control_sequence = true;
    EXPECT_FALSE(sequenceCollisionFree(braking));
    settings_.shift_control_sequence = false;
    state_.pose.pose.position.x = 1.;
    // Stopping cannot erase measured momentum into the wall.
    state_.speed.linear.x = 8.; control_sequence_.vx.fill(.25f);
    EXPECT_THROW(validateOutput(), std::runtime_error);
    state_.speed.linear.x = 0.;
    // An already colliding footprint must still fail closed.
    std::fill(costmap_->getCharMap(), costmap_->getCharMap()+10000, 254);
    EXPECT_THROW(validateOutput(), std::runtime_error);
  }

  void prepareArrival(double x, nav2_core::GoalChecker * checker, double exit_radius = 0.13,
    double enter_radius = 0.10)
  {
    goal_rotate_enter_radius_ = enter_radius;
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
  double position_tolerance = 0.15;
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    const std::string &, const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist &) override
  {pose.position.x = pose.position.y = position_tolerance; return true;}
};

TEST(OptimizerTests, ArrivalRadiusEqualToDoubleToleranceRemainsEnabled)
{
  ArrivalGoalChecker checker;
  checker.position_tolerance = .008;
  OptimizerTester optimizer;
  optimizer.prepareArrival(.005, &checker, .008, .006);
  EXPECT_TRUE(optimizer.rotating());
  optimizer.prepareArrival(.007, &checker, .008, .006);
  EXPECT_TRUE(optimizer.rotating());
  optimizer.prepareArrival(.009, &checker, .008, .006);
  EXPECT_FALSE(optimizer.rotating());
  optimizer.prepareArrival(.005, &checker, .0080001, .006);
  EXPECT_FALSE(optimizer.rotating());
}

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

TEST(OptimizerTests, PathSeedsReachTranslationAndRotationWaypointsBeforeAdvancing)
{
  const auto translating = pathSeed({0.f, 0.f, 0.f},
    {{.01f, 0.f, 0.f}, {.01f, .05f, 0.f}}, 2, .05f, .25f, .35f, .0635f, .4769f, 4.f);
  ASSERT_EQ(translating.size(), 2u);
  EXPECT_GT(translating[0].x, 0.f);
  EXPECT_FLOAT_EQ(translating[0].y, 0.f);
  EXPECT_GT(translating[1].y, 0.f);
  const auto rotating = pathSeed({0.f, 0.f, 0.f},
    {{0.f, 0.f, .02f}, {.05f, 0.f, .02f}}, 2, .05f, .25f, .35f, .0635f, .4769f, 4.f);
  EXPECT_FLOAT_EQ(rotating[0].x, 0.f);
  EXPECT_FLOAT_EQ(rotating[0].y, 0.f);
  EXPECT_GT(rotating[0].yaw, 0.f);
}

TEST(OptimizerTests, StandstillRequiresCollisionFreeMeasuredMomentumAndFootprint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("stop_output_test");
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("stop_costmap");
  map->on_configure(rclcpp_lifecycle::State());
  OptimizerTester optimizer;
  optimizer.verifyValidatedStandstill(node, map);
  map->on_cleanup(rclcpp_lifecycle::State());
}
