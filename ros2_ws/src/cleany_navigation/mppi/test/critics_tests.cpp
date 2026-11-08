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

#include <numeric>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "cleany_navigation/mppi/tools/utils.hpp"
#include "cleany_navigation/mppi/motion_models.hpp"
#include "cleany_navigation/mppi/critics/constraint_critic.hpp"
#include "cleany_navigation/mppi/critics/goal_angle_critic.hpp"
#include "cleany_navigation/mppi/critics/goal_critic.hpp"
#include "cleany_navigation/mppi/critics/obstacles_critic.hpp"
#include "cleany_navigation/mppi/critics/cost_critic.hpp"
#include "cleany_navigation/mppi/critics/path_align_critic.hpp"
#include "cleany_navigation/mppi/critics/path_align_legacy_critic.hpp"
#include "cleany_navigation/mppi/critics/path_angle_critic.hpp"
#include "cleany_navigation/mppi/critics/path_follow_critic.hpp"
#include "cleany_navigation/mppi/critics/prefer_forward_critic.hpp"
#include "cleany_navigation/mppi/critics/twirling_critic.hpp"
#include "cleany_navigation/mppi/critics/directional_motion_critic.hpp"
#include "cleany_navigation/mppi/critics/velocity_deadband_critic.hpp"
#include "nav2_core/exceptions.hpp"
#include "utils_test.cpp"  // NOLINT

// Tests the various critic plugin functions

// ROS lock used from utils_test.cpp

using namespace cleany_navigation::mppi;  // NOLINT
using namespace cleany_navigation::mppi::critics;  // NOLINT
using namespace cleany_navigation::mppi::utils;  // NOLINT
using xt::evaluation_strategy::immediate;

TEST(CriticTests, ConstraintsCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly and that defaults are reasonable
  ConstraintCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
  EXPECT_TRUE(critic.getMaxVelConstraint() > 0.0);
  EXPECT_TRUE(critic.getMinVelConstraint() < 0.0);

  // Scoring testing

  // provide velocities in constraints, should not have any costs
  state.vx = 0.40 * xt::ones<float>({1000, 30});
  state.vy = xt::zeros<float>({1000, 30});
  state.wz = xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // provide out of maximum velocity constraint
  auto last_batch_traj_in_full = xt::view(state.vx, -1, xt::all());
  last_batch_traj_in_full = 0.60 * xt::ones<float>({30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(999), 1.2, 0.01);
  costs = xt::zeros<float>({1000});

  // provide out of minimum velocity constraint
  auto first_batch_traj_in_full = xt::view(state.vx, 1, xt::all());
  first_batch_traj_in_full = -0.45 * xt::ones<float>({30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * 0.1 error introduced * 30 timesteps = 1.2
  EXPECT_NEAR(costs(1), 1.2, 0.01);
  costs = xt::zeros<float>({1000});

  // Now with ackermann, all in constraint so no costs to score
  state.vx = 0.40 * xt::ones<float>({1000, 30});
  state.wz = 1.5 * xt::ones<float>({1000, 30});
  data.motion_model = std::make_shared<AckermannMotionModel>(&param_handler, node->get_name());
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // Now violating the ackermann constraints
  state.wz = 2.5 * xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  // 4.0 weight * 0.1 model_dt * (0.2 - 0.4/2.5) * 30 timesteps = 0.48
  EXPECT_NEAR(costs(1), 0.48, 0.01);
}

TEST(CriticTests, ObstacleCriticMisalignedParams) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", true);

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  ObstaclesCritic critic;
  // Expect throw when settings mismatched
  EXPECT_THROW(
    critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler),
    nav2_core::PlannerException
  );
}

TEST(CriticTests, ObstacleCriticAlignedParams) {
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", false);

  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  ObstaclesCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
}


TEST(CriticTests, CostCriticMisAlignedParams) {
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", true);
  costmap_ros->on_configure(lstate);

  CostCritic critic;
  // Expect throw when settings mismatched
  EXPECT_THROW(
    critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler),
    nav2_core::PlannerException
  );
}

TEST(CriticTests, CostCriticAlignedParams) {
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  auto getParam = param_handler.getParamGetter("critic");
  bool consider_footprint;
  getParam(consider_footprint, "consider_footprint", false);
  costmap_ros->on_configure(lstate);

  CostCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");
}

TEST(CriticTests, GoalAngleCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly
  GoalAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path too far from `threshold_to_consider` to consider
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  path.yaws(9) = 3.14;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // Lets move it even closer, just to be sure it still doesn't trigger
  state.pose.pose.position.x = 9.2;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // provide state pose and path below `threshold_to_consider` to consider
  state.pose.pose.position.x = 9.7;
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0);
  EXPECT_NEAR(costs(0), 9.42, 0.02);  // (3.14 - 0.0) * 3.0 weight
}

TEST(CriticTests, GoalCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();

  // Initialization testing

  // Make sure initializes correctly
  GoalCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing with all trajectories set to 0

  // provide state poses and path far, should not trigger
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  path.y(9) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs(2), 0.0, 1e-6);  // (0 * 5.0 weight
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);  // Should all be 0 * 1000
  costs = xt::zeros<float>({1000});

  // provide state pose and path close
  path.x(9) = 0.5;
  path.y(9) = 0.0;
  critic.score(data);
  EXPECT_NEAR(costs(2), 2.5, 1e-6);  // (sqrt(10.0 * 10.0) * 5.0 weight
  EXPECT_NEAR(xt::sum(costs, immediate)(), 2500.0, 1e-6);  // should be 2.5 * 1000
}

TEST(CriticTests, PathAngleCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally

  // Initialization testing

  // Make sure initializes correctly
  PathAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path close, within pose tolerance so won't do anything
  state.pose.pose.position.x = 0.0;
  state.pose.pose.position.y = 0.0;
  path.reset(10);
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with less than PI/2 angular diff.
  path.x(9) = 0.95;
  data.furthest_reached_path_point = 2;  // So it grabs the 2 + offset_from_furthest_ = 6th point
  path.x(6) = 1.0;  // angle between path point and pose = 0 < max_angle_to_furthest_
  path.y(6) = 0.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close but outside of tol. with more than PI/2 angular diff.
  path.x(6) = -1.0;  // angle between path point and pose > max_angle_to_furthest_
  path.y(6) = 4.0;
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0.0);
  EXPECT_NEAR(costs(0), 3.6315, 1e-2);  // atan2(4,-1) [1.81] * 2.0 weight
}

TEST(CriticTests, LeadingLatticeRotationUsesItsEndHeading)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("rotation_test");
  node->declare_parameter("critic.respect_in_place_rotations", true);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "rotation_costmap", "", "rotation_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  ParametersHandler parameters(node);
  PathAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &parameters);
  models::State state;
  state.reset(3, 10);
  state.pose.pose.orientation.w = 1.0;
  models::Trajectories trajectories;
  trajectories.reset(3, 10);
  models::Path path;
  path.reset(5);
  path.yaws(1) = M_PI_4;
  path.yaws(2) = M_PI_2;
  path.x(3) = 1.0;
  path.x(4) = 2.0;
  xt::view(trajectories.yaws, 0, xt::all()) = M_PI_2;
  xt::view(trajectories.yaws, 2, xt::all()) = -M_PI_2;
  xt::xtensor<float, 1> costs = xt::zeros<float>({3});
  float dt = .1;
  CriticData data{state, trajectories, path, costs, dt, false, nullptr,
    nullptr, std::nullopt, std::nullopt};
  critic.score(data);
  EXPECT_NEAR(costs(0), 0.0, 1e-5);
  EXPECT_GT(costs(1), costs(0));
  EXPECT_GT(costs(2), costs(1));

  // The heading wraps at pi; crossing that seam is a small positive turn.
  path.yaws(0) = 3.1;
  path.yaws(1) = -3.1;
  path.yaws(2) = -3.0;
  xt::view(trajectories.yaws, 0, xt::all()) = -3.0;
  xt::view(trajectories.yaws, 1, xt::all()) = 3.1;
  costs.fill(0.0);
  critic.score(data);
  EXPECT_NEAR(costs(0), 0.0, 1e-5);
  EXPECT_LT(costs(1), 0.5);

  // A nearby final goal can point elsewhere; its angle critic owns arrival.
  TestGoalChecker goal_checker;
  data.goal_checker = &goal_checker;
  path.x(4) = .01;
  costs.fill(0.0);
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs)(), 0.0, 1e-5);
  path.x(4) = 2.0;
  data.goal_checker = nullptr;

  // An ordinary translated path does not create an in-place rotation target.
  path.yaws.fill(0.0);
  path.x(1) = .5;
  path.x(2) = 1.0;
  costs.fill(0.0);
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs)(), 0.0, 1e-5);
}

TEST(CriticTests, HolonomicPathAngleOnlyScoresExplicitRotations)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("holonomic_angle_test");
  node->declare_parameter("critic.respect_in_place_rotations", true);
  node->declare_parameter("critic.rotation_only", true);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "holonomic_angle_costmap", "", "holonomic_angle_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  ParametersHandler parameters(node);
  PathAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &parameters);
  models::State state;state.reset(2, 10);state.pose.pose.orientation.w = 1.;
  models::Trajectories trajectories;trajectories.reset(2, 10);
  xt::view(trajectories.yaws, 1, xt::all()) = M_PI_2;
  models::Path path;path.reset(5);
  xt::xtensor<float, 1> costs = xt::zeros<float>({2});
  float dt = .1;
  CriticData data{state, trajectories, path, costs, dt, false, nullptr,
    nullptr, std::nullopt, std::nullopt};
  for(const auto &direction : std::vector<std::pair<float,float>>{
    {0,1},{0,-1},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}})
  {
    for(size_t i=0;i<5;++i) {
      path.x(i)=.5f*i*direction.first;path.y(i)=.5f*i*direction.second;
    }
    costs.fill(0.);critic.score(data);
    EXPECT_NEAR(xt::sum(costs)(), 0., 1e-5);
  }
  path.x.fill(0.);path.y.fill(0.);
  path.yaws(1)=M_PI_4;path.yaws(2)=M_PI_2;
  path.x(3)=1.;path.x(4)=2.;
  costs.fill(0.);critic.score(data);
  EXPECT_GT(costs(0),0.);
  EXPECT_NEAR(costs(1),0.,1e-5);
}

TEST(CriticTests, HolonomicSegmentHeadingDoesNotAnticipateArrivalYaw)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("holonomic_angle_test");
  node->declare_parameter("critic.respect_in_place_rotations", true);
  node->declare_parameter("critic.rotation_only", true);
  node->declare_parameter("critic.follow_segment_heading", true);
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "holonomic_angle_costmap", "", "holonomic_angle_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  ParametersHandler parameters(node);
  PathAngleCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &parameters);
  models::State state;state.reset(2, 10);state.pose.pose.orientation.w = 1.;
  models::Trajectories trajectories;trajectories.reset(2, 10);
  xt::view(trajectories.yaws, 1, xt::all()) = M_PI_2;
  models::Path path;path.reset(5);
  xt::xtensor<float, 1> costs = xt::zeros<float>({2});
  float dt = .1;
  CriticData data{state, trajectories, path, costs, dt, false, nullptr,
    nullptr, std::nullopt, std::nullopt};
  for(const auto &direction : std::vector<std::pair<float,float>>{
    {0,1},{0,-1},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}})
  {
    for(size_t i=0;i<5;++i) {
      path.x(i)=.5f*i*direction.first;path.y(i)=.5f*i*direction.second;
    }
    costs.fill(0.);critic.score(data);
    EXPECT_NEAR(costs(0),0.,1e-5);EXPECT_GT(costs(1),0.);
  }
  // A future goal rotation must not turn a translating robot early.
  path.x=xt::xtensor<float,1>{0.f,0.f,0.f,0.f,0.f};
  path.y=xt::xtensor<float,1>{0.f,.2f,.4f,.5f,.5f};path.yaws(4)=M_PI_2;
  costs.fill(0.);critic.score(data);EXPECT_NEAR(costs(0),0.,1e-5);EXPECT_GT(costs(1),0.);
  path.x.fill(0.);path.y.fill(0.);
  path.yaws(1)=M_PI_4;path.yaws(2)=M_PI_2;
  path.x(3)=1.;path.x(4)=2.;
  costs.fill(0.);critic.score(data);
  EXPECT_GT(costs(0),0.);
  EXPECT_NEAR(costs(1),0.,1e-5);
}

TEST(CriticTests, PreferForwardCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally

  // Initialization testing

  // Make sure initializes correctly
  PreferForwardCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path far away, not within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0f, 1e-6f);

  // provide state pose and path close to trigger behavior but with all forward motion
  path.x(9) = 0.15;
  state.vx = xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0f, 1e-6f);

  // provide state pose and path close to trigger behavior but with all reverse motion
  state.vx = -1.0 * xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_GT(xt::sum(costs, immediate)(), 0.0f);
  EXPECT_NEAR(costs(0), 15.0f, 1e-3f);  // 1.0 * 0.1 model_dt * 5.0 weight * 30 length
}

TEST(CriticTests, TwirlingCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  TwirlingCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path far away, not within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 10.0;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path close to trigger behavior but with no angular variation
  path.x(9) = 0.15;
  state.wz = xt::zeros<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // Provide nearby with some motion
  auto traj_view = xt::view(state.wz, 0, xt::all());
  traj_view = 10.0;
  critic.score(data);
  EXPECT_NEAR(costs(0), 100.0, 1e-6);  // (mean(10.0) * 10.0 weight
  costs = xt::zeros<float>({1000});

  // Now try again with some wiggling noise
  traj_view = xt::random::randn<float>({30}, 0.0, 0.5);
  critic.score(data);
  EXPECT_NEAR(costs(0), 3.3, 4e-1);  // (mean of noise with mu=0, sigma=0.5 * 10.0 weight
}

TEST(CriticTests, PathFollowCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  PathFollowCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path close within positional tolerances
  state.pose.pose.position.x = 2.0;
  path.reset(6);
  path.x(5) = 1.7;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path far enough to enable
  // pose differential is (0, 0) and (0.15, 0)
  path.x(5) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 750.0, 1e-2);  // 0.15 * 5 weight * 1000
}

TEST(CriticTests, PathAlignCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  PathAlignCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path close within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 0.85;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path far enough to enable
  // but data furthest point reached is 0 and offset default is 20, so returns
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path far enough to enable, with data to pass condition
  // but with empty trajectories and paths, should still be zero
  *data.furthest_reached_path_point = 21;
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path far enough to enable, with data to pass condition
  // and with a valid path to pass invalid path condition
  state.pose.pose.position.x = 0.0;
  data.path_pts_valid.reset();  // Recompute on new path
  path.reset(22);
  path.x(0) = 0;
  path.x(1) = 0.1;
  path.x(2) = 0.2;
  path.x(3) = 0.3;
  path.x(4) = 0.4;
  path.x(5) = 0.5;
  path.x(6) = 0.6;
  path.x(7) = 0.7;
  path.x(8) = 0.8;
  path.x(9) = 0.9;
  path.x(10) = 0.9;
  path.x(11) = 0.9;
  path.x(12) = 0.9;
  path.x(13) = 0.9;
  path.x(14) = 0.9;
  path.x(15) = 0.9;
  path.x(16) = 0.9;
  path.x(17) = 0.9;
  path.x(18) = 0.9;
  path.x(19) = 0.9;
  path.x(20) = 0.9;
  path.x(21) = 0.9;
  generated_trajectories.x = 0.66 * xt::ones<float>({1000, 30});
  critic.score(data);
  // 0.66 * 1000 * 10 weight * 6 num pts eval / 6 normalization term
  // Test critic scoring, not float-reduction roundoff across 1000 trajectories.
  const double total_cost = std::accumulate(costs.cbegin(), costs.cend(), 0.0);
  EXPECT_NEAR(total_cost, 6600.0, 1e-2);

  // provide state pose and path far enough to enable, with data to pass condition
  // but path is blocked in collision
  auto * costmap = costmap_ros->getCostmap();
  // island in the middle of lethal cost to cross. Costmap defaults to size 5x5 @ 10cm resolution
  for (unsigned int i = 11; i <= 30; ++i) {  // 1.1m-3m
    for (unsigned int j = 11; j <= 30; ++j) {  // 1.1m-3m
      costmap->setCost(i, j, 254);
    }
  }

  data.path_pts_valid.reset();  // Recompute on new path
  costs = xt::zeros<float>({1000});
  path.x = 1.5 * xt::ones<float>({22});
  path.y = 1.5 * xt::ones<float>({22});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);
}

TEST(CriticTests, PathAlignLegacyCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  state.reset(1000, 30);
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  generated_trajectories.reset(1000, 30);
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  TestGoalChecker goal_checker;  // from utils_tests tolerance of 0.25 positionally
  data.goal_checker = &goal_checker;

  // Initialization testing

  // Make sure initializes correctly
  PathAlignLegacyCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide state poses and path close within positional tolerances
  state.pose.pose.position.x = 1.0;
  path.reset(10);
  path.x(9) = 0.85;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path far enough to enable
  // but data furthest point reached is 0 and offset default is 20, so returns
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path far enough to enable, with data to pass condition
  // but with empty trajectories and paths, should still be zero
  *data.furthest_reached_path_point = 21;
  path.x(9) = 0.15;
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);

  // provide state pose and path far enough to enable, with data to pass condition
  // and with a valid path to pass invalid path condition
  state.pose.pose.position.x = 0.0;
  data.path_pts_valid.reset();  // Recompute on new path
  path.reset(22);
  path.x(0) = 0;
  path.x(1) = 0.1;
  path.x(2) = 0.2;
  path.x(3) = 0.3;
  path.x(4) = 0.4;
  path.x(5) = 0.5;
  path.x(6) = 0.6;
  path.x(7) = 0.7;
  path.x(8) = 0.8;
  path.x(9) = 0.9;
  path.x(10) = 0.9;
  path.x(11) = 0.9;
  path.x(12) = 0.9;
  path.x(13) = 0.9;
  path.x(14) = 0.9;
  path.x(15) = 0.9;
  path.x(16) = 0.9;
  path.x(17) = 0.9;
  path.x(18) = 0.9;
  path.x(19) = 0.9;
  path.x(20) = 0.9;
  path.x(21) = 0.9;
  generated_trajectories.x = 0.66 * xt::ones<float>({1000, 30});
  critic.score(data);
  // 0.04 * 1000 * 10 weight * 6 num pts eval / 6 normalization term
  EXPECT_NEAR(xt::sum(costs, immediate)(), 400.0, 1e-2);

  // provide state pose and path far enough to enable, with data to pass condition
  // but path is blocked in collision
  auto * costmap = costmap_ros->getCostmap();
  // island in the middle of lethal cost to cross. Costmap defaults to size 5x5 @ 10cm resolution
  for (unsigned int i = 11; i <= 30; ++i) {  // 1.1m-3m
    for (unsigned int j = 11; j <= 30; ++j) {  // 1.1m-3m
      costmap->setCost(i, j, 254);
    }
  }

  data.path_pts_valid.reset();  // Recompute on new path
  costs = xt::zeros<float>({1000});
  path.x = 1.5 * xt::ones<float>({22});
  path.y = 1.5 * xt::ones<float>({22});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0.0, 1e-6);
}

TEST(CriticTests, VelocityDeadbandCritic)
{
  // Standard preamble
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler param_handler(node);
  auto getParam = param_handler.getParamGetter("critic");
  std::vector<double> deadband_velocities_;
  getParam(deadband_velocities_, "deadband_velocities", std::vector<double>{0.08, 0.08, 0.08});
  rclcpp_lifecycle::State lstate;
  costmap_ros->on_configure(lstate);

  models::State state;
  models::ControlSequence control_sequence;
  models::Trajectories generated_trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({1000});
  float model_dt = 0.1;
  CriticData data =
  {state, generated_trajectories, path, costs, model_dt, false, nullptr, nullptr, std::nullopt,
    std::nullopt};
  data.motion_model = std::make_shared<OmniMotionModel>();

  // Initialization testing

  // Make sure initializes correctly and that defaults are reasonable
  VelocityDeadbandCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap_ros, &param_handler);
  EXPECT_EQ(critic.getName(), "critic");

  // Scoring testing

  // provide velocities out of deadband bounds, should not have any costs
  state.vx = 0.80 * xt::ones<float>({1000, 30});
  state.vy = 0.60 * xt::ones<float>({1000, 30});
  state.wz = 0.80 * xt::ones<float>({1000, 30});
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs, immediate)(), 0, 1e-6);

  // Test cost value
  state.vx = 0.01 * xt::ones<float>({1000, 30});
  state.vy = 0.02 * xt::ones<float>({1000, 30});
  state.wz = 0.021 * xt::ones<float>({1000, 30});
  critic.score(data);
  // 35.0 weight * 0.1 model_dt * (0.07 + 0.06 + 0.059) * 30 timesteps = 56.7
  EXPECT_NEAR(costs(1), 19.845, 0.01);
}

TEST(CriticTests, DirectionalTravelCost)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("directional_test");
  node->declare_parameter("critic.lateral_weight", 2.0);
  node->declare_parameter("critic.diagonal_weight", 1.0);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler params(node);
  models::State state;
  state.reset(7, 10);
  models::Trajectories trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({7});
  float dt = 0.1f;
  CriticData data{state, trajectories, path, costs, dt, false, nullptr,
    std::make_shared<OmniMotionModel>(), std::nullopt, std::nullopt};
  DirectionalMotionCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap, &params);
  // Forward, lateral, both diagonal signs, reverse, rotation, standstill.
  xt::view(state.vx, 0, xt::all()) = 0.2f;
  xt::view(state.vy, 1, xt::all()) = 0.2f;
  xt::view(state.vx, 2, xt::all()) = 0.2f;
  xt::view(state.vy, 2, xt::all()) = 0.2f;
  xt::view(state.vx, 3, xt::all()) = -0.2f;
  xt::view(state.vy, 3, xt::all()) = -0.2f;
  xt::view(state.vx, 4, xt::all()) = -0.2f;
  xt::view(state.wz, 5, xt::all()) = 0.3f;
  critic.score(data);
  EXPECT_NEAR(costs(0), 0.0f, 1e-6);
  EXPECT_NEAR(costs(1), 0.4f, 1e-6);
  EXPECT_NEAR(costs(2), 0.5656854f, 1e-6);
  EXPECT_NEAR(costs(3), costs(2), 1e-6);
  EXPECT_NEAR(costs(4), 0.0f, 1e-6);
  EXPECT_NEAR(costs(5), 0.0f, 1e-6);
  EXPECT_NEAR(costs(6), 0.0f, 1e-6);
  costs.fill(0.0f);
  dt = 0.05f;
  critic.score(data);
  EXPECT_NEAR(costs(2), 0.2828427f, 1e-6);
  state.reset(7, 20);
  state.vx.fill(0.2f);
  state.vy.fill(-0.2f);
  costs.fill(0.0f);
  critic.score(data);
  EXPECT_NEAR(costs(2), 0.5656854f, 1e-6);  // Same duration at twice the sample count.
  costs.fill(0.0f);
  data.motion_model = std::make_shared<DiffDriveMotionModel>();
  critic.score(data);
  EXPECT_NEAR(xt::sum(costs)(), 0.0f, 1e-6);
}

TEST(CriticTests, DirectionalPeakFavorsBothAxesAtEqualTravelDistance)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("angular_directional_test");
  node->declare_parameter("critic.lateral_weight", 0.1);
  node->declare_parameter("critic.diagonal_weight", 0.45);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler params(node);
  models::State state;
  state.reset(361, 50);
  models::Trajectories trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({361});
  float dt = 0.1f;
  CriticData data{state, trajectories, path, costs, dt, false, nullptr,
    std::make_shared<OmniMotionModel>(), std::nullopt, std::nullopt};
  DirectionalMotionCritic critic;
  critic.on_configure(node, "mppi", "critic", costmap, &params);
  // Every rollout travels one metre; compare direction without a speed bias.
  for (size_t angle = 0; angle <= 360; ++angle) {
    const float radians = angle * 0.017453292519943295f;
    xt::view(state.vx, angle, xt::all()) = 0.2f * std::cos(radians);
    xt::view(state.vy, angle, xt::all()) = 0.2f * std::sin(radians);
  }
  critic.score(data);
  EXPECT_NEAR(costs(0), 0.f, 1e-6);
  EXPECT_NEAR(costs(180), 0.f, 1e-6);
  EXPECT_NEAR(costs(90), 0.1f, 1e-6);
  EXPECT_NEAR(costs(270), 0.1f, 1e-6);
  for (const size_t angle : {45, 135, 225, 315}) {
    EXPECT_NEAR(costs(angle), 0.5f, 1e-6);
  }
  EXPECT_LT(costs(30), costs(45));
  EXPECT_LT(costs(60), costs(45));
  EXPECT_GT(costs(47), costs(45));  // The lateral baseline shifts the peak slightly.
  EXPECT_LT(costs(47), 0.502f);
  for (size_t angle = 0; angle < 360; ++angle) {
    EXPECT_LT(std::abs(costs(angle + 1) - costs(angle)), 0.02f);
    EXPECT_NEAR(costs(angle), costs(360 - angle), 1e-6);
  }
  state.vx *= 0.5f;
  state.vy *= 0.5f;
  costs.fill(0.f);
  critic.score(data);
  EXPECT_NEAR(costs(45), 0.25f, 1e-6);  // Half the distance, half the cost.
}

TEST(CriticTests, DirectionalWeightsRejectInvalid)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("invalid_directional_test");
  node->declare_parameter("critic.lateral_weight", 0.0);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  for (const double value : {-1.0, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    node->set_parameter(rclcpp::Parameter("critic.lateral_weight", value));
    ParametersHandler params(node);
    DirectionalMotionCritic critic;
    EXPECT_THROW(critic.on_configure(node, "mppi", "critic", costmap, &params), std::invalid_argument);
  }
}

TEST(CriticTests, DirectionalDisabledZeroAndGoalTolerance)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("optional_directional_test");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "dummy_costmap", "", "dummy_costmap");
  ParametersHandler params(node);
  models::State state;
  state.reset(2, 10); state.vx.fill(0.2f); state.vy.fill(0.2f);
  models::Trajectories trajectories;
  models::Path path;
  xt::xtensor<float, 1> costs = xt::zeros<float>({2});
  float dt = 0.1f;
  CriticData data{state, trajectories, path, costs, dt, false, nullptr,
    std::make_shared<OmniMotionModel>(), std::nullopt, std::nullopt};
  DirectionalMotionCritic zero;
  zero.on_configure(node, "mppi", "zero", costmap, &params);
  zero.score(data);
  EXPECT_NEAR(xt::sum(costs)(), 0.0f, 1e-6);
  node->declare_parameter("disabled.enabled", false);
  node->declare_parameter("disabled.lateral_weight", 2.0);
  DirectionalMotionCritic disabled;
  disabled.on_configure(node, "mppi", "disabled", costmap, &params);
  disabled.score(data);
  EXPECT_NEAR(xt::sum(costs)(), 0.0f, 1e-6);
  node->declare_parameter("goal.lateral_weight", 2.0);
  DirectionalMotionCritic goal;
  goal.on_configure(node, "mppi", "goal", costmap, &params);
  TestGoalChecker checker;
  data.goal_checker = &checker;
  path.reset(2); path.x(1) = 0.1f;
  goal.score(data);
  EXPECT_NEAR(xt::sum(costs)(), 0.0f, 1e-6);
  path.x(1) = 1.0f;
  goal.score(data);
  EXPECT_GT(costs(0), 0.0f);
}
