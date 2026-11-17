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

#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "cleany_navigation/mppi/tools/noise_generator.hpp"
#include "cleany_navigation/mppi/tools/parameters_handler.hpp"
#include "cleany_navigation/mppi/models/optimizer_settings.hpp"
#include "cleany_navigation/mppi/models/state.hpp"
#include "cleany_navigation/mppi/models/control_sequence.hpp"

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace cleany_navigation::mppi;  // NOLINT

class TestNoiseGenerator : public NoiseGenerator
{
public:
  ~TestNoiseGenerator() {shutdown();}

  void clearNoise()
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    noises_vx_.fill(0.0f);
    noises_vy_.fill(0.0f);
    noises_wz_.fill(0.0f);
  }

  bool waitForNoise()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::unique_lock<std::mutex> guard(noise_lock_, std::try_to_lock);
        // The worker clears ready_ and writes all samples under this same lock.
        if (guard.owns_lock() && !ready_) {
          return true;
        }
      }
      std::this_thread::yield();
    }
    return false;
  }
};

class NoiseGeneratorTest : public ::testing::Test
{
protected:
  NoiseGeneratorTest()
  : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>("noise_test")),
    handler_(node_)
  {
    settings_.batch_size = 100;
    settings_.time_steps = 25;
    settings_.sampling_std.vx = 0.1;
    settings_.sampling_std.vy = 0.1;
    settings_.sampling_std.wz = 0.1;
  }

  void initialize(bool background)
  {
    node_->declare_parameter("test_name.regenerate_noises", background);
    generator_.initialize(settings_, false, "test_name", &handler_);
  }

  bool resetAndRead(bool holonomic)
  {
    control_.reset(settings_.time_steps);
    for (unsigned int i = 0; i < settings_.time_steps; ++i) {
      control_.vx(i) = i;
      control_.vy(i) = i;
      control_.wz(i) = i;
    }
    state_.reset(settings_.batch_size, settings_.time_steps);
    generator_.reset(settings_, holonomic);
    if (!generator_.waitForNoise()) {
      return false;
    }
    generator_.setNoisedControls(state_, control_);
    return true;
  }

  void expectPassthrough()
  {
    EXPECT_TRUE(xt::all(xt::equal(state_.cvx, control_.vx)));
    EXPECT_TRUE(xt::all(xt::equal(state_.cvy, control_.vy)));
    EXPECT_TRUE(xt::all(xt::equal(state_.cwz, control_.wz)));
  }

  void expectNoise(bool holonomic)
  {
    EXPECT_EQ(state_.cvx.shape()[0], settings_.batch_size);
    EXPECT_EQ(state_.cvx.shape()[1], settings_.time_steps);
    EXPECT_EQ(state_.cvy.shape(), state_.cvx.shape());
    EXPECT_EQ(state_.cwz.shape(), state_.cvx.shape());
    EXPECT_TRUE(xt::all(xt::isfinite(state_.cvx)));
    EXPECT_TRUE(xt::all(xt::isfinite(state_.cvy)));
    EXPECT_TRUE(xt::all(xt::isfinite(state_.cwz)));
    EXPECT_TRUE(xt::any(xt::not_equal(state_.cvx, control_.vx)));
    EXPECT_TRUE(xt::any(xt::not_equal(state_.cwz, control_.wz)));
    if (holonomic) {
      EXPECT_TRUE(xt::any(xt::not_equal(state_.cvy, control_.vy)));
    } else {
      EXPECT_TRUE(xt::all(xt::equal(state_.cvy, control_.vy)));
    }
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  ParametersHandler handler_;
  TestNoiseGenerator generator_;
  models::OptimizerSettings settings_;
  models::ControlSequence control_;
  models::State state_;
};

TEST_F(NoiseGeneratorTest, ZeroNoiseSynchronous)
{
  initialize(false);
  for (bool holonomic : {false, true}) {
    ASSERT_TRUE(resetAndRead(holonomic));
    generator_.clearNoise();
    generator_.setNoisedControls(state_, control_);
    expectPassthrough();
  }
}

TEST_F(NoiseGeneratorTest, ZeroNoiseBackground)
{
  initialize(true);
  for (bool holonomic : {false, true}) {
    ASSERT_TRUE(resetAndRead(holonomic));
    generator_.clearNoise();
    generator_.setNoisedControls(state_, control_);
    expectPassthrough();
  }
}

TEST_F(NoiseGeneratorTest, SeededSynchronousSamples)
{
  initialize(false);
  const auto expect_distribution = [](const auto & samples, const auto & controls, float stddev) {
      const auto residual = samples - controls;
      EXPECT_NEAR(xt::mean(residual)(), 0.0f, 0.1f * stddev);
      EXPECT_NEAR(xt::mean(xt::square(residual))(), stddev * stddev, 0.2f * stddev * stddev);
    };
  for (bool holonomic : {false, true}) {
    xt::random::seed(42);
    ASSERT_TRUE(resetAndRead(holonomic));
    expectNoise(holonomic);
    expect_distribution(state_.cvx, control_.vx, settings_.sampling_std.vx);
    expect_distribution(state_.cwz, control_.wz, settings_.sampling_std.wz);
    if (holonomic) {
      expect_distribution(state_.cvy, control_.vy, settings_.sampling_std.vy);
    }
    const auto first = state_;

    xt::random::seed(42);
    ASSERT_TRUE(resetAndRead(holonomic));
    EXPECT_TRUE(xt::all(xt::equal(state_.cvx, first.cvx)));
    EXPECT_TRUE(xt::all(xt::equal(state_.cvy, first.cvy)));
    EXPECT_TRUE(xt::all(xt::equal(state_.cwz, first.cwz)));

    // Fixed-noise mode reuses its samples between resets.
    generator_.generateNextNoises();
    generator_.setNoisedControls(state_, control_);
    EXPECT_TRUE(xt::all(xt::equal(state_.cvx, first.cvx)));
    EXPECT_TRUE(xt::all(xt::equal(state_.cwz, first.cwz)));
  }
}

TEST_F(NoiseGeneratorTest, BackgroundRequestsComplete)
{
  initialize(true);
  ASSERT_TRUE(resetAndRead(false));
  for (int i = 0; i < 20; ++i) {
    const auto previous = state_.cvx;
    generator_.generateNextNoises();
    ASSERT_TRUE(generator_.waitForNoise());
    generator_.setNoisedControls(state_, control_);
    expectNoise(false);
    EXPECT_TRUE(xt::any(xt::not_equal(state_.cvx, previous)));
  }
}

TEST_F(NoiseGeneratorTest, ResetWhileGenerationIsPending)
{
  initialize(true);
  ASSERT_TRUE(resetAndRead(false));
  for (int i = 0; i < 20; ++i) {
    generator_.generateNextNoises();
    const bool holonomic = i % 2 == 0;
    settings_.batch_size = holonomic ? 64 : 100;
    settings_.time_steps = holonomic ? 12 : 25;
    ASSERT_TRUE(resetAndRead(holonomic));
    expectNoise(holonomic);
  }
}

TEST_F(NoiseGeneratorTest, ShutdownIdleAndPendingWorker)
{
  initialize(true);
  for (int i = 0; i < 20; ++i) {
    if (i != 0) {
      generator_.initialize(settings_, false, "test_name", &handler_);
    }
    ASSERT_TRUE(resetAndRead(false));
    if (i % 2 == 0) {
      generator_.generateNextNoises();
    }
    generator_.shutdown();
    generator_.shutdown();
  }
}

TEST_F(NoiseGeneratorTest, ShutdownBeforeFirstRequest)
{
  initialize(true);
  generator_.shutdown();
  for (int i = 0; i < 20; ++i) {
    generator_.initialize(settings_, false, "test_name", &handler_);
    generator_.shutdown();
  }
}
