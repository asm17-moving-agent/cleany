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
#include <cmath>
#include <thread>
#include <xtensor/xmath.hpp>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "cleany_navigation/mppi/tools/noise_generator.hpp"
#include "cleany_navigation/mppi/tools/parameters_handler.hpp"
#include "cleany_navigation/mppi/models/optimizer_settings.hpp"
#include "cleany_navigation/mppi/models/state.hpp"
#include "cleany_navigation/mppi/models/control_sequence.hpp"

// Tests noise generator object

class RosLockGuard
{
public:
  RosLockGuard() {rclcpp::init(0, nullptr);}
  ~RosLockGuard() {rclcpp::shutdown();}
};
RosLockGuard g_rclcpp;

using namespace cleany_navigation::mppi;  // NOLINT

TEST(NoiseGeneratorTest, NoiseGeneratorLifecycle)
{
  // Tests shuts down internal thread cleanly
  NoiseGenerator generator;
  cleany_navigation::mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(false));
  ParametersHandler handler(node);

  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);
  generator.shutdown();
}


TEST(NoiseGeneratorTest, ResetHasACompleteBatchWithoutWaiting)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node");
  node->declare_parameter("test.regenerate_noises", true);
  ParametersHandler handler(node);
  NoiseGenerator generator;
  models::OptimizerSettings settings;
  settings.batch_size = 100; settings.time_steps = 25;
  settings.sampling_std.vx = settings.sampling_std.vy = settings.sampling_std.wz = .1;
  models::ControlSequence control;
  control.reset(25);
  models::State state;
  state.reset(100, 25);
  generator.initialize(settings, true, "test", &handler);
  for (int i = 0; i < 40; ++i) {
    const bool omni = i % 2 == 0;
    generator.generateNextNoises();
    generator.reset(settings, omni);
    generator.setNoisedControls(state, control);
    const float x_rms = std::sqrt(xt::mean(xt::square(state.cvx))());
    EXPECT_GT(x_rms, .07f); EXPECT_LT(x_rms, .13f);
    EXPECT_GT(xt::sum(xt::abs(state.cwz))(), 1.f);
    if (omni) {EXPECT_GT(xt::sum(xt::abs(state.cvy))(), 1.f);}
    else {EXPECT_EQ(xt::sum(xt::abs(state.cvy))(), 0.f);}
  }
  auto previous = state.cvx;
  generator.generateNextNoises();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  bool changed = false;
  while (std::chrono::steady_clock::now() < deadline && !changed) {
    generator.setNoisedControls(state, control);
    changed = xt::sum(xt::abs(state.cvx-previous))() > 1.f;
    std::this_thread::yield();
  }
  EXPECT_TRUE(changed);
  generator.shutdown();
}

TEST(NoiseGeneratorTest, SeparateControllersDoNotConsumeEachOthersRandomStream)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("independent_noise");
  ParametersHandler handler(node);
  models::OptimizerSettings settings;
  settings.batch_size = 100; settings.time_steps = 25;
  settings.sampling_std.vx = settings.sampling_std.vy = settings.sampling_std.wz = .1;
  models::ControlSequence control; control.reset(25);
  models::State a, b; a.reset(100,25); b.reset(100,25);
  NoiseGenerator first, second;
  first.initialize(settings,true,"first",&handler);
  second.initialize(settings,true,"second",&handler);
  first.setNoisedControls(a,control); second.setNoisedControls(b,control);
  EXPECT_EQ(xt::sum(xt::abs(a.cvx-b.cvx))(),0.f);
  first.reset(settings,true);
  second.setNoisedControls(b,control);
  first.setNoisedControls(a,control);
  EXPECT_GT(xt::sum(xt::abs(a.cvx-b.cvx))(),1.f);
  first.shutdown(); second.shutdown();
}

class ControlledNoiseGenerator : public NoiseGenerator
{
public:
  ~ControlledNoiseGenerator() {shutdown();}

  bool waitForGeneration()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> guard(noise_lock_);
        if (!ready_) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  void prepareZeroNoises(unsigned int batch_size, unsigned int time_steps)
  {
    noises_vx_ = xt::zeros<float>({batch_size, time_steps});
    noises_vy_ = xt::zeros<float>({batch_size, time_steps});
    noises_wz_ = xt::zeros<float>({batch_size, time_steps});
  }
};

TEST(NoiseGeneratorTest, NoiseGeneratorMain)
{
  // Tests shuts down internal thread cleanly
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node");
  node->declare_parameter("test_name.regenerate_noises", rclcpp::ParameterValue(true));
  ParametersHandler handler(node);
  ControlledNoiseGenerator generator;
  cleany_navigation::mppi::models::OptimizerSettings settings;
  settings.batch_size = 100;
  settings.time_steps = 25;
  settings.sampling_std.vx = 0.1;
  settings.sampling_std.vy = 0.1;
  settings.sampling_std.wz = 0.1;

  // Populate a potential control sequence
  cleany_navigation::mppi::models::ControlSequence control_sequence;
  control_sequence.reset(25);
  for (unsigned int i = 0; i != control_sequence.vx.shape(0); i++) {
    control_sequence.vx(i) = i;
    control_sequence.vy(i) = i;
    control_sequence.wz(i) = i;
  }

  cleany_navigation::mppi::models::State state;
  state.reset(settings.batch_size, settings.time_steps);

  // 생성 전의 0 잡음은 제어 시퀀스를 그대로 전달해야 합니다.
  generator.prepareZeroNoises(settings.batch_size, settings.time_steps);
  generator.setNoisedControls(state, control_sequence);
  EXPECT_EQ(state.cvx(0), 0);
  EXPECT_EQ(state.cvy(0), 0);
  EXPECT_EQ(state.cwz(0), 0);
  EXPECT_EQ(state.cvx(9), 9);
  EXPECT_EQ(state.cvy(9), 9);
  EXPECT_EQ(state.cwz(9), 9);

  // 고정 난수열로 수치 검사를 재현하고, 실제 작업 스레드의 생성을 검증합니다.
  // Each generator owns a deterministic random stream.
  generator.initialize(settings, false, "test_name", &handler);
  generator.reset(settings, false);
  ASSERT_TRUE(generator.waitForGeneration());
  generator.generateNextNoises();
  ASSERT_TRUE(generator.waitForGeneration());
  generator.setNoisedControls(state, control_sequence);
  EXPECT_NE(state.cvx(0), 0);
  EXPECT_EQ(state.cvy(0), 0);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(0), 0);
  EXPECT_NE(state.cvx(9), 9);
  EXPECT_EQ(state.cvy(9), 9);  // Not populated in non-holonomic
  EXPECT_NE(state.cwz(9), 9);

  EXPECT_NEAR(state.cvx(0), 0, 0.3);
  EXPECT_NEAR(state.cvy(0), 0, 0.3);
  EXPECT_NEAR(state.cwz(0), 0, 0.3);
  EXPECT_NEAR(state.cvx(9), 9, 0.3);
  EXPECT_NEAR(state.cvy(9), 9, 0.3);
  EXPECT_NEAR(state.cwz(9), 9, 0.3);

  // Test holonomic setting
  generator.reset(settings, true);  // Now holonomically
  ASSERT_TRUE(generator.waitForGeneration());
  generator.generateNextNoises();
  ASSERT_TRUE(generator.waitForGeneration());
  generator.setNoisedControls(state, control_sequence);
  EXPECT_NE(state.cvx(0), 0);
  EXPECT_NE(state.cvy(0), 0);  // Now populated in non-holonomic
  EXPECT_NE(state.cwz(0), 0);
  EXPECT_NE(state.cvx(9), 9);
  EXPECT_NE(state.cvy(9), 9);  // Now populated in non-holonomic
  EXPECT_NE(state.cwz(9), 9);

  EXPECT_NEAR(state.cvx(0), 0, 0.3);
  EXPECT_NEAR(state.cvy(0), 0, 0.3);
  EXPECT_NEAR(state.cwz(0), 0, 0.3);
  EXPECT_NEAR(state.cvx(9), 9, 0.3);
  EXPECT_NEAR(state.cvy(9), 9, 0.3);
  EXPECT_NEAR(state.cwz(9), 9, 0.3);

  generator.shutdown();
}
