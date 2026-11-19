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

#include "cleany_navigation/mppi/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace cleany_navigation::mppi
{

void NoiseGenerator::initialize(
  cleany_navigation::mppi::models::OptimizerSettings & settings, bool is_holonomic,
  const std::string & name, ParametersHandler * param_handler)
{
  active_ = true;

  auto getParam = param_handler->getParamGetter(name);
  getParam(regenerate_noises_, "regenerate_noises", false);

  // The first optimization must see a complete batch, even under CPU load.
  reset(settings, is_holonomic);
  if (regenerate_noises_) {
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  }
}

void NoiseGenerator::shutdown()
{
  {
    std::lock_guard<std::mutex> guard(noise_lock_);
    active_ = false;
  }
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  // Trigger the thread to run in parallel to this iteration
  // to generate the next iteration's noises (if applicable).
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = active_ && regenerate_noises_;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  std::unique_lock<std::mutex> guard(noise_lock_);

  xt::noalias(state.cvx) = control_sequence.vx + noises_vx_;
  xt::noalias(state.cvy) = control_sequence.vy + noises_vy_;
  xt::noalias(state.cwz) = control_sequence.wz + noises_wz_;
}

void NoiseGenerator::reset(cleany_navigation::mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  // Settings, model selection and buffers form one atomic generation.
  std::lock_guard<std::mutex> guard(noise_lock_);
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  ready_ = false;
  generateNoisedControls();
}

void NoiseGenerator::noiseThread()
{
  std::unique_lock<std::mutex> guard(noise_lock_);
  while (true) {
    noise_cond_.wait(guard, [this]() {return !active_ || ready_;});
    if (!active_) {return;}
    ready_ = false;
    generateNoisedControls();
  }
}

void NoiseGenerator::generateNoisedControls()
{
  auto & s = settings_;

  xt::noalias(noises_vx_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx, random_engine_);
  xt::noalias(noises_wz_) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz, random_engine_);
  if (is_holonomic_) {
    xt::noalias(noises_vy_) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy, random_engine_);
  } else {
    xt::noalias(noises_vy_) = xt::zeros<float>({s.batch_size, s.time_steps});
  }
}

}  // namespace cleany_navigation::mppi
