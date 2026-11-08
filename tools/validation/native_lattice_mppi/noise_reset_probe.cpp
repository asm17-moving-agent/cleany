// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <iostream>
#include <algorithm>
#include "cleany_navigation/mppi/tools/noise_generator.hpp"
#include "cleany_navigation/mppi/tools/parameters_handler.hpp"
#include <xtensor/xmath.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("noise_reset_probe");
  node->declare_parameter("probe.regenerate_noises", true);
  cleany_navigation::mppi::ParametersHandler params(node);
  cleany_navigation::mppi::models::OptimizerSettings settings;
  settings.batch_size = 1000; settings.time_steps = 60;
  settings.sampling_std.vx = .1; settings.sampling_std.vy = .1; settings.sampling_std.wz = .2;
  cleany_navigation::mppi::NoiseGenerator generator;
  cleany_navigation::mppi::models::State state;
  state.reset(1000, 60);
  cleany_navigation::mppi::models::ControlSequence control;
  control.reset(60);
  generator.initialize(settings, true, "probe", &params);
  int empty = 0;
  std::vector<double> elapsed;
  for (int i = 0; i < 100; ++i) {
    const auto begin = std::chrono::steady_clock::now();
    generator.reset(settings, true);
    generator.setNoisedControls(state, control);
    elapsed.push_back(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now()-begin).count());
    empty += xt::sum(xt::abs(state.cvx)+xt::abs(state.cvy)+xt::abs(state.cwz))() == 0;
  }
  generator.shutdown();
  std::sort(elapsed.begin(), elapsed.end());
  std::cout << "{\"resets\":100,\"empty_batches\":" << empty
            << ",\"reset_and_copy_p50_ms\":" << elapsed[50]
            << ",\"reset_and_copy_p99_ms\":" << elapsed[98] << "}\n";
  rclcpp::shutdown();
}
