// Copyright 2020 ROS2-Control Development Team
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

#include <errno.h>
#include <atomic>
#include "cleany_mujoco_observer/control_cycle.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <hardware_interface/version.h>
#include <controller_manager/controller_manager.hpp>
#include <rclcpp/executors.hpp>
#include <realtime_tools/realtime_helpers.hpp>

using namespace std::chrono_literals;

namespace
{
// Reference: https://man7.org/linux/man-pages/man2/sched_setparam.2.html
// This value is used when configuring the main loop to use SCHED_FIFO scheduling
// We use a midpoint RT priority to allow maximum flexibility to users
int const kSchedPriority = 50;

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  rclcpp::NodeOptions cm_node_options = controller_manager::get_cm_node_options();
  std::vector<std::string> node_arguments = cm_node_options.arguments();
  for (int i = 1; i < argc; ++i)
  {
    if (node_arguments.empty() && std::string(argv[i]) != "--ros-args")
    {
      // A simple way to reject non ros args
      continue;
    }
    node_arguments.push_back(argv[i]);
  }
  cm_node_options.arguments(node_arguments);

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    executor, manager_node_name, "", cm_node_options);

  const bool use_sim_time = cm->get_parameter_or("use_sim_time", false);

  const bool has_realtime = realtime_tools::has_realtime_kernel();
  const bool lock_memory = cm->get_parameter_or<bool>("lock_memory", has_realtime);
  if (lock_memory)
  {
    const auto lock_result = realtime_tools::lock_memory();
    if (!lock_result.first)
    {
      RCLCPP_WARN(cm->get_logger(), "Unable to lock the memory: '%s'", lock_result.second.c_str());
    }
  }

  RCLCPP_INFO(cm->get_logger(), "update rate is %d Hz", cm->get_update_rate());
  const int thread_priority = cm->get_parameter_or<int>("thread_priority", kSchedPriority);
  RCLCPP_INFO(
    cm->get_logger(), "Spawning %s RT thread with scheduler priority: %d", cm->get_name(),
    thread_priority);

  std::atomic<bool> control_failed{false};
  std::thread cm_thread(
    [cm, thread_priority, use_sim_time, &control_failed]()
    {
      rclcpp::Parameter cpu_affinity_param;
      if (cm->get_parameter("cpu_affinity", cpu_affinity_param))
      {
        std::vector<int> cpus = {};
        if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          cpus = {static_cast<int>(cpu_affinity_param.as_int())};
        }
        else if (cpu_affinity_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
          const auto cpu_affinity_param_array = cpu_affinity_param.as_integer_array();
          std::for_each(
            cpu_affinity_param_array.begin(), cpu_affinity_param_array.end(),
            [&cpus](int cpu) { cpus.push_back(static_cast<int>(cpu)); });
        }
        const auto affinity_result = realtime_tools::set_current_thread_affinity(cpus);
        if (!affinity_result.first)
        {
          RCLCPP_WARN(
            cm->get_logger(), "Unable to set the CPU affinity : '%s'",
            affinity_result.second.c_str());
        }
      }

      if (!realtime_tools::configure_sched_fifo(thread_priority))
      {
        RCLCPP_WARN(
          cm->get_logger(),
          "Could not enable FIFO RT scheduling policy: with error number <%i>(%s). See "
          "[https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html] "
          "for details on how to enable realtime scheduling.",
          errno, strerror(errno));
      }
      else
      {
        RCLCPP_INFO(
          cm->get_logger(), "Successful set up FIFO RT scheduling policy with priority %i.",
          thread_priority);
      }

      // Based on mujoco_ros2_control 0.0.3. Keep its hardware/physics backend,
      // but bind all three control operations to one captured ROS timestamp.
      // The minimum-period gate prevents repeated feedback during overruns.
      try
      {
        if (!use_sim_time) {
          throw std::runtime_error("Cleany MuJoCo control requires use_sim_time=true");
        }
        auto clock = cm->get_clock();
        if (!clock->wait_until_started()) {return;}
        const auto rate = cm->get_update_rate();
        if (rate == 0) {throw std::runtime_error("Control update_rate must be positive");}
        cleany_mujoco_observer::ControlCycleGate gate(1'000'000'000 / rate);
        struct Manager
        {
          controller_manager::ControllerManager& cm;
          void read(int64_t stamp, int64_t period) {
            cm.read(rclcpp::Time(stamp, RCL_ROS_TIME), rclcpp::Duration(std::chrono::nanoseconds(period)));
          }
          void update(int64_t stamp, int64_t period) {
            cm.update(rclcpp::Time(stamp, RCL_ROS_TIME), rclcpp::Duration(std::chrono::nanoseconds(period)));
          }
          void write(int64_t stamp, int64_t period) {
            cm.write(rclcpp::Time(stamp, RCL_ROS_TIME), rclcpp::Duration(std::chrono::nanoseconds(period)));
          }
        } manager{*cm};
        while (rclcpp::ok())
        {
          const auto cycle = gate.take(clock->now().nanoseconds());
          if (cycle) {cycle->execute(manager);}
          // An expired deadline is harmless: the gate still requires a new
          // full period. A paused clock does not publish pretend fresh states.
          if (!clock->sleep_until(rclcpp::Time(gate.next_stamp_ns(), RCL_ROS_TIME))) {break;}
        }
      }
      catch (const std::exception& error)
      {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(cm->get_logger(), "MuJoCo control clock failure: %s", error.what());
          control_failed.store(true);
          rclcpp::shutdown();
        }
      }
    });

  executor->add_node(cm);
  executor->spin();
  cm_thread.join();
  rclcpp::shutdown();
  return control_failed.load() ? 1 : 0;
}
