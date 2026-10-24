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

#include "cleany_navigation/mppi/critic_manager.hpp"
#include <fstream>
#include <cstdlib>
#include <iomanip>

namespace cleany_navigation::mppi
{

void CriticManager::on_configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros, ParametersHandler * param_handler)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  name_ = name;
  auto node = parent_.lock();
  logger_ = node->get_logger();
  parameters_handler_ = param_handler;

  getParams();
  loadCritics();
}

void CriticManager::getParams()
{
  auto node = parent_.lock();
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(critic_names_, "critics", std::vector<std::string>{}, ParameterType::Static);
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "cleany_navigation", "cleany_navigation::mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      parent_, name_, name_ + "." + name, costmap_ros_,
      parameters_handler_);
    RCLCPP_INFO(logger_, "Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "cleany_navigation::mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  // Optional experiment telemetry; never changes scores or rollout selection.
  static std::ofstream trace;
  static double previous_stamp = -1e9;
  static size_t snapshot = 0;
  const double stamp = data.state.pose.header.stamp.sec +
    data.state.pose.header.stamp.nanosec * 1e-9;
  const char * filename = std::getenv("MPPI_TRACE_FILE");
  const bool record = filename && (stamp < previous_stamp || stamp - previous_stamp >= 2.0);
  const size_t count = data.costs.size();
  std::vector<std::vector<float>> deltas;
  if (record) {deltas.resize(critics_.size(), std::vector<float>(count, 0.0f));}
  for (size_t q = 0; q < critics_.size(); q++) {
    if (data.fail_flag) {break;}
    xt::xtensor<float, 1> before;
    if (record) {before = data.costs;}
    critics_[q]->score(data);
    if (record) {
      for (size_t n = 0; n < count; ++n) {deltas[q][n] = data.costs(n) - before(n);}
    }
  }
  if (record) {
    if (!trace.is_open()) {
      trace.open(filename);
      trace << "snapshot,sim_s,candidate,robot_x,robot_y,robot_yaw,path0_x,path0_y,path0_yaw,furthest,failed";
      for (const auto & name : critic_names_) {trace << "," << name;}
      trace << ",critic_total";
      for (int k = 0; k < 5; ++k) {trace << ",x" << k << ",y" << k << ",yaw" << k;}
      trace << "\n";
      trace << std::setprecision(9);
    }
    const size_t last = data.trajectories.x.shape(1) - 1;
    for (size_t n = 0; n < count; ++n) {
      trace << snapshot << ',' << stamp << ',' << n << ',' << data.state.pose.pose.position.x
        << ',' << data.state.pose.pose.position.y << ',' << tf2::getYaw(data.state.pose.pose.orientation)
        << ',' << data.path.x(0) << ',' << data.path.y(0) << ',' << data.path.yaws(0)
        << ',' << data.furthest_reached_path_point.value_or(0) << ',' << data.fail_flag;
      for (const auto & cost : deltas) {trace << ',' << cost[n];}
      trace << ',' << data.costs(n);
      for (size_t k = 0; k < 5; ++k) {
        const size_t t = k * last / 4;
        trace << ',' << data.trajectories.x(n,t) << ',' << data.trajectories.y(n,t)
          << ',' << data.trajectories.yaws(n,t);
      }
      trace << "\n";
    }
    trace.flush(); previous_stamp = stamp; ++snapshot;
  }
}

}  // namespace cleany_navigation::mppi
