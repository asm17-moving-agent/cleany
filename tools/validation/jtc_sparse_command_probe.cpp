/** @file Compare dense and sparse commands with the installed JTC sampler. */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "joint_trajectory_controller/trajectory.hpp"

using Message = trajectory_msgs::msg::JointTrajectory;
using Point = trajectory_msgs::msg::JointTrajectoryPoint;
using Trajectory = joint_trajectory_controller::Trajectory;

static std::shared_ptr<Message> read_message(size_t width, size_t count)
{
  auto message = std::make_shared<Message>();
  message->header.stamp.sec = 1;
  for (size_t j = 0; j < width; ++j) message->joint_names.push_back("joint_" + std::to_string(j));
  for (size_t i = 0; i < count; ++i) {
    int64_t ns;
    Point point;
    std::cin >> ns;
    point.time_from_start.sec = ns / 1000000000;
    point.time_from_start.nanosec = ns % 1000000000;
    point.positions.resize(width);
    point.velocities.resize(width);
    for (auto & q : point.positions) std::cin >> q;
    for (auto & v : point.velocities) std::cin >> v;
    message->points.push_back(point);
  }
  if (!std::cin) throw std::runtime_error("Malformed command input");
  return message;
}

int main()
{
  size_t width, dense_count, sparse_count, probes;
  std::cin >> width >> dense_count >> sparse_count >> probes;
  auto dense_msg = read_message(width, dense_count);
  auto sparse_msg = read_message(width, sparse_count);
  Trajectory dense(dense_msg), sparse(sparse_msg);
  const rclcpp::Time before(999999999LL, RCL_ROS_TIME);
  dense.set_point_before_trajectory_msg(before, dense_msg->points.front());
  sparse.set_point_before_trajectory_msg(before, sparse_msg->points.front());
  double max_position = 0., max_velocity = 0.;
  for (size_t i = 0; i < probes; ++i) {
    int64_t ns;
    std::cin >> ns;
    Point a, b;
    joint_trajectory_controller::TrajectoryPointConstIter begin, end;
    const rclcpp::Time at(1000000000LL + ns, RCL_ROS_TIME);
    auto method = joint_trajectory_controller::interpolation_methods::DEFAULT_INTERPOLATION;
    if (!dense.sample(at, method, a, begin, end) || !sparse.sample(at, method, b, begin, end))
      throw std::runtime_error("Installed JTC rejected a trajectory sample");
    if (a.positions.size() != width || b.positions.size() != width ||
        a.velocities.size() != width || b.velocities.size() != width)
      throw std::runtime_error("Installed JTC returned an incomplete state");
    for (size_t j = 0; j < width; ++j) {
      max_position = std::max(max_position, std::abs(a.positions[j] - b.positions[j]));
      max_velocity = std::max(max_velocity, std::abs(a.velocities[j] - b.velocities[j]));
    }
  }
  std::cout << std::setprecision(17) << "{\"samples\":" << probes
            << ",\"max_position_error_rad\":" << max_position
            << ",\"max_velocity_error_rad_s\":" << max_velocity << "}\n";
  return max_position < 1e-8 && max_velocity < 1e-6 ? 0 : 1;
}
