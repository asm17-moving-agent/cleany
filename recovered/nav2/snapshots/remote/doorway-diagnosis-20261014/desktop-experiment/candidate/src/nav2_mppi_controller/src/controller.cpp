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

#include <stdint.h>
#include "nav2_core/exceptions.hpp"
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace nav2_mppi_controller
{

void MPPIController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  costmap_ros_ = costmap_ros;
  tf_buffer_ = tf;
  name_ = name;
  parameters_handler_ = std::make_unique<ParametersHandler>(parent);

  auto node = parent_.lock();
  clock_ = node->get_clock();
  last_time_called_ = clock_->now();
  // Get high-level controller parameters
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(visualize_, "visualize", false);
  getParam(reset_period_, "reset_period", 1.0);
  getParam(path_progress_timeout_, "path_progress_timeout", 0.0);
  getParam(path_progress_distance_, "path_progress_distance", 0.08);

  // Configure composed objects
  optimizer_.initialize(parent_, name_, costmap_ros_, parameters_handler_.get());
  path_handler_.initialize(parent_, name_, costmap_ros_, tf_buffer_, parameters_handler_.get());
  trajectory_visualizer_.on_configure(
    parent_, name_,
    costmap_ros_->getGlobalFrameID(), parameters_handler_.get());

  RCLCPP_INFO(logger_, "Configured MPPI Controller: %s", name_.c_str());
}

void MPPIController::cleanup()
{
  optimizer_.shutdown();
  trajectory_visualizer_.on_cleanup();
  parameters_handler_.reset();
  RCLCPP_INFO(logger_, "Cleaned up MPPI Controller: %s", name_.c_str());
}

void MPPIController::activate()
{
  trajectory_visualizer_.on_activate();
  parameters_handler_->start();
  RCLCPP_INFO(logger_, "Activated MPPI Controller: %s", name_.c_str());
}

void MPPIController::deactivate()
{
  trajectory_visualizer_.on_deactivate();
  RCLCPP_INFO(logger_, "Deactivated MPPI Controller: %s", name_.c_str());
}

void MPPIController::reset()
{
  optimizer_.reset();
}

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker * goal_checker)
{
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  if (clock_->now() - last_time_called_ > rclcpp::Duration::from_seconds(reset_period_)) {
    reset();
    progress_gate_.reset();
  }
  last_time_called_ = clock_->now();

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  nav_msgs::msg::Path transformed_plan = path_handler_.transformPath(robot_pose);
  if (path_progress_timeout_ > 0.0 && !progress_path_.poses.empty()) {
    geometry_msgs::msg::PoseStamped pose;
    if (robot_pose.header.frame_id == progress_path_.header.frame_id) {pose=robot_pose;}
    else {tf_buffer_->transform(robot_pose,pose,progress_path_.header.frame_id,tf2::durationFromSec(0.1));}
    std::vector<mppi::ProgressPoint> points;
    for (const auto & p : progress_path_.poses) {points.push_back({p.pose.position.x,p.pose.position.y});}
    const auto & goal=progress_path_.poses.back().pose;
    geometry_msgs::msg::Pose tolerance;geometry_msgs::msg::Twist velocity_tolerance;
    const bool valid=goal_checker && goal_checker->getTolerances(tolerance,velocity_tolerance);
    const bool terminal=valid && std::hypot(pose.pose.position.x-goal.position.x,
      pose.pose.position.y-goal.position.y)<=tolerance.position.x;
    const double yaw_error=std::abs(angles::shortest_angular_distance(
      tf2::getYaw(pose.pose.orientation),tf2::getYaw(goal.orientation)));
    const double progress=mppi::projectProgress(points,pose.pose.position.x,pose.pose.position.y);
    if (!progress_gate_.observe(clock_->now().seconds(),progress,yaw_error,terminal,
      path_progress_timeout_,path_progress_distance_)) {
      RCLCPP_WARN(logger_, "Path progress timeout: %.2fs without %.3fm route advance",path_progress_timeout_,path_progress_distance_);
      throw nav2_core::PlannerException("Path progress timeout");
    }
  }


  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  geometry_msgs::msg::TwistStamped cmd =
    optimizer_.evalControl(robot_pose, robot_speed, transformed_plan, goal_checker);

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(logger_, "Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    visualize(std::move(transformed_plan));
  }

  return cmd;
}

void MPPIController::visualize(nav_msgs::msg::Path transformed_plan)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory");
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIController::setPlan(const nav_msgs::msg::Path & path)
{
  bool new_goal=progress_path_.poses.empty() || path.poses.empty();
  if (!new_goal) {
    const auto & a=progress_path_.poses.back().pose;const auto & b=path.poses.back().pose;
    new_goal=std::hypot(a.position.x-b.position.x,a.position.y-b.position.y)>0.01 ||
      std::abs(angles::shortest_angular_distance(tf2::getYaw(a.orientation),tf2::getYaw(b.orientation)))>0.01;
  }
  if (new_goal) {progress_gate_.reset();} else {progress_gate_.replan();}
  progress_path_=path;
  path_handler_.setPath(path);
}

void MPPIController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  optimizer_.setSpeedLimit(speed_limit, percentage);
}

}  // namespace nav2_mppi_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mppi_controller::MPPIController, nav2_core::Controller)
