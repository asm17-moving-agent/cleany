// Copyright 2021 ros2_control Development Team
// Copyright 2026 Cleany Team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <atomic>
#include <functional>
#include <memory>
#include <vector>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/waitable.hpp"
#include "rclcpp_action/create_server.hpp"

namespace cleany_trajectory_controller
{

/** Wake the node's executor without publishing from its control update loop. */
class CompletionWaitable final : public rclcpp::Waitable
{
public:
  CompletionWaitable(rclcpp::Context::SharedPtr context, std::function<void()> callback)
  : guard_(context), callback_(std::move(callback)) {}

  void notify() {guard_.trigger();}
  size_t get_number_of_ready_guard_conditions() override {return 1;}
  void add_to_wait_set(rcl_wait_set_t * wait_set) override
  {
    const auto ret = rcl_wait_set_add_guard_condition(
      wait_set, &guard_.get_rcl_guard_condition(), &index_);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Adding completion guard");
    }
  }
  bool is_ready(rcl_wait_set_t * wait_set) override
  {
    return index_ < wait_set->size_of_guard_conditions &&
           wait_set->guard_conditions[index_] != nullptr;
  }
  std::shared_ptr<void> take_data() override {return nullptr;}
  void execute(std::shared_ptr<void> &) override {callback_();}

private:
  rclcpp::GuardCondition guard_;
  std::function<void()> callback_;
  size_t index_{0};
};

/** Preserve upstream motion semantics; dispatch terminal results by executor event.
 * Action callbacks, feedback and result delivery share the default mutually
 * exclusive callback group. In particular, accepted cancellation must finish
 * its ROS state transition before the completion callback sends its result.
 */
class JointTrajectoryController final
  : public joint_trajectory_controller::JointTrajectoryController
{
  using Base = joint_trajectory_controller::JointTrajectoryController;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJTrajAction>;
  using CallbackReturn = controller_interface::CallbackReturn;
  struct PendingGoal
  {
    RealtimeGoalHandlePtr goal;
    bool cancel_requested{false};
  };

public:
  ~JointTrajectoryController() override {remove_completion_waitable();}

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    remove_completion_waitable();
    const auto result = Base::on_configure(state);
    if (result != CallbackReturn::SUCCESS) {
      return result;
    }
    pending_.clear();
    pending_count_.store(0);
    completion_ = std::make_shared<CompletionWaitable>(
      get_node()->get_node_base_interface()->get_context(), [this]() {flush(false);});
    get_node()->get_node_waitables_interface()->add_waitable(completion_, nullptr);
    action_server_.reset();
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
      get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
      std::string(get_node()->get_name()) + "/follow_joint_trajectory",
      [this](const auto & id, const auto & goal) {return goal_received_callback(id, goal);},
      [this](const auto & goal) {return cancel(goal);},
      [this](const auto & goal) {accept(goal);});
    goal_handle_timer_ = get_node()->create_wall_timer(
      action_monitor_period_.to_chrono<std::chrono::nanoseconds>(), [this]() {flush(true);});
    goal_handle_timer_->cancel();
    RCLCPP_INFO(get_node()->get_logger(),
      "Terminal results use executor events; Action feedback rate %.2f Hz", params_.action_monitor_rate);
    return result;
  }

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    const auto result = Base::update(time, period);
    // A pending terminal flag is visible even when RealtimeBuffer's try-lock
    // returns its previous handle. No new timer or joint-state query is used.
    if (pending_count_.load() != 0 && !rt_has_pending_goal_.load()) {
      completion_->notify();
    }
    return result;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    const auto result = Base::on_deactivate(state);
    completion_->notify();
    return result;
  }

private:
  rclcpp_action::CancelResponse cancel(const std::shared_ptr<GoalHandle> & handle)
  {
    const auto active = *rt_active_goal_.readFromNonRT();
    if (!active || active->gh_ != handle) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    const auto result = Base::goal_cancelled_callback(handle);
    for (auto & pending : pending_) {
      if (pending.goal == active) {
        pending.cancel_requested = true;
      }
    }
    completion_->notify();
    return result;
  }

  void accept(const std::shared_ptr<GoalHandle> & handle)
  {
    // Adapted from upstream 2.53.3 goal_accepted_callback: preserve trajectory
    // setup and tolerance handling, retain handles until their result is sent.
    rt_has_pending_goal_ = true;
    const auto previous = *rt_active_goal_.readFromNonRT();
    if (previous) {
      // Server-side replacement is not a client cancellation transition.
      auto result = std::make_shared<FollowJTrajAction::Result>();
      result->error_code = FollowJTrajAction::Result::INVALID_GOAL;
      result->error_string = "Goal superseded by a newer command";
      previous->setAborted(result);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
    auto trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>(
      handle->get_goal()->trajectory);
    add_new_trajectory_msg(trajectory);
    rt_is_holding_ = false;
    auto mutable_handle = handle;
    auto goal = std::make_shared<RealtimeGoalHandle>(mutable_handle);
    goal->preallocated_feedback_->joint_names = params_.joints;
    goal->execute();
    pending_.push_back({goal, false});
    pending_count_.store(pending_.size());
    auto logger = get_node()->get_logger();
    active_tolerances_.writeFromNonRT(joint_trajectory_controller::get_segment_tolerances(
      logger, default_tolerances_, *(handle->get_goal()), params_.joints));
    rt_active_goal_.writeFromNonRT(goal);
    goal_handle_timer_->reset();
    // A new request may have preempted a still-unreported old request.
    completion_->notify();
  }

  void flush(bool feedback)
  {
    const auto active = *rt_active_goal_.readFromNonRT();
    for (auto & pending : pending_) {
      const auto & goal = pending.goal;
      if (!goal->gh_->is_active()) {
        continue;
      }
      if (pending.cancel_requested && goal->gh_->is_canceling()) {
        // An accepted cancel wins if completion was queued during its callback.
        // Upstream's pending-success flag otherwise suppresses setCanceled().
        auto result = std::make_shared<FollowJTrajAction::Result>();
        result->error_string = "Goal canceled by client";
        goal->gh_->canceled(result);
      } else if (goal != active || feedback) {
        goal->runNonRealtime();
      }
    }
    pending_.erase(std::remove_if(pending_.begin(), pending_.end(),
      [](const auto & entry) {return !entry.goal->gh_->is_active();}), pending_.end());
    pending_count_.store(pending_.size());
    if (pending_.empty()) {
      goal_handle_timer_->cancel();
    }
  }

  void remove_completion_waitable()
  {
    if (completion_) {
      get_node()->get_node_waitables_interface()->remove_waitable(completion_, nullptr);
      completion_.reset();
    }
  }

  std::shared_ptr<CompletionWaitable> completion_;
  std::vector<PendingGoal> pending_;
  std::atomic<size_t> pending_count_{0};
  static_assert(std::atomic<size_t>::is_always_lock_free);
};

}  // namespace cleany_trajectory_controller

PLUGINLIB_EXPORT_CLASS(
  cleany_trajectory_controller::JointTrajectoryController, controller_interface::ControllerInterface)
