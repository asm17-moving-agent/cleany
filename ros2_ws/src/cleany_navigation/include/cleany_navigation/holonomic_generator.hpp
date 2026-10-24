#pragma once
#include <vector>
#include <optional>
#include <cstdint>
#include <string>
#include "dwb_plugins/standard_traj_generator.hpp"
namespace cleany_navigation {
class HolonomicGenerator : public dwb_plugins::StandardTrajectoryGenerator {
public:
  ~HolonomicGenerator() override;
  void initialize(const nav2_util::LifecycleNode::SharedPtr &, const std::string &) override;
  void startNewIteration(const nav_2d_msgs::msg::Twist2D &) override;
  bool hasMoreTwists() override {return index_<candidates_.size();}
  nav_2d_msgs::msg::Twist2D nextTwist() override {return candidates_.at(index_++);}
  void reset() override;
  // Same-cycle controller input, in one common world frame. Consumed once;
  // never retain a goal across reset, replan, or a failed control cycle.
  void setGoalContext(const geometry_msgs::msg::Pose2D &pose,
    const geometry_msgs::msg::Pose2D &goal);
  void clearGoalContext() {goal_context_.reset();}
  // Extra candidates, never a post-collision velocity filter. One-cycle input.
  void setContinuityContext(const nav_2d_msgs::msg::Twist2D &velocity) {continuity_context_=velocity;}
  void clearContinuityContext() {continuity_context_.reset();}
  dwb_msgs::msg::Trajectory2D generateTrajectory(const geometry_msgs::msg::Pose2D &,
    const nav_2d_msgs::msg::Twist2D &,const nav_2d_msgs::msg::Twist2D &) override;
protected:
  std::vector<double> getTimeSteps(const nav_2d_msgs::msg::Twist2D &) override;
  nav_2d_msgs::msg::Twist2D computeNewVelocity(const nav_2d_msgs::msg::Twist2D &,
    const nav_2d_msgs::msg::Twist2D &,double) override;
private:
  struct TimingRecord {
    int64_t stamp_ns, previous_stamp_ns;
    uint64_t reset_generation;
    bool had_previous;
    double raw_dt_s, command_dt_s;
    nav_2d_msgs::msg::Twist2D measured;
    std::optional<double> first_dt_min, first_dt_max;
    size_t trajectories{0}, nonzero_targets{0}, zero_first_commands{0};
  };
  // Opt-in bounded memory only while controlling. Write once on destruction.
  std::string timing_trace_file_;
  std::vector<TimingRecord> timing_records_;
  uint64_t reset_generation_{0};
  size_t timing_dropped_{0};
  bool timing_current_recorded_{false};
  rclcpp::Clock::SharedPtr clock_;
  std::optional<rclcpp::Time> last_iteration_;
  double command_dt_{0.};
  bool use_elapsed_command_time_{true};
  bool preserve_translation_direction_{true};
  bool prefer_target_translation_direction_{false};
  double max_wheel_speed_{0.},wheel_radius_{.0635},wheel_lever_{(.35+.6038)/2.};
  std::vector<nav_2d_msgs::msg::Twist2D> candidates_;
  size_t index_{0};
  double fine_scale_{.25};
  struct GoalContext {geometry_msgs::msg::Pose2D pose, goal;};
  std::optional<GoalContext> goal_context_;
  double goal_directed_radius_{0.};
  bool continuity_samples_{false};
  std::optional<nav_2d_msgs::msg::Twist2D> continuity_context_;
};
}
