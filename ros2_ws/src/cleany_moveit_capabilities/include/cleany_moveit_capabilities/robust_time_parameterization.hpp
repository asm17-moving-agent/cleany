#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>

namespace cleany
{
struct RetimingOptions
{
  double path_tolerance = 0.0001;
  double resample_dt = 0.1;
  double min_angle_change = 0.;
  double collinear_tolerance = 1e-12;
  double default_velocity_limit = 1.;
  double default_acceleration_limit = 1.;
};

enum class RetimingResult { FAILED, TOTG };

/** Time a joint path with the numerically corrected TOTG solver.
 * Positions use model joint units and times are seconds. Collinear cleanup
 * checks every removed point. Geometric refinement bounds the returned quintic
 * against the cleaned path and position limits. Uniform time dilation bounds
 * speed and acceleration, including nanosecond timestamp quantization.
 * Failed attempts leave the input unchanged; there is no alternate solver.
 */
RetimingResult retime_path(robot_trajectory::RobotTrajectory& trajectory,
                          double velocity_scaling, double acceleration_scaling,
                          const RetimingOptions& options);

/** Reject unusable timing instead of returning a successful untimed path. */
class RobustTimeParameterization : public planning_request_adapter::PlanningRequestAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override;
  std::string getDescription() const override;
  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& scene,
                    const planning_interface::MotionPlanRequest& request,
                    planning_interface::MotionPlanResponse& response,
                    std::vector<std::size_t>& added_path_index) const override;
private:
  RetimingOptions options_;
};
}  // namespace cleany
