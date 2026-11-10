#include <cleany_moveit_capabilities/robust_time_parameterization.hpp>
#include "totg/time_optimal_trajectory_generation.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>

namespace cleany
{
namespace
{
const auto LOGGER = rclcpp::get_logger("cleany.robust_time_parameterization");
using Limits = std::unordered_map<std::string, double>;

bool positive(double value) { return std::isfinite(value) && value > 0.; }

double scaling(double value) { return value == 0. ? 1. : value; }

bool limits(const robot_trajectory::RobotTrajectory& trajectory, double velocity_scaling,
            double acceleration_scaling, const RetimingOptions& options, Limits& velocity, Limits& acceleration)
{
  if (!trajectory.getGroup() || !trajectory.getGroup()->getVariableCount() || trajectory.empty() || !positive(velocity_scaling) || velocity_scaling > 1. ||
      !positive(acceleration_scaling) || acceleration_scaling > 1. ||
      !positive(options.default_velocity_limit) || !positive(options.default_acceleration_limit)) return false;
  for (const auto& name : trajectory.getGroup()->getVariableNames())
  {
    const auto& bounds = trajectory.getRobotModel()->getVariableBounds(name);
    velocity[name] = (bounds.velocity_bounded_ ? std::min(std::abs(bounds.min_velocity_), bounds.max_velocity_) :
                      options.default_velocity_limit) * velocity_scaling;
    acceleration[name] = (bounds.acceleration_bounded_ ? std::min(std::abs(bounds.min_acceleration_), bounds.max_acceleration_) :
                          options.default_acceleration_limit) * acceleration_scaling;
    if (!positive(velocity[name]) || !positive(acceleration[name])) return false;
  }
  return true;
}

bool valid(const robot_trajectory::RobotTrajectory& timed, const robot_trajectory::RobotTrajectory& original,
           const Limits& velocity, const Limits& acceleration)
{
  if (timed.empty() || !std::isfinite(timed.getDuration())) return false;
  bool moving = false;
  for (std::size_t i = 1; i < original.getWayPointCount(); ++i)
    for (const auto& name : original.getGroup()->getVariableNames())
      moving |= original.getWayPoint(i).getVariablePosition(name) != original.getFirstWayPoint().getVariablePosition(name);
  bool timed_moving = false;
  for (std::size_t i = 1; i < timed.getWayPointCount(); ++i)
    for (const auto& name : timed.getGroup()->getVariableNames())
      timed_moving |= timed.getWayPoint(i).getVariablePosition(name) != timed.getFirstWayPoint().getVariablePosition(name);
  if (moving && (!positive(timed.getDuration()) || !timed_moving)) return false;
  for (const auto& name : timed.getGroup()->getVariableNames())
    if (std::abs(timed.getFirstWayPoint().getVariablePosition(name)-original.getFirstWayPoint().getVariablePosition(name)) > 1e-10 ||
        std::abs(timed.getLastWayPoint().getVariablePosition(name)-original.getLastWayPoint().getVariablePosition(name)) > 1e-10)
      return false;
  for (std::size_t i = 0; i < timed.getWayPointCount(); ++i)
  {
    const auto& point = timed.getWayPoint(i);
    const double dt = timed.getWayPointDurationFromPrevious(i);
    if (!std::isfinite(dt) || dt < 0. || (i && dt < 1e-8) || !point.hasVelocities() || !point.hasAccelerations()) return false;
    for (const auto& name : timed.getGroup()->getVariableNames())
      if (!std::isfinite(point.getVariablePosition(name)) || !std::isfinite(point.getVariableVelocity(name)) ||
          !std::isfinite(point.getVariableAcceleration(name)) ||
          std::abs(point.getVariableVelocity(name)) > velocity.at(name)*(1.+1e-6) ||
          std::abs(point.getVariableAcceleration(name)) > acceleration.at(name)*(1.+1e-6)) return false;
  }
  return true;
}

/** Enclose derivative Bernstein controls for every ROS nanosecond rounding.
 * Uniform time scaling preserves q(u). A rounded interval corresponds to a
 * smaller uncertainty interval in the original clock, so one dilation suffices.
 */
bool boundQuinticTiming(robot_trajectory::RobotTrajectory& timed, const Limits& velocity, const Limits& acceleration)
{
  long double scale = 1.;
  const auto& names = timed.getGroup()->getVariableNames();
  for (std::size_t i = 1; i < timed.getWayPointCount(); ++i)
  {
    const long double dt = timed.getWayPointDurationFromPrevious(i);
    const auto& first = timed.getWayPoint(i-1);
    const auto& last = timed.getWayPoint(i);
    if (!positive(dt) || !first.hasVelocities() || !first.hasAccelerations() ||
        !last.hasVelocities() || !last.hasAccelerations()) return false;
    const long double uncertainty = std::min(dt/8., 2.e-9L);
    scale = std::max(scale, 16.e-9L/dt);
    const long double low=dt-uncertainty, high=dt+uncertainty;
    for (const auto& name : names)
    {
      const long double dq=static_cast<long double>(last.getVariablePosition(name))-first.getVariablePosition(name);
      const long double v0=first.getVariableVelocity(name), v1=last.getVariableVelocity(name);
      const long double a0=first.getVariableAcceleration(name), a1=last.getVariableAcceleration(name);
      auto bound = [&](long double a, long double b, long double c, bool second) {
        auto value = [&](long double h) { return second ? a/(h*h)+b/h+c : a/h+b+c*h; };
        long double maximum=std::max(std::abs(value(low)),std::abs(value(high)));
        const long double extremum=second ? (b!=0. ? -2.*a/b : -1.) :
            (c!=0. && a/c>0. ? std::sqrt(a/c) : -1.);
        if (extremum>low && extremum<high) maximum=std::max(maximum,std::abs(value(extremum)));
        const long double ratio=maximum/(second ? acceleration.at(name) : velocity.at(name));
        scale=std::max(scale,second ? std::sqrt(ratio) : ratio);
        return std::isfinite(ratio);
      };
      if (!bound(0.,v0,0.,false) || !bound(0.,v0,a0/4.,false) ||
          !bound(5.*dq,-2.*(v0+v1),(a1-a0)/4.,false) ||
          !bound(0.,v1,-a1/4.,false) || !bound(0.,v1,0.,false) ||
          !bound(0.,0.,a0,true) || !bound(20.*dq,-12.*v0-8.*v1,a1-2.*a0,true) ||
          !bound(-20.*dq,8.*v0+12.*v1,a0-2.*a1,true) || !bound(0.,0.,a1,true)) return false;
    }
  }
  if (scale > 1.)
  {
    scale *= 1.+1e-9L;
    for (std::size_t i = 0; i < timed.getWayPointCount(); ++i)
    {
      timed.setWayPointDurationFromPrevious(i, timed.getWayPointDurationFromPrevious(i)*scale);
      auto point = timed.getWayPointPtr(i);
      for (const auto& name : names)
      {
        point->setVariableVelocity(name, point->getVariableVelocity(name)/scale);
        point->setVariableAcceleration(name, point->getVariableAcceleration(name)/(scale*scale));
      }
    }
  }
  return true;
}

bool between(const Eigen::VectorXd& a, const Eigen::VectorXd& point, const Eigen::VectorXd& b, double tolerance)
{
  const Eigen::VectorXd delta = b-a;
  const double norm = delta.squaredNorm();
  if (norm == 0.) return (point-a).cwiseAbs().maxCoeff() == 0.;
  const double u = (point-a).dot(delta)/norm;
  return u >= 0. && u <= 1. && (point-a-u*delta).cwiseAbs().maxCoeff() <= tolerance;
}

std::vector<std::size_t> corners(const std::vector<Eigen::VectorXd>& points, double tolerance)
{
  std::vector<std::size_t> kept;
  for (std::size_t i = 0; i < points.size(); ++i)
  {
    if (!kept.empty() && (points[i]-points[kept.back()]).cwiseAbs().maxCoeff() == 0.) continue;
    while (kept.size() >= 2 && between(points[kept[kept.size()-2]], points[kept.back()], points[i], tolerance))
      kept.pop_back();
    kept.push_back(i);
  }
  // Bound error against every original point, so successive removals cannot
  // accumulate a larger shortcut around a shallow bend.
  for (std::size_t i = 1; i < kept.size(); ++i)
    for (std::size_t j = kept[i-1]+1; j < kept[i]; ++j)
      if (!between(points[kept[i-1]], points[j], points[kept[i]], tolerance))
      {
        kept.resize(points.size()); std::iota(kept.begin(), kept.end(), 0); return kept;
      }
  return kept;
}
}  // namespace

RetimingResult retime_path(robot_trajectory::RobotTrajectory& trajectory, double velocity_scaling,
                          double acceleration_scaling, const RetimingOptions& options)
{
  Limits velocity, acceleration;
  if (!std::isfinite(options.path_tolerance) || options.path_tolerance < 0. || !positive(options.resample_dt) ||
      !std::isfinite(options.min_angle_change) || options.min_angle_change < 0. ||
      !std::isfinite(options.collinear_tolerance) || options.collinear_tolerance < 0. ||
      !limits(trajectory, scaling(velocity_scaling), scaling(acceleration_scaling), options, velocity, acceleration))
    return RetimingResult::FAILED;
  const auto& names = trajectory.getGroup()->getVariableNames();
  std::vector<Eigen::VectorXd> points;
  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i)
  {
    Eigen::VectorXd point(names.size());
    for (std::size_t j = 0; j < names.size(); ++j) point[j] = trajectory.getWayPoint(i).getVariablePosition(names[j]);
    if (!point.allFinite() || !trajectory.getWayPoint(i).satisfiesBounds(trajectory.getGroup()))
      return RetimingResult::FAILED;
    points.push_back(point);
  }
  robot_trajectory::RobotTrajectory timed(trajectory, true);
  timed.clear();
  for (const auto index : corners(points, options.collinear_tolerance))
    timed.addSuffixWayPoint(trajectory.getWayPoint(index), 0.);
  totg::TimeOptimalTrajectoryGeneration solver(options.path_tolerance, options.resample_dt,
                                               options.min_angle_change);
  if (!solver.computeTimeStamps(timed, velocity, acceleration) ||
      !boundQuinticTiming(timed, velocity, acceleration) || !valid(timed, trajectory, velocity, acceleration))
    return RetimingResult::FAILED;
  trajectory = timed;
  return RetimingResult::TOTG;
}

void RobustTimeParameterization::initialize(const rclcpp::Node::SharedPtr& node, const std::string& ns)
{
  options_.path_tolerance = getParam(node, LOGGER, ns, "path_tolerance", options_.path_tolerance);
  options_.resample_dt = getParam(node, LOGGER, ns, "resample_dt", options_.resample_dt);
  options_.min_angle_change = getParam(node, LOGGER, ns, "min_angle_change", options_.min_angle_change);
  options_.collinear_tolerance = getParam(node, LOGGER, ns, "retiming_collinear_tolerance", options_.collinear_tolerance);
  options_.default_velocity_limit = getParam(node, LOGGER, ns, "retiming_default_velocity_limit", options_.default_velocity_limit);
  options_.default_acceleration_limit = getParam(node, LOGGER, ns, "retiming_default_acceleration_limit", options_.default_acceleration_limit);
}

std::string RobustTimeParameterization::getDescription() const { return "Corrected Time Optimal Parameterization"; }

bool RobustTimeParameterization::adaptAndPlan(const PlannerFn& planner,
    const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& request,
    planning_interface::MotionPlanResponse& response, std::vector<std::size_t>&) const
{
  if (!planner(scene, request, response))
  {
    if (response.error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      response.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    response.trajectory_.reset();
    return false;
  }
  const auto result = response.trajectory_ ? retime_path(*response.trajectory_, request.max_velocity_scaling_factor,
      request.max_acceleration_scaling_factor, options_) : RetimingResult::FAILED;
  if (result == RetimingResult::FAILED)
  {
    response.error_code_.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    response.trajectory_.reset();
    RCLCPP_ERROR(LOGGER, "Time parameterization failed; refusing an untimed trajectory");
    return false;
  }

  return true;
}
}  // namespace cleany

PLUGINLIB_EXPORT_CLASS(cleany::RobustTimeParameterization, planning_request_adapter::PlanningRequestAdapter)
