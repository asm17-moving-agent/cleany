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

#include "cleany_navigation/mppi/optimizer.hpp"
#include "cleany_navigation/body_speed_limit.hpp"
#include "cleany_navigation/mppi/wheel_projection.hpp"
#include "cleany_navigation/mppi/path_seed.hpp"
#include "cleany_navigation/mppi/path_rotation.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include <numeric>

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace cleany_navigation::mppi
{

using namespace xt::placeholders;  // NOLINT
using xt::evaluation_strategy::immediate;

void Optimizer::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string & name,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  ParametersHandler * param_handler)
{
  parent_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  parameters_handler_ = param_handler;

  auto node = parent_.lock();
  logger_ = node->get_logger();

  getParams();

  critic_manager_.on_configure(parent_, name_, costmap_ros_, parameters_handler_);
  noise_generator_.initialize(settings_, isHolonomic(), name_, parameters_handler_);

  reset();
}

void Optimizer::shutdown()
{
  noise_generator_.shutdown();
}

void Optimizer::getParams()
{
  std::string motion_model_name;

  auto & s = settings_;
  auto getParam = parameters_handler_->getParamGetter(name_);
  auto getParentParam = parameters_handler_->getParamGetter("");
  getParam(s.model_dt, "model_dt", 0.05f);
  getParam(s.time_steps, "time_steps", 56);
  getParam(s.batch_size, "batch_size", 1000);
  getParam(s.iteration_count, "iteration_count", 1);
  getParam(s.temperature, "temperature", 0.3f);
  getParam(s.gamma, "gamma", 0.015f);
  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
  getParam(s.base_constraints.vx_min, "vx_min", -0.35);
  getParam(s.base_constraints.vy, "vy_max", 0.5);
  getParam(s.base_constraints.wz, "wz_max", 1.9);
  getParam(s.sampling_std.vx, "vx_std", 0.2);
  getParam(s.sampling_std.vy, "vy_std", 0.2);
  getParam(s.sampling_std.wz, "wz_std", 0.4);
  getParam(s.retry_attempt_limit, "retry_attempt_limit", 1);
  getParam(validate_output_, "validate_output", false);
  getParam(path_seed_count_, "path_seed_count", 0);
  getParam(path_seed_max_speed_, "path_seed_max_speed", 0.25f);
  if (path_seed_count_ < 0 || path_seed_max_speed_ <= 0.0f) {throw std::runtime_error("Invalid path seed settings");}
  getParam(wheel_max_rad_s_, "wheel_max_rad_s", 0.0f);
  getParam(wheel_radius_m_, "wheel_radius_m", 0.0635f);
  getParam(wheelbase_m_, "wheelbase_m", 0.35f);
  getParam(wheel_separation_m_, "wheel_separation_m", 0.6038f);
  if (wheel_max_rad_s_ < 0.0f || wheel_radius_m_ <= 0.0f ||
    wheelbase_m_ <= 0.0f || wheel_separation_m_ <= 0.0f)
  {throw std::runtime_error("Invalid wheel projection geometry or limit");}
  getParam(acceleration_limits_, "acceleration_limits", false);
  getParam(goal_approach_time_, "goal_approach_time", 1.f);
  getParam(ax_max_, "ax_max", .2f);
  getParam(ay_max_, "ay_max", .15f);
  getParam(az_max_, "az_max", .4f);
  getParam(wheel_acceleration_, "wheel_acceleration_max", 3.f);
  getParam(path_velocity_weight_, "path_velocity_weight", 0.f);
  getParam(control_variation_weight_, "control_variation_weight", 0.f);
  getParam(proximity_wheel_limit_, "proximity_wheel_limit", 0.f);
  getParam(proximity_clearance_, "proximity_clearance", .11f);
  getParam(translation_max_, "translation_max", 0.f);
  getParam(proximity_translation_max_, "proximity_translation_max", 0.f);
  getParam(proximity_yaw_max_, "proximity_yaw_max", 0.f);
  getParam(goal_translation_max_, "goal_translation_max", 0.f);
  getParam(goal_yaw_max_, "goal_yaw_max", 0.f);
  getParam(goal_slow_radius_, "goal_slow_radius", .2f);
  getParam(speed_limit_reaction_time_, "speed_limit_reaction_time", .3f);
  for (float value : {translation_max_, proximity_translation_max_, proximity_yaw_max_,
      goal_translation_max_, goal_yaw_max_, goal_slow_radius_, speed_limit_reaction_time_}) {
    if (!finiteLimit(value) || value < 0.f) {throw std::runtime_error("Invalid body speed limit");}
  }
  if ((translation_max_ > 0.f && (!acceleration_limits_ || translation_max_ >
      std::min(s.base_constraints.vx_max, s.base_constraints.vy))) ||
      proximity_translation_max_ > translation_max_ || goal_translation_max_ > translation_max_ ||
      proximity_yaw_max_ > s.base_constraints.wz || goal_yaw_max_ > s.base_constraints.wz ||
      ((proximity_translation_max_ > 0.f) != (proximity_yaw_max_ > 0.f)) ||
      ((goal_translation_max_ > 0.f) != (goal_yaw_max_ > 0.f))) {
    throw std::runtime_error("Body speed bounds require acceleration control and ordered limits");
  }
  if (!(goal_approach_time_>0.f && ax_max_>0.f && ay_max_>0.f && az_max_>0.f && wheel_acceleration_>0.f) ||
      !std::isfinite(goal_approach_time_+ax_max_+ay_max_+az_max_+wheel_acceleration_+path_velocity_weight_+
      control_variation_weight_+proximity_wheel_limit_+proximity_clearance_) ||
      path_velocity_weight_<0.f || control_variation_weight_<0.f || proximity_wheel_limit_<0.f ||
      proximity_clearance_<0.f || (proximity_wheel_limit_>0.f &&
      (!acceleration_limits_ || wheel_max_rad_s_<=0.f || proximity_wheel_limit_>wheel_max_rad_s_))) {
    throw std::runtime_error("Invalid motion continuity limits");
  }
  getParam(goal_rotate_enter_radius_, "goal_rotate_enter_radius", 0.0f);
  getParam(goal_rotate_exit_radius_, "goal_rotate_exit_radius", 0.0f);
  getParam(path_rotate_enabled_, "path_rotate_enabled", false);
  getParam(path_rotate_position_tolerance_, "path_rotate_position_tolerance", 0.075f);
  getParam(path_rotate_heading_tolerance_, "path_rotate_heading_tolerance", 0.1f);
  if (path_rotate_position_tolerance_ <= 0.0f || path_rotate_heading_tolerance_ <= 0.0f ||
    !std::isfinite(path_rotate_position_tolerance_) || !std::isfinite(path_rotate_heading_tolerance_))
  {throw std::runtime_error("Path rotation tolerances must be finite and positive");}
  if (goal_rotate_enter_radius_ < 0.0f ||
    goal_rotate_exit_radius_ < goal_rotate_enter_radius_)
  {
    throw std::runtime_error("Goal rotation radii must be nonnegative and ordered");
  }

  getParam(motion_model_name, "motion_model", std::string("DiffDrive"));

  s.constraints = s.base_constraints;
  setMotionModel(motion_model_name);
  parameters_handler_->addPostCallback([this]() {reset();});

  double controller_frequency;
  getParentParam(controller_frequency, "controller_frequency", 0.0, ParameterType::Static);
  setOffset(controller_frequency);
}

void Optimizer::setOffset(double controller_frequency)
{
  const double controller_period = 1.0 / controller_frequency;
  constexpr double eps = 1e-6;

  if ((controller_period + eps) < settings_.model_dt) {
    RCLCPP_WARN(
      logger_,
      "Controller period is less then model dt, consider setting it equal");
  } else if (abs(controller_period - settings_.model_dt) < eps) {
    RCLCPP_INFO(
      logger_,
      "Controller period is equal to model dt. Control sequence "
      "shifting is ON");
    settings_.shift_control_sequence = true;
  } else {
    throw std::runtime_error(
            "Controller period more then model dt, set it equal to model dt");
  }
}

void Optimizer::reset()
{
  // Keep the arrival domain through an internal fallback retry; prepare()
  // recomputes it from the current path and pose on the next control cycle.
  state_.reset(settings_.batch_size, settings_.time_steps);
  control_sequence_.reset(settings_.time_steps);
  control_history_[0] = {0.0, 0.0, 0.0};
  control_history_[1] = {0.0, 0.0, 0.0};
  control_history_[2] = {0.0, 0.0, 0.0};
  control_history_[3] = {0.0, 0.0, 0.0};

  settings_.constraints = settings_.base_constraints;

  costs_ = xt::zeros<float>({settings_.batch_size});
  generated_trajectories_.reset(settings_.batch_size, settings_.time_steps);

  noise_generator_.reset(settings_, isHolonomic());
  RCLCPP_INFO(logger_, "Optimizer reset");
}

geometry_msgs::msg::TwistStamped Optimizer::evalControl(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  const auto now=std::chrono::steady_clock::now();
  acceleration_origin_={float(robot_speed.linear.x),float(robot_speed.linear.y),float(robot_speed.angular.z)};
  if(last_output_at_.time_since_epoch().count() &&
    std::chrono::duration<double>(now-last_output_at_).count()<.2)acceleration_origin_=last_output_;
  if(wheel_max_rad_s_>0.f)projectWheelTwist(acceleration_origin_.x,acceleration_origin_.y,
    acceleration_origin_.yaw,wheel_radius_m_,.5f*(wheelbase_m_+wheel_separation_m_),wheel_max_rad_s_);
  prepare(robot_pose, robot_speed, plan, goal_checker);
  effective_wheel_limit_=proximityWheelLimit();
  updateBodyLimits();

  do {
    optimize();
  } while (fallback(critics_data_.fail_flag));

  utils::savitskyGolayFilter(control_sequence_, control_history_, settings_);
  // The Savitzky-Golay filter has negative coefficients and can overshoot.
  applyControlSequenceConstraints();
  // The filter carries prior translations. Clear those values before emission;
  // this is the same zero-translation domain scored by all active critics.
  if (!acceleration_limits_ && (goal_rotation_active_ || path_rotation_active_)) {
    control_sequence_.vx.fill(0.0f);
    control_sequence_.vy.fill(0.0f);
    for (auto & control : control_history_) {control.vx = control.vy = 0.0f;}
  }
  if (validate_output_) {validateOutput();}
  auto control = getControlFromSequenceAsTwist(plan.header.stamp);
  last_output_={float(control.twist.linear.x),float(control.twist.linear.y),float(control.twist.angular.z)};
  last_output_at_=std::chrono::steady_clock::now();

  if (settings_.shift_control_sequence) {
    shiftControlSequence();
  }

  return control;
}

bool Optimizer::sequenceCollisionFree(const models::ControlSequence & sequence) const
{
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(costmap_);
  const auto & footprint=costmap_ros_->getRobotFootprint();
  double x=state_.pose.pose.position.x,y=state_.pose.pose.position.y;
  double yaw=tf2::getYaw(state_.pose.pose.orientation);
  const double dt=settings_.model_dt;
  for (size_t t=0;t<sequence.vx.size();++t) {
    const double vx=t==0?state_.speed.linear.x:sequence.vx(t-1);
    const double vy=t==0?state_.speed.linear.y:sequence.vy(t-1);
    const double wz=t==0?state_.speed.angular.z:sequence.wz(t-1);
    const double dx=dt*(std::cos(yaw)*vx-std::sin(yaw)*vy);
    const double dy=dt*(std::sin(yaw)*vx+std::cos(yaw)*vy),da=dt*wz;
    const int steps=std::max({1,static_cast<int>(std::ceil(std::hypot(dx,dy)/.01)),
      static_cast<int>(std::ceil(std::abs(da)/.01745329252))});
    for (int k=1;k<=steps;++k) {
      const double u=static_cast<double>(k)/steps;
      const double cost=checker.footprintCostAtPose(x+u*dx,y+u*dy,yaw+u*da,footprint);
      if (cost<0 || cost>=254) {return false;}
    }
    x+=dx;y+=dy;yaw+=da;
  }
  return true;
}

void Optimizer::validateOutput()
{
  if (sequenceCollisionFree(control_sequence_)) {return;}
  // A weighted mean of feasible trajectories need not be feasible in SE(2).
  // Choose an already scored, native-checked proposal when averaging/filtering
  // crosses an obstacle. No collision cost or footprint threshold is relaxed.
  std::vector<size_t> order(costs_.size());std::iota(order.begin(),order.end(),0);
  std::sort(order.begin(),order.end(),[this](size_t a,size_t b){return costs_(a)<costs_(b);});
  for (const size_t row:order) {
    control_sequence_.vx=xt::view(state_.cvx,row,xt::all());
    control_sequence_.vy=xt::view(state_.cvy,row,xt::all());
    control_sequence_.wz=xt::view(state_.cwz,row,xt::all());
    applyControlSequenceConstraints();
    if (sequenceCollisionFree(control_sequence_)) {
      RCLCPP_WARN_THROTTLE(logger_,*parent_.lock()->get_clock(),2000,
        "MPPI mean rollout rejected; using scored collision-free candidate %zu",row);
      return;
    }
  }
  throw std::runtime_error("MPPI: no collision-free emitted rollout");
}

void Optimizer::optimize()
{
  for (size_t i = 0; i < settings_.iteration_count; ++i) {
    generateNoisedTrajectories();
    critic_manager_.evalTrajectoriesScores(critics_data_);
    scoreMotionContinuity();
    updateControlSequence();
  }
}

bool Optimizer::fallback(bool fail)
{
  static size_t counter = 0;

  if (!fail) {
    counter = 0;
    return false;
  }

  reset();

  if (++counter > settings_.retry_attempt_limit) {
    counter = 0;
    throw std::runtime_error("Optimizer fail to compute path");
  }

  return true;
}

void Optimizer::prepare(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  const nav_msgs::msg::Path & plan, nav2_core::GoalChecker * goal_checker)
{
  state_.pose = robot_pose;
  state_.speed = robot_speed;
  path_ = utils::toTensor(plan);
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  const bool valid_tolerance = goal_checker &&
    goal_checker->getTolerances(tolerance, velocity_tolerance);
  const bool bounded = valid_tolerance && goal_rotate_enter_radius_ > 0.0f &&
    goal_rotate_exit_radius_ <= tolerance.position.x;
  const float radius = goal_rotation_active_ ?
    goal_rotate_exit_radius_ : goal_rotate_enter_radius_;
  const bool next_rotation = bounded &&
    utils::withinPositionGoalTolerance(radius, state_.pose.pose, path_);
  if (next_rotation != goal_rotation_active_) {
    RCLCPP_INFO(logger_, "MPPI goal rotation phase: %s", next_rotation ? "enter" : "exit");
  }
  goal_rotation_active_ = next_rotation;
  path_rotation_active_ = false;
  if (path_rotate_enabled_ && !utils::withinPositionGoalTolerance(
      goal_checker, state_.pose.pose, path_))
  {
    const auto target = leadingRotationYaw(path_, state_.pose.pose, path_rotate_position_tolerance_);
    if (target) {
      const double error = *target - tf2::getYaw(state_.pose.pose.orientation);
      path_rotation_active_ = std::abs(std::atan2(std::sin(error), std::cos(error))) >
        path_rotate_heading_tolerance_;
    }
  }
  if (!acceleration_limits_ && (goal_rotation_active_ || path_rotation_active_)) {
    control_sequence_.vx.fill(0.0f);
    control_sequence_.vy.fill(0.0f);
    for (auto & control : control_history_) {control.vx = control.vy = 0.0f;}
  }
  costs_.fill(0);

  critics_data_.fail_flag = false;
  critics_data_.goal_checker = goal_checker;
  critics_data_.motion_model = motion_model_;
  critics_data_.furthest_reached_path_point.reset();
  critics_data_.path_pts_valid.reset();
}

void Optimizer::shiftControlSequence()
{
  using namespace xt::placeholders;  // NOLINT
  control_sequence_.vx = xt::roll(control_sequence_.vx, -1);
  control_sequence_.wz = xt::roll(control_sequence_.wz, -1);


  xt::view(control_sequence_.vx, -1) =
    xt::view(control_sequence_.vx, -2);

  xt::view(control_sequence_.wz, -1) =
    xt::view(control_sequence_.wz, -2);


  if (isHolonomic()) {
    control_sequence_.vy = xt::roll(control_sequence_.vy, -1);
    xt::view(control_sequence_.vy, -1) =
      xt::view(control_sequence_.vy, -2);
  }
}

void Optimizer::generateNoisedTrajectories()
{
  noise_generator_.setNoisedControls(state_, control_sequence_);
  noise_generator_.generateNextNoises();
  if (path_seed_count_ > 0 && isHolonomic() && !goal_rotation_active_) {
    std::vector<SeedPose> path;
    for (size_t j=0; j<path_.x.size(); ++j) {
      path.push_back({path_.x(j),path_.y(j),path_.yaws(j)});
    }
    const float yaw=tf2::getYaw(state_.pose.pose.orientation), dt=settings_.model_dt;
    // The motion model's first sample is the measured velocity.
    SeedPose pose{static_cast<float>(state_.pose.pose.position.x)+dt*
      (std::cos(yaw)*static_cast<float>(state_.speed.linear.x)-std::sin(yaw)*static_cast<float>(state_.speed.linear.y)),
      static_cast<float>(state_.pose.pose.position.y)+dt*
      (std::sin(yaw)*static_cast<float>(state_.speed.linear.x)+std::cos(yaw)*static_cast<float>(state_.speed.linear.y)),
      yaw+dt*static_cast<float>(state_.speed.angular.z)};
    const size_t count=std::min(static_cast<size_t>(path_seed_count_),static_cast<size_t>(settings_.batch_size));
    // Explore different short connectors onto the planner path, rather than
    // repeating eight identical proposals. Every connector is evaluated by all
    // critics and the native output checker; no map-specific pose is inserted.
    size_t connector_end=0;float connector_arc=0;
    while (connector_end+1<path.size() && connector_end<15) {
      const float step=std::hypot(path[connector_end+1].x-path[connector_end].x,
        path[connector_end+1].y-path[connector_end].y);
      if (connector_arc+step>.35f) {break;}
      connector_arc+=step;++connector_end;
    }
    for (size_t n=0; n<count; ++n) {
      const float speed=path_seed_max_speed_*(0.2f+0.8f*(n%8)/7.0f);
      auto proposal=pathSeed(pose,path,settings_.time_steps,dt,speed,
        std::min(settings_.constraints.wz,0.35f),wheel_radius_m_,
        0.5f*(wheelbase_m_+wheel_separation_m_),wheel_max_rad_s_,
        (n/8)%(connector_end+1));
      for (size_t t=0; t<proposal.size(); ++t) {
        state_.cvx(n,t)=proposal[t].x;state_.cvy(n,t)=proposal[t].y;state_.cwz(n,t)=proposal[t].yaw;
      }
    }
  }

  // Constrain the sampled controls before rollout/collision scoring, rather than
  // deleting XY from a trajectory that was evaluated as translating.
  if (goal_rotation_active_ || path_rotation_active_) {
    state_.cvx.fill(0.0f);
    state_.cvy.fill(0.0f);
  }
  if (wheel_max_rad_s_ > 0.0f && isHolonomic()) {
    const float lever = 0.5f * (wheelbase_m_ + wheel_separation_m_);
    for (size_t batch = 0; batch < settings_.batch_size; ++batch) {
      for (size_t t = 0; t < settings_.time_steps; ++t) {
        projectWheelTwist(state_.cvx(batch, t), state_.cvy(batch, t), state_.cwz(batch, t),
          wheel_radius_m_, lever, wheel_max_rad_s_);
      }
    }
  }
  if(acceleration_limits_) {
    for(size_t batch=0;batch<settings_.batch_size;++batch) {
      auto previous=acceleration_origin_;
      for(size_t t=0;t<settings_.time_steps;++t)constrainMotion(
        state_.cvx(batch,t),state_.cvy(batch,t),state_.cwz(batch,t),previous);
    }
  }
  updateStateVelocities(state_);
  integrateStateVelocities(generated_trajectories_, state_);
}

bool Optimizer::isHolonomic() const {return motion_model_->isHolonomic();}

void Optimizer::applyControlSequenceConstraints()
{
  auto & s = settings_;

  if (isHolonomic()) {
    control_sequence_.vy = xt::clip(control_sequence_.vy, -s.constraints.vy, s.constraints.vy);
  }

  control_sequence_.vx = xt::clip(control_sequence_.vx, s.constraints.vx_min, s.constraints.vx_max);
  control_sequence_.wz = xt::clip(control_sequence_.wz, -s.constraints.wz, s.constraints.wz);

  motion_model_->applyConstraints(control_sequence_);
  if (wheel_max_rad_s_ > 0.0f && isHolonomic()) {
    const float lever = 0.5f * (wheelbase_m_ + wheel_separation_m_);
    for (size_t t = 0; t < control_sequence_.vx.size(); ++t) {
      projectWheelTwist(control_sequence_.vx(t), control_sequence_.vy(t), control_sequence_.wz(t),
        wheel_radius_m_, lever, wheel_max_rad_s_);
    }
  }
  if(acceleration_limits_) {
    auto previous=acceleration_origin_;
    for(size_t t=0;t<control_sequence_.vx.size();++t)constrainMotion(
      control_sequence_.vx(t),control_sequence_.vy(t),control_sequence_.wz(t),previous);
  }
}

void Optimizer::constrainMotion(float &x,float &y,float &z,PlanarVelocity &previous) const
{
  if(goal_rotation_active_ || path_rotation_active_)x=y=0.f;
  if(path_.x.size()>0 && goal_rotate_enter_radius_>0.f) {
    const auto end=path_.x.size()-1;
    const double d=std::hypot(state_.pose.pose.position.x-path_.x(end),state_.pose.pose.position.y-path_.y(end));
    const double limit=std::max(0.,d-.8*goal_rotate_enter_radius_)/goal_approach_time_;
    const double speed=std::hypot(x,y);
    if(speed>limit && speed>0.){x*=limit/speed;y*=limit/speed;}
  }
  const auto &c=settings_.constraints;
  x=std::clamp(x,c.vx_min,c.vx_max);y=isHolonomic()?std::clamp(y,-c.vy,c.vy):0.f;
  z=std::clamp(z,-c.wz,c.wz);
  projectBodyVelocity(x,y,z,effective_translation_limit_,effective_yaw_limit_);
  if(effective_wheel_limit_>0.f)projectWheelTwist(x,y,z,wheel_radius_m_,
    .5f*(wheelbase_m_+wheel_separation_m_),effective_wheel_limit_);
  previous=boundedVelocityStep(previous,{x,y,z},settings_.model_dt,
    ax_max_,ay_max_,az_max_,wheel_acceleration_,wheel_radius_m_,.5f*(wheelbase_m_+wheel_separation_m_));
  x=previous.x;y=previous.y;z=previous.yaw;
}

float Optimizer::proximityWheelLimit() const
{
  if(proximity_wheel_limit_<=0.f || !costmap_)return wheel_max_rad_s_;
  // The padded footprint is 5 cm outside the wheels; orange starts another 11 cm away.
  // This is anticipatory comfort control. Collision Monitor still owns hard limits.
  const double braking=(wheel_max_rad_s_*wheel_max_rad_s_-proximity_wheel_limit_*proximity_wheel_limit_)*wheel_radius_m_/(2*wheel_acceleration_);
  const double clearance=obstacleClearance(proximity_clearance_+braking);
  return std::min(wheel_max_rad_s_,float(std::sqrt(proximity_wheel_limit_*proximity_wheel_limit_+
    2*wheel_acceleration_*std::max(0.,clearance-proximity_clearance_)/wheel_radius_m_)));
}

double Optimizer::obstacleClearance(double search_distance) const
{
  if (!costmap_) {return search_distance;}
  const auto footprint=costmap_ros_->getRobotFootprint();
  double minx=0,maxx=0,miny=0,maxy=0;
  for(const auto&p:footprint){minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
  const double angle=tf2::getYaw(state_.pose.pose.orientation),cs=std::cos(angle),sn=std::sin(angle);
  const auto &p=state_.pose.pose.position;
  double clearance=search_distance;
  const double reach=std::max({std::abs(minx),std::abs(maxx),std::abs(miny),std::abs(maxy)})*1.42+clearance;
  int x0,y0,x1,y1;
  costmap_->worldToMapEnforceBounds(p.x-reach,p.y-reach,x0,y0);
  costmap_->worldToMapEnforceBounds(p.x+reach,p.y+reach,x1,y1);
  for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x) {
    if(costmap_->getCost(x,y)<254)continue;
    double wx,wy;costmap_->mapToWorld(x,y,wx,wy);wx-=p.x;wy-=p.y;
    const double bx=cs*wx+sn*wy,by=-sn*wx+cs*wy;
    const double dx=std::max({minx-bx,0.,bx-maxx}),dy=std::max({miny-by,0.,by-maxy});
    clearance=std::min(clearance,std::max(0.,std::hypot(dx,dy)-costmap_->getResolution()*.7072));
  }
  return clearance;
}

void Optimizer::updateBodyLimits()
{
  const double maximum=std::min(double(translation_max_),
    std::max(double(settings_.constraints.vx_max),double(settings_.constraints.vy)));
  effective_translation_limit_=maximum;
  effective_yaw_limit_=translation_max_ > 0.f ? settings_.constraints.wz : 0.f;
  if (translation_max_ <= 0.f) {return;}
  // This is a conservative deceleration for every planar direction, including
  // diagonal wheel motion. The same wheel acceleration is enforced by rollout.
  const double decel=std::min({double(ax_max_),double(ay_max_),
    wheel_acceleration_*wheel_radius_m_/std::sqrt(2.)});
  const double lever=.5*(wheelbase_m_+wheel_separation_m_);
  const double angular_decel=std::min(double(az_max_),wheel_acceleration_*wheel_radius_m_/lever);
  const double yaw_max=settings_.constraints.wz;
  if (proximity_translation_max_ > 0.f && proximity_yaw_max_ > 0.f) {
    double radius=lever;
    if (costmap_ros_) {
      for (const auto &p:costmap_ros_->getRobotFootprint())radius=std::max(radius,std::hypot(p.x,p.y));
    }
    const double linear_distance=(maximum*maximum-
      proximity_translation_max_*proximity_translation_max_)/(2.*decel)+maximum*speed_limit_reaction_time_;
    const double angular_distance=radius*((yaw_max*yaw_max-proximity_yaw_max_*proximity_yaw_max_)/
      (2.*angular_decel)+yaw_max*speed_limit_reaction_time_);
    const double gap=obstacleClearance(proximity_clearance_+std::max(linear_distance,angular_distance))-proximity_clearance_;
    effective_translation_limit_=approachingSpeed(maximum,proximity_translation_max_,decel,gap,speed_limit_reaction_time_);
    effective_yaw_limit_=approachingSpeed(yaw_max,proximity_yaw_max_,angular_decel,gap/radius,speed_limit_reaction_time_);
  }
  if (path_.x.size()>0 && goal_translation_max_>0.f && goal_yaw_max_>0.f) {
    const size_t end=path_.x.size()-1;
    const double gap=std::hypot(state_.pose.pose.position.x-path_.x(end),
      state_.pose.pose.position.y-path_.y(end))-goal_slow_radius_;
    effective_translation_limit_=std::min(double(effective_translation_limit_),
      approachingSpeed(maximum,goal_translation_max_,decel,gap,speed_limit_reaction_time_));
    effective_yaw_limit_=std::min(double(effective_yaw_limit_),
      approachingSpeed(yaw_max,goal_yaw_max_,angular_decel,gap/lever,speed_limit_reaction_time_));
  }
}

void Optimizer::scoreMotionContinuity()
{
  if(path_velocity_weight_<=0.f && control_variation_weight_<=0.f)return;
  // Apply local cross-track velocity damping only on an approximately straight
  // prefix; curved paths and in-place turns retain the existing critics.
  bool straight=false;double nx=0,ny=0,error=0;
  if(path_.x.size()>1 && !goal_rotation_active_ && !path_rotation_active_) {
    size_t end=1;
    while(end+1<path_.x.size() && std::hypot(path_.x(end)-path_.x(0),path_.y(end)-path_.y(0))<.5)++end;
    const double dx=path_.x(end)-path_.x(0),dy=path_.y(end)-path_.y(0),length=std::hypot(dx,dy);
    if(length>.1) {
      nx=-dy/length;ny=dx/length;straight=true;
      for(size_t i=1;i<end;++i)if(std::abs(nx*(path_.x(i)-path_.x(0))+ny*(path_.y(i)-path_.y(0)))>.03)straight=false;
      error=nx*(state_.pose.pose.position.x-path_.x(0))+ny*(state_.pose.pose.position.y-path_.y(0));
    }
  }
  const size_t count=std::min(size_t(settings_.time_steps),size_t(std::ceil(.5/settings_.model_dt)));
  for(size_t b=0;b<settings_.batch_size;++b) {
    double lateral=0,variation=0;auto previous=acceleration_origin_;
    for(size_t t=0;t<count;++t) {
      const double x=state_.cvx(b,t),y=state_.cvy(b,t),z=state_.cwz(b,t),a=generated_trajectories_.yaws(b,t);
      if(straight)lateral+=std::abs(nx*(std::cos(a)*x-std::sin(a)*y)+ny*(std::sin(a)*x+std::cos(a)*y)+std::clamp(error,-.05,.05));
      variation+=std::abs(x-previous.x)+std::abs(y-previous.y)+.5f*(wheelbase_m_+wheel_separation_m_)*std::abs(z-previous.yaw);
      previous={float(x),float(y),float(z)};
    }
    costs_(b)+=(path_velocity_weight_*lateral+control_variation_weight_*variation)/count;
  }
}

void Optimizer::updateStateVelocities(
  models::State & state) const
{
  updateInitialStateVelocities(state);
  propagateStateVelocitiesFromInitials(state);
}

void Optimizer::updateInitialStateVelocities(
  models::State & state) const
{
  xt::noalias(xt::view(state.vx, xt::all(), 0)) = state.speed.linear.x;
  xt::noalias(xt::view(state.wz, xt::all(), 0)) = state.speed.angular.z;

  if (isHolonomic()) {
    xt::noalias(xt::view(state.vy, xt::all(), 0)) = state.speed.linear.y;
  }
}

void Optimizer::propagateStateVelocitiesFromInitials(
  models::State & state) const
{
  motion_model_->predict(state);
}

void Optimizer::integrateStateVelocities(
  xt::xtensor<float, 2> & trajectory,
  const xt::xtensor<float, 2> & sequence) const
{
  float initial_yaw = tf2::getYaw(state_.pose.pose.orientation);

  const auto vx = xt::view(sequence, xt::all(), 0);
  const auto vy = xt::view(sequence, xt::all(), 2);
  const auto wz = xt::view(sequence, xt::all(), 1);

  auto traj_x = xt::view(trajectory, xt::all(), 0);
  auto traj_y = xt::view(trajectory, xt::all(), 1);
  auto traj_yaws = xt::view(trajectory, xt::all(), 2);

  xt::noalias(traj_yaws) = xt::cumsum(wz * settings_.model_dt, 0) + initial_yaw;

  auto && yaw_cos = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 1>::from_shape(traj_yaws.shape());

  const auto yaw_offseted = xt::view(traj_yaws, xt::range(1, _));

  xt::noalias(xt::view(yaw_cos, 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::range(1, _))) = xt::cos(yaw_offseted);
  xt::noalias(xt::view(yaw_sin, xt::range(1, _))) = xt::sin(yaw_offseted);

  auto && dx = xt::eval(vx * yaw_cos);
  auto && dy = xt::eval(vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - vy * yaw_sin;
    dy = dy + vy * yaw_cos;
  }

  xt::noalias(traj_x) = state_.pose.pose.position.x + xt::cumsum(dx * settings_.model_dt, 0);
  xt::noalias(traj_y) = state_.pose.pose.position.y + xt::cumsum(dy * settings_.model_dt, 0);
}

void Optimizer::integrateStateVelocities(
  models::Trajectories & trajectories,
  const models::State & state) const
{
  const float initial_yaw = tf2::getYaw(state.pose.pose.orientation);

  xt::noalias(trajectories.yaws) =
    xt::cumsum(state.wz * settings_.model_dt, 1) + initial_yaw;

  const auto yaws_cutted = xt::view(trajectories.yaws, xt::all(), xt::range(0, -1));

  auto && yaw_cos = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  auto && yaw_sin = xt::xtensor<float, 2>::from_shape(trajectories.yaws.shape());
  xt::noalias(xt::view(yaw_cos, xt::all(), 0)) = cosf(initial_yaw);
  xt::noalias(xt::view(yaw_sin, xt::all(), 0)) = sinf(initial_yaw);
  xt::noalias(xt::view(yaw_cos, xt::all(), xt::range(1, _))) = xt::cos(yaws_cutted);
  xt::noalias(xt::view(yaw_sin, xt::all(), xt::range(1, _))) = xt::sin(yaws_cutted);

  auto && dx = xt::eval(state.vx * yaw_cos);
  auto && dy = xt::eval(state.vx * yaw_sin);

  if (isHolonomic()) {
    dx = dx - state.vy * yaw_sin;
    dy = dy + state.vy * yaw_cos;
  }

  xt::noalias(trajectories.x) = state.pose.pose.position.x +
    xt::cumsum(dx * settings_.model_dt, 1);
  xt::noalias(trajectories.y) = state.pose.pose.position.y +
    xt::cumsum(dy * settings_.model_dt, 1);
}

xt::xtensor<float, 2> Optimizer::getOptimizedTrajectory()
{
  auto && sequence =
    xt::xtensor<float, 2>::from_shape({settings_.time_steps, isHolonomic() ? 3u : 2u});
  auto && trajectories = xt::xtensor<float, 2>::from_shape({settings_.time_steps, 3});

  xt::noalias(xt::view(sequence, xt::all(), 0)) = control_sequence_.vx;
  xt::noalias(xt::view(sequence, xt::all(), 1)) = control_sequence_.wz;

  if (isHolonomic()) {
    xt::noalias(xt::view(sequence, xt::all(), 2)) = control_sequence_.vy;
  }

  integrateStateVelocities(trajectories, sequence);
  return std::move(trajectories);
}

void Optimizer::updateControlSequence()
{
  auto & s = settings_;
  auto bounded_noises_vx = state_.cvx - control_sequence_.vx;
  auto bounded_noises_wz = state_.cwz - control_sequence_.wz;
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.vx, 2) * xt::sum(
    xt::view(control_sequence_.vx, xt::newaxis(), xt::all()) * bounded_noises_vx, 1, immediate);
  xt::noalias(costs_) +=
    s.gamma / powf(s.sampling_std.wz, 2) * xt::sum(
    xt::view(control_sequence_.wz, xt::newaxis(), xt::all()) * bounded_noises_wz, 1, immediate);

  if (isHolonomic()) {
    auto bounded_noises_vy = state_.cvy - control_sequence_.vy;
    xt::noalias(costs_) +=
      s.gamma / powf(s.sampling_std.vy, 2) * xt::sum(
      xt::view(control_sequence_.vy, xt::newaxis(), xt::all()) * bounded_noises_vy,
      1, immediate);
  }

  auto && costs_normalized = costs_ - xt::amin(costs_, immediate);
  auto && exponents = xt::eval(xt::exp(-1 / settings_.temperature * costs_normalized));
  auto && softmaxes = xt::eval(exponents / xt::sum(exponents, immediate));
  auto && softmaxes_extened = xt::eval(xt::view(softmaxes, xt::all(), xt::newaxis()));

  xt::noalias(control_sequence_.vx) = xt::sum(state_.cvx * softmaxes_extened, 0, immediate);
  xt::noalias(control_sequence_.wz) = xt::sum(state_.cwz * softmaxes_extened, 0, immediate);
  if (isHolonomic()) {
    xt::noalias(control_sequence_.vy) = xt::sum(state_.cvy * softmaxes_extened, 0, immediate);
  }

  applyControlSequenceConstraints();
}

geometry_msgs::msg::TwistStamped Optimizer::getControlFromSequenceAsTwist(
  const builtin_interfaces::msg::Time & stamp)
{
  unsigned int offset = !acceleration_limits_ && settings_.shift_control_sequence ? 1 : 0;

  auto vx = control_sequence_.vx(offset);
  auto wz = control_sequence_.wz(offset);

  if (isHolonomic()) {
    auto vy = control_sequence_.vy(offset);
    return utils::toTwistStamped(vx, vy, wz, stamp, costmap_ros_->getBaseFrameID());
  }

  return utils::toTwistStamped(vx, wz, stamp, costmap_ros_->getBaseFrameID());
}

void Optimizer::setMotionModel(const std::string & model)
{
  if (model == "DiffDrive") {
    motion_model_ = std::make_shared<DiffDriveMotionModel>();
  } else if (model == "Omni") {
    motion_model_ = std::make_shared<OmniMotionModel>();
  } else if (model == "Ackermann") {
    motion_model_ = std::make_shared<AckermannMotionModel>(parameters_handler_, name_);
  } else {
    throw std::runtime_error(
            std::string(
              "Model " + model + " is not valid! Valid options are DiffDrive, Omni, "
              "or Ackermann"));
  }
}

void Optimizer::setSpeedLimit(double speed_limit, bool percentage)
{
  auto & s = settings_;
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
    s.constraints.vx_max = s.base_constraints.vx_max;
    s.constraints.vx_min = s.base_constraints.vx_min;
    s.constraints.vy = s.base_constraints.vy;
    s.constraints.wz = s.base_constraints.wz;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      double ratio = speed_limit / 100.0;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    } else {
      // Speed limit is expressed in absolute value
      double ratio = speed_limit / s.base_constraints.vx_max;
      s.constraints.vx_max = s.base_constraints.vx_max * ratio;
      s.constraints.vx_min = s.base_constraints.vx_min * ratio;
      s.constraints.vy = s.base_constraints.vy * ratio;
      s.constraints.wz = s.base_constraints.wz * ratio;
    }
  }
}

models::Trajectories & Optimizer::getGeneratedTrajectories()
{
  return generated_trajectories_;
}

}  // namespace cleany_navigation::mppi
