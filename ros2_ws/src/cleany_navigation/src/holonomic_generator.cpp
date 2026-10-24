#include "cleany_navigation/holonomic_generator.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include "pluginlib/class_list_macros.hpp"
namespace cleany_navigation {
namespace {
// Offer one direction at several speeds, clamped to the velocity box.
void addDirectionCandidates(std::vector<nav_2d_msgs::msg::Twist2D> &candidates,
  double ux,double uy,double speed,dwb_plugins::KinematicParameters limits) {
  const double unit[]={ux,uy},lower[]={limits.getMinX(),limits.getMinY()},
    upper[]={limits.getMaxX(),limits.getMaxY()};
  for(int i=0;i<2;++i) {
    if(unit[i]>1e-12)speed=std::min(speed,upper[i]/unit[i]);
    else if(unit[i]<-1e-12)speed=std::min(speed,lower[i]/unit[i]);
  }
  // Stay inside the mathematical cap after division/multiplication
  // round-off. Otherwise a legal max-speed vector can round one ULP above
  // max_speed_xy and disappear, changing the candidate set discontinuously.
  // Keep the strict bound checks below; do not widen any velocity limit.
  speed=std::nextafter(std::nextafter(speed,0.),0.);
  if(!std::isfinite(speed) || speed<=0.)return;
  for(double fraction:{.25,.5,1.}) {
    nav_2d_msgs::msg::Twist2D cmd;cmd.x=ux*speed*fraction;cmd.y=uy*speed*fraction;
    const double magnitude=std::hypot(cmd.x,cmd.y);
    if(cmd.x<lower[0] || cmd.x>upper[0] || cmd.y<lower[1] || cmd.y>upper[1] ||
       magnitude<limits.getMinSpeedXY() || magnitude>limits.getMaxSpeedXY())continue;
    candidates.push_back(cmd);
  }
}
}
HolonomicGenerator::~HolonomicGenerator() {
  if(timing_trace_file_.empty())return;
  try {
    using Json=nlohmann::json;
    Json records=Json::array();
    for(const auto &r:timing_records_) {
      const char *reason=!r.had_previous ? "after_reset" :
        r.raw_dt_s<0. ? "clock_backward" : r.raw_dt_s==0. ? "clock_unchanged" : "clock_advanced";
      records.push_back({{"stamp_ns",r.stamp_ns},{"previous_stamp_ns",r.had_previous ? Json(r.previous_stamp_ns) : Json(nullptr)},
        {"reset_generation",r.reset_generation},{"reason",reason},{"raw_dt_s",r.raw_dt_s},
        {"command_dt_s",r.command_dt_s},{"measured",{r.measured.x,r.measured.y,r.measured.theta}},
        {"first_dt_min",r.first_dt_min ? Json(*r.first_dt_min) : Json(nullptr)},
        {"first_dt_max",r.first_dt_max ? Json(*r.first_dt_max) : Json(nullptr)},
        {"trajectories",r.trajectories},{"nonzero_targets",r.nonzero_targets},
        {"zero_first_commands",r.zero_first_commands}});
    }
    const auto target=std::filesystem::path(timing_trace_file_);
    if(!target.parent_path().empty())std::filesystem::create_directories(target.parent_path());
    const auto temporary=target.string()+".tmp";
    std::ofstream out(temporary);out.exceptions(std::ios::failbit|std::ios::badbit);
    out<<Json{{"schema",1},{"max_records",10000},{"dropped",timing_dropped_},{"records",records}}.dump();
    out.close();std::filesystem::rename(temporary,target);
  } catch(const std::exception &error) {
    RCLCPP_WARN(rclcpp::get_logger("holonomic_generator"),"Timing trace write failed: %s",error.what());
  }
}
void HolonomicGenerator::reset() {
  clearGoalContext();
  clearContinuityContext();
  candidates_.clear();index_=0;last_iteration_.reset();command_dt_=0.;++reset_generation_;
  timing_current_recorded_=false;
}
void HolonomicGenerator::initialize(const nav2_util::LifecycleNode::SharedPtr &node,const std::string &name) {
  StandardTrajectoryGenerator::initialize(node,name);
  clock_=node->get_clock();
  const auto elapsed_key=name+".use_elapsed_command_time";
  if(!node->has_parameter(elapsed_key))node->declare_parameter(elapsed_key,true);
  use_elapsed_command_time_=node->get_parameter(elapsed_key).as_bool();
  const auto direction_key=name+".preserve_translation_direction";
  if(!node->has_parameter(direction_key))node->declare_parameter(direction_key,true);
  preserve_translation_direction_=node->get_parameter(direction_key).as_bool();
  const auto target_direction_key=name+".prefer_target_translation_direction";
  if(!node->has_parameter(target_direction_key))node->declare_parameter(target_direction_key,false);
  prefer_target_translation_direction_=node->get_parameter(target_direction_key).as_bool();
  double wheelbase=.35,separation=.6038;
  for(auto entry:{std::make_pair("max_wheel_speed_rad_s",&max_wheel_speed_),
      std::make_pair("mecanum_wheel_radius_m",&wheel_radius_),
      std::make_pair("mecanum_wheelbase_m",&wheelbase),
      std::make_pair("mecanum_wheel_separation_m",&separation)}) {
    const auto key=name+"."+entry.first;
    if(!node->has_parameter(key))node->declare_parameter(key,*entry.second);
    *entry.second=node->get_parameter(key).as_double();
  }
  if(!std::isfinite(max_wheel_speed_) || max_wheel_speed_<0. ||
     !std::isfinite(wheel_radius_) || wheel_radius_<=0. ||
     !std::isfinite(wheelbase) || wheelbase<=0. ||
     !std::isfinite(separation) || separation<=0.)
    throw std::invalid_argument(name+" mecanum wheel limits/geometry");
  wheel_lever_=(wheelbase+separation)/2.;
  const auto continuity_key=name+".continuity_samples";
  if(!node->has_parameter(continuity_key))node->declare_parameter(continuity_key,false);
  continuity_samples_=node->get_parameter(continuity_key).as_bool();
  const auto trace_key=name+".timing_trace_file";
  if(!node->has_parameter(trace_key))node->declare_parameter(trace_key,std::string());
  timing_trace_file_=node->get_parameter(trace_key).as_string();
  if(!timing_trace_file_.empty())timing_records_.reserve(10000);
  reset();
  const auto key=name+".fine_translation_scale";
  if(!node->has_parameter(key))node->declare_parameter(key,.25);
  fine_scale_=node->get_parameter(key).as_double();
  if(!std::isfinite(fine_scale_) || fine_scale_<=0 || fine_scale_>=1)throw std::invalid_argument(key);
  const auto radius_key=name+".goal_directed_radius";
  if(!node->has_parameter(radius_key))node->declare_parameter(radius_key,0.);
  goal_directed_radius_=node->get_parameter(radius_key).as_double();
  if(!std::isfinite(goal_directed_radius_) || goal_directed_radius_<0.)
    throw std::invalid_argument(radius_key);
}
void HolonomicGenerator::setGoalContext(const geometry_msgs::msg::Pose2D &pose,
  const geometry_msgs::msg::Pose2D &goal) {
  clearGoalContext();
  for(double value:{pose.x,pose.y,pose.theta,goal.x,goal.y,goal.theta})
    if(!std::isfinite(value))return;
  goal_context_=GoalContext{pose,goal};
}
void HolonomicGenerator::startNewIteration(const nav_2d_msgs::msg::Twist2D &current) {
  const auto goal_context=goal_context_;clearGoalContext();
  const auto continuity_context=continuity_context_;clearContinuityContext();
  candidates_.clear();index_=0;
  const auto now=clock_->now();command_dt_=0.;
  const bool had_previous=last_iteration_.has_value();
  const auto previous_ns=had_previous ? last_iteration_->nanoseconds() : 0;
  const auto raw_dt=had_previous ? (now-*last_iteration_).seconds() : 0.;
  command_dt_=std::max(0.,raw_dt);
  timing_current_recorded_=false;
  if(!timing_trace_file_.empty()) {
    if(timing_records_.size()<10000) {
      timing_records_.push_back({now.nanoseconds(),previous_ns,reset_generation_,had_previous,
        raw_dt,command_dt_,current,std::nullopt,std::nullopt});
      timing_current_recorded_=true;
    } else ++timing_dropped_;
  }
  last_iteration_=now;
  StandardTrajectoryGenerator::startNewIteration(current);
  std::vector<nav_2d_msgs::msg::Twist2D> fine;
  auto limits=kinematics_handler_->getKinematics();
  while(StandardTrajectoryGenerator::hasMoreTwists()) {
    auto cmd=StandardTrajectoryGenerator::nextTwist();
    // Exact zero matters to RotateToGoal, which tests fabs(v)>0.
    for(auto value:{&cmd.x,&cmd.y,&cmd.theta})if(std::abs(*value)<1e-9)*value=0.;
    candidates_.push_back(cmd);
    if(cmd.theta==0. && (cmd.x!=0. || cmd.y!=0.)) {
      cmd.x*=fine_scale_;cmd.y*=fine_scale_;
      if(cmd.x>=limits.getMinX() && cmd.x<=limits.getMaxX() &&
         cmd.y>=limits.getMinY() && cmd.y<=limits.getMaxY() &&
         std::hypot(cmd.x,cmd.y)>=limits.getMinSpeedXY() &&
         std::hypot(cmd.x,cmd.y)<=limits.getMaxSpeedXY())fine.push_back(cmd);
    }
  }
  // Only the zero-yaw plane gains fine samples, not a larger XYZ product.
  // Inherited rollout still models measured velocity and acceleration/braking.
  candidates_.insert(candidates_.end(),fine.begin(),fine.end());
  if(continuity_samples_) {
    // A Cartesian velocity grid loses a selected diagonal as the acceleration
    // window moves. Offer that direction again at several speeds, together
    // with measured travel. All normal rollouts and critics still run.
    std::vector<nav_2d_msgs::msg::Twist2D> directions{current};
    if(continuity_context)directions.push_back(*continuity_context);
    for(const auto &direction:directions) {
      const double length=std::hypot(direction.x,direction.y);
      if(!std::isfinite(length) || length<=1e-6)continue;
      const double ux=direction.x/length,uy=direction.y/length;
      addDirectionCandidates(candidates_,ux,uy,limits.getMaxSpeedXY(),limits);
    }
  }
  if(goal_context && goal_directed_radius_>0.) {
    const auto &pose=goal_context->pose;const auto &goal=goal_context->goal;
    const double dx=goal.x-pose.x,dy=goal.y-pose.y,distance=std::hypot(dx,dy);
    if(distance>1e-9 && distance<=goal_directed_radius_ &&
       std::isfinite(distance) && std::isfinite(sim_time_) && sim_time_>0.) {
      // Rotate the world-frame error into robot axes. This supplies candidate
      // targets only: inherited rollout and every critic still evaluate them.
      const double ux=(std::cos(pose.theta)*dx+std::sin(pose.theta)*dy)/distance;
      const double uy=(-std::sin(pose.theta)*dx+std::cos(pose.theta)*dy)/distance;
      addDirectionCandidates(candidates_,ux,uy,std::min(limits.getMaxSpeedXY(),distance/sim_time_),limits);
    }
  }
}
std::vector<double> HolonomicGenerator::getTimeSteps(const nav_2d_msgs::msg::Twist2D &command) {
  auto steps=StandardTrajectoryGenerator::getTimeSteps(command);
  // Command acceleration uses elapsed robot/ROS time, not the fixed rollout
  // discretization. Split just its first interval; retain the full horizon and
  // existing resolution. Startup, reset and paused/backward clocks grant no
  // acceleration budget. Large gaps cannot exceed the old first-step budget.
  const auto first=steps.front();const auto dt=std::min(command_dt_,first);
  if(limit_vel_cmd_in_traj_ && use_elapsed_command_time_ && dt<first) {
    steps.front()=first-dt;steps.insert(steps.begin(),dt);
  }
  if(timing_current_recorded_) {
    auto &record=timing_records_.back();const auto actual=steps.front();
    record.first_dt_min=record.first_dt_min ? std::min(*record.first_dt_min,actual) : actual;
    record.first_dt_max=record.first_dt_max ? std::max(*record.first_dt_max,actual) : actual;
  }
  return steps;
}
dwb_msgs::msg::Trajectory2D HolonomicGenerator::generateTrajectory(
  const geometry_msgs::msg::Pose2D &pose,const nav_2d_msgs::msg::Twist2D &current,
  const nav_2d_msgs::msg::Twist2D &target) {
  auto result=StandardTrajectoryGenerator::generateTrajectory(pose,current,target);
  if(timing_current_recorded_) {
    auto &record=timing_records_.back();++record.trajectories;
    record.nonzero_targets+=(target.x!=0. || target.y!=0. || target.theta!=0.);
    record.zero_first_commands+=(result.velocity.x==0. && result.velocity.y==0. && result.velocity.theta==0.);
  }
  return result;
}
nav_2d_msgs::msg::Twist2D HolonomicGenerator::computeNewVelocity(
  const nav_2d_msgs::msg::Twist2D &target,const nav_2d_msgs::msg::Twist2D &current,double dt) {
  auto result=StandardTrajectoryGenerator::computeNewVelocity(target,current,dt);
  if(preserve_translation_direction_) {
    const auto projected=result;
    // Share one progress fraction between X/Y while respecting BOTH inherited
    // component budgets. Keep this segment ramp as the fallback when reaching
    // the target bearing would require too much lateral acceleration/braking.
    double progress=1.;
    const double delta[]={target.x-current.x,target.y-current.y};
    const double allowed[]={result.x-current.x,result.y-current.y};
    for(int i=0;i<2;++i)if(std::abs(delta[i])>1e-12)
      progress=std::min(progress,std::clamp(std::abs(allowed[i]/delta[i]),0.,1.));
    result.x=current.x+progress*delta[0];result.y=current.y+progress*delta[1];
    if(prefer_target_translation_direction_ && std::hypot(target.x,target.y)>1e-12) {
      // Intersect the target ray with the inherited reachable component box.
      // This can correct small measured cross-track velocity immediately,
      // instead of amplifying it into bearing jitter at low commanded speed.
      // The sampled target is unchanged; all normal rollouts/critics still run.
      double lower=0.,upper=1.;bool feasible=true;
      const double target_xy[]={target.x,target.y},current_xy[]={current.x,current.y},
        projected_xy[]={projected.x,projected.y};
      for(int i=0;i<2;++i) {
        const double lo=std::min(current_xy[i],projected_xy[i]);
        const double hi=std::max(current_xy[i],projected_xy[i]);
        if(std::abs(target_xy[i])<=1e-12) {
          if(lo>0. || hi<0.)feasible=false;
        } else {
          const double a=lo/target_xy[i],b=hi/target_xy[i];
          lower=std::max(lower,std::min(a,b));upper=std::min(upper,std::max(a,b));
        }
      }
      if(feasible && lower<=upper) {
        result.x=upper*target.x;result.y=upper*target.y;
      }
    }
  }
  // Measured odometry can overshoot the commanded box. Acceleration limiting
  // alone would then emit out-of-box commands while braking. Apply component
  // caps inside the rollout so critics check the same first command we emit.
  auto limits=kinematics_handler_->getKinematics();
  result.x=std::clamp(result.x,limits.getMinX(),limits.getMaxX());
  result.y=std::clamp(result.y,limits.getMinY(),limits.getMaxY());
  result.theta=std::clamp(result.theta,-limits.getMaxTheta(),limits.getMaxTheta());
  if(max_wheel_speed_>0.) {
    const double rotation=wheel_lever_*result.theta;
    const double wheel_max=std::max({std::abs(result.x-result.y-rotation),
      std::abs(result.x+result.y+rotation),std::abs(result.x+result.y-rotation),
      std::abs(result.x-result.y+rotation)})/wheel_radius_;
    if(wheel_max>max_wheel_speed_) {
      const double scale=max_wheel_speed_/wheel_max;
      result.x*=scale;result.y*=scale;result.theta*=scale;
    }
  }
  return result;
}
}
PLUGINLIB_EXPORT_CLASS(cleany_navigation::HolonomicGenerator,dwb_core::TrajectoryGenerator)
