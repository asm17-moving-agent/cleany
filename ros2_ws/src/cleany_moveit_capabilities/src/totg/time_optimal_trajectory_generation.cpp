/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Tobias Kunz <tobias@gatech.edu>
 * Date: 05/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * Algorithm details and publications:
 * http://www.golems.org/node/1570
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <limits>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include "time_optimal_trajectory_generation.hpp"
#include <vector>
#include <array>
#include <functional>
#include <numeric>

namespace cleany::totg
{
namespace
{
static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("cleany.time_optimal_trajectory_generation");
constexpr double DEFAULT_TIMESTEP = 1e-3;

}  // namespace

class LinearPathSegment : public PathSegment
{
public:
  LinearPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& end)
    : PathSegment((end - start).norm()), end_(end), start_(start)
  {
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    s /= length_;
    s = std::max(0.0, std::min(1.0, s));
    return (1.0 - s) * start_ + s * end_;
  }

  Eigen::VectorXd getTangent(double /* s */) const override
  {
    return (end_ - start_) / length_;
  }

  Eigen::VectorXd getCurvature(double /* s */) const override
  {
    return Eigen::VectorXd::Zero(start_.size());
  }

  std::list<double> getSwitchingPoints() const override
  {
    return std::list<double>();
  }

  LinearPathSegment* clone() const override
  {
    return new LinearPathSegment(*this);
  }

private:
  Eigen::VectorXd end_;
  Eigen::VectorXd start_;
};

class CircularPathSegment : public PathSegment
{
public:
  CircularPathSegment(const Eigen::VectorXd& start, const Eigen::VectorXd& intersection,
                      const Eigen::VectorXd& end, double max_deviation)
    : start_(intersection), end_(intersection), tangent_(Eigen::VectorXd::Zero(start.size())),
      normal_(Eigen::VectorXd::Zero(start.size())), radius_(1.)
  {
    const double before = (intersection-start).norm(), after = (end-intersection).norm();
    if (before == 0. || after == 0.) return;
    tangent_ = (intersection-start)/before;
    const Eigen::VectorXd outgoing = (end-intersection)/after;
    const double cosine = std::clamp(tangent_.dot(outgoing), -1., 1.);
    normal_ = outgoing-cosine*tangent_;
    const double sine = normal_.norm();
    // Only a numerically collinear corner is degenerate. Absolute length and
    // broad dot-product cutoffs used to discard tiny paths and near reversals.
    if (sine <= 64.*std::numeric_limits<double>::epsilon()) return;
    normal_ /= sine;
    const double angle = std::atan2(sine, cosine);
    const double distance = std::min({before, after, max_deviation/std::tan(angle/4.)});
    radius_ = distance/std::tan(angle/2.);
    length_ = angle*radius_;
    start_ = distance == before ? start : Eigen::VectorXd(intersection-distance*tangent_);
    end_ = distance == after ? end : Eigen::VectorXd(intersection+distance*outgoing);
  }

  Eigen::VectorXd getConfig(double s) const override
  {
    if (s <= 0.) return start_;
    if (s >= length_) return end_;
    const double angle = s/radius_, half_sine = std::sin(angle/2.);
    // Local coordinates avoid subtracting a large circle centre at shallow bends.
    return start_+radius_*(std::sin(angle)*tangent_+2.*half_sine*half_sine*normal_);
  }
  Eigen::VectorXd getTangent(double s) const override
  {
    const double angle = s/radius_;
    return std::cos(angle)*tangent_+std::sin(angle)*normal_;
  }
  Eigen::VectorXd getCurvature(double s) const override
  {
    const double angle = s/radius_;
    return (-std::sin(angle)*tangent_+std::cos(angle)*normal_)/radius_;
  }
  std::list<double> getSwitchingPoints() const override
  {
    std::list<double> points;
    for (Eigen::Index i=0; i<tangent_.size(); ++i)
    {
      double angle = std::atan2(-tangent_[i], normal_[i]);
      if (angle <= 0.) angle += M_PI;
      const double position = angle*radius_;
      if (position > 0. && position < length_) points.push_back(position);
    }
    points.sort();
    return points;
  }
  CircularPathSegment* clone() const override { return new CircularPathSegment(*this); }

private:
  Eigen::VectorXd start_, end_, tangent_, normal_;
  double radius_;
};

Path::Path(const std::list<Eigen::VectorXd>& path, double max_deviation) : length_(0.0)
{
  if (path.size() < 2)
    return;
  std::list<Eigen::VectorXd>::const_iterator path_iterator1 = path.begin();
  std::list<Eigen::VectorXd>::const_iterator path_iterator2 = path_iterator1;
  ++path_iterator2;
  std::list<Eigen::VectorXd>::const_iterator path_iterator3;
  Eigen::VectorXd start_config = *path_iterator1;
  while (path_iterator2 != path.end())
  {
    path_iterator3 = path_iterator2;
    ++path_iterator3;
    if (max_deviation > 0.0 && path_iterator3 != path.end())
    {
      CircularPathSegment* blend_segment =
          new CircularPathSegment(0.5 * (*path_iterator1 + *path_iterator2), *path_iterator2,
                                  0.5 * (*path_iterator2 + *path_iterator3), max_deviation);
      Eigen::VectorXd end_config = blend_segment->getConfig(0.0);
      if ((end_config - start_config).norm() > 0.)
      {
        path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, end_config));
      }
      start_config = blend_segment->getConfig(blend_segment->getLength());
      if (blend_segment->getLength() > 0.) path_segments_.emplace_back(blend_segment);
      else delete blend_segment;
    }
    else
    {
      if ((*path_iterator2-start_config).norm() > 0.)
        path_segments_.push_back(std::make_unique<LinearPathSegment>(start_config, *path_iterator2));
      start_config = *path_iterator2;
    }
    path_iterator1 = path_iterator2;
    ++path_iterator2;
  }

  // Create list of switching point candidates, calculate total path length and
  // absolute positions of path segments
  for (std::unique_ptr<PathSegment>& path_segment : path_segments_)
  {
    path_segment->position_ = length_;
    std::list<double> local_switching_points = path_segment->getSwitchingPoints();
    for (std::list<double>::const_iterator point = local_switching_points.begin();
         point != local_switching_points.end(); ++point)
    {
      switching_points_.push_back(std::make_pair(length_ + *point, false));
    }
    length_ += path_segment->getLength();
    while (!switching_points_.empty() && switching_points_.back().first >= length_)
      switching_points_.pop_back();
    switching_points_.push_back(std::make_pair(length_, true));
  }
  if (!switching_points_.empty()) switching_points_.pop_back();
}

Path::Path(const Path& path) : length_(path.length_), switching_points_(path.switching_points_)
{
  for (const std::unique_ptr<PathSegment>& path_segment : path.path_segments_)
  {
    path_segments_.emplace_back(path_segment->clone());
  }
}

double Path::getLength() const
{
  return length_;
}

PathSegment* Path::getPathSegment(double& s) const
{
  std::list<std::unique_ptr<PathSegment>>::const_iterator it = path_segments_.begin();
  std::list<std::unique_ptr<PathSegment>>::const_iterator next = it;
  ++next;
  while (next != path_segments_.end() && s >= (*next)->position_)
  {
    it = next;
    ++next;
  }
  s -= (*it)->position_;
  return (*it).get();
}

Eigen::VectorXd Path::getConfig(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getConfig(s);
}

Eigen::VectorXd Path::getTangent(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getTangent(s);
}

Eigen::VectorXd Path::getCurvature(double s) const
{
  const PathSegment* path_segment = getPathSegment(s);
  return path_segment->getCurvature(s);
}

double Path::getNextSwitchingPoint(double s, bool& discontinuity) const
{
  std::list<std::pair<double, bool>>::const_iterator it = switching_points_.begin();
  while (it != switching_points_.end() && it->first <= s)
  {
    ++it;
  }
  if (it == switching_points_.end())
  {
    discontinuity = true;
    return length_;
  }
  discontinuity = it->second;
  return it->first;
}

std::vector<double> Path::getBlendMidpoints() const
{
  std::vector<double> positions;
  for (const auto& segment : path_segments_)
    if (segment->getCurvature(segment->getLength()/2.).squaredNorm() > 0.)
      positions.push_back(segment->position_+segment->getLength()/2.);
  return positions;
}

const std::list<std::pair<double, bool>>& Path::getSwitchingPoints() const
{
  return switching_points_;
}

Trajectory::Trajectory(const Path& path, const Eigen::VectorXd& max_velocity, const Eigen::VectorXd& max_acceleration,
                       double time_step)
  : path_(path)
  , max_velocity_(max_velocity)
  , max_acceleration_(max_acceleration)
  , joint_num_(max_velocity.size())
  , valid_(true)
  , time_step_(time_step)
  , cached_time_(std::numeric_limits<double>::max())
{
  // A fixed 1e-6 difference crosses tiny blends at near-reversal corners.
  // Keep each finite-difference stencil within the local switching intervals,
  // while retaining a roundoff floor at the path's coordinate scale.
  epsilon_ = 1e-6;
  double previous_switch = 0.;
  for (const auto& point : path_.getSwitchingPoints())
  {
    const double interval = point.first - previous_switch;
    if (interval > 0.) epsilon_ = std::min(epsilon_, interval * 1e-3);
    previous_switch = point.first;
  }
  epsilon_ = std::max(epsilon_, 64. * std::numeric_limits<double>::epsilon() * path_.getLength());
  if (time_step_ == 0)
  {
    valid_ = false;
    RCLCPP_ERROR(LOGGER, "The trajectory is invalid because the time step is 0.");
    return;
  }
  trajectory_.push_back(TrajectoryStep(0.0, 0.0));
  double after_acceleration = getMinMaxPathAcceleration(0.0, 0.0, true);
  while (valid_ && !integrateForward(trajectory_, after_acceleration) && valid_)
  {
    double before_acceleration;
    TrajectoryStep switching_point;
    if (getNextSwitchingPoint(trajectory_.back().path_pos_, switching_point, before_acceleration, after_acceleration))
    {
      break;
    }
    integrateBackward(trajectory_, switching_point.path_pos_, switching_point.path_vel_, before_acceleration);
  }

  if (valid_)
  {
    double before_acceleration = getMinMaxPathAcceleration(path_.getLength(), 0.0, false);
    integrateBackward(trajectory_, path_.getLength(), 0.0, before_acceleration);
  }

  if (valid_)
  {
    // Calculate timing
    std::list<TrajectoryStep>::iterator previous = trajectory_.begin();
    std::list<TrajectoryStep>::iterator it = previous;
    it->time_ = 0.0;
    ++it;
    while (it != trajectory_.end())
    {
      it->time_ =
          previous->time_ + (it->path_pos_ - previous->path_pos_) / ((it->path_vel_ + previous->path_vel_) / 2.0);
      previous = it;
      ++it;
    }
  }
}

Trajectory::~Trajectory()
{
}

// Returns true if end of path is reached.
bool Trajectory::getNextSwitchingPoint(double path_pos, TrajectoryStep& next_switching_point,
                                       double& before_acceleration, double& after_acceleration)
{
  TrajectoryStep acceleration_switching_point(path_pos, 0.0);
  double acceleration_before_acceleration, acceleration_after_acceleration;
  bool acceleration_reached_end;
  do
  {
    acceleration_reached_end =
        getNextAccelerationSwitchingPoint(acceleration_switching_point.path_pos_, acceleration_switching_point,
                                          acceleration_before_acceleration, acceleration_after_acceleration);
  } while (!acceleration_reached_end &&
           acceleration_switching_point.path_vel_ > getVelocityMaxPathVelocity(acceleration_switching_point.path_pos_));

  TrajectoryStep velocity_switching_point(path_pos, 0.0);
  double velocity_before_acceleration, velocity_after_acceleration;
  bool velocity_reached_end;
  do
  {
    velocity_reached_end = getNextVelocitySwitchingPoint(velocity_switching_point.path_pos_, velocity_switching_point,
                                                         velocity_before_acceleration, velocity_after_acceleration);
  } while (
      !velocity_reached_end && velocity_switching_point.path_pos_ <= acceleration_switching_point.path_pos_ &&
      (velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(velocity_switching_point.path_pos_ - epsilon_) ||
       velocity_switching_point.path_vel_ > getAccelerationMaxPathVelocity(velocity_switching_point.path_pos_ + epsilon_)));

  if (acceleration_reached_end && velocity_reached_end)
  {
    return true;
  }
  else if (!acceleration_reached_end &&
           (velocity_reached_end || acceleration_switching_point.path_pos_ <= velocity_switching_point.path_pos_))
  {
    next_switching_point = acceleration_switching_point;
    before_acceleration = acceleration_before_acceleration;
    after_acceleration = acceleration_after_acceleration;
    return false;
  }
  else
  {
    next_switching_point = velocity_switching_point;
    before_acceleration = velocity_before_acceleration;
    after_acceleration = velocity_after_acceleration;
    return false;
  }
}

bool Trajectory::getNextAccelerationSwitchingPoint(double path_pos, TrajectoryStep& next_switching_point,
                                                   double& before_acceleration, double& after_acceleration)
{
  double switching_path_pos = path_pos;
  double switching_path_vel;
  while (true)
  {
    bool discontinuity;
    switching_path_pos = path_.getNextSwitchingPoint(switching_path_pos, discontinuity);

    if (switching_path_pos > path_.getLength() - epsilon_)
    {
      return true;
    }

    if (discontinuity)
    {
      const double before_path_vel = getAccelerationMaxPathVelocity(switching_path_pos - epsilon_);
      const double after_path_vel = getAccelerationMaxPathVelocity(switching_path_pos + epsilon_);
      switching_path_vel = std::min(before_path_vel, after_path_vel);
      before_acceleration = getMinMaxPathAcceleration(switching_path_pos - epsilon_, switching_path_vel, false);
      after_acceleration = getMinMaxPathAcceleration(switching_path_pos + epsilon_, switching_path_vel, true);

      if ((before_path_vel > after_path_vel ||
           getMinMaxPhaseSlope(switching_path_pos - epsilon_, switching_path_vel, false) >
               getAccelerationMaxPathVelocityDeriv(switching_path_pos - 2.0 * epsilon_)) &&
          (before_path_vel < after_path_vel || getMinMaxPhaseSlope(switching_path_pos + epsilon_, switching_path_vel, true) <
                                                   getAccelerationMaxPathVelocityDeriv(switching_path_pos + 2.0 * epsilon_)))
      {
        break;
      }
    }
    else
    {
      switching_path_vel = getAccelerationMaxPathVelocity(switching_path_pos);
      before_acceleration = 0.0;
      after_acceleration = 0.0;

      if (getAccelerationMaxPathVelocityDeriv(switching_path_pos - epsilon_) < 0.0 &&
          getAccelerationMaxPathVelocityDeriv(switching_path_pos + epsilon_) > 0.0)
      {
        break;
      }
    }
  }

  next_switching_point = TrajectoryStep(switching_path_pos, switching_path_vel);
  return false;
}

bool Trajectory::getNextVelocitySwitchingPoint(double path_pos, TrajectoryStep& next_switching_point,
                                               double& before_acceleration, double& after_acceleration)
{
  bool start = false;
  path_pos -= DEFAULT_TIMESTEP;
  do
  {
    path_pos += DEFAULT_TIMESTEP;

    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >=
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      start = true;
    }
  } while ((!start || getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
                          getVelocityMaxPathVelocityDeriv(path_pos)) &&
           path_pos < path_.getLength());

  if (path_pos >= path_.getLength())
  {
    return true;  // end of trajectory reached
  }

  double before_path_pos = path_pos - DEFAULT_TIMESTEP;
  double after_path_pos = path_pos;
  while (after_path_pos - before_path_pos > epsilon_)
  {
    path_pos = (before_path_pos + after_path_pos) / 2.0;
    if (getMinMaxPhaseSlope(path_pos, getVelocityMaxPathVelocity(path_pos), false) >
        getVelocityMaxPathVelocityDeriv(path_pos))
    {
      before_path_pos = path_pos;
    }
    else
    {
      after_path_pos = path_pos;
    }
  }

  before_acceleration = getMinMaxPathAcceleration(before_path_pos, getVelocityMaxPathVelocity(before_path_pos), false);
  after_acceleration = getMinMaxPathAcceleration(after_path_pos, getVelocityMaxPathVelocity(after_path_pos), true);
  next_switching_point = TrajectoryStep(after_path_pos, getVelocityMaxPathVelocity(after_path_pos));
  return false;
}

double Trajectory::integrationStep(double path_pos, double path_vel, double acceleration, bool forward) const
{
  const double direction = forward ? 1. : -1.;
  const double query = std::nextafter(path_pos, direction * std::numeric_limits<double>::infinity());
  const double curvature = path_.getCurvature(query).norm();
  double step = time_step_;
  if (curvature > 0.)
  {
    // Resolve the turn locally instead of taking a millisecond step across it.
    const double rate = std::max(path_vel, std::sqrt(std::abs(acceleration)/curvature));
    if (rate > 0.) step = std::min(step, 0.01/(curvature*rate));
  }
  if (direction*acceleration < 0. && path_vel > 0.)
    step = std::min(step, 0.5*path_vel/std::abs(acceleration));
  double distance = forward ? path_.getLength()-path_pos : path_pos;
  for (const auto& point : path_.getSwitchingPoints())
    if (point.second && direction*(point.first-path_pos) > epsilon_)
      distance = std::min(distance, direction*(point.first-path_pos));
  const double squared_velocity = path_vel*path_vel + 2.*direction*acceleration*distance;
  if (distance > epsilon_ && squared_velocity >= 0.)
  {
    const double sum = path_vel + std::sqrt(squared_velocity);
    if (sum > 0.) step = std::min(step, 2.*distance/sum);
  }
  return step;
}

// Returns true if end of path is reached
bool Trajectory::integrateForward(std::list<TrajectoryStep>& trajectory, double acceleration)
{
  double path_pos = trajectory.back().path_pos_;
  double path_vel = trajectory.back().path_vel_;

  std::list<std::pair<double, bool>> switching_points = path_.getSwitchingPoints();
  std::list<std::pair<double, bool>>::iterator next_discontinuity = switching_points.begin();

  while (true)
  {
    while ((next_discontinuity != switching_points.end()) &&
           (next_discontinuity->first <= path_pos || !next_discontinuity->second))
    {
      ++next_discontinuity;
    }

    double old_path_pos = path_pos;
    double old_path_vel = path_vel;

    const double step = integrationStep(path_pos, path_vel, acceleration, true);
    path_vel += step * acceleration;
    path_pos += step * 0.5 * (old_path_vel + path_vel);
    if (!std::isfinite(path_vel) || !std::isfinite(path_pos) || path_pos <= old_path_pos)
    {
      valid_ = false;
      RCLCPP_ERROR(LOGGER, "Forward integration cannot make finite path progress");
      return true;
    }

    if (next_discontinuity != switching_points.end() && path_pos > next_discontinuity->first)
    {
      // Constant acceleration is linear in squared speed over path distance.
      path_vel = std::sqrt(std::max(0., old_path_vel*old_path_vel +
                         2.*acceleration*(next_discontinuity->first-old_path_pos)));
      path_pos = next_discontinuity->first;
    }

    if (path_pos > path_.getLength())
    {
      trajectory.push_back(TrajectoryStep(path_pos, path_vel));
      return true;
    }
    else if (path_vel < 0.0)
    {
      valid_ = false;
      RCLCPP_ERROR(LOGGER, "Error while integrating forward: Negative path velocity");
      return true;
    }

    if (path_vel > getVelocityMaxPathVelocity(path_pos) &&
        getMinMaxPhaseSlope(old_path_pos, getVelocityMaxPathVelocity(old_path_pos), false) <=
            getVelocityMaxPathVelocityDeriv(old_path_pos))
    {
      path_vel = getVelocityMaxPathVelocity(path_pos);
    }

    trajectory.push_back(TrajectoryStep(path_pos, path_vel));
    acceleration = getMinMaxPathAcceleration(path_pos, path_vel, true);

    if (path_vel == 0 && acceleration == 0)
    {
      // The position will never change if velocity and acceleration are zero.
      // The loop will spin indefinitely as no exit condition is met.
      valid_ = false;
      RCLCPP_ERROR(LOGGER, "Error while integrating forward: zero acceleration and velocity. Are any relevant "
                           "acceleration components limited to zero?");
      return true;
    }

    if (path_vel > getAccelerationMaxPathVelocity(path_pos) || path_vel > getVelocityMaxPathVelocity(path_pos))
    {
      // Find more accurate intersection with max-velocity curve using bisection
      TrajectoryStep overshoot = trajectory.back();
      trajectory.pop_back();
      double before = trajectory.back().path_pos_;
      double before_path_vel = trajectory.back().path_vel_;
      double after = overshoot.path_pos_;
      double after_path_vel = overshoot.path_vel_;
      while (after - before > epsilon_)
      {
        const double midpoint = 0.5 * (before + after);
        double midpoint_path_vel = 0.5 * (before_path_vel + after_path_vel);

        if (midpoint_path_vel > getVelocityMaxPathVelocity(midpoint) &&
            getMinMaxPhaseSlope(before, getVelocityMaxPathVelocity(before), false) <=
                getVelocityMaxPathVelocityDeriv(before))
        {
          midpoint_path_vel = getVelocityMaxPathVelocity(midpoint);
        }

        if (midpoint_path_vel > getAccelerationMaxPathVelocity(midpoint) ||
            midpoint_path_vel > getVelocityMaxPathVelocity(midpoint))
        {
          after = midpoint;
          after_path_vel = midpoint_path_vel;
        }
        else
        {
          before = midpoint;
          before_path_vel = midpoint_path_vel;
        }
      }
      trajectory.push_back(TrajectoryStep(before, before_path_vel));

      if (getAccelerationMaxPathVelocity(after) < getVelocityMaxPathVelocity(after))
      {
        if (next_discontinuity != switching_points.end() && after > next_discontinuity->first)
        {
          return false;
        }
        else if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory.back().path_vel_, true) >
                 getAccelerationMaxPathVelocityDeriv(trajectory.back().path_pos_))
        {
          return false;
        }
      }
      else
      {
        if (getMinMaxPhaseSlope(trajectory.back().path_pos_, trajectory_.back().path_vel_, false) >
            getVelocityMaxPathVelocityDeriv(trajectory_.back().path_pos_))
        {
          return false;
        }
      }
    }
  }
}

void Trajectory::integrateBackward(std::list<TrajectoryStep>& start_trajectory, double path_pos, double path_vel,
                                   double acceleration)
{
  std::list<TrajectoryStep>::iterator start2 = start_trajectory.end();
  --start2;
  std::list<TrajectoryStep>::iterator start1 = start2;
  --start1;
  std::list<TrajectoryStep> trajectory;
  double slope;
  assert(start1->path_pos_ <= path_pos);

  while (start1 != start_trajectory.begin() || path_pos >= 0.0)
  {
    if (start1->path_pos_ <= path_pos)
    {
      trajectory.push_front(TrajectoryStep(path_pos, path_vel));
      const double step = integrationStep(path_pos, path_vel, acceleration, false);
      path_vel -= step * acceleration;
      path_pos -= step * 0.5 * (path_vel + trajectory.front().path_vel_);
      if (!std::isfinite(path_vel) || !std::isfinite(path_pos) || path_pos >= trajectory.front().path_pos_)
      {
        valid_ = false;
        RCLCPP_ERROR(LOGGER, "Backward integration cannot make finite path progress");
        return;
      }
      acceleration = getMinMaxPathAcceleration(
          std::nextafter(path_pos, -std::numeric_limits<double>::infinity()), path_vel, false);
      slope = (trajectory.front().path_vel_ - path_vel) / (trajectory.front().path_pos_ - path_pos);

      if (path_vel < 0.0)
      {
        valid_ = false;
        RCLCPP_ERROR(LOGGER, "Error while integrating backward: Negative path velocity");
        end_trajectory_ = trajectory;
        return;
      }
    }
    else
    {
      --start1;
      --start2;
    }

    // Check for intersection between current start trajectory and backward
    // trajectory segments
    const double start_slope = (start2->path_vel_ - start1->path_vel_) / (start2->path_pos_ - start1->path_pos_);
    const double intersection_path_pos =
        (start1->path_vel_ - path_vel + slope * path_pos - start_slope * start1->path_pos_) / (slope - start_slope);
    if (std::max(start1->path_pos_, path_pos) - epsilon_ <= intersection_path_pos &&
        intersection_path_pos <= epsilon_ + std::min(start2->path_pos_, trajectory.front().path_pos_))
    {
      const double intersection_path_vel =
          start1->path_vel_ + start_slope * (intersection_path_pos - start1->path_pos_);
      start_trajectory.erase(start2, start_trajectory.end());
      start_trajectory.push_back(TrajectoryStep(intersection_path_pos, intersection_path_vel));
      start_trajectory.splice(start_trajectory.end(), trajectory);
      return;
    }
  }

  valid_ = false;
  RCLCPP_ERROR(LOGGER, "Error while integrating backward: Did not hit start trajectory");
  end_trajectory_ = trajectory;
}

double Trajectory::getMinMaxPathAcceleration(double path_pos, double path_vel, bool max)
{
  Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  double factor = max ? 1.0 : -1.0;
  double max_path_acceleration = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (config_deriv[i] != 0.0)
    {
      max_path_acceleration =
          std::min(max_path_acceleration, max_acceleration_[i] / std::abs(config_deriv[i]) -
                                              factor * config_deriv2[i] * path_vel * path_vel / config_deriv[i]);
    }
  }
  return factor * max_path_acceleration;
}

double Trajectory::getMinMaxPhaseSlope(double path_pos, double path_vel, bool max)
{
  return getMinMaxPathAcceleration(path_pos, path_vel, max) / path_vel;
}

double Trajectory::getAccelerationMaxPathVelocity(double path_pos) const
{
  double max_path_velocity = std::numeric_limits<double>::infinity();
  const Eigen::VectorXd config_deriv = path_.getTangent(path_pos);
  const Eigen::VectorXd config_deriv2 = path_.getCurvature(path_pos);
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    if (config_deriv[i] != 0.0)
    {
      for (unsigned int j = i + 1; j < joint_num_; ++j)
      {
        if (config_deriv[j] != 0.0)
        {
          double a_ij = config_deriv2[i] / config_deriv[i] - config_deriv2[j] / config_deriv[j];
          if (a_ij != 0.0)
          {
            max_path_velocity = std::min(max_path_velocity, sqrt((max_acceleration_[i] / std::abs(config_deriv[i]) +
                                                                  max_acceleration_[j] / std::abs(config_deriv[j])) /
                                                                 std::abs(a_ij)));
          }
        }
      }
    }
    else if (config_deriv2[i] != 0.0)
    {
      max_path_velocity = std::min(max_path_velocity, sqrt(max_acceleration_[i] / std::abs(config_deriv2[i])));
    }
  }
  return max_path_velocity;
}

double Trajectory::getVelocityMaxPathVelocity(double path_pos) const
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    max_path_velocity = std::min(max_path_velocity, max_velocity_[i] / std::abs(tangent[i]));
  }
  return max_path_velocity;
}

double Trajectory::getAccelerationMaxPathVelocityDeriv(double path_pos)
{
  return (getAccelerationMaxPathVelocity(path_pos + epsilon_) - getAccelerationMaxPathVelocity(path_pos - epsilon_)) /
         (2.0 * epsilon_);
}

double Trajectory::getVelocityMaxPathVelocityDeriv(double path_pos)
{
  const Eigen::VectorXd tangent = path_.getTangent(path_pos);
  double max_path_velocity = std::numeric_limits<double>::max();
  unsigned int active_constraint = 0;
  for (unsigned int i = 0; i < joint_num_; ++i)
  {
    const double this_max_path_velocity = max_velocity_[i] / std::abs(tangent[i]);
    if (this_max_path_velocity < max_path_velocity)
    {
      max_path_velocity = this_max_path_velocity;
      active_constraint = i;
    }
  }
  return -(max_velocity_[active_constraint] * path_.getCurvature(path_pos)[active_constraint]) /
         (tangent[active_constraint] * std::abs(tangent[active_constraint]));
}

bool Trajectory::isValid() const
{
  return valid_;
}

double Trajectory::getDuration() const
{
  return trajectory_.back().time_;
}

std::list<Trajectory::TrajectoryStep>::const_iterator Trajectory::getTrajectorySegment(double time) const
{
  if (time >= trajectory_.back().time_)
  {
    std::list<TrajectoryStep>::const_iterator last = trajectory_.end();
    last--;
    return last;
  }
  else
  {
    if (time < cached_time_)
    {
      cached_trajectory_segment_ = trajectory_.begin();
    }
    while (time >= cached_trajectory_segment_->time_)
    {
      ++cached_trajectory_segment_;
    }
    cached_time_ = time;
    return cached_trajectory_segment_;
  }
}

Eigen::VectorXd Trajectory::getPosition(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 0.5 * time_step * time_step * acceleration;

  return path_.getConfig(path_pos);
}

Eigen::VectorXd Trajectory::getVelocity(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;

  return path_.getTangent(path_pos) * path_vel;
}

Eigen::VectorXd Trajectory::getAcceleration(double time) const
{
  std::list<TrajectoryStep>::const_iterator it = getTrajectorySegment(time);
  std::list<TrajectoryStep>::const_iterator previous = it;
  previous--;

  double time_step = it->time_ - previous->time_;
  const double acceleration =
      2.0 * (it->path_pos_ - previous->path_pos_ - time_step * previous->path_vel_) / (time_step * time_step);

  time_step = time - previous->time_;
  const double path_pos =
      previous->path_pos_ + time_step * previous->path_vel_ + 0.5 * time_step * time_step * acceleration;
  const double path_vel = previous->path_vel_ + time_step * acceleration;
  return path_.getCurvature(path_pos)*path_vel*path_vel+path_.getTangent(path_pos)*acceleration;
}

std::vector<double> Trajectory::getPathEventTimes() const
{
  std::vector<double> times;
  auto next = std::next(trajectory_.begin());
  for (const double position : path_.getBlendMidpoints())
  {
    while (next != trajectory_.end() && next->path_pos_ < position) ++next;
    if (next == trajectory_.end()) break;
    const auto previous = std::prev(next);
    const double dt = next->time_-previous->time_;
    const double acceleration = (next->path_vel_-previous->path_vel_)/dt;
    const double distance = position-previous->path_pos_;
    const double speed = std::sqrt(std::max(0., previous->path_vel_*previous->path_vel_+2.*acceleration*distance));
    const double sum = previous->path_vel_+speed;
    const double offset = sum > 0. ? 2.*distance/sum : 0.;
    times.push_back(previous->time_+std::clamp(offset, 0., dt));
  }
  return times;
}

TimeOptimalTrajectoryGeneration::TimeOptimalTrajectoryGeneration(const double path_tolerance, const double resample_dt,
                                                                 const double min_angle_change)
  : path_tolerance_(path_tolerance), resample_dt_(resample_dt), min_angle_change_(min_angle_change)
{
}

bool TimeOptimalTrajectoryGeneration::computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
                                                        const double max_velocity_scaling_factor,
                                                        const double max_acceleration_scaling_factor) const
{
  if (trajectory.empty())
    return true;

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  // Validate scaling
  double velocity_scaling_factor = 1.0;
  if (max_velocity_scaling_factor > 0.0 && max_velocity_scaling_factor <= 1.0)
  {
    velocity_scaling_factor = max_velocity_scaling_factor;
  }
  else if (max_velocity_scaling_factor == 0.0)
  {
    RCLCPP_DEBUG(LOGGER, "A max_velocity_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                 velocity_scaling_factor);
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Invalid max_velocity_scaling_factor %f specified, defaulting to %f instead.",
                max_velocity_scaling_factor, velocity_scaling_factor);
  }

  double acceleration_scaling_factor = 1.0;
  if (max_acceleration_scaling_factor > 0.0 && max_acceleration_scaling_factor <= 1.0)
  {
    acceleration_scaling_factor = max_acceleration_scaling_factor;
  }
  else if (max_acceleration_scaling_factor == 0.0)
  {
    RCLCPP_DEBUG(LOGGER, "A max_acceleration_scaling_factor of 0.0 was specified, defaulting to %f instead.",
                 acceleration_scaling_factor);
  }
  else
  {
    RCLCPP_WARN(LOGGER, "Invalid max_acceleration_scaling_factor %f specified, defaulting to %f instead.",
                max_acceleration_scaling_factor, acceleration_scaling_factor);
  }

  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();
  const unsigned num_joints = group->getVariableCount();

  // Get the vel/accel limits
  Eigen::VectorXd max_velocity(num_joints);
  Eigen::VectorXd max_acceleration(num_joints);
  for (size_t j = 0; j < num_joints; ++j)
  {
    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars[j]);

    // Limits need to be non-zero, otherwise we never exit
    max_velocity[j] = 1.0;
    if (bounds.velocity_bounded_)
    {
      if (bounds.max_velocity_ <= 0.0)
      {
        RCLCPP_ERROR(LOGGER, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                     bounds.max_velocity_, vars[j].c_str());
        return false;
      }
      max_velocity[j] =
          std::min(std::fabs(bounds.max_velocity_), std::fabs(bounds.min_velocity_)) * velocity_scaling_factor;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(
          LOGGER, "Joint velocity limits are not defined. Using the default "
                      << max_velocity[j] << " rad/s. You can define velocity limits in the URDF or joint_limits.yaml.");
    }

    max_acceleration[j] = 1.0;
    if (bounds.acceleration_bounded_)
    {
      if (bounds.max_acceleration_ < 0.0)
      {
        RCLCPP_ERROR(LOGGER, "Invalid max_acceleration %f specified for '%s', must be greater than 0.0",
                     bounds.max_acceleration_, vars[j].c_str());
        return false;
      }
      max_acceleration[j] = std::min(std::fabs(bounds.max_acceleration_), std::fabs(bounds.min_acceleration_)) *
                            acceleration_scaling_factor;
    }
    else
    {
      RCLCPP_WARN_STREAM_ONCE(LOGGER,
                              "Joint acceleration limits are not defined. Using the default "
                                  << max_acceleration[j]
                                  << " rad/s^2. You can define acceleration limits in the URDF or joint_limits.yaml.");
    }
  }

  return doTimeParameterizationCalculations(trajectory, max_velocity, max_acceleration);
}

bool TimeOptimalTrajectoryGeneration::computeTimeStamps(
    robot_trajectory::RobotTrajectory& trajectory, const std::unordered_map<std::string, double>& velocity_limits,
    const std::unordered_map<std::string, double>& acceleration_limits) const
{
  if (trajectory.empty())
    return true;

  // Get the default joint limits from the robot model, then overwrite any that are provided as arguments
  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }
  const unsigned num_joints = group->getVariableCount();
  const std::vector<std::string>& vars = group->getVariableNames();
  const moveit::core::RobotModel& rmodel = group->getParentModel();

  Eigen::VectorXd max_velocity(num_joints);
  Eigen::VectorXd max_acceleration(num_joints);
  for (size_t j = 0; j < num_joints; ++j)
  {
    const moveit::core::VariableBounds& bounds = rmodel.getVariableBounds(vars[j]);

    // VELOCITY LIMIT
    // Check if a custom limit was specified as an argument
    bool set_velocity_limit = false;
    auto it = velocity_limits.find(vars[j]);
    if (it != velocity_limits.end())
    {
      max_velocity[j] = it->second;
      set_velocity_limit = true;
    }

    if (bounds.velocity_bounded_ && !set_velocity_limit)
    {
      // Set the default velocity limit, from robot model
      if (bounds.max_velocity_ < 0.0)
      {
        RCLCPP_ERROR(LOGGER, "Invalid max_velocity %f specified for '%s', must be greater than 0.0",
                     bounds.max_velocity_, vars[j].c_str());
        return false;
      }
      max_velocity[j] = std::min(std::fabs(bounds.max_velocity_), std::fabs(bounds.min_velocity_));
      set_velocity_limit = true;
    }

    if (!set_velocity_limit)
    {
      max_velocity[j] = 1.0;
      RCLCPP_WARN_STREAM_ONCE(
          LOGGER, "Joint velocity limits are not defined. Using the default "
                      << max_velocity[j] << " rad/s. You can define velocity limits in the URDF or joint_limits.yaml.");
    }

    // ACCELERATION LIMIT
    // Check if a custom limit was specified as an argument
    bool set_acceleration_limit = false;
    it = acceleration_limits.find(vars[j]);
    if (it != acceleration_limits.end())
    {
      max_acceleration[j] = it->second;
      set_acceleration_limit = true;
    }

    if (bounds.acceleration_bounded_ && !set_acceleration_limit)
    {
      // Set the default acceleration limit, from robot model
      if (bounds.max_acceleration_ < 0.0)
      {
        RCLCPP_ERROR(LOGGER, "Invalid max_acceleration %f specified for '%s', must be greater than 0.0",
                     bounds.max_acceleration_, vars[j].c_str());
        return false;
      }
      max_acceleration[j] = std::min(std::fabs(bounds.max_acceleration_), std::fabs(bounds.min_acceleration_));
      set_acceleration_limit = true;
    }
    if (!set_acceleration_limit)
    {
      max_acceleration[j] = 1.0;
      RCLCPP_WARN_STREAM_ONCE(LOGGER,
                              "Joint acceleration limits are not defined. Using the default "
                                  << max_acceleration[j]
                                  << " rad/s^2. You can define acceleration limits in the URDF or joint_limits.yaml.");
    }
  }

  return doTimeParameterizationCalculations(trajectory, max_velocity, max_acceleration);
}

bool TimeOptimalTrajectoryGeneration::doTimeParameterizationCalculations(robot_trajectory::RobotTrajectory& trajectory,
                                                                         const Eigen::VectorXd& max_velocity,
                                                                         const Eigen::VectorXd& max_acceleration) const
{
  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  const unsigned num_points = trajectory.getWayPointCount();
  const std::vector<int>& idx = group->getVariableIndexList();
  const unsigned num_joints = group->getVariableCount();

  // Have to convert into Eigen data structs and remove repeated points
  //  (https://github.com/tobiaskunz/trajectories/issues/3)
  std::list<Eigen::VectorXd> points;
  for (size_t p = 0; p < num_points; ++p)
  {
    moveit::core::RobotStatePtr waypoint = trajectory.getWayPointPtr(p);
    Eigen::VectorXd new_point(num_joints);
    // The first point should always be kept
    bool diverse_point = (p == 0);

    for (size_t j = 0; j < num_joints; ++j)
    {
      new_point[j] = waypoint->getVariablePosition(idx[j]);
      // If any joint angle is different, it's a unique waypoint
      if (p > 0 && std::fabs(new_point[j] - points.back()[j]) > min_angle_change_)
      {
        diverse_point = true;
      }
    }

    if (diverse_point)
      points.push_back(new_point);
    // If the last point is not a diverse_point we replace the last added point with it to make sure to always have the
    // input end point as the last point
    else if (p == num_points - 1)
      points.back() = new_point;
  }

  // Return trajectory with only the first waypoint if there are not multiple diverse points
  if (points.size() == 1)
  {
    moveit::core::RobotState waypoint = moveit::core::RobotState(trajectory.getWayPoint(0));
    waypoint.zeroVelocities();
    waypoint.zeroAccelerations();
    trajectory.clear();
    trajectory.addSuffixWayPoint(waypoint, 0.0);
    return true;
  }

  const std::vector<Eigen::VectorXd> knots(points.begin(), points.end());
  std::vector<std::size_t> stops{0};
  for (std::size_t i=1; i+1<knots.size(); ++i)
  {
    const Eigen::VectorXd incoming = (knots[i]-knots[i-1]).normalized();
    const Eigen::VectorXd outgoing = (knots[i+1]-knots[i]).normalized();
    const double cosine = std::clamp(incoming.dot(outgoing), -1., 1.);
    const double sine = (outgoing-cosine*incoming).norm();
    // A tangent discontinuity requires zero path speed for finite joint
    // acceleration. Solve each smooth section with that exact boundary condition.
    if ((cosine < 0. && sine <= 64.*std::numeric_limits<double>::epsilon()) ||
        (path_tolerance_ == 0. && (incoming-outgoing).norm() > 64.*std::numeric_limits<double>::epsilon()))
      stops.push_back(i);
  }
  stops.push_back(knots.size()-1);

  struct Sample { double time; Eigen::VectorXd position, velocity, acceleration; };
  std::vector<Sample> output;
  const auto& names = group->getVariableNames();
  const double roundoff = 128.*std::numeric_limits<double>::epsilon()*
      std::max(1., std::accumulate(knots.begin(), knots.end(), 0.,
        [](double extent, const auto& point) { return std::max(extent, point.cwiseAbs().maxCoeff()); }));
  auto certified = [&](const Sample& first, const Sample& last) {
    const double dt = last.time-first.time;
    const double uncertainty=std::min(dt/8.,2.e-9), low=dt-uncertainty, high=dt+uncertainty;
    const Eigen::VectorXd zero=Eigen::VectorXd::Zero(num_joints);
    std::vector<Eigen::VectorXd> control;
    auto enclose = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& a) {
      // Bernstein hull in the uncertain interval duration, enclosing each
      // quintic control even after ROS rounds both endpoint timestamps.
      const Eigen::VectorXd begin=q+v*low+a*low*low;
      control.push_back(begin);
      control.push_back(begin+(high-low)/2.*(v+2.*a*low));
      control.push_back(q+v*high+a*high*high);
    };
    enclose(first.position,zero,zero);
    enclose(first.position,first.velocity/5.,zero);
    enclose(first.position,first.velocity*2./5.,first.acceleration/20.);
    enclose(last.position,-last.velocity*2./5.,last.acceleration/20.);
    enclose(last.position,-last.velocity/5.,zero);
    enclose(last.position,zero,zero);
    for (const auto& point : control)
      for (std::size_t joint=0; joint<names.size(); ++joint)
      {
        const auto& bounds = trajectory.getRobotModel()->getVariableBounds(names[joint]);
        if (!point.allFinite() || (bounds.position_bounded_ &&
            (point[joint] < bounds.min_position_-roundoff || point[joint] > bounds.max_position_+roundoff)))
          return false;
      }
    // Each capsule is convex: enclosing all Bernstein controls encloses the
    // entire quintic, not just sampled positions. Use one original segment.
    for (std::size_t i=1; i<knots.size(); ++i)
    {
      const Eigen::VectorXd delta = knots[i]-knots[i-1];
      const double squared = delta.squaredNorm();
      bool inside = true;
      for (const auto& point : control)
      {
        const double u = std::clamp((point-knots[i-1]).dot(delta)/squared, 0., 1.);
        if ((point-knots[i-1]-u*delta).norm() > path_tolerance_+roundoff) { inside=false; break; }
      }
      if (inside) return true;
    }
    return false;
  };

  double elapsed = 0.;
  for (std::size_t section=1; section<stops.size(); ++section)
  {
    std::list<Eigen::VectorXd> section_points(knots.begin()+stops[section-1], knots.begin()+stops[section]+1);
    // Reserve half the geometric budget for representing the internal curve
    // with the controller's quintic interpolation.
    Trajectory parameterized(Path(section_points, path_tolerance_/2.), max_velocity, max_acceleration, DEFAULT_TIMESTEP);
    if (!parameterized.isValid() || !std::isfinite(parameterized.getDuration()) || parameterized.getDuration() <= 0.)
    {
      RCLCPP_ERROR(LOGGER, "Unable to parameterize trajectory.");
      return false;
    }
    const double duration = parameterized.getDuration();
    auto sample = [&](double time) -> Sample {
      if (time == 0.) return {time, section_points.front(), Eigen::VectorXd::Zero(num_joints), Eigen::VectorXd::Zero(num_joints)};
      if (time == duration) return {time, section_points.back(), Eigen::VectorXd::Zero(num_joints), Eigen::VectorXd::Zero(num_joints)};
      return {time, parameterized.getPosition(time), parameterized.getVelocity(time), parameterized.getAcceleration(time)};
    };
    std::vector<double> anchors = parameterized.getPathEventTimes();
    anchors.push_back(0.); anchors.push_back(duration);
    std::sort(anchors.begin(), anchors.end());
    anchors.erase(std::unique(anchors.begin(), anchors.end(), [duration](double a, double b) {
      return std::abs(a-b) <= 64.*std::numeric_limits<double>::epsilon()*duration;
    }), anchors.end());
    anchors.front() = 0.; anchors.back() = duration;
    // Subdivide between geometric anchors instead of overlaying two grids.
    // Almost coincident event/grid samples amplify timestamp quantization.
    std::vector<double> times{0.};
    for (std::size_t i=1; i<anchors.size(); ++i)
    {
      const double interval=anchors[i]-anchors[i-1];
      const std::size_t count=std::max<std::size_t>(1,std::ceil(interval/resample_dt_));
      for (std::size_t j=1; j<=count; ++j)
        times.push_back(anchors[i-1]+interval*(double(j)/count));
      times.back()=anchors[i];
    }
    std::vector<Sample> samples{sample(0.)};
    std::function<bool(const Sample&, const Sample&, unsigned)> refine;
    refine = [&](const Sample& first, const Sample& last, unsigned depth) {
      if (certified(first,last)) { samples.push_back(last); return true; }
      const double middle = first.time+(last.time-first.time)/2.;
      if (depth >= 40 || middle == first.time || middle == last.time) return false;
      const Sample midpoint = sample(middle);
      return refine(first,midpoint,depth+1) && refine(midpoint,last,depth+1);
    };
    for (std::size_t i=1; i<times.size(); ++i)
      if (!refine(samples.back(), sample(times[i]), 0))
      {
        RCLCPP_ERROR(LOGGER, "Cannot represent time-parameterized path within geometry and position bounds");
        return false;
      }
    for (std::size_t i=section==1 ? 0 : 1; i<samples.size(); ++i)
    {
      samples[i].time += elapsed;
      output.push_back(std::move(samples[i]));
    }
    elapsed += duration;
  }

  moveit::core::RobotState waypoint(trajectory.getWayPoint(0));
  trajectory.clear();
  double previous_time = 0.;
  for (const auto& sample : output)
  {
    for (std::size_t j=0; j<num_joints; ++j)
    {
      waypoint.setVariablePosition(idx[j], sample.position[j]);
      waypoint.setVariableVelocity(idx[j], sample.velocity[j]);
      waypoint.setVariableAcceleration(idx[j], sample.acceleration[j]);
    }
    trajectory.addSuffixWayPoint(waypoint, sample.time-previous_time);
    previous_time = sample.time;
  }

  return true;
}
}  // namespace cleany::totg
