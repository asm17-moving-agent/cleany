#include "cleany_axis_controller/axis_generator.hpp"
#include <cmath>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"
namespace cleany_axis_controller {
void AxisGenerator::initialize(const nav2_util::LifecycleNode::SharedPtr &node, const std::string &name) {
  StandardTrajectoryGenerator::initialize(node, name);
  clock_ = node->get_clock();
  auto parameter = [&](const std::string &key, double value) {
    const auto full = name + "." + key;
    if (!node->has_parameter(full)) node->declare_parameter(full, value);
    const auto result = node->get_parameter(full).as_double();
    if (!std::isfinite(result) || result <= 0) throw std::invalid_argument(full);
    return result;
  };
  linear_stopped_ = parameter("axis_linear_stopped", .01);
  angular_stopped_ = parameter("axis_angular_stopped", .02);
  settle_s_ = parameter("axis_settle_s", .30);
  reset();
}
void AxisGenerator::startNewIteration(const nav_2d_msgs::msg::Twist2D &current) {
  candidates_.clear(); index_ = 0;
  // A stop is always available; the inherited rollout models braking, not an instant stop.
  candidates_.emplace_back();
  const double now = clock_->now().seconds();
  if (!std::isfinite(current.x) || !std::isfinite(current.y) || !std::isfinite(current.theta)) {
    quiet_since_.reset(); return;
  }
  if (last_time_ && now < *last_time_) quiet_since_.reset();
  last_time_ = now;
  const bool moving[] = {std::abs(current.x) > linear_stopped_, std::abs(current.y) > linear_stopped_, std::abs(current.theta) > angular_stopped_};
  const int count = int(moving[0]) + int(moving[1]) + int(moving[2]);
  int permitted = -1;
  if (count > 0) {
    quiet_since_.reset();
    if (count > 1) return;  // Mixed residual motion: brake before choosing a new axis.
    for (int i=0;i<3;++i) if (moving[i]) permitted=i;
  } else {
    if (!quiet_since_) quiet_since_=now;
    if (now-*quiet_since_ < settle_s_) return;
  }
  StandardTrajectoryGenerator::startNewIteration(current);
  while (StandardTrajectoryGenerator::hasMoreTwists()) {
    auto cmd = StandardTrajectoryGenerator::nextTwist();
    double *values[] = {&cmd.x, &cmd.y, &cmd.theta};
    int active=0, axis=-1;
    for (int i=0;i<3;++i) {
      if (std::abs(*values[i]) <= 1e-9) *values[i]=0.;
      else {++active; axis=i;}
    }
    if (active == 1 && (permitted < 0 || permitted == axis)) candidates_.push_back(cmd);
  }
}
}
PLUGINLIB_EXPORT_CLASS(cleany_axis_controller::AxisGenerator, dwb_core::TrajectoryGenerator)
