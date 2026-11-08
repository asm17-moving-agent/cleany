// SPDX-License-Identifier: Apache-2.0
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include "cleany_navigation/mppi/critics/directional_motion_critic.hpp"
#include "cleany_navigation/mppi/motion_models.hpp"
#include <xtensor/xmath.hpp>
#include "cleany_navigation/mppi/tools/utils.hpp"

namespace cleany_navigation::mppi::critics
{
void DirectionalMotionCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(lateral_weight_, "lateral_weight", 0.0, ParameterType::Static);
  getParam(diagonal_weight_, "diagonal_weight", 0.0, ParameterType::Static);
  // Inspect IEEE bits because this library is compiled with fast-math.
  for (const float weight : {lateral_weight_, diagonal_weight_}) {
    uint32_t bits;
    std::memcpy(&bits, &weight, sizeof(bits));
    if ((bits & 0x7f800000u) == 0x7f800000u || weight < 0.0f) {
      throw std::invalid_argument("DirectionalMotionCritic weights must be finite and nonnegative");
    }
  }
}

void DirectionalMotionCritic::score(CriticData & data)
{
  if (!enabled_ || !data.motion_model || !data.motion_model->isHolonomic() ||
    (lateral_weight_ == 0.0f && diagonal_weight_ == 0.0f))
  {
    return;
  }
  // Once XY is reached, translation preference must not compete with final yaw.
  if (data.goal_checker && data.path.x.size() > 0 &&
    utils::withinPositionGoalTolerance(data.goal_checker, data.state.pose.pose, data.path))
  {
    return;
  }
  const auto x_squared = xt::square(data.state.vx);
  const auto y_squared = xt::square(data.state.vy);
  const auto speed_squared = xt::eval(x_squared + y_squared);
  const auto denominator = xt::maximum(speed_squared, 1e-12f);
  // Per-metre angular preference: lateral*sin(theta)^2 + diagonal*sin(2*theta)^2.
  // The clamp only regularizes near-zero translation; standstill costs zero.
  const auto lateral_fraction = y_squared / denominator;
  const auto diagonal_fraction = 4.0f * (x_squared / denominator) * lateral_fraction;
  data.costs += data.model_dt * xt::sum(
    xt::sqrt(speed_squared) *
    (lateral_weight_ * lateral_fraction + diagonal_weight_ * diagonal_fraction), {1},
    xt::evaluation_strategy::immediate);
}
}  // namespace cleany_navigation::mppi::critics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  cleany_navigation::mppi::critics::DirectionalMotionCritic,
  cleany_navigation::mppi::critics::CriticFunction)
