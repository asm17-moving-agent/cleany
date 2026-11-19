// SPDX-License-Identifier: Apache-2.0
#ifndef CLEANY_NAVIGATION_MPPI__CRITICS__DIRECTIONAL_MOTION_CRITIC_HPP_
#define CLEANY_NAVIGATION_MPPI__CRITICS__DIRECTIONAL_MOTION_CRITIC_HPP_

#include "cleany_navigation/mppi/critic_function.hpp"

namespace cleany_navigation::mppi::critics
{
/** @brief Fixed body-frame lateral and diagonal travel costs for Omni rollouts.
 *
 * Integrates travel distance times lateral*sin(theta)^2 + diagonal*sin(2*theta)^2,
 * where theta is the body-frame translation angle. Weights are cost per metre;
 * zero disables each term. Zero translation has zero cost.
 * No measured slip, map location, or recovery state changes these weights.
 */
class DirectionalMotionCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  float lateral_weight_{0.0f};
  float diagonal_weight_{0.0f};
};
}  // namespace cleany_navigation::mppi::critics
#endif  // CLEANY_NAVIGATION_MPPI__CRITICS__DIRECTIONAL_MOTION_CRITIC_HPP_
