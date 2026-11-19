// SPDX-License-Identifier: Apache-2.0
#ifndef CLEANY_NAVIGATION_MPPI__CRITICS__DIRECTIONAL_MOTION_CRITIC_HPP_
#define CLEANY_NAVIGATION_MPPI__CRITICS__DIRECTIONAL_MOTION_CRITIC_HPP_

#include "cleany_navigation/mppi/critic_function.hpp"

namespace cleany_navigation::mppi::critics
{
/** @brief Omni 후보 궤적에 차체 좌표계 기준 횡이동 및 대각 이동 비용을 부과합니다.
 *
 * 이동 거리에 lateral*sin(theta)^2 + diagonal*sin(2*theta)^2를 곱해 적분합니다.
 * theta는 차체 기준 이동 방향이며 가중치는 m당 비용입니다. 각 가중치가 0이면
 * 해당 항을 끄고, 병진 이동이 없으면 비용도 0입니다.
 * 측정된 미끄러짐, 지도 위치나 복구 상태에 따라 가중치를 바꾸지 않습니다.
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
