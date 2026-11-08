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

#ifndef CLEANY_NAVIGATION_MPPI__CRITICS__PATH_ANGLE_CRITIC_HPP_
#define CLEANY_NAVIGATION_MPPI__CRITICS__PATH_ANGLE_CRITIC_HPP_

#include "cleany_navigation/mppi/critic_function.hpp"
#include "cleany_navigation/mppi/models/state.hpp"
#include "cleany_navigation/mppi/tools/utils.hpp"

namespace cleany_navigation::mppi::critics
{

/**
 * @brief Score path alignment and optionally preserve in-place lattice turns.
 */
class PathAngleCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize() override;

  /**
   * @brief Add orientation costs to the candidate trajectories.
   */
  void score(CriticData & data) override;

protected:
  float max_angle_to_furthest_{0};
  float threshold_to_consider_{0};

  size_t offset_from_furthest_{0};
  bool reversing_allowed_{true};
  bool forward_preference_{true};
  bool respect_in_place_rotations_{false};
  bool rotation_only_{false};
  bool follow_segment_heading_{false};
  float rotation_position_tolerance_{0.075f};

  unsigned int power_{0};
  float weight_{0};
};

}  // namespace cleany_navigation::mppi::critics

#endif  // CLEANY_NAVIGATION_MPPI__CRITICS__PATH_ANGLE_CRITIC_HPP_
