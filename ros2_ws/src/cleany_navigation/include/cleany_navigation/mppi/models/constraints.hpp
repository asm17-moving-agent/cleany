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

#ifndef CLEANY_NAVIGATION_MPPI__MODELS__CONSTRAINTS_HPP_
#define CLEANY_NAVIGATION_MPPI__MODELS__CONSTRAINTS_HPP_

namespace cleany_navigation::mppi::models
{

/**
 * @struct cleany_navigation::mppi::models::ControlConstraints
 * @brief Constraints on control
 */
struct ControlConstraints
{
  float vx_max;
  float vx_min;
  float vy;
  float wz;
};

/**
 * @struct cleany_navigation::mppi::models::SamplingStd
 * @brief Noise parameters for sampling trajectories
 */
struct SamplingStd
{
  float vx;
  float vy;
  float wz;
};

}  // namespace cleany_navigation::mppi::models

#endif  // CLEANY_NAVIGATION_MPPI__MODELS__CONSTRAINTS_HPP_
