#pragma once

#include <array>
#include <cstddef>

namespace cleany {
// ROS contract is FL, FR, RL, RR; the preserved driver stores FL, FR, RR, RL.
constexpr std::array<size_t, 4> kWireToMotorIndex = {0, 1, 3, 2};
}  // namespace cleany
