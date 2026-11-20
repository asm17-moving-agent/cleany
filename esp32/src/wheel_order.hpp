#pragma once

#include <array>
#include <cstddef>

namespace cleany {
// ROS 순서는 FL, FR, RL, RR이며 기존 드라이버의 저장 순서는 FL, FR, RR, RL입니다.
constexpr std::array<size_t, 4> kWireToMotorIndex = {0, 1, 3, 2};
}  // namespace cleany
