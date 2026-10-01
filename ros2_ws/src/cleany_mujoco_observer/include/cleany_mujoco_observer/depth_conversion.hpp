#pragma once

#include <cstddef>

namespace cleany_mujoco_observer {

inline void convert_depth_row(const float* source, float* destination,
                              std::size_t width, float near, float far) {
  const float ratio = 1.0f - near / far;
  for (std::size_t x = 0; x < width; ++x) {
    destination[x] = near / (1.0f - source[x] * ratio);
  }
}

}
