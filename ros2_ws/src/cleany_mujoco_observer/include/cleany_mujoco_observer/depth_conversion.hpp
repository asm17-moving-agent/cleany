#pragma once

#include <cstddef>

namespace cleany_mujoco_observer {

/**
 * @brief 정규화된 렌더러 깊이를 카메라 축 방향의 실제 거리로 변환합니다.
 * 원본과 출력 버퍼에는 최소 width개의 float가 있어야 합니다. 절단면은
 * 0 < near < far를 만족하며 원하는 출력 거리 단위를 사용해야 합니다.
 */
inline void convert_depth_row(const float* source, float* destination,
                              std::size_t width, float near, float far) {
  const float ratio = 1.0f - near / far;
  for (std::size_t x = 0; x < width; ++x) {
    destination[x] = near / (1.0f - source[x] * ratio);
  }
}

}
