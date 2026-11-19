#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace cleany_navigation {
inline bool finiteLimit(double value) {
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return (bits & UINT64_C(0x7ff0000000000000)) != UINT64_C(0x7ff0000000000000);
}

/// 병진 크기와 yaw를 독립적으로 제한하며 0이면 해당 한계를 끕니다.
template<typename T>
inline void projectBodyVelocity(T & x, T & y, T & yaw, double translation, double rotation) {
  const double speed = std::hypot(x, y);
  if (translation > 0. && speed > translation) {
    x *= translation / speed;
    y *= translation / speed;
  }
  if (rotation > 0.) {yaw = std::clamp(yaw, T(-rotation), T(rotation));}
}

/// 요청 twist 곡률을 유지하는 장애물 근처의 절대 속도 상한입니다.
struct BodySpeedLimit {
  double translation, rotation;
  BodySpeedLimit(double linear, double angular) : translation(linear), rotation(angular) {
    if (!finiteLimit(linear) || !finiteLimit(angular) || linear <= 0. || angular <= 0.) {
      throw std::invalid_argument("Body speed limits must be finite and positive");
    }
  }
  double ratio(double x, double y, double yaw) const {
    if (!finiteLimit(x) || !finiteLimit(y) || !finiteLimit(yaw)) {return 0.;}
    const double speed = std::hypot(x, y);
    double scale = 1.;
    if (speed > translation) {scale = translation / speed;}
    if (std::abs(yaw) > rotation) {scale = std::min(scale, rotation / std::abs(yaw));}
    return scale;
  }
};

/// 남은 거리, 일정 감속도와 반응 시간으로 속도를 제한합니다.
inline double approachingSpeed(double maximum, double near, double deceleration,
  double distance, double reaction) {
  const double delayed = deceleration * reaction;
  return std::min(maximum, std::max(near,
    std::sqrt(near * near + delayed * delayed +
    2. * deceleration * std::max(0., distance)) - delayed));
}
}
