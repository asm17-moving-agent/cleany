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

/// Limit translation magnitude and yaw independently; zero disables a bound.
template<typename T>
inline void projectBodyVelocity(T & x, T & y, T & yaw, double translation, double rotation) {
  const double speed = std::hypot(x, y);
  if (translation > 0. && speed > translation) {
    x *= translation / speed;
    y *= translation / speed;
  }
  if (rotation > 0.) {yaw = std::clamp(yaw, T(-rotation), T(rotation));}
}

/// An absolute near-obstacle cap that preserves the requested twist curvature.
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

/// Cap speed using remaining distance, constant deceleration and reaction time.
inline double approachingSpeed(double maximum, double near, double deceleration,
  double distance, double reaction) {
  const double delayed = deceleration * reaction;
  return std::min(maximum, std::max(near,
    std::sqrt(near * near + delayed * delayed +
    2. * deceleration * std::max(0., distance)) - delayed));
}
}
