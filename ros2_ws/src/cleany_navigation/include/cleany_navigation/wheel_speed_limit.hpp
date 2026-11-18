#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cleany_navigation {
struct WheelSpeedLimit {
  double radius;
  double wheelbase;
  double separation;
  double maximum;

  WheelSpeedLimit(double r, double b, double s, double m)
  : radius(r), wheelbase(b), separation(s), maximum(m) {
    if (!std::isfinite(r) || !std::isfinite(b) || !std::isfinite(s) ||
      !std::isfinite(m) || r <= 0 || b <= 0 || s <= 0 || m <= 0 || m > 10) {
      throw std::invalid_argument("Invalid wheel limit or mecanum geometry");
    }
  }

  /// Preserve the requested twist direction and never amplify a small command.
  double ratio(double x, double y, double yaw) const {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {return 0.;}
    const double peak = (std::abs(x) + std::abs(y) +
      (wheelbase + separation) * .5 * std::abs(yaw)) / radius;
    return peak > maximum ? maximum / peak : 1.;
  }
};
}
