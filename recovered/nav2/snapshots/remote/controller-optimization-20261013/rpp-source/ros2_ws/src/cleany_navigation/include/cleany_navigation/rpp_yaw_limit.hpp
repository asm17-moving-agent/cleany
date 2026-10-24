#ifndef CLEANY_NAVIGATION__RPP_YAW_LIMIT_HPP_
#define CLEANY_NAVIGATION__RPP_YAW_LIMIT_HPP_
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace cleany_navigation
{
inline double rpp_yaw_scale(double angular, double limit)
{
  if (!std::isfinite(limit) || limit <= 0.0) {
    throw std::invalid_argument("tracking_yaw_limit must be finite and positive");
  }
  if (!std::isfinite(angular)) {
    throw std::invalid_argument("native RPP returned a non-finite angular velocity");
  }
  return std::abs(angular) > limit ? limit / std::abs(angular) : 1.0;
}
}  // namespace cleany_navigation
#endif  // CLEANY_NAVIGATION__RPP_YAW_LIMIT_HPP_
