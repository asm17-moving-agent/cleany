#pragma once

#include <algorithm>
#include <cmath>

namespace cleany {

// 80 MHz APB / (20 kHz * 2048) = 1.953125, an exact LEDC divider.
constexpr int kPwmBits = 11;
constexpr int kPwmPeriodTicks = 1 << kPwmBits;
constexpr int kMaximumDutyTicks = kPwmPeriodTicks * 255 / 256;
constexpr int kDutySlewTicks = (kPwmPeriodTicks + 50) / 100;

inline int dutyTicks(float percent) {
  return std::clamp(static_cast<int>(std::lround(percent * kPwmPeriodTicks / 100.0F)),
                    -kMaximumDutyTicks, kMaximumDutyTicks);
}

inline float dutyPercent(int ticks) {
  return ticks * (100.0F / kPwmPeriodTicks);
}

}  // namespace cleany
