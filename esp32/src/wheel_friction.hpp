#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "modular_int32.hpp"
#include "motor_command_filter.hpp"

namespace cleany {

struct DirectionFriction {
  float startPercent;
  std::array<float, 4> holdPercent;
};

struct WheelFriction {
  DirectionFriction forward, reverse;
};

/** Interpolate measured holding output; blend back to the original FF by 1 rad/s. */
inline float holdingFeedForward(const DirectionFriction& profile, float speed, float nominal) {
  if (speed >= 1.0F) return nominal;
  if (speed <= 0.2F) return profile.holdPercent.front() * (speed / 0.2F);
  if (speed >= 0.5F) {
    const float weight = (speed - 0.5F) / 0.5F;
    return profile.holdPercent.back() * (1.0F - weight) + nominal * weight;
  }
  const float index = (speed - 0.2F) * 10.0F;
  const int lower = std::min(2, static_cast<int>(index));
  return profile.holdPercent[lower] +
         (index - lower) * (profile.holdPercent[lower + 1] - profile.holdPercent[lower]);
}

struct StartupOutput {
  float floorPercent;
  float anglePercent;
  float integralTransferPercent;
};

/** Track cold-start angle and compensate short low-speed losses of encoder progress. */
class StartFriction {
 public:
  StartupOutput update(float target, float feedback, int32_t count, int64_t nowUs,
                       const DirectionFriction& profile) {
    const int sign = (target > 0) - (target < 0);
    if (sign != sign_) {
      sign_ = sign;
      startCount_ = count;
      progressCount_ = count;
      progressUs_ = nowUs;
      startUs_ = lastUs_ = nowUs;
      referenceRad_ = 0.0F;
      lastTarget_ = 0.0F;
      lastAnglePercent_ = 0.0F;
      floorActive_ = sign != 0 && std::fabs(feedback) < 0.1F;
      trackAngle_ = floorActive_;
    }
    const int64_t delta = modularDifference(count, startCount_);
    const int64_t ageUs = nowUs - startUs_;
    const bool lowSpeed = sign && std::fabs(target) <= 0.5F;
    if (!lowSpeed || nowUs-lastUs_ > 50000 ||
        sign * static_cast<int64_t>(modularDifference(count, progressCount_)) > 0) {
      progressCount_ = count;
      progressUs_ = nowUs;
    }
    if (sign * delta >= 3 || ageUs >= 150000) floorActive_ = false;
    if (nowUs-lastUs_ > 50000 || std::fabs(target) > 0.5F) trackAngle_ = false;
    float integralTransfer = 0.0F;
    if (trackAngle_ && ageUs >= 1000000) {
      // Preserve the last applied position-loop bias when handing off to velocity PI.
      integralTransfer = lastAnglePercent_;
      trackAngle_ = false;
    }
    float anglePercent = 0.0F;
    if (trackAngle_) {
      referenceRad_ += lastTarget_ * static_cast<float>(nowUs-lastUs_) * 1e-6F;
      const float observedRad = static_cast<float>(delta) * kTwoPi /
                                kEncoderCountsPerOutputRevolution;
      // Measured holding slopes plus velocity Kp are about 13 %/(rad/s).
      // 80 %/rad corresponds to an approximately 0.16 s angle-error time scale.
      anglePercent = std::clamp(80.0F * (referenceRad_-observedRad), -3.0F, 3.0F);
    }
    lastAnglePercent_ = anglePercent;
    lastTarget_ = target;
    lastUs_ = nowUs;
    float floor = floorActive_ ? profile.startPercent : 0.0F;
    if (lowSpeed) {
      // At RR +0.2 rad/s the same encoder phase repeatedly stuck at 8–12% duty.
      // Allow three expected encoder edges before applying a bounded breakaway pulse.
      const float edgeUs = 1e6F*kTwoPi / (kEncoderCountsPerOutputRevolution*std::fabs(target));
      const float lateUs = static_cast<float>(nowUs-progressUs_) - std::max(25000.0F, 3.0F*edgeUs);
      if (lateUs > 0.0F && lateUs <= 150000.0F) {
        floor = profile.startPercent + std::min(6.0F, lateUs*0.00012F);
      }
    }
    return {floor, anglePercent, integralTransfer};
  }

  void reset() { sign_ = 0; floorActive_ = trackAngle_ = false; }

 private:
  int sign_ = 0;
  bool floorActive_ = false, trackAngle_ = false;
  int32_t startCount_ = 0;
  int32_t progressCount_ = 0;
  int64_t progressUs_ = 0;
  int64_t startUs_ = 0, lastUs_ = 0;
  float lastTarget_ = 0.0F, referenceRad_ = 0.0F, lastAnglePercent_ = 0.0F;
};

}  // namespace cleany
