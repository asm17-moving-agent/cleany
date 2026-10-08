#pragma once

#include <cmath>
#include <cstdint>

namespace cleany {

constexpr int32_t kEncoderCountsPerOutputRevolution = 3172;
constexpr float kTwoPi = 6.28318530717958647692F;

struct MotorFilterConfig {
  int slewStepPercent = 1;
  float zeroSpeedThresholdRadPerSecond = 0.5F;
  int64_t zeroSpeedDwellUs = 50000;
};

inline int speedSign(int speed) {
  return (speed > 0) - (speed < 0);
}

inline int moveToward(int current, int target, int maximumStep) {
  if (current < target) {
    const int next = current + maximumStep;
    return next < target ? next : target;
  }
  if (current > target) {
    const int next = current - maximumStep;
    return next > target ? next : target;
  }
  return current;
}

inline float encoderVelocityRadPerSecond(int32_t deltaCount,
                                         float elapsedSeconds) {
  if (elapsedSeconds <= 0.0F) {
    return 0.0F;
  }
  return static_cast<float>(deltaCount) * kTwoPi /
         static_cast<float>(kEncoderCountsPerOutputRevolution) /
         elapsedSeconds;
}

class MotorCommandFilter {
 public:
  MotorCommandFilter() = default;

  explicit MotorCommandFilter(MotorFilterConfig config) : config_(config) {}

  void setTargetSpeed(int targetSpeed) {
    targetSpeed_ = targetSpeed;
  }

  void forceStop() {
    targetSpeed_ = 0;
    appliedSpeed_ = 0;
    zeroSpeedSinceUs_ = -1;
    waitingForReverse_ = false;
  }

  int step(float feedbackVelocityRadPerSecond, int64_t nowUs) {
    const int targetSign = speedSign(targetSpeed_);
    const int appliedSign = speedSign(appliedSpeed_);

    if (appliedSign != 0) {
      lastDriveSign_ = appliedSign;
      zeroSpeedSinceUs_ = -1;
      if (targetSign != 0 && targetSign != appliedSign) {
        waitingForReverse_ = true;
        appliedSpeed_ =
            moveToward(appliedSpeed_, 0, config_.slewStepPercent);
      } else {
        waitingForReverse_ = false;
        appliedSpeed_ = moveToward(appliedSpeed_, targetSpeed_,
                                   config_.slewStepPercent);
      }
      return appliedSpeed_;
    }

    observeStoppedSpeed(feedbackVelocityRadPerSecond, nowUs);
    if (targetSign == 0) {
      waitingForReverse_ = false;
      if (zeroSpeedIsStable(nowUs)) {
        lastDriveSign_ = 0;
      }
      return 0;
    }

    const bool directionWouldReverse =
        lastDriveSign_ != 0 && targetSign != lastDriveSign_;
    if (directionWouldReverse && !zeroSpeedIsStable(nowUs)) {
      waitingForReverse_ = true;
      return 0;
    }

    waitingForReverse_ = false;
    lastDriveSign_ = targetSign;
    zeroSpeedSinceUs_ = -1;
    appliedSpeed_ =
        moveToward(0, targetSpeed_, config_.slewStepPercent);
    return appliedSpeed_;
  }

  int targetSpeed() const {
    return targetSpeed_;
  }

  int appliedSpeed() const {
    return appliedSpeed_;
  }

  bool waitingForReverse() const {
    return waitingForReverse_;
  }

 private:
  void observeStoppedSpeed(float feedbackVelocityRadPerSecond, int64_t nowUs) {
    if (std::fabs(feedbackVelocityRadPerSecond) <=
        config_.zeroSpeedThresholdRadPerSecond) {
      if (zeroSpeedSinceUs_ < 0) {
        zeroSpeedSinceUs_ = nowUs;
      }
    } else {
      zeroSpeedSinceUs_ = -1;
    }
  }

  bool zeroSpeedIsStable(int64_t nowUs) const {
    return zeroSpeedSinceUs_ >= 0 &&
           nowUs - zeroSpeedSinceUs_ >= config_.zeroSpeedDwellUs;
  }

  MotorFilterConfig config_;
  int targetSpeed_ = 0;
  int appliedSpeed_ = 0;
  int lastDriveSign_ = 0;
  int64_t zeroSpeedSinceUs_ = -1;
  bool waitingForReverse_ = false;
};

}  // namespace cleany
