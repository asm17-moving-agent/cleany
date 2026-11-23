#pragma once

#include <cmath>
#include <cstdint>

namespace cleany {

constexpr int32_t kEncoderCountsPerOutputRevolution = 3172;
constexpr float kTwoPi = 6.28318530717958647692F;

struct MotorFilterConfig {
  int slewStepPercent = 1;
  int64_t slewStepIntervalUs = 0;
  float zeroSpeedThresholdRadPerSecond = 0.5F;
  int64_t zeroSpeedDwellUs = 50000;
  int decelSlewStepPercent = 0;
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

/**
 * @brief PWM 변화를 제한하고 방향 반전 전에 측정된 정지를 확인합니다.
 *
 * slewStepIntervalUs가 양수이면 해당 간격이 지난 step()에서만 PWM을 갱신합니다.
 * 간격이 0 이하이면 호출마다 갱신하므로 호출자가 주기를 관리해야 합니다.
 * 목표와 결과는 부호 있는 PWM %, 피드백은 rad/s, 시각은 MCU 단조 증가 μs입니다.
 * forceStop()은 증분 제한을 우회하지만 이전 구동 방향은 보존합니다.
 */
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
    lastSlewUpdateUs_ = -1;
    waitingForReverse_ = false;
  }

  int step(float feedbackVelocityRadPerSecond, int64_t nowUs, bool still = true) {
    const int targetSign = speedSign(targetSpeed_);
    const int appliedSign = speedSign(appliedSpeed_);

    if (appliedSign != 0) {
      lastDriveSign_ = appliedSign;
      zeroSpeedSinceUs_ = -1;
      const int decelStep = (config_.decelSlewStepPercent > 0)
                                ? config_.decelSlewStepPercent
                                : config_.slewStepPercent;
      if (targetSign != 0 && targetSign != appliedSign) {
        waitingForReverse_ = true;
        if (canAdvanceSlew(nowUs)) {
          appliedSpeed_ =
              moveToward(appliedSpeed_, 0, decelStep);
          lastSlewUpdateUs_ = nowUs;
        }
      } else {
        waitingForReverse_ = false;
        if (canAdvanceSlew(nowUs)) {
          const bool isDecel = std::abs(targetSpeed_) < std::abs(appliedSpeed_);
          const int stepAmount = isDecel ? decelStep : config_.slewStepPercent;
          appliedSpeed_ = moveToward(appliedSpeed_, targetSpeed_, stepAmount);
          lastSlewUpdateUs_ = nowUs;
        }
      }
      return appliedSpeed_;
    }

    observeStoppedSpeed(feedbackVelocityRadPerSecond, nowUs, still);
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
    if (canAdvanceSlew(nowUs)) {
      appliedSpeed_ =
          moveToward(0, targetSpeed_, config_.slewStepPercent);
      lastSlewUpdateUs_ = nowUs;
    }
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
  bool canAdvanceSlew(int64_t nowUs) const {
    if (config_.slewStepIntervalUs <= 0) {
      return true;
    }
    return lastSlewUpdateUs_ < 0 ||
           (nowUs - lastSlewUpdateUs_) >= config_.slewStepIntervalUs;
  }

  void observeStoppedSpeed(float feedbackVelocityRadPerSecond, int64_t nowUs, bool still) {
    if (still && std::fabs(feedbackVelocityRadPerSecond) <=
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
  int64_t lastSlewUpdateUs_ = -1;
  bool waitingForReverse_ = false;
};

}  // namespace cleany
