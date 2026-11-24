#pragma once

#include <algorithm>
#include <cmath>

#include "wheel_friction.hpp"

namespace cleany {

struct WheelVelocityControllerConfig {
  float maximumTargetRadPerSecond = 11.0F;
  float proportionalGain = 6.0F;
  float integralGain = 6.0F;
  float outputLimitPercent = 100.0F;
};

/**
 * @brief PI 적분 포화 방지와 함께 휠 속도 오차를 부호 있는 PWM %로 바꿉니다.
 *
 * 입력 단위는 rad/s와 초이며 출력 방향은 목표 방향을 거스르지 않습니다.
 * encoder로 정지를 확인한 뒤의 방향 반전은 MotorCommandFilter가 담당합니다.
 * 호출자는 유한한 입력과 유효한 양의 속도 기준값을 전달해야 합니다.
 */
class WheelVelocityController {
 public:
  WheelVelocityController() = default;

  explicit WheelVelocityController(WheelVelocityControllerConfig config,
                                   const WheelFriction* friction = nullptr)
      : config_(config), friction_(friction) {}

  /**
   * @brief PI 상태를 갱신하고 제한된 PWM %를 반환합니다.
   * 목표가 0이거나 경과 시간이 0 이하이면 적분 상태를 초기화하고 0을 반환합니다.
   */
  float update(float targetRadPerSecond, float measuredRadPerSecond,
               float elapsedSeconds, int32_t encoderCount = 0, int64_t nowUs = 0) {
    if (targetRadPerSecond == 0.0F || elapsedSeconds <= 0.0F) {
      reset();
      return 0.0F;
    }

    const float error = targetRadPerSecond - measuredRadPerSecond;
    float feedForward =
        targetRadPerSecond / config_.maximumTargetRadPerSecond *
        config_.outputLimitPercent;
    float minimumOutput = 0.0F;
    if (friction_) {
      const auto& direction = targetRadPerSecond > 0.0F ? friction_->forward : friction_->reverse;
      feedForward = std::copysign(holdingFeedForward(direction, std::fabs(targetRadPerSecond),
                                                     std::fabs(feedForward)), targetRadPerSecond);
      const auto startup = start_.update(targetRadPerSecond, measuredRadPerSecond,
                                         encoderCount, nowUs, direction);
      minimumOutput = startup.floorPercent;
      feedForward += startup.anglePercent;
      if (startup.integralTransferPercent != 0.0F && config_.integralGain > 0.0F) {
        const float limit = config_.outputLimitPercent / config_.integralGain;
        integralError_ = std::clamp(integralError_ + startup.integralTransferPercent /
                                    config_.integralGain, -limit, limit);
      }
    }
    const float lowerOutput =
        targetRadPerSecond > 0.0F ? minimumOutput : -config_.outputLimitPercent;
    const float upperOutput =
        targetRadPerSecond > 0.0F ? config_.outputLimitPercent : -minimumOutput;
    float candidateIntegral = integralError_ + error * elapsedSeconds;
    if (config_.integralGain > 0.0F) {
      const float integralLimit =
          config_.outputLimitPercent / config_.integralGain;
      candidateIntegral =
          std::clamp(candidateIntegral, -integralLimit, integralLimit);
    }

    const float candidateOutput =
        feedForward + config_.proportionalGain * error +
        config_.integralGain * candidateIntegral;
    const bool windingUp =
        (candidateOutput > upperOutput && error > 0.0F) ||
        (candidateOutput < lowerOutput && error < 0.0F);
    if (!windingUp) {
      integralError_ = candidateIntegral;
    }

    const float output =
        feedForward + config_.proportionalGain * error +
        config_.integralGain * integralError_;
    // 방향 반전은 encoder로 정지를 확인한 뒤 별도로 처리합니다.
    return std::clamp(output, lowerOutput, upperOutput);
  }

  void reset() {
    start_.reset();
    integralError_ = 0.0F;
  }

  float integralError() const {
    return integralError_;
  }

 private:
  WheelVelocityControllerConfig config_;
  const WheelFriction* friction_ = nullptr;
  StartFriction start_;
  float integralError_ = 0.0F;
};

}  // namespace cleany
