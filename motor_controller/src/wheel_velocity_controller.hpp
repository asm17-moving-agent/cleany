#pragma once

#include <algorithm>
#include <cmath>

namespace cleany {

struct WheelVelocityControllerConfig {
  float maximumTargetRadPerSecond = 11.0F;
  float proportionalGain = 6.0F;
  float integralGain = 6.0F;
  float outputLimitPercent = 100.0F;
};

class WheelVelocityController {
 public:
  WheelVelocityController() = default;

  explicit WheelVelocityController(WheelVelocityControllerConfig config)
      : config_(config) {}

  float update(float targetRadPerSecond, float measuredRadPerSecond,
               float elapsedSeconds) {
    if (targetRadPerSecond == 0.0F || elapsedSeconds <= 0.0F) {
      reset();
      return 0.0F;
    }

    const float error = targetRadPerSecond - measuredRadPerSecond;
    const float feedForward =
        targetRadPerSecond / config_.maximumTargetRadPerSecond *
        config_.outputLimitPercent;
    const float lowerOutput =
        targetRadPerSecond > 0.0F ? 0.0F : -config_.outputLimitPercent;
    const float upperOutput =
        targetRadPerSecond > 0.0F ? config_.outputLimitPercent : 0.0F;
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
    // Direction changes are handled separately after encoder-confirmed stop.
    return std::clamp(output, lowerOutput, upperOutput);
  }

  void reset() {
    integralError_ = 0.0F;
  }

  float integralError() const {
    return integralError_;
  }

 private:
  WheelVelocityControllerConfig config_;
  float integralError_ = 0.0F;
};

}  // namespace cleany
