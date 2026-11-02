#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "gyro_calibration.hpp"

namespace cleany {

enum class ImuCalibrationState : uint8_t {
  kCalibrating = 0, kRunning = 1, kFallback = 2
};
enum class ImuCalibrationReason : uint8_t {
  kNone = 0, kTimeout = 1, kSensor = 2, kUnstable = 3, kSample = 4
};
struct ImuCalibrationValue {
  ImuCalibrationState state = ImuCalibrationState::kCalibrating;
  ImuCalibrationReason reason = ImuCalibrationReason::kNone;
  std::array<double, 3> bias{};
  std::array<double, 9> covariance{};
};
struct ImuStartupConfig {
  size_t calibrationSamples = 1000, validationSamples = 500;
  int64_t settlingUs = 5000000, timeoutUs = 25000000;
  double noiseFactor = 5.;
  std::array<double, 3> fallbackBias{};
  std::array<double, 9> fallbackCovariance{};
};

class ImuStartupCalibration {
 public:
  explicit ImuStartupCalibration(ImuStartupConfig config = {},
                                 int64_t startedAtUs = 0)
      : config_(config), startedAtUs_(startedAtUs),
        core_([&] {
          CalibrationConfig c{};
          c.calibrationSamples = config.calibrationSamples;
          c.validationSamples = config.validationSamples;
          c.limitsReviewed = false;
          return c;
        }()) {}
  const ImuCalibrationValue& value() const { return value_; }
  bool terminal() const { return value_.state != ImuCalibrationState::kCalibrating; }
  void sensorFailure() { finishFallback(ImuCalibrationReason::kSensor); }
  void poll(int64_t nowUs) {
    if (terminal()) return;
    if (nowUs >= startedAtUs_ + config_.timeoutUs)
      finishFallback(ImuCalibrationReason::kTimeout);
  }
  void addSample(const ImuSample& sample) {
    if (terminal()) return;
    if (sample.timestampUs < startedAtUs_ + config_.settlingUs) return;
    if (!core_.addSample(sample)) {
      finishFallback(ImuCalibrationReason::kSample);
      return;
    }
    if (core_.state() == CalibrationState::kCompleted) assess();
    else if (core_.state() == CalibrationState::kFailed)
      finishFallback(ImuCalibrationReason::kSample);
  }
 private:
  ImuStartupConfig config_;
  int64_t startedAtUs_;
  GyroCalibration core_;
  ImuCalibrationValue value_{};
  void finishFallback(ImuCalibrationReason reason) {
    if (terminal()) return;
    value_.state = ImuCalibrationState::kFallback;
    value_.reason = reason;
    value_.bias = config_.fallbackBias;
    value_.covariance = config_.fallbackCovariance;
  }
  void assess() {
    const auto& r = core_.result();
    for (size_t i = 0; i < 3; ++i) {
      const double ref = config_.fallbackCovariance[i * 3 + i];
      const double trainVariance = r.calibration.gyroCovariance[i * 3 + i];
      const double validationVariance = r.validation.gyroCovariance[i * 3 + i];
      if (std::sqrt(trainVariance) > config_.noiseFactor * std::sqrt(ref) ||
          std::sqrt(validationVariance) > config_.noiseFactor * std::sqrt(ref)) {
        finishFallback(ImuCalibrationReason::kUnstable);
        return;
      }
      const double train = std::max(trainVariance, ref);
      const double valid = std::max(validationVariance, ref);
      if (std::fabs(r.validationResidualRadS[i]) >
          config_.noiseFactor *
              std::sqrt(train / config_.calibrationSamples +
                        valid / config_.validationSamples)) {
        finishFallback(ImuCalibrationReason::kUnstable);
        return;
      }
    }
    value_.state = ImuCalibrationState::kRunning;
    value_.bias = r.candidateBiasRadS;
    for (size_t i = 0; i < 9; ++i) value_.covariance[i] = 0;
    for (size_t i = 0; i < 3; ++i)
      value_.covariance[i * 3 + i] =
          std::max(r.validation.gyroCovariance[i * 3 + i],
                   config_.fallbackCovariance[i * 3 + i]);
  }
};
}  // namespace cleany
