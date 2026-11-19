#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "mpu6050_sample.hpp"

namespace cleany {

enum class CalibrationState { kCalibrating, kValidating, kCompleted, kFailed };
enum class CalibrationError {
  kNone, kInvalidConfig, kInvalidSample, kSaturated, kTiming, kMotion,
  kTemperatureDrift, kValidation, kLimitsNotReviewed, kSensor
};

inline const char* calibrationStateName(CalibrationState state) {
  switch (state) {
    case CalibrationState::kCalibrating: return "calibrating";
    case CalibrationState::kValidating: return "validating";
    case CalibrationState::kCompleted: return "completed";
    case CalibrationState::kFailed: return "failed";
  }
  return "unknown";
}

inline const char* calibrationErrorName(CalibrationError error) {
  switch (error) {
    case CalibrationError::kNone: return "none";
    case CalibrationError::kInvalidConfig: return "invalid_config";
    case CalibrationError::kInvalidSample: return "invalid_sample";
    case CalibrationError::kSaturated: return "saturated";
    case CalibrationError::kTiming: return "timing";
    case CalibrationError::kMotion: return "motion";
    case CalibrationError::kTemperatureDrift: return "temperature_drift";
    case CalibrationError::kValidation: return "validation";
    case CalibrationError::kLimitsNotReviewed: return "limits_not_reviewed";
    case CalibrationError::kSensor: return "sensor";
  }
  return "unknown";
}

struct CalibrationConfig {
  size_t calibrationSamples = 1000;
  size_t validationSamples = 500;
  int64_t nominalPeriodUs = 10000;
  int64_t maxSampleGapUs = 30000;
  bool limitsReviewed = false;
  double maxGyroStdRadS = 0;
  double maxAccelStdMps2 = 0;
  double maxValidationMeanRadS = 0;
  double maxTemperatureSpanC = 0;
};

struct WindowStatistics {
  size_t samples = 0;
  int64_t firstTimestampUs = 0, lastTimestampUs = 0;
  std::array<double, 3> gyroMeanRadS{}, accelMeanMps2{};
  std::array<double, 9> gyroCovariance{}, accelCovariance{};
  double minTemperatureC = 0, maxTemperatureC = 0;
};

struct CalibrationResult {
  bool accepted = false;
  CalibrationError error = CalibrationError::kNone;
  WindowStatistics calibration{}, validation{};
  std::array<double, 3> candidateBiasRadS{}, validationResidualRadS{};
};

class GyroCalibration {
 public:
  explicit GyroCalibration(CalibrationConfig config = {}) : config_(config) {
    if (!validConfig()) {
      state_ = CalibrationState::kFailed;
      result_.error = CalibrationError::kInvalidConfig;
    }
  }

  void reset() {
    result_ = {};
    state_ = CalibrationState::kCalibrating;
    if (!validConfig()) {
      state_ = CalibrationState::kFailed;
      result_.error = CalibrationError::kInvalidConfig;
    }
    gyroM2_ = {};
    accelM2_ = {};
    validation_gm2_ = {};
    validation_am2_ = {};
    previous_timestamp_ = 0;
    have_previous_ = false;
  }

  bool addSample(const ImuSample& sample) {
    if (state_ == CalibrationState::kCompleted ||
        state_ == CalibrationState::kFailed) {
      return false;
    }
    if (sample.saturated) {
      fail(CalibrationError::kSaturated);
      return false;
    }
    if (!finite(sample)) {
      fail(CalibrationError::kInvalidSample);
      return false;
    }
    if (sample.timestampUs < 0) {
      fail(CalibrationError::kTiming);
      return false;
    }
    if (have_previous_ &&
        (sample.timestampUs <= previous_timestamp_ ||
         sample.timestampUs - previous_timestamp_ > config_.maxSampleGapUs)) {
      fail(CalibrationError::kTiming);
      return false;
    }
    previous_timestamp_ = sample.timestampUs;
    have_previous_ = true;
    const bool validating = state_ == CalibrationState::kValidating;
    WindowStatistics& w = validating ? result_.validation : result_.calibration;
    auto& gm = validating ? validation_gm2_ : gyroM2_;
    auto& am = validating ? validation_am2_ : accelM2_;
    if (w.samples == 0) {
      w.firstTimestampUs = sample.timestampUs;
      w.minTemperatureC = w.maxTemperatureC = sample.temperatureC;
    }
    ++w.samples;
    w.lastTimestampUs = sample.timestampUs;
    w.minTemperatureC = std::fmin(w.minTemperatureC, sample.temperatureC);
    w.maxTemperatureC = std::fmax(w.maxTemperatureC, sample.temperatureC);
    update(sample.angularVelocityRadS, w.gyroMeanRadS, gm, w.samples);
    update(sample.accelerationMps2, w.accelMeanMps2, am, w.samples);
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        w.gyroCovariance[i * 3 + j] =
            w.samples > 1 ? gm[i * 3 + j] / (w.samples - 1) : 0;
        w.accelCovariance[i * 3 + j] =
            w.samples > 1 ? am[i * 3 + j] / (w.samples - 1) : 0;
      }
    }
    if (!validating && w.samples == config_.calibrationSamples) {
      result_.candidateBiasRadS = w.gyroMeanRadS;
      if (!durationValid(w)) {
        fail(CalibrationError::kTiming);
        return false;
      }
      state_ = CalibrationState::kValidating;
      return true;
    }
    const size_t target = validating ? config_.validationSamples : config_.calibrationSamples;
    if (validating && w.samples == target) {
      if (!durationValid(w)) {
        fail(CalibrationError::kTiming);
        return false;
      }
      for (size_t i = 0; i < 3; ++i) {
        result_.validationResidualRadS[i] =
            w.gyroMeanRadS[i] - result_.candidateBiasRadS[i];
      }
      state_ = CalibrationState::kCompleted;
      assess();
    }
    return true;
  }
  void fail(CalibrationError error) {
    if (state_ == CalibrationState::kCompleted || state_ == CalibrationState::kFailed) return;
    result_.accepted = false;
    result_.error =
        error == CalibrationError::kNone ? CalibrationError::kSensor : error;
    state_ = CalibrationState::kFailed;
  }
  CalibrationState state() const { return state_; }
  const CalibrationResult& result() const { return result_; }

 private:
  using Matrix = std::array<double,9>;
  CalibrationConfig config_;
  CalibrationState state_ = CalibrationState::kCalibrating;
  CalibrationResult result_{};
  Matrix gyroM2_{}, accelM2_{}, validation_gm2_{}, validation_am2_{};
  int64_t previous_timestamp_ = 0;
  bool have_previous_ = false;

  bool validConfig() const {
    return config_.calibrationSamples >= 2 &&
           config_.validationSamples >= 2 &&
           config_.nominalPeriodUs > 0 &&
           config_.maxSampleGapUs >= config_.nominalPeriodUs &&
           (!config_.limitsReviewed ||
            (positive(config_.maxGyroStdRadS) &&
             positive(config_.maxAccelStdMps2) &&
             positive(config_.maxValidationMeanRadS) &&
             positive(config_.maxTemperatureSpanC)));
  }

  static bool positive(double x) { return std::isfinite(x) && x > 0; }

  static bool finite(const ImuSample& s) {
    if (!std::isfinite(s.temperatureC)) return false;
    for (size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(s.angularVelocityRadS[i]) ||
          !std::isfinite(s.accelerationMps2[i])) {
        return false;
      }
    }
    return true;
  }

  bool durationValid(const WindowStatistics& w) const {
    // Require at least 90% of the nominal elapsed window duration.
    const long double expected =
        static_cast<long double>(w.samples - 1) * config_.nominalPeriodUs *
        0.9L;
    return static_cast<long double>(w.lastTimestampUs - w.firstTimestampUs) >=
           expected;
  }

  static void update(const std::array<double, 3>& x,
                     std::array<double, 3>& mean, Matrix& m2, size_t n) {
    // Welford's two-delta update accumulates the full covariance online.
    std::array<double, 3> delta{}, after{};
    for (size_t i = 0; i < 3; ++i) {
      delta[i] = x[i] - mean[i];
      mean[i] += delta[i] / n;
      after[i] = x[i] - mean[i];
    }
    for (size_t i = 0; i < 3; ++i) {
      for (size_t j = 0; j < 3; ++j) {
        m2[i * 3 + j] += delta[i] * after[j];
      }
    }
  }

  void assess() {
    // Collect both windows for diagnostics, but never accept unreviewed limits.
    if (!config_.limitsReviewed) {
      result_.error = CalibrationError::kLimitsNotReviewed;
      return;
    }
    const auto exceeds = [](const WindowStatistics& w, double gyro, double accel) {
      for (size_t i = 0; i < 3; ++i) {
        if (std::sqrt(w.gyroCovariance[i * 3 + i]) > gyro ||
            std::sqrt(w.accelCovariance[i * 3 + i]) > accel) {
          return true;
        }
      }
      return false;
    };
    if (exceeds(result_.calibration, config_.maxGyroStdRadS,
                config_.maxAccelStdMps2) ||
        exceeds(result_.validation, config_.maxGyroStdRadS,
                config_.maxAccelStdMps2)) {
      result_.error = CalibrationError::kMotion;
      return;
    }
    const double span =
        std::fmax(result_.calibration.maxTemperatureC,
                  result_.validation.maxTemperatureC) -
        std::fmin(result_.calibration.minTemperatureC,
                  result_.validation.minTemperatureC);
    if (span > config_.maxTemperatureSpanC) {
      result_.error = CalibrationError::kTemperatureDrift;
      return;
    }
    for (double residual : result_.validationResidualRadS) {
      if (std::fabs(residual) > config_.maxValidationMeanRadS) {
        result_.error = CalibrationError::kValidation;
        return;
      }
    }
    result_.accepted = true;
    result_.error = CalibrationError::kNone;
  }
};
}  // namespace cleany
