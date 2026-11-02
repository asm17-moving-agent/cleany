#include <cassert>
#include <cmath>
#include <limits>

#include "gyro_calibration.hpp"

using namespace cleany;

static ImuSample sample(int64_t timestamp, double gyro = 0) {
  ImuSample s{};
  s.timestampUs = timestamp;
  s.angularVelocityRadS = {gyro, gyro * 2, -gyro};
  s.accelerationMps2 = {1, 2, 3};
  s.temperatureC = 25;
  return s;
}

static ImuSample varyingSample(int64_t timestamp, double value) {
  ImuSample s = sample(timestamp);
  s.angularVelocityRadS = {value, value, value};
  s.accelerationMps2 = {value, value, value};
  return s;
}

static CalibrationConfig config(bool reviewed = true) {
  CalibrationConfig c{};
  c.calibrationSamples = 4;
  c.validationSamples = 4;
  c.nominalPeriodUs = 10000;
  c.maxSampleGapUs = 30000;
  c.limitsReviewed = reviewed;
  c.maxGyroStdRadS = 10;
  c.maxAccelStdMps2 = 10;
  c.maxValidationMeanRadS = 10;
  c.maxTemperatureSpanC = 10;
  return c;
}

static void feedWindow(GyroCalibration& calibration, int first, int count,
                       double gyro, double temperature = 25) {
  for (int i = 0; i < count; ++i) {
    ImuSample s = sample((first + i) * 10000, gyro);
    s.temperatureC = temperature;
    assert(calibration.addSample(s));
  }
}

int main() {
  // Measurement-only calibration retains bias/full covariance but never accepts.
  {
    GyroCalibration c(config(false));
    const double values[] = {0, 1, 2, 3};
    for (int i = 0; i < 4; ++i) {
      assert(c.addSample(sample(i * 10000, values[i])));
    }
    for (int i = 0; i < 4; ++i) {
      assert(c.addSample(sample((i + 4) * 10000, values[i])));
    }
    const auto& r = c.result();
    assert(c.state() == CalibrationState::kCompleted && !r.accepted);
    assert(r.error == CalibrationError::kLimitsNotReviewed);
    assert(std::fabs(r.calibration.gyroMeanRadS[0] - 1.5) < 1e-12);
    // Welford's online covariance retains off-diagonal cross-axis terms.
    assert(std::fabs(r.calibration.gyroCovariance[0] - 5.0 / 3) < 1e-12);
    assert(std::fabs(r.calibration.gyroCovariance[1] - 10.0 / 3) < 1e-12);
    assert(std::fabs(r.candidateBiasRadS[0] - 1.5) < 1e-12);
    assert(std::fabs(r.validationResidualRadS[0]) < 1e-12);
    // Measurement-only mode reports statistics, but reviewed limits gate acceptance.
  }
  // Reviewed calibration recovers known bias and checks a separate holdout.
  {
    GyroCalibration c(config());
    feedWindow(c, 0, 4, 0.25);
    // The holdout window is separate: it validates, rather than fits, the bias.
    feedWindow(c, 4, 4, 0.26);
    assert(c.result().accepted);
    assert(std::fabs(c.result().candidateBiasRadS[0] - 0.25) < 1e-12);
    assert(std::fabs(c.result().validationResidualRadS[0] - 0.01) < 1e-12);
    assert(!c.addSample(sample(80000, 4)));
    c.reset();
    assert(c.state() == CalibrationState::kCalibrating);
  }
  // A holdout mean outside the residual limit rejects the candidate calibration.
  {
    auto cfig = config();
    cfig.maxValidationMeanRadS = 0.005;
    GyroCalibration c(cfig);
    feedWindow(c, 0, 4, 0.1);
    for (int i = 0; i < 3; ++i) {
      assert(c.addSample(sample((i + 4) * 10000, 0.2)));
    }
    assert(c.addSample(sample(70000, 0.2)));
    assert(c.state() == CalibrationState::kCompleted);
    assert(!c.result().accepted && c.result().error == CalibrationError::kValidation);
  }
  // Missing reviewed quality limits fail configuration before taking samples.
  {
    auto bad = config();
    bad.maxGyroStdRadS = 0;
    GyroCalibration c(bad);
    assert(c.state() == CalibrationState::kFailed);
    assert(!c.addSample(sample(0)));
  }
  // Duplicate timestamps are rejected instead of counting the same sample twice.
  {
    GyroCalibration c(config());
    assert(c.addSample(sample(0)));
    assert(!c.addSample(sample(0)));
    assert(c.result().error == CalibrationError::kTiming && !c.result().accepted);
  }
  // Saturation and nonfinite temperature fail the attempt; reset permits retry.
  {
    GyroCalibration c(config());
    auto s = sample(0);
    s.saturated = true;
    assert(!c.addSample(s));
    assert(c.result().error == CalibrationError::kSaturated);
    c.reset();
    s = sample(0);
    s.temperatureC = std::numeric_limits<double>::quiet_NaN();
    assert(!c.addSample(s));
    assert(c.result().error == CalibrationError::kInvalidSample);
  }
  // Samples arriving ten times too fast cannot satisfy a nominal 100 Hz window.
  {
    GyroCalibration c(config());
    for (int i = 0; i < 3; ++i) {
      assert(c.addSample(sample(i * 1000)));
    }
    assert(!c.addSample(sample(3000)));
    assert(c.state() == CalibrationState::kFailed);
    assert(c.result().error == CalibrationError::kTiming);
  }
  // The maximum sample gap also applies across the calibration/holdout boundary.
  {
    GyroCalibration c(config());
    feedWindow(c, 0, 4, 0);
    assert(!c.addSample(sample(70000)));
    assert(c.state() == CalibrationState::kFailed);
    assert(c.result().error == CalibrationError::kTiming);
  }
  // Motion-quality rejection covers both sensor types in both sampling windows.
  {
    auto cfig = config();
    cfig.maxGyroStdRadS = 0.1;
    // Excessive calibration-window gyro variance rejects the result.
    GyroCalibration calibrationWindow(cfig);
    assert(calibrationWindow.addSample(varyingSample(0, -1)));
    assert(calibrationWindow.addSample(varyingSample(10000, 1)));
    assert(calibrationWindow.addSample(varyingSample(20000, -1)));
    assert(calibrationWindow.addSample(varyingSample(30000, 1)));
    feedWindow(calibrationWindow, 4, 4, 0);
    assert(calibrationWindow.result().error == CalibrationError::kMotion);
    assert(!calibrationWindow.result().accepted);

    cfig = config();
    cfig.maxAccelStdMps2 = 0.1;
    // Excessive calibration-window accelerometer variance rejects the result.
    GyroCalibration accelCalibrationWindow(cfig);
    for (int i = 0; i < 4; ++i) {
      assert(accelCalibrationWindow.addSample(varyingSample(i * 10000, i % 2)));
    }
    feedWindow(accelCalibrationWindow, 4, 4, 0);
    assert(accelCalibrationWindow.result().error == CalibrationError::kMotion);

    auto gyroValidationConfig = config();
    gyroValidationConfig.maxGyroStdRadS = 0.1;
    // Excessive independent-window gyro variance rejects the result.
    GyroCalibration validationWindow(gyroValidationConfig);
    feedWindow(validationWindow, 0, 4, 0);
    for (int i = 0; i < 3; ++i) {
      assert(validationWindow.addSample(varyingSample((i + 4) * 10000, i % 2)));
    }
    assert(validationWindow.addSample(varyingSample(70000, 1)));
    assert(validationWindow.result().error == CalibrationError::kMotion);

    cfig = config();
    cfig.maxAccelStdMps2 = 0.1;
    // Excessive independent-window accelerometer variance rejects the result.
    GyroCalibration accelValidationWindow(cfig);
    feedWindow(accelValidationWindow, 0, 4, 0);
    for (int i = 0; i < 3; ++i) {
      assert(accelValidationWindow.addSample(
          varyingSample((i + 4) * 10000, i % 2)));
    }
    assert(accelValidationWindow.addSample(varyingSample(70000, 1)));
    assert(accelValidationWindow.result().error == CalibrationError::kMotion);
    assert(!accelValidationWindow.result().accepted);
  }
  // Temperature limits span both windows, not just variation inside each one.
  {
    auto cfig = config();
    cfig.maxTemperatureSpanC = 1;
    GyroCalibration c(cfig);
    feedWindow(c, 0, 4, 0, 25);
    feedWindow(c, 4, 4, 0, 27);
    assert(c.result().error == CalibrationError::kTemperatureDrift);
    assert(!c.result().accepted);
  }
  // Negative time and nonfinite gyro/acceleration values fail independently.
  {
    GyroCalibration c(config());
    ImuSample s = sample(-1);
    assert(!c.addSample(s));
    assert(c.result().error == CalibrationError::kTiming);

    c.reset();
    s = sample(0);
    s.angularVelocityRadS[1] = std::numeric_limits<double>::infinity();
    assert(!c.addSample(s));
    assert(c.result().error == CalibrationError::kInvalidSample);

    c.reset();
    s = sample(0);
    s.accelerationMps2[2] = std::numeric_limits<double>::quiet_NaN();
    assert(!c.addSample(s));
    assert(c.result().error == CalibrationError::kInvalidSample);
  }
  // Negative finite sensor-axis values are valid signed measurements.
  {
    GyroCalibration c(config());
    ImuSample s = sample(0, -0.5);
    s.accelerationMps2 = {-1, -2, -3};
    assert(c.addSample(s));
    assert(c.state() == CalibrationState::kCalibrating);
  }
  // Neither a fitted bias nor an incomplete holdout makes calibration accepted.
  {
    auto cfig = config();
    cfig.calibrationSamples = 3;
    cfig.validationSamples = 3;
    GyroCalibration c(cfig);
    feedWindow(c, 0, 3, 0);
    assert(c.state() == CalibrationState::kValidating);
    assert(!c.result().accepted);
    feedWindow(c, 3, 2, 0);
    assert(c.state() == CalibrationState::kValidating);
    assert(!c.result().accepted);
  }
  // Too-small sample counts and nonfinite reviewed limits fail configuration.
  {
    for (size_t count : {size_t{0}, size_t{1}}) {
      auto cfig = config();
      cfig.calibrationSamples = count;
      GyroCalibration c(cfig);
      assert(c.state() == CalibrationState::kFailed);
      assert(c.result().error == CalibrationError::kInvalidConfig);

      cfig = config();
      cfig.validationSamples = count;
      GyroCalibration validationCount(cfig);
      assert(validationCount.state() == CalibrationState::kFailed);
      assert(validationCount.result().error == CalibrationError::kInvalidConfig);
    }
    for (double limit : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}) {
      auto cfig = config();
      cfig.maxValidationMeanRadS = limit;
      GyroCalibration c(cfig);
      assert(c.state() == CalibrationState::kFailed);
      assert(c.result().error == CalibrationError::kInvalidConfig);
    }
  }
  // Sensor failure invalidates an active run; reset clears it; terminal data is immutable.
  {
    GyroCalibration c(config());
    c.fail(CalibrationError::kSensor);
    assert(c.state() == CalibrationState::kFailed);
    assert(c.result().error == CalibrationError::kSensor);
    assert(!c.result().accepted);
    assert(!c.addSample(sample(0)));

    c.reset();
    c.reset();
    assert(c.state() == CalibrationState::kCalibrating);
    assert(c.result().error == CalibrationError::kNone);
    c.fail(CalibrationError::kNone);
    assert(c.result().error == CalibrationError::kSensor);

    c.reset();
    feedWindow(c, 0, 4, 0);
    feedWindow(c, 4, 4, 0);
    c.fail(CalibrationError::kSensor);
    assert(c.state() == CalibrationState::kCompleted);
    assert(c.result().accepted);
  }
}
