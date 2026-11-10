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
  // 측정 전용 보정은 편향과 전체 공분산을 보존하지만 보정 승인으로 처리하지 않습니다.
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
    // Welford 온라인 공분산 계산에서 축 간 비대각 항도 유지합니다.
    assert(std::fabs(r.calibration.gyroCovariance[0] - 5.0 / 3) < 1e-12);
    assert(std::fabs(r.calibration.gyroCovariance[1] - 10.0 / 3) < 1e-12);
    assert(std::fabs(r.candidateBiasRadS[0] - 1.5) < 1e-12);
    assert(std::fabs(r.validationResidualRadS[0]) < 1e-12);
    // 측정 전용 모드는 통계만 보고하며 검토된 한계값이 있어야 승인할 수 있습니다.
  }
  // 검토된 보정으로 알려진 편향을 추정하고 별도의 평가 표본을 검사합니다.
  {
    GyroCalibration c(config());
    feedWindow(c, 0, 4, 0.25);
    // 평가 구간은 편향 추정에 사용하지 않고 별도로 검증합니다.
    feedWindow(c, 4, 4, 0.26);
    assert(c.result().accepted);
    assert(std::fabs(c.result().candidateBiasRadS[0] - 0.25) < 1e-12);
    assert(std::fabs(c.result().validationResidualRadS[0] - 0.01) < 1e-12);
    assert(!c.addSample(sample(80000, 4)));
    c.reset();
    assert(c.state() == CalibrationState::kCalibrating);
  }
  // 평가 평균이 잔차 한계를 넘으면 보정 후보를 거부합니다.
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
  // 검토된 품질 한계가 없으면 표본 수집 전에 설정 검증이 실패합니다.
  {
    auto bad = config();
    bad.maxGyroStdRadS = 0;
    GyroCalibration c(bad);
    assert(c.state() == CalibrationState::kFailed);
    assert(!c.addSample(sample(0)));
  }
  // 같은 표본을 중복 집계하지 않도록 중복 시각을 거부합니다.
  {
    GyroCalibration c(config());
    assert(c.addSample(sample(0)));
    assert(!c.addSample(sample(0)));
    assert(c.result().error == CalibrationError::kTiming && !c.result().accepted);
  }
  // 포화나 유한하지 않은 온도는 시도를 실패시키며 reset 후 재시도할 수 있습니다.
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
  // 명목 100 Hz보다 열 배 빠른 표본으로는 요구된 수집 시간을 충족할 수 없습니다.
  {
    GyroCalibration c(config());
    for (int i = 0; i < 3; ++i) {
      assert(c.addSample(sample(i * 1000)));
    }
    assert(!c.addSample(sample(3000)));
    assert(c.state() == CalibrationState::kFailed);
    assert(c.result().error == CalibrationError::kTiming);
  }
  // 최대 표본 간격은 보정 구간과 평가 구간 사이에도 적용합니다.
  {
    GyroCalibration c(config());
    feedWindow(c, 0, 4, 0);
    assert(!c.addSample(sample(70000)));
    assert(c.state() == CalibrationState::kFailed);
    assert(c.result().error == CalibrationError::kTiming);
  }
  // 두 수집 구간의 두 센서 종류 모두 움직임 품질을 검사합니다.
  {
    auto cfig = config();
    cfig.maxGyroStdRadS = 0.1;
    // 보정 구간의 자이로 분산이 과도하면 결과를 거부합니다.
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
    // 보정 구간의 가속도계 분산이 과도하면 결과를 거부합니다.
    GyroCalibration accelCalibrationWindow(cfig);
    for (int i = 0; i < 4; ++i) {
      assert(accelCalibrationWindow.addSample(varyingSample(i * 10000, i % 2)));
    }
    feedWindow(accelCalibrationWindow, 4, 4, 0);
    assert(accelCalibrationWindow.result().error == CalibrationError::kMotion);

    auto gyroValidationConfig = config();
    gyroValidationConfig.maxGyroStdRadS = 0.1;
    // 독립 평가 구간의 자이로 분산이 과도하면 결과를 거부합니다.
    GyroCalibration validationWindow(gyroValidationConfig);
    feedWindow(validationWindow, 0, 4, 0);
    for (int i = 0; i < 3; ++i) {
      assert(validationWindow.addSample(varyingSample((i + 4) * 10000, i % 2)));
    }
    assert(validationWindow.addSample(varyingSample(70000, 1)));
    assert(validationWindow.result().error == CalibrationError::kMotion);

    cfig = config();
    cfig.maxAccelStdMps2 = 0.1;
    // 독립 평가 구간의 가속도계 분산이 과도하면 결과를 거부합니다.
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
  // 온도 한계는 각 구간 내부뿐 아니라 두 구간 전체에 적용합니다.
  {
    auto cfig = config();
    cfig.maxTemperatureSpanC = 1;
    GyroCalibration c(cfig);
    feedWindow(c, 0, 4, 0, 25);
    feedWindow(c, 4, 4, 0, 27);
    assert(c.result().error == CalibrationError::kTemperatureDrift);
    assert(!c.result().accepted);
  }
  // 음수 시각과 유한하지 않은 자이로·가속도 값은 각각 실패 조건입니다.
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
  // 유한한 음수 센서 축 값은 유효한 부호 있는 측정값입니다.
  {
    GyroCalibration c(config());
    ImuSample s = sample(0, -0.5);
    s.accelerationMps2 = {-1, -2, -3};
    assert(c.addSample(s));
    assert(c.state() == CalibrationState::kCalibrating);
  }
  // 편향을 추정했거나 평가 구간이 미완료인 것만으로 보정을 승인하지 않습니다.
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
  // 표본 수가 너무 작거나 검토 한계가 유한하지 않으면 설정이 실패합니다.
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
  // 센서 고장은 수집을 무효화하고 reset은 상태를 지우며 종료된 결과는 변경되지 않습니다.
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
