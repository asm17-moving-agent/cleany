#include <cassert>
#include <cmath>

#include "imu_startup_calibration.hpp"

namespace {
cleany::ImuStartupConfig config() {
  cleany::ImuStartupConfig c{};
  c.calibrationSamples = 20;
  c.validationSamples = 10;
  c.settlingUs = 5000000;
  c.timeoutUs = 25000000;
  c.fallbackBias = {0.1, 0.2, 0.3};
  c.fallbackCovariance = {1e-6, 1e-7, 2e-7, 1e-7, 2e-6,
                          3e-7, 2e-7, 3e-7, 3e-6};
  return c;
}

void feed(cleany::ImuStartupCalibration& calibration, int64_t start,
          bool noisyTrain, bool noisyValidation) {
  for (int n = 0; n < 30; ++n) {
    cleany::ImuSample sample{};
    sample.timestampUs = start + (n + 1) * 10000;
    sample.accelerationMps2[2] = 9.8;
    sample.temperatureC = 20;
    const bool validation = n >= 20;
    const bool noise = validation ? noisyValidation : noisyTrain;
    const double variation = noise ? ((n % 2) ? 0.02 : -0.02) : 0.0;
    sample.angularVelocityRadS = {0.01 + variation, 0.02, -0.03};
    calibration.addSample(sample);
  }
}
}  // namespace

int main() {
  auto cfg = config();
  constexpr int64_t origin = 10000000;
  cleany::ImuStartupCalibration success(cfg, origin);
  cleany::ImuSample settling{};
  settling.timestampUs = origin + cfg.settlingUs - 1;
  success.addSample(settling);
  assert(success.value().state == cleany::ImuCalibrationState::kCalibrating);
  feed(success, origin + cfg.settlingUs, false, false);
  assert(success.value().state == cleany::ImuCalibrationState::kRunning);
  assert(std::fabs(success.value().bias[0] - 0.01) < 1e-9);
  assert(success.value().covariance[1] == 0);
  assert(success.value().covariance[0] == cfg.fallbackCovariance[0]);
  const auto savedBias = success.value().bias;
  success.sensorFailure();
  success.poll(origin + cfg.timeoutUs);
  assert(success.value().state == cleany::ImuCalibrationState::kRunning);
  assert(success.value().bias == savedBias);

  cleany::ImuStartupCalibration fallback(cfg, origin);
  fallback.sensorFailure();
  assert(fallback.value().state == cleany::ImuCalibrationState::kFallback);
  assert(fallback.value().bias == cfg.fallbackBias);
  assert(fallback.value().covariance == cfg.fallbackCovariance);
  const auto savedCovariance = fallback.value().covariance;
  feed(fallback, origin + cfg.settlingUs, false, false);
  assert(fallback.value().covariance == savedCovariance);

  cleany::ImuStartupCalibration trainNoise(cfg, origin);
  feed(trainNoise, origin + cfg.settlingUs, true, false);
  assert(trainNoise.value().reason == cleany::ImuCalibrationReason::kUnstable);
  cleany::ImuStartupCalibration holdoutNoise(cfg, origin);
  feed(holdoutNoise, origin + cfg.settlingUs, false, true);
  assert(holdoutNoise.value().reason == cleany::ImuCalibrationReason::kUnstable);

  cleany::ImuStartupCalibration residual(cfg, origin);
  for (int n = 0; n < 30; ++n) {
    cleany::ImuSample s{};
    s.timestampUs = origin + cfg.settlingUs + (n + 1) * 10000;
    s.angularVelocityRadS = {n < 20 ? 0.01 : 0.03, 0.02, -0.03};
    residual.addSample(s);
  }
  assert(residual.value().reason == cleany::ImuCalibrationReason::kUnstable);

  cleany::ImuStartupCalibration timeout(cfg, origin);
  timeout.poll(origin + cfg.timeoutUs - 1);
  assert(!timeout.terminal());
  timeout.poll(origin + cfg.timeoutUs);
  assert(timeout.value().state == cleany::ImuCalibrationState::kFallback);
  assert(timeout.value().reason == cleany::ImuCalibrationReason::kTimeout);
  assert(timeout.value().covariance == cfg.fallbackCovariance);

  cleany::ImuStartupCalibration saturated(cfg, origin);
  cleany::ImuSample clipped{};
  clipped.timestampUs = origin + cfg.settlingUs;
  clipped.saturated = true;
  saturated.addSample(clipped);
  assert(saturated.value().state == cleany::ImuCalibrationState::kFallback);
  assert(saturated.value().reason == cleany::ImuCalibrationReason::kSample);
  assert(saturated.value().bias == cfg.fallbackBias);

  cleany::ImuStartupCalibration interrupted(cfg, origin);
  cleany::ImuSample sample{};
  sample.timestampUs = origin + cfg.settlingUs;
  interrupted.addSample(sample);
  sample.timestampUs += 100000;  // Acquisition stall exceeds the 30 ms gap limit.
  interrupted.addSample(sample);
  assert(interrupted.value().state == cleany::ImuCalibrationState::kFallback);
  assert(interrupted.value().reason == cleany::ImuCalibrationReason::kSample);
  assert(interrupted.value().covariance == cfg.fallbackCovariance);
}
