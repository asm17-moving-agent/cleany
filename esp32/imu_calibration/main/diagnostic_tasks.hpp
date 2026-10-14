#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gyro_calibration.hpp"
#include "sdkconfig.h"

namespace cleany::diagnostic {
constexpr int kImuCore = 1;
constexpr int kSerialCore = 0;
constexpr size_t kSampleRateHz = 100;
static_assert(portNUM_PROCESSORS >= 2, "Calibration diagnostic requires two cores");

struct RestartCommand {
  // A host-generated nonce distinguishes a new request from old buffered acks.
  std::array<char, 17> requestId{};
};

struct RunStarted {
  uint32_t runId = 0;
  bool commandTriggered = false;
  std::array<char, 17> requestId{};
};

struct Snapshot {
  uint32_t runId = 0;
  bool settling = true;
  CalibrationState state = CalibrationState::kCalibrating;
  CalibrationConfig config{};
  CalibrationResult result{};
  esp_err_t sensorError = ESP_OK;
  const char* sensorOperation = "none";  // Driver-owned static string literals.
  uint8_t identity = 0;

  bool terminal() const {
    return state == CalibrationState::kCompleted ||
           state == CalibrationState::kFailed;
  }
  const char* phase() const {
    return settling && !terminal() ? "settling" : calibrationStateName(state);
  }
};

// FreeRTOS queues copy fixed-size snapshots, never a live mutable calibrator.
static_assert(std::is_trivially_copyable<Snapshot>::value);
static_assert(std::is_trivially_copyable<RunStarted>::value);
static_assert(std::is_trivially_copyable<RestartCommand>::value);
struct Channels {
  QueueHandle_t latest = nullptr;     // Length 1, overwrite-only IMU -> serial.
  QueueHandle_t restarts = nullptr;   // Length 4, serial -> IMU.
  QueueHandle_t started = nullptr;    // Length 4, IMU -> serial acknowledgements.
};

inline CalibrationConfig calibrationConfig() {
  CalibrationConfig config{};
  config.calibrationSamples = CONFIG_CLEANY_IMU_CALIBRATION_SECONDS * kSampleRateHz;
  config.validationSamples = CONFIG_CLEANY_IMU_VALIDATION_SECONDS * kSampleRateHz;
  config.nominalPeriodUs = 1000000 / kSampleRateHz;
  config.maxSampleGapUs = CONFIG_CLEANY_IMU_MAX_SAMPLE_GAP_MS * 1000LL;
#ifdef CONFIG_CLEANY_IMU_LIMITS_REVIEWED
  config.limitsReviewed = true;
#endif
  config.maxGyroStdRadS = CONFIG_CLEANY_IMU_MAX_GYRO_STD_MICRORAD_S / 1e6;
  config.maxAccelStdMps2 = CONFIG_CLEANY_IMU_MAX_ACCEL_STD_MM_S2 / 1e3;
  config.maxValidationMeanRadS =
      CONFIG_CLEANY_IMU_MAX_VALIDATION_MEAN_MICRORAD_S / 1e6;
  config.maxTemperatureSpanC =
      CONFIG_CLEANY_IMU_MAX_TEMPERATURE_SPAN_MILLIC / 1e3;
  return config;
}

void imuTask(void* channels);
void serialTask(void* channels);
}  // namespace cleany::diagnostic
