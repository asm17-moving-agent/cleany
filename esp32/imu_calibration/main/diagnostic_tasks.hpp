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
  // 호스트가 만든 nonce로 새 요청과 버퍼에 남은 이전 ACK를 구분합니다.
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
  const char* sensorOperation = "none";  // 드라이버가 소유하는 정적 문자열 리터럴입니다.
  uint8_t identity = 0;

  bool terminal() const {
    return state == CalibrationState::kCompleted ||
           state == CalibrationState::kFailed;
  }
  const char* phase() const {
    return settling && !terminal() ? "settling" : calibrationStateName(state);
  }
};

// FreeRTOS 큐는 고정 크기 스냅샷을 복사하며 변경 중인 보정 객체를 공유하지 않습니다.
static_assert(std::is_trivially_copyable<Snapshot>::value);
static_assert(std::is_trivially_copyable<RunStarted>::value);
static_assert(std::is_trivially_copyable<RestartCommand>::value);
struct Channels {
  QueueHandle_t latest = nullptr;     // 길이 1, 덮어쓰기 전용 IMU → serial 큐입니다.
  QueueHandle_t restarts = nullptr;   // 길이 4, serial → IMU 큐입니다.
  QueueHandle_t started = nullptr;    // 길이 4, IMU → serial 응답 큐입니다.
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
