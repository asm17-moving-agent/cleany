#include "diagnostic_tasks.hpp"

#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "mpu6050.hpp"

namespace cleany::diagnostic {
namespace {
Mpu6050Config sensorConfig() {
  Mpu6050Config config{};
  config.timeoutMs = CONFIG_CLEANY_IMU_I2C_TIMEOUT_MS;
  return config;
}

class CalibrationRunner {
 public:
  explicit CalibrationRunner(Channels& channels)
      : channels_(channels), config_(calibrationConfig()), sensor_(sensorConfig()),
        calibration_(config_), runId_(esp_random()) {}

  void start(bool commandTriggered, const RestartCommand& command = {}) {
    if (++runId_ == 0) runId_ = 1;
    calibration_.reset();
    settling_ = true;
    updateSnapshot();
    const RunStarted acknowledgement{runId_, commandTriggered, command.requestId};
    // start is called only when an acknowledgement slot is available.
    // This is nonblocking even when serial output is stalled.
    (void)xQueueSend(channels_.started, &acknowledgement, 0);
    if (calibration_.state() == CalibrationState::kFailed) {
      sensor_.shutdown();
      finish();
      return;
    }
    if (sensor_.initialize() != ESP_OK) {
      calibration_.fail(CalibrationError::kSensor);
      finish();
      return;
    }
    const int64_t now = esp_timer_get_time();
    settlingUntilUs_ = now + CONFIG_CLEANY_IMU_SETTLING_MS * 1000LL;
    sampleDeadlineUs_ = now + config_.maxSampleGapUs;
    updateSnapshot();
  }

  void tick() {
    // Defer restart consumption if its acknowledgement cannot be retained.
    // Existing acquisition keeps running; the IMU task never waits for serial.
    if (uxQueueSpacesAvailable(channels_.started) != 0) {
      RestartCommand command{};
      if (xQueueReceive(channels_.restarts, &command, 0) == pdTRUE) {
        start(true, command);
      }
    }
    if (terminal()) return;
    ImuSample sample{};
    const auto read = sensor_.readSample(sample);
    const int64_t now = esp_timer_get_time();
    if (read == Mpu6050ReadStatus::kError) {
      calibration_.fail(CalibrationError::kSensor);
      finish();
      return;
    }
    if (now > sampleDeadlineUs_) {
      calibration_.fail(CalibrationError::kTiming);
      finish();
      return;
    }
    if (read == Mpu6050ReadStatus::kNotReady) return;
    sampleDeadlineUs_ = sample.timestampUs + config_.maxSampleGapUs;
    if (settling_ && now < settlingUntilUs_) return;
    settling_ = false;
    (void)calibration_.addSample(sample);
    updateSnapshot();
  }

 private:
  Channels& channels_;
  CalibrationConfig config_;
  Mpu6050 sensor_;
  GyroCalibration calibration_;
  uint32_t runId_;
  bool settling_ = false;
  int64_t settlingUntilUs_ = 0;
  int64_t sampleDeadlineUs_ = 0;

  bool terminal() const {
    return calibration_.state() == CalibrationState::kCompleted ||
           calibration_.state() == CalibrationState::kFailed;
  }

  void finish() {
    settling_ = false;
    updateSnapshot();
  }

  void updateSnapshot() {
    const Snapshot snapshot{runId_, settling_, calibration_.state(), config_,
        calibration_.result(), sensor_.lastError(), sensor_.lastOperation(),
        sensor_.identity()};
    (void)xQueueOverwrite(channels_.latest, &snapshot);
  }
};
}  // namespace

void imuTask(void* context) {
  auto& channels = *static_cast<Channels*>(context);
  CalibrationRunner runner(channels);
  runner.start(false);
  while (true) {
    runner.tick();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
}  // namespace cleany::diagnostic
