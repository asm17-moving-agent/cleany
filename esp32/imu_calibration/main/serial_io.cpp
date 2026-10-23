#include "diagnostic_tasks.hpp"

#include <algorithm>
#include <array>
#include <deque>
#include <string>

#include "cJSON.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/task.h"

namespace cleany::diagnostic {
namespace {
constexpr char kSchema[] = "cleany.imu_calibration/v1";
constexpr size_t kMaximumQueuedMessages = 8;

template <size_t N>
void addArray(cJSON* object, const char* name, const std::array<double, N>& data) {
  cJSON* values = cJSON_CreateDoubleArray(data.data(), static_cast<int>(N));
  if (values != nullptr) cJSON_AddItemToObject(object, name, values);
}

cJSON* envelope(const char* type, uint32_t runId, const char* phase) {
  cJSON* message = cJSON_CreateObject();
  if (message != nullptr) {
    cJSON_AddStringToObject(message, "schema", kSchema);
    cJSON_AddStringToObject(message, "type", type);
    cJSON_AddNumberToObject(message, "run_id", runId);
    cJSON_AddStringToObject(message, "phase", phase);
  }
  return message;
}

cJSON* windowJson(const WindowStatistics& statistics) {
  cJSON* object = cJSON_CreateObject();
  if (object == nullptr) return nullptr;
  cJSON_AddNumberToObject(object, "samples", statistics.samples);
  cJSON_AddNumberToObject(object, "first_timestamp_us", statistics.firstTimestampUs);
  cJSON_AddNumberToObject(object, "last_timestamp_us", statistics.lastTimestampUs);
  cJSON_AddNumberToObject(object, "duration_s",
      (statistics.lastTimestampUs - statistics.firstTimestampUs) / 1e6);
  addArray(object, "gyro_mean_rad_s", statistics.gyroMeanRadS);
  addArray(object, "accel_mean_m_s2", statistics.accelMeanMps2);
  addArray(object, "gyro_covariance_rad2_s2", statistics.gyroCovariance);
  addArray(object, "accel_covariance_m2_s4", statistics.accelCovariance);
  cJSON_AddNumberToObject(object, "temperature_min_c", statistics.minTemperatureC);
  cJSON_AddNumberToObject(object, "temperature_max_c", statistics.maxTemperatureC);
  return object;
}

class SerialIo {
 public:
  explicit SerialIo(Channels& channels) : channels_(channels) {}

  void tick() {
    flushOutput();
    if (txQueue_.size() >= kMaximumQueuedMessages - 2) return;
    RunStarted started{};
    if (xQueueReceive(channels_.started, &started, 0) == pdTRUE) {
      cJSON* message = envelope("started", started.runId, "settling");
      if (message != nullptr) {
        cJSON_AddStringToObject(message, "trigger",
            started.commandTriggered ? "command" : "boot");
        cJSON_AddStringToObject(message, "request_id", started.requestId.data());
        enqueue(message);
      }
    }
    if (xQueuePeek(channels_.latest, &snapshot_, 0) == pdTRUE) {
      if (snapshot_.terminal() &&
          (!haveTerminal_ || lastTerminalRun_ != snapshot_.runId)) {
        emitResult();
        haveTerminal_ = true;
        lastTerminalRun_ = snapshot_.runId;
      }
      receiveCommands();
    }
  }

 private:
  Channels& channels_;
  Snapshot snapshot_{};
  std::deque<std::string> txQueue_;
  size_t txOffset_ = 0;
  std::string command_;
  bool commandOverflow_ = false;
  bool haveTerminal_ = false;
  uint32_t lastTerminalRun_ = 0;

  void enqueue(cJSON* object) {
    if (object == nullptr) return;
    char* text = cJSON_PrintUnformatted(object);
    cJSON_Delete(object);
    if (text == nullptr) return;
    if (txQueue_.size() < kMaximumQueuedMessages) {
      txQueue_.emplace_back(text);
      txQueue_.back().push_back('\n');
    }
    cJSON_free(text);
  }

  void flushOutput() {
    if (txQueue_.empty()) return;
    const auto& pending = txQueue_.front();
    const size_t count = std::min<size_t>(512, pending.size() - txOffset_);
    const int written = usb_serial_jtag_write_bytes(
        pending.data() + txOffset_, count, 0);
    if (written > 0) txOffset_ += static_cast<size_t>(written);
    if (txOffset_ == pending.size()) {
      txQueue_.pop_front();
      txOffset_ = 0;
    }
  }

  void emitError(const char* error) {
    cJSON* message = envelope("error", snapshot_.runId, snapshot_.phase());
    if (message != nullptr) {
      cJSON_AddStringToObject(message, "error", error);
      enqueue(message);
    }
  }

  void emitStatus() {
    if (snapshot_.terminal()) {
      emitResult();
      return;
    }
    cJSON* message = envelope("status", snapshot_.runId, snapshot_.phase());
    if (message == nullptr) return;
    cJSON_AddNumberToObject(message, "calibration_samples",
        snapshot_.result.calibration.samples);
    cJSON_AddNumberToObject(message, "validation_samples",
        snapshot_.result.validation.samples);
    enqueue(message);
  }

  void emitResult() {
    cJSON* message = envelope("result", snapshot_.runId, snapshot_.phase());
    if (message == nullptr) return;
    const auto& result = snapshot_.result;
    const auto& cfg = snapshot_.config;
    cJSON_AddBoolToObject(message, "accepted", result.accepted);
    cJSON_AddStringToObject(message, "error", calibrationErrorName(result.error));
    cJSON_AddStringToObject(message, "sensor_error",
        esp_err_to_name(snapshot_.sensorError));
    cJSON_AddStringToObject(message, "sensor_operation", snapshot_.sensorOperation);
    cJSON_AddNumberToObject(message, "who_am_i", snapshot_.identity);
    cJSON_AddBoolToObject(message, "motor_outputs_low", true);
    if (result.calibration.samples >= cfg.calibrationSamples) {
      addArray(message, "candidate_bias_rad_s", result.candidateBiasRadS);
    } else {
      cJSON_AddNullToObject(message, "candidate_bias_rad_s");
    }
    if (result.accepted) addArray(message, "bias_rad_s", result.candidateBiasRadS);
    else cJSON_AddNullToObject(message, "bias_rad_s");
    if (result.validation.samples >= cfg.validationSamples) {
      addArray(message, "validation_residual_rad_s", result.validationResidualRadS);
    } else {
      cJSON_AddNullToObject(message, "validation_residual_rad_s");
    }
    cJSON_AddItemToObject(message, "calibration", windowJson(result.calibration));
    cJSON_AddItemToObject(message, "validation", windowJson(result.validation));
    cJSON* config = cJSON_AddObjectToObject(message, "config");
    if (config != nullptr) {
      cJSON_AddNumberToObject(config, "sample_rate_hz", kSampleRateHz);
      cJSON_AddNumberToObject(config, "calibration_samples", cfg.calibrationSamples);
      cJSON_AddNumberToObject(config, "validation_samples", cfg.validationSamples);
      cJSON_AddNumberToObject(config, "settling_ms", CONFIG_CLEANY_IMU_SETTLING_MS);
      cJSON_AddNumberToObject(config, "max_sample_gap_us", cfg.maxSampleGapUs);
      cJSON_AddBoolToObject(config, "limits_reviewed", cfg.limitsReviewed);
      cJSON_AddNumberToObject(config, "max_gyro_std_rad_s", cfg.maxGyroStdRadS);
      cJSON_AddNumberToObject(config, "max_accel_std_m_s2", cfg.maxAccelStdMps2);
      cJSON_AddNumberToObject(config, "max_validation_mean_rad_s",
          cfg.maxValidationMeanRadS);
      cJSON_AddNumberToObject(config, "max_temperature_span_c", cfg.maxTemperatureSpanC);
      cJSON_AddNumberToObject(config, "gyro_range_deg_s", 250);
      cJSON_AddNumberToObject(config, "accel_range_g", 2);
      cJSON_AddNumberToObject(config, "gyro_dlpf_hz", 42);
      cJSON_AddNumberToObject(config, "i2c_clock_hz", 100000);
      cJSON_AddNumberToObject(config, "i2c_timeout_ms", CONFIG_CLEANY_IMU_I2C_TIMEOUT_MS);
      cJSON_AddNumberToObject(config, "imu_core", kImuCore);
      cJSON_AddNumberToObject(config, "serial_core", kSerialCore);
    }
    enqueue(message);
  }

  void restart() {
    RestartCommand request{};
    if (command_ != "calibrate") {
      // Optional explicit 16-hex-digit nonce: "calibrate <request_id>".
      if (command_.size() != 26 || command_.compare(0, 10, "calibrate ") != 0) {
        emitError("invalid_request_id");
        return;
      }
      for (size_t i = 0; i < 16; ++i) {
        const char value = command_[10 + i];
        if (!((value >= '0' && value <= '9') || (value >= 'a' && value <= 'f'))) {
          emitError("invalid_request_id");
          return;
        }
        request.requestId[i] = value;
      }
    }
    if (xQueueSend(channels_.restarts, &request, 0) != pdTRUE) {
      emitError("restart_queue_full");
    }
  }

  void receiveCommands() {
    if (txQueue_.size() >= kMaximumQueuedMessages - 2) return;
    char input[32];
    const int received = usb_serial_jtag_read_bytes(input, sizeof(input), 0);
    for (int i = 0; i < received; ++i) {
      const char byte = input[i];
      if (byte == '\r') continue;
      if (byte != '\n') {
        if (command_.size() < 32 && !commandOverflow_) command_.push_back(byte);
        else commandOverflow_ = true;
        continue;
      }
      if (commandOverflow_) emitError("command_too_long");
      else if (command_ == "status") emitStatus();
      else if (command_ == "calibrate" || command_.compare(0, 10, "calibrate ") == 0) {
        restart();
      } else emitError("unknown_command");
      command_.clear();
      commandOverflow_ = false;
    }
  }
};
}  // namespace

void serialTask(void* context) {
  auto& channels = *static_cast<Channels*>(context);
  SerialIo io(channels);
  while (true) {
    io.tick();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
}  // namespace cleany::diagnostic
