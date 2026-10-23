#pragma once

#include <cstdint>
#include <array>
#include <atomic>

#include "mpu6050_sample.hpp"
#include "imu_startup_calibration.hpp"
#include "wheel_command_gate.hpp"

namespace cleany {

constexpr uint32_t kImuFaultSensor = 1;
constexpr uint32_t kImuFaultSaturated = 2;
constexpr int64_t kImuMaximumAgeUs = 100000;

struct ImuRuntimeValue {
  ImuSample sample{};
  uint8_t whoAmI = 0;
  uint32_t faultBits = kImuFaultSensor;
  ImuCalibrationValue calibration{};
};

class ImuCalibrationGate {
 public:
  bool pending() const { return pending_.load(std::memory_order_acquire); }
  void complete() { pending_.store(false, std::memory_order_release); }
 private:
  std::atomic<bool> pending_{true};
};

inline bool acceptImuGatedWheelCommand(WheelCommandGate& wheelGate,
                                       const ImuCalibrationGate& calibrationGate,
                                       const WheelCommand& command,
                                       uint64_t nowUs, bool allowMotion = true) {
  if (command.mode != WheelMode::kStop &&
      (calibrationGate.pending() || !allowMotion)) return false;
  return wheelGate.accept(command, nowUs);
}

inline ImuRuntimeValue makeImuValue(const ImuSample& sample, uint8_t whoAmI) {
  ImuRuntimeValue value{};
  value.sample = sample;
  value.whoAmI = whoAmI;
  value.faultBits = sample.saturated ? kImuFaultSaturated : 0;
  return value;
}

inline bool imuValueFresh(const ImuRuntimeValue& value, int64_t nowUs) {
  return value.sample.timestampUs > 0 && nowUs >= value.sample.timestampUs &&
         nowUs - value.sample.timestampUs <= kImuMaximumAgeUs;
}

struct ImuPublicationGate {
  bool consume(const ImuRuntimeValue& value, int64_t nowUs) {
    if (!imuValueFresh(value, nowUs)) return false;
    if (value.sample.timestampUs <= lastSequenceTimestampUs) return false;
    lastSequenceTimestampUs = value.sample.timestampUs;
    return true;
  }

 private:
  int64_t lastSequenceTimestampUs = 0;
};

}  // namespace cleany
