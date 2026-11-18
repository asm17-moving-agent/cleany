#pragma once

#include <cstdint>
#include <cmath>

#include "mpu6050_sample.hpp"

namespace cleany {

constexpr uint32_t kImuFaultSensor = 1;
constexpr uint32_t kImuFaultSaturated = 2;
constexpr int64_t kImuMaximumAgeUs = 100000;

struct ImuRuntimeValue {
  ImuSample sample{};
  uint8_t whoAmI = 0;
  uint32_t faultBits = kImuFaultSensor;
};

inline ImuRuntimeValue makeImuValue(const ImuSample& sample, uint8_t whoAmI) {
  ImuRuntimeValue value{};
  value.sample = sample;
  value.whoAmI = whoAmI;
  value.faultBits = sample.saturated ? kImuFaultSaturated : 0;
  return value;
}

inline bool imuValueFresh(const ImuRuntimeValue& value, int64_t nowUs) {
  if (value.faultBits != 0 || value.whoAmI != 0x68 || value.sample.saturated ||
      value.sample.timestampUs <= 0 || nowUs < value.sample.timestampUs ||
      nowUs - value.sample.timestampUs > kImuMaximumAgeUs) return false;
  for (size_t i = 0; i < 3; ++i)
    if (!std::isfinite(value.sample.angularVelocityRadS[i]) ||
        !std::isfinite(value.sample.accelerationMps2[i])) return false;
  return true;
}

inline bool acquisitionEpochNanoseconds(int64_t epochNowNs, int64_t sampleAgeUs,
                                        int64_t& result) {
  if (epochNowNs <= 0 || sampleAgeUs < 0 ||
      sampleAgeUs > epochNowNs / 1000) return false;
  result = epochNowNs - sampleAgeUs * 1000;
  return result > 0 && result / 1000000000LL <= INT32_MAX;
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
