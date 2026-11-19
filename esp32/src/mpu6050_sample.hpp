#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace cleany {

// Sensor-frame data, decoded for MPU6050 ranges +/-2 g and +/-250 degrees/s.
struct ImuSample {
  int64_t timestampUs = 0;
  std::array<double, 3> accelerationMps2{};
  std::array<double, 3> angularVelocityRadS{};
  double temperatureC = 0.0;
  bool saturated = false;
};

constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;

inline ImuSample decodeMpu6050Sample(const std::array<uint8_t, 14>& bytes,
                                    int64_t timestampUs) {
  std::array<int32_t, 7> raw{};
  bool saturated = false;
  for (size_t i = 0; i < raw.size(); ++i) {
    const uint32_t value = (static_cast<uint32_t>(bytes[2 * i]) << 8) |
                           bytes[2 * i + 1];
    raw[i] = value >= 0x8000 ? static_cast<int32_t>(value) - 65536
                            : static_cast<int32_t>(value);
    if (i != 3 && (raw[i] == -32768 || raw[i] == 32767)) saturated = true;
  }
  ImuSample result{};
  result.timestampUs = timestampUs;
  result.saturated = saturated;
  for (size_t axis = 0; axis < 3; ++axis) {
    result.accelerationMps2[axis] =
        static_cast<double>(raw[axis]) * kStandardGravityMps2 / 16384.0;
    result.angularVelocityRadS[axis] =
        static_cast<double>(raw[axis + 4]) * kDegreesToRadians / 131.0;
  }
  result.temperatureC = static_cast<double>(raw[3]) / 340.0 + 36.53;
  return result;
}

}  // namespace cleany
