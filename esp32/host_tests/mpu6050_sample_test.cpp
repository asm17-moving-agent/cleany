#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "mpu6050_sample.hpp"

namespace {
std::array<uint8_t, 14> encode(const std::array<int32_t, 7>& raw) {
  std::array<uint8_t, 14> bytes{};
  for (size_t i = 0; i < raw.size(); ++i) {
    const auto value = static_cast<uint16_t>(raw[i]);
    bytes[2 * i] = static_cast<uint8_t>(value >> 8);
    bytes[2 * i + 1] = static_cast<uint8_t>(value);
  }
  return bytes;
}

void near(double actual, double expected) {
  assert(std::fabs(actual - expected) < 1e-10);
}
}  // namespace

int main() {
  // 부호 있는 big-endian 축 값을 SI 단위로 변환하고 측정 시각을 보존합니다.
  const auto s = cleany::decodeMpu6050Sample(
      encode({{16384, -16384, -1, -340, 131, -131, -1}}), 12345);
  assert(s.timestampUs == 12345 && !s.saturated);
  near(s.accelerationMps2[0], cleany::kStandardGravityMps2);
  near(s.accelerationMps2[1], -cleany::kStandardGravityMps2);
  near(s.accelerationMps2[2], -cleany::kStandardGravityMps2 / 16384);
  near(s.angularVelocityRadS[0], cleany::kDegreesToRadians);
  near(s.angularVelocityRadS[1], -cleany::kDegreesToRadians);
  near(s.angularVelocityRadS[2], -cleany::kDegreesToRadians / 131);
  // MPU6050 변환식으로 부호 있는 온도 레지스터 값을 환산합니다.
  near(s.temperatureC, 35.53);
  // 가속도계 ADC의 음수 끝값은 포화를 나타냅니다.
  assert(cleany::decodeMpu6050Sample(
      encode({{-32768, 0, 0, 0, 0, 0, 0}}), 0).saturated);
  // 자이로 ADC의 양수 끝값도 포화를 나타냅니다.
  assert(cleany::decodeMpu6050Sample(
      encode({{0, 0, 0, 0, 0, 0, 32767}}), 0).saturated);
  // 온도 레지스터의 끝값은 가속도계나 자이로의 포화가 아닙니다.
  assert(!cleany::decodeMpu6050Sample(
      encode({{0, 0, 0, -32768, 0, 0, 0}}), 0).saturated);
}
