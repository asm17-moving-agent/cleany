#include <cassert>
#include <limits>

#include "imu_runtime.hpp"

int main() {
  cleany::ImuSample sample{};
  sample.timestampUs = 1000000;
  auto value = cleany::makeImuValue(sample, 0x68);
  assert(value.whoAmI == 0x68 && value.faultBits == 0);
  assert(cleany::imuValueFresh(value, 1100000));
  assert(!cleany::imuValueFresh(value, 1100001));
  assert(!cleany::imuValueFresh(value, 999999));
  cleany::ImuPublicationGate gate{};
  assert(gate.consume(value, 1000000));
  assert(!gate.consume(value, 1000000));
  sample.timestampUs++;
  value = cleany::makeImuValue(sample, 0x68);
  assert(gate.consume(value, 1000001));
  assert(!gate.consume(value, 1100002));
  sample.timestampUs = 1000000;
  assert(!gate.consume(cleany::makeImuValue(sample, 0x68), 1000002));
  sample.timestampUs = 0;
  assert(!cleany::imuValueFresh(cleany::makeImuValue(sample, 0x68), 1));

  sample.saturated = true;
  sample.timestampUs = 1000000;
  value = cleany::makeImuValue(sample, 0x68);
  assert(value.faultBits == cleany::kImuFaultSaturated);
  assert(!cleany::imuValueFresh(value, 1000000));
  value = {};
  assert(value.faultBits == cleany::kImuFaultSensor);
  assert(!cleany::imuValueFresh(value, 1000000));
  sample.saturated = false;
  sample.timestampUs = 1000000;
  sample.accelerationMps2[0] = std::numeric_limits<double>::infinity();
  assert(!cleany::imuValueFresh(cleany::makeImuValue(sample, 0x68), 1000000));
  sample.accelerationMps2[0] = 0;
  sample.angularVelocityRadS[2] = std::numeric_limits<double>::quiet_NaN();
  assert(!cleany::imuValueFresh(cleany::makeImuValue(sample, 0x68), 1000000));
  sample.angularVelocityRadS[2] = 0.1;
  value = cleany::makeImuValue(sample, 0x68);
  assert(value.sample.angularVelocityRadS[2] == 0.1);
  value.faultBits = cleany::kImuFaultSensor;
  assert(!cleany::imuValueFresh(value, 1000000));
  int64_t epoch = 0;
  assert(cleany::acquisitionEpochNanoseconds(2000000000, 500, epoch));
  assert(epoch == 1999500000);
  assert(!cleany::acquisitionEpochNanoseconds(0, 0, epoch));
  assert(!cleany::acquisitionEpochNanoseconds(2000000000, -1, epoch));
  assert(!cleany::acquisitionEpochNanoseconds(1000, 2, epoch));
  assert(!cleany::acquisitionEpochNanoseconds(2147483648000000000LL, 0, epoch));
  assert(!cleany::imuValueFresh(cleany::makeImuValue(sample, 0x69), 1000000));
}
