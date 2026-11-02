#include <cassert>

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
  value = cleany::makeImuValue(sample, 0x68);
  assert(value.faultBits == cleany::kImuFaultSaturated);
  value = {};
  assert(value.faultBits == cleany::kImuFaultSensor);
  assert(!cleany::imuValueFresh(value, 1000000));
  cleany::ImuCalibrationGate calibrationGate;
  assert(calibrationGate.pending());
  cleany::WheelCommandGate wheelGate({250000, 10.0F});
  cleany::WheelCommand enable{};
  enable.sequence = 1;
  enable.validUntilUs = 200000;
  enable.mode = cleany::WheelMode::kEnable;
  assert(!cleany::acceptImuGatedWheelCommand(wheelGate, calibrationGate,
                                              enable, 100000));
  cleany::WheelCommand stop{};
  stop.mode = cleany::WheelMode::kStop;
  assert(cleany::acceptImuGatedWheelCommand(wheelGate, calibrationGate,
                                            stop, 100000));
  calibrationGate.complete();
  assert(!calibrationGate.pending());
  assert(!wheelGate.enabled());
  enable.sequence = 2;
  assert(cleany::acceptImuGatedWheelCommand(wheelGate, calibrationGate,
                                            enable, 100000, false) == false);
  assert(cleany::acceptImuGatedWheelCommand(wheelGate, calibrationGate,
                                            enable, 100000));
  cleany::WheelCommand velocity{};
  velocity.sequence = 3;
  velocity.validUntilUs = 200000;
  velocity.mode = cleany::WheelMode::kVelocity;
  velocity.velocity[0] = 1.0F;
  assert(cleany::acceptImuGatedWheelCommand(wheelGate, calibrationGate,
                                            velocity, 100000));
}
