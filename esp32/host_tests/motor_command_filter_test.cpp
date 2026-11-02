#include <cassert>
#include <cmath>

#include "motor_command_filter.hpp"

namespace {

void testRampsCommand() {
  cleany::MotorCommandFilter filter;
  filter.setTargetSpeed(100);

  int64_t nowUs = 0;
  for (int expected = 1; expected <= 100; ++expected) {
    const int speed = filter.step(0.0F, nowUs);
    assert(speed == expected);
    nowUs += 10000;
  }
  const int steadySpeed = filter.step(0.0F, nowUs);
  assert(steadySpeed == 100);
}

void testWaitsForStoppedFeedbackBeforeReverse() {
  cleany::MotorCommandFilter filter;
  filter.setTargetSpeed(40);

  int64_t nowUs = 0;
  for (int i = 0; i < 40; ++i) {
    filter.step(5.0F, nowUs);
    nowUs += 10000;
  }
  assert(filter.appliedSpeed() == 40);

  filter.setTargetSpeed(-40);
  for (int i = 0; i < 40; ++i) {
    const int speed = filter.step(5.0F, nowUs);
    assert(speed >= 0);
    nowUs += 10000;
  }
  assert(filter.appliedSpeed() == 0);

  for (int i = 0; i < 10; ++i) {
    const int speed = filter.step(1.0F, nowUs);
    assert(speed == 0);
    assert(filter.waitingForReverse());
    nowUs += 10000;
  }

  for (int i = 0; i < 5; ++i) {
    const int speed = filter.step(0.0F, nowUs);
    assert(speed == 0);
    nowUs += 10000;
  }
  const int reversedSpeed = filter.step(0.0F, nowUs);
  assert(reversedSpeed == -1);
  assert(!filter.waitingForReverse());
}

void testForceStopIsImmediate() {
  cleany::MotorCommandFilter filter;
  filter.setTargetSpeed(100);
  filter.step(0.0F, 0);
  assert(filter.appliedSpeed() == 1);

  filter.forceStop();
  assert(filter.targetSpeed() == 0);
  assert(filter.appliedSpeed() == 0);
}

void testEncoderVelocityConversion() {
  const float velocity = cleany::encoderVelocityRadPerSecond(
      cleany::kEncoderCountsPerOutputRevolution, 1.0F);
  assert(std::fabs(velocity - cleany::kTwoPi) < 0.0001F);
}

void testConservativeRampInterval() {
  cleany::MotorFilterConfig config;
  config.slewStepIntervalUs = 20000;
  cleany::MotorCommandFilter filter(config);
  filter.setTargetSpeed(50);

  assert(filter.step(0.0F, 0) == 1);
  assert(filter.step(0.0F, 5000) == 1);
  assert(filter.step(0.0F, 15000) == 1);
  assert(filter.step(0.0F, 20000) == 2);
  assert(filter.step(0.0F, 39999) == 2);
  assert(filter.step(0.0F, 40000) == 3);
}

void testAsymmetricDecelSlew() {
  cleany::MotorFilterConfig config;
  config.slewStepPercent = 1;
  config.decelSlewStepPercent = 3;
  cleany::MotorCommandFilter filter(config);

  // Accel: steps by 1
  filter.setTargetSpeed(10);
  assert(filter.step(0.0F, 0) == 1);
  assert(filter.step(0.0F, 0) == 2);
  assert(filter.step(0.0F, 0) == 3);

  // Decel to 0: steps down by 3
  filter.setTargetSpeed(0);
  assert(filter.step(0.0F, 0) == 0);  // 3 -> 0 directly (3 step)
}

}  // namespace

int main() {
  testRampsCommand();
  testWaitsForStoppedFeedbackBeforeReverse();
  testForceStopIsImmediate();
  testEncoderVelocityConversion();
  testConservativeRampInterval();
  testAsymmetricDecelSlew();
}
