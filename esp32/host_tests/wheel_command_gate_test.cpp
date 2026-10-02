#include "wheel_command_gate.hpp"

#include <cassert>
#include <limits>

using cleany::WheelCommand;
using cleany::WheelCommandGate;
using cleany::WheelMode;

static WheelCommand command(WheelMode mode, uint32_t seq, uint64_t deadline) {
  WheelCommand c;
  c.sequence = seq;
  c.validUntilUs = deadline;
  c.mode = mode;
  return c;
}

static void validationAndReplay() {
  WheelCommandGate gate({250000, 10.0F});
  auto enable = command(WheelMode::kEnable, 10, 350);
  assert(!gate.accept(command(WheelMode::kVelocity, 11, 350), 100));
  enable.velocity[0] = 1;
  assert(!gate.accept(enable, 100));
  enable.velocity.fill(0);
  auto bad = enable;
  bad.protocolVersion = 1;
  assert(!gate.accept(bad, 100));
  bad = enable; bad.validUntilUs = 100;
  assert(!gate.accept(bad, 100));
  bad = enable; bad.validUntilUs = 350101;
  assert(!gate.accept(bad, 100));
  bad = enable; bad.velocity[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!gate.accept(bad, 100));
  bad = command(static_cast<WheelMode>(255), 10, 350);
  assert(!gate.accept(bad, 100));
  assert(gate.lastSequence() == 0 && gate.target()[0] == 0);
  assert(gate.accept(enable, 100));
  const auto deadline = enable.validUntilUs;
  const auto targetBefore = gate.target();
  assert(!gate.accept(enable, 101));  // Duplicate cannot refresh.
  assert(!gate.accept(command(WheelMode::kEnable, 11, 400), 101));
  auto velocity = command(WheelMode::kVelocity, 11, 350);
  velocity.velocity = {1, 2, 3, 4};
  assert(gate.accept(velocity, 110));
  const auto target = gate.target();
  const auto acceptedAt = gate.lastAcceptedUs();
  for (float v : {11.0F, -11.0F, std::numeric_limits<float>::quiet_NaN(),
                  std::numeric_limits<float>::infinity(),
                  -std::numeric_limits<float>::infinity()}) {
    bad = command(WheelMode::kVelocity, 12, 550);
    bad.velocity[0] = v;
    assert(!gate.accept(bad, 111));
    assert(gate.lastSequence() == 11);
    assert(gate.target() == target && gate.lastAcceptedUs() == acceptedAt);
  }
  assert(!gate.accept(command(WheelMode::kEnable, 12, 550), 111));
  assert(gate.target() == target && gate.lastAcceptedUs() == acceptedAt);
  assert(gate.lastAcceptedUs() >= 100 && deadline == 350 && targetBefore[0] == 0);
  auto halfRange = command(WheelMode::kVelocity, 11U + 0x80000000U, 350);
  assert(!gate.accept(halfRange, 111));
  assert(!gate.expire(349));
  assert(gate.expire(350));
  assert(!gate.enabled() && gate.target()[0] == 0 && gate.timedOut() &&
         gate.lastAcceptedUs() == 0);
  assert(!gate.accept(velocity, 350));  // Permission is not renewed by motion.
  auto replay = command(WheelMode::kEnable, 10, 600);
  assert(!gate.accept(replay, 351));  // Replaying sequence with a new deadline is stale.
  auto freshVelocity = command(WheelMode::kVelocity, 12, 600);
  freshVelocity.velocity = {1, 2, 3, 4};
  assert(!gate.accept(freshVelocity, 351));  // Must explicitly enable again.
  auto fresh = command(WheelMode::kEnable, 12, 600);
  assert(gate.accept(fresh, 351));
  assert(!gate.timedOut());
}

static void stopSequenceAndReconnect() {
  WheelCommandGate gate({250000, 10.0F});
  assert(gate.accept(command(WheelMode::kEnable, 0xfffffffeU, 1100), 1000));
  auto stop = command(WheelMode::kStop, 0xffffffffU, 0);
  assert(gate.accept(stop, 1001));
  assert(!gate.enabled() && gate.lastSequence() == 0xffffffffU);
  assert(!gate.accept(command(WheelMode::kEnable, 0xfffffffeU, 1200), 1002));
  assert(gate.accept(command(WheelMode::kEnable, 0, 1200), 1002));
  gate.disconnect();
  assert(!gate.enabled());
  assert(!gate.accept(command(WheelMode::kEnable, 0, 1200), 1003));
  assert(gate.accept(command(WheelMode::kEnable, 1, 1200), 1003));
  gate.disconnect();
  assert(gate.lastSequence() == 1);
  auto oldStop = command(WheelMode::kStop, 0, 0);
  oldStop.protocolVersion = 99;
  oldStop.velocity[0] = std::numeric_limits<float>::quiet_NaN();
  assert(gate.accept(oldStop, 1004));  // Output stop unconditional, sequence stays high.
  assert(gate.lastSequence() == 1);

  WheelCommandGate wrap({250000, 10.0F});
  assert(wrap.accept(command(WheelMode::kEnable, 0xffffffffU, 1200), 1000));
  wrap.disconnect();
  assert(wrap.lastSequence() == 0xffffffffU);
  assert(wrap.accept(command(WheelMode::kEnable, 0, 1200), 1001));
}

int main() {
  validationAndReplay();
  stopSequenceAndReconnect();
}
