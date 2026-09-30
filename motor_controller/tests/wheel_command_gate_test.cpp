#include "wheel_command_gate.hpp"

#include <cassert>
#include <limits>

using cleany::WheelCommand;
using cleany::WheelCommandGate;
using cleany::WheelMode;

static WheelCommand command(WheelMode mode, uint32_t seq, uint64_t now,
                            uint32_t session = 7) {
  WheelCommand c;
  c.bootId = 42;
  c.sessionId = session;
  c.sequence = seq;
  c.validUntilUs = now + 250000;
  c.mode = mode;
  return c;
}

static void arm(WheelCommandGate& gate, uint64_t now, uint32_t session = 7) {
  assert(gate.accept(command(WheelMode::kBeginSession, 0, now, session), now));
  assert(!gate.armed() && gate.target()[0] == 0);
  assert(gate.accept(command(WheelMode::kArm, 1, now + 1, session), now + 1));
  assert(gate.armed() && gate.target()[0] == 0);
}

static void validation() {
  WheelCommandGate gate({42, 250000, 10.0F});
  auto begin = command(WheelMode::kBeginSession, 0, 100);
  auto bad = begin;
  bad.bootId = 41;
  assert(!gate.accept(bad, 100));
  bad = begin; bad.protocolVersion = 2;
  assert(!gate.accept(bad, 100));
  bad = begin; bad.sessionId = 0;
  assert(!gate.accept(bad, 100));
  bad = begin; bad.validUntilUs = 100;
  assert(!gate.accept(bad, 100));  // Equality is expired.
  bad = begin; bad.validUntilUs = 250101;
  assert(!gate.accept(bad, 100));  // Future horizon is bounded.
  bad = begin; bad.velocity[0] = 1;
  assert(!gate.accept(bad, 100));
  assert(gate.accept(begin, 100));
  auto velocity = command(WheelMode::kVelocity, 1, 101);
  assert(!gate.accept(velocity, 101));  // Explicit ARM is mandatory.
  auto a = command(WheelMode::kArm, 1, 102);
  a.velocity[0] = 1;
  assert(!gate.accept(a, 102));
  a.velocity.fill(0);
  assert(gate.accept(a, 102));
  velocity = command(WheelMode::kVelocity, 2, 103);
  velocity.velocity = {1, 2, 3, 4};
  assert(gate.accept(velocity, 103));
  const auto target = gate.target();
  const auto accepted = gate.lastAcceptedUs();
  assert(!gate.accept(velocity, 104));  // Duplicates cannot refresh watchdog.
  for (float invalid : {10.1F, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity(),
                        -std::numeric_limits<float>::infinity()}) {
    bad = command(WheelMode::kVelocity, 3, 105);
    bad.velocity[0] = invalid;
    assert(!gate.accept(bad, 105));
    assert(gate.lastAcceptedUs() == accepted && gate.target() == target);
  }
  bad = command(WheelMode::kVelocity, 3, 105);
  bad.mode = static_cast<WheelMode>(255);
  assert(!gate.accept(bad, 105));
  bad = command(WheelMode::kVelocity, 3, 105, 8);
  assert(!gate.accept(bad, 105));
  bad = command(WheelMode::kVelocity, 0x80000002U, 105);
  assert(!gate.accept(bad, 105));
  assert(!gate.watchdog(250102));
  assert(gate.watchdog(250103));
  assert(!gate.armed() && gate.sessionId() == 0 && gate.target()[0] == 0);
  assert(!gate.watchdog(250104));  // Idle/disarmed is not a timeout event.
  assert(!gate.accept(command(WheelMode::kBeginSession, 4, 250104), 250104));
  assert(!gate.accept(command(WheelMode::kArm, 4, 250104), 250104));
  assert(!gate.accept(command(WheelMode::kVelocity, 4, 250104), 250104));
}

static void deadlineAndStop() {
  WheelCommandGate gate({42, 250000, 10.0F});
  arm(gate, 100);
  auto v = command(WheelMode::kVelocity, 2, 110);
  v.validUntilUs = 140;
  v.velocity = {1, 2, 3, 4};
  assert(gate.accept(v, 110));
  assert(!gate.watchdog(139));
  assert(gate.watchdog(140));  // Deadline is rechecked at each control tick.
  assert(!gate.accept(command(WheelMode::kBeginSession, 3, 141), 141));
  arm(gate, 150, 8);
  auto stop = command(WheelMode::kStop, 0, 160);
  stop.bootId = stop.sessionId = 0;
  stop.protocolVersion = 255;
  stop.validUntilUs = 0;
  stop.velocity[0] = std::numeric_limits<float>::quiet_NaN();
  assert(gate.accept(stop, 160));  // STOP ignores all metadata.
  assert(!gate.armed() && gate.sessionId() == 0);
  assert(!gate.accept(command(WheelMode::kBeginSession, 4, 161, 8), 161));
  assert(!gate.accept(command(WheelMode::kBeginSession, 4, 161, 7), 161));
  arm(gate, 170, 9);
  gate.disconnect(172);
  assert(!gate.armed() && gate.target()[0] == 0);
  assert(!gate.accept(command(WheelMode::kBeginSession, 2, 173, 9), 173));
}

static void wrapAndRetirementCapacity() {
  WheelCommandGate wrap({42, 250000, 10.0F});
  assert(wrap.accept(command(WheelMode::kBeginSession, 0xfffffffeU, 100), 100));
  assert(wrap.accept(command(WheelMode::kArm, 0xffffffffU, 101), 101));
  assert(wrap.accept(command(WheelMode::kVelocity, 0, 102), 102));
  assert(!wrap.accept(command(WheelMode::kVelocity, 0x80000000U, 103), 103));
  assert(!wrap.accept(command(WheelMode::kBeginSession, 1, 104), 104));
  assert(wrap.armed());  // A replayed BEGIN cannot change the active controller.

  WheelCommandGate gate({42, 250000, 10.0F});
  for (uint32_t session = 1; session <= 9; ++session) {
    const uint64_t now = 100 + session * 10;
    assert(gate.accept(command(WheelMode::kBeginSession, 0, now, session), now));
  }
  assert(!gate.accept(command(WheelMode::kBeginSession, 1, 200, 1), 200));
  assert(!gate.accept(command(WheelMode::kBeginSession, 1, 200, 10), 200));
  assert(gate.sessionId() == 9);  // Saturation never evicts live retirement records.
  assert(gate.accept(command(WheelMode::kStop, 0, 201), 201));
  assert(!gate.accept(command(WheelMode::kBeginSession, 1, 202, 10), 202));
  assert(gate.accept(command(WheelMode::kBeginSession, 1, 250201, 10), 250201));
}

int main() {
  validation();
  deadlineAndStop();
  wrapAndRetirementCapacity();
}
