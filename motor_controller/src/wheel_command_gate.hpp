#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace cleany {

enum class WheelMode : uint8_t { kStop = 0, kBeginSession = 1, kArm = 2, kVelocity = 3 };

struct WheelCommand {
  uint8_t protocolVersion = 1;
  uint32_t bootId = 0;
  uint32_t sessionId = 0;
  uint32_t sequence = 0;
  uint64_t validUntilUs = 0;
  WheelMode mode = WheelMode::kStop;
  std::array<float, 4> velocity{};
};

struct WheelGateConfig {
  uint32_t bootId = 1;
  uint32_t watchdogUs = 250000;
  float maxVelocity = 10.0F;
};

class WheelCommandGate {
 public:
  explicit WheelCommandGate(WheelGateConfig config) : config_(config) {}

  bool accept(const WheelCommand& command, uint64_t nowUs) {
    if (command.mode == WheelMode::kStop) {
      disarm(nowUs);
      return true;
    }
    if (command.protocolVersion != 1 || command.bootId != config_.bootId ||
        command.sessionId == 0 || command.validUntilUs <= nowUs ||
        command.validUntilUs - nowUs > config_.watchdogUs) return false;
    if (command.mode == WheelMode::kBeginSession) {
      if (!zero(command)) return false;
      if (nowUs < sessionHoldoffUntilUs_ || command.sessionId == sessionId_ ||
          command.sessionId == retiredSessionId_ || recentlyRetired(command.sessionId, nowUs))
        return false;
      if (!rememberSession(nowUs)) return false;
      sessionId_ = command.sessionId;
      haveSequence_ = true;
      lastSequence_ = command.sequence;
      armed_ = false;
      target_.fill(0.0F);
      lastAcceptedUs_ = 0;
      validUntilUs_ = 0;
      return true;
    }
    if (command.sessionId != sessionId_ || !newer(command.sequence)) return false;
    if (!finiteAndBounded(command)) return false;
    if (command.mode == WheelMode::kArm) {
      if (!zero(command)) return false;
      armed_ = true;
      target_.fill(0.0F);
    } else if (command.mode == WheelMode::kVelocity) {
      if (!armed_) return false;
      target_ = command.velocity;
    } else {
      return false;
    }
    lastSequence_ = command.sequence;
    haveSequence_ = true;
    lastAcceptedUs_ = nowUs;
    validUntilUs_ = command.validUntilUs;
    return true;
  }

  bool watchdog(uint64_t nowUs) {
    if (armed_ && (nowUs >= validUntilUs_ ||
                   nowUs - lastAcceptedUs_ >= config_.watchdogUs)) {
      disarm(nowUs);
      return true;
    }
    return false;
  }
  void disconnect(uint64_t nowUs) { disarm(nowUs); }
  bool armed() const { return armed_; }
  uint32_t sessionId() const { return sessionId_; }
  uint32_t lastSequence() const { return lastSequence_; }
  const std::array<float, 4>& target() const { return target_; }
  uint64_t lastAcceptedUs() const { return lastAcceptedUs_; }

 private:
  static bool newer(uint32_t n, uint32_t old) {
    const uint32_t delta = n - old;
    return delta != 0 && delta < 0x80000000U;
  }
  bool newer(uint32_t n) const { return !haveSequence_ || newer(n, lastSequence_); }
  static bool zero(const WheelCommand& c) {
    for (float v : c.velocity) if (v != 0.0F) return false;
    return true;
  }
  bool finiteAndBounded(const WheelCommand& c) const {
    for (float v : c.velocity)
      if (!std::isfinite(v) || std::fabs(v) > config_.maxVelocity) return false;
    return true;
  }
  bool recentlyRetired(uint32_t id, uint64_t nowUs) const {
    for (const auto& retired : retiredSessions_)
      if (retired.id == id && nowUs < retired.untilUs) return true;
    return false;
  }
  bool rememberSession(uint64_t nowUs) {
    if (sessionId_ == 0) return true;
    retiredSessionId_ = sessionId_;
    for (auto& retired : retiredSessions_) {
      if (retired.id == sessionId_ || retired.id == 0 || nowUs >= retired.untilUs) {
        retired = {sessionId_, nowUs + config_.watchdogUs};
        return true;
      }
    }
    // Never evict a session whose queued packets can still be valid.
    sessionHoldoffUntilUs_ = nowUs + config_.watchdogUs;
    return false;
  }
  void disarm(uint64_t nowUs) {
    (void)rememberSession(nowUs);
    armed_ = false;
    target_.fill(0.0F);
    lastAcceptedUs_ = 0;
    validUntilUs_ = 0;
    sessionId_ = 0;
    haveSequence_ = false;
  }
  WheelGateConfig config_;
  struct RetiredSession { uint32_t id = 0; uint64_t untilUs = 0; };
  std::array<RetiredSession, 8> retiredSessions_{};
  uint32_t sessionId_ = 0, retiredSessionId_ = 0, lastSequence_ = 0;
  bool armed_ = false, haveSequence_ = false;
  uint64_t lastAcceptedUs_ = 0, validUntilUs_ = 0, sessionHoldoffUntilUs_ = 0;
  std::array<float, 4> target_{};
};
}  // namespace cleany
