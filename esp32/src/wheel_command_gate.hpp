#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace cleany {

enum class WheelMode : uint8_t { kStop = 0, kEnable = 1, kVelocity = 2 };

struct WheelCommand {
  uint8_t protocolVersion = 2;
  uint32_t sequence = 0;
  uint64_t validUntilUs = 0;
  WheelMode mode = WheelMode::kStop;
  std::array<float, 4> velocity{};
};

struct WheelGateConfig {
  uint32_t maximumCommandLifetimeUs = 250000;
  float maxVelocity = 10.0F;
};

class WheelCommandGate {
 public:
  explicit WheelCommandGate(WheelGateConfig config) : config_(config) {}

  bool accept(const WheelCommand& c, uint64_t nowUs) {
    if (c.mode == WheelMode::kStop) {
      if (!haveSequence_ || newer(c.sequence, lastSequence_)) {
        lastSequence_ = c.sequence;
        haveSequence_ = true;
      }
      disable();
      return true;
    }
    if (c.protocolVersion != 2 || c.validUntilUs <= nowUs ||
        c.validUntilUs - nowUs > config_.maximumCommandLifetimeUs || !newer(c.sequence))
      return false;
    if (c.mode == WheelMode::kEnable) {
      if (!zero(c)) return false;
      if (enabled_) return false;
      enabled_ = true;
      timedOut_ = false;
    } else if (c.mode == WheelMode::kVelocity) {
      if (!enabled_ || !finiteAndBounded(c)) return false;
      target_ = c.velocity;
    } else {
      return false;
    }
    lastSequence_ = c.sequence;
    haveSequence_ = true;
    validUntilUs_ = c.validUntilUs;
    lastAcceptedUs_ = nowUs;
    return true;
  }

  bool expire(uint64_t nowUs) {
    if (enabled_ && nowUs >= validUntilUs_) {
      disable();
      timedOut_ = true;
      return true;
    }
    return false;
  }
  void disconnect() { disable(); }
  bool enabled() const { return enabled_; }
  bool timedOut() const { return timedOut_; }
  uint32_t lastSequence() const { return lastSequence_; }
  uint64_t lastAcceptedUs() const { return lastAcceptedUs_; }
  const std::array<float, 4>& target() const { return target_; }

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
  void disable() {
    enabled_ = false;
    target_.fill(0.0F);
    validUntilUs_ = 0;
    lastAcceptedUs_ = 0;
  }
  WheelGateConfig config_;
  uint32_t lastSequence_ = 0;
  bool haveSequence_ = false, enabled_ = false, timedOut_ = false;
  uint64_t validUntilUs_ = 0;
  uint64_t lastAcceptedUs_ = 0;
  std::array<float, 4> target_{};
};
}  // namespace cleany
