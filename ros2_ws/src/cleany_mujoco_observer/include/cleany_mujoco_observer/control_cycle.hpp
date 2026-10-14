#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace cleany_mujoco_observer
{
struct ControlCycle
{
  int64_t stamp_ns;
  int64_t period_ns;

  template<class Manager>
  void execute(Manager& manager) const
  {
    manager.read(stamp_ns, period_ns);
    manager.update(stamp_ns, period_ns);
    manager.write(stamp_ns, period_ns);
  }
};

// A delayed read must not change the timestamp passed to update/write.
// A paused clock or a catch-up iteration cannot produce another state sample.
class ControlCycleGate
{
public:
  explicit ControlCycleGate(int64_t period_ns) : nominal_period_ns_(period_ns)
  {
    if (period_ns <= 0) {throw std::invalid_argument("Control period must be positive");}
  }

  std::optional<ControlCycle> take(int64_t now_ns)
  {
    if (previous_ns_ && now_ns < *previous_ns_) {
      throw std::runtime_error("Simulation clock moved backwards during control");
    }
    if (!previous_ns_) {
      previous_ns_ = now_ns;
      return std::nullopt;
    }
    const auto elapsed = now_ns - *previous_ns_;
    if (elapsed < nominal_period_ns_) {return std::nullopt;}
    previous_ns_ = now_ns;
    return ControlCycle{now_ns, elapsed};
  }

  int64_t next_stamp_ns() const {return *previous_ns_ + nominal_period_ns_;}

private:
  int64_t nominal_period_ns_;
  std::optional<int64_t> previous_ns_;
};
}  // namespace cleany_mujoco_observer
