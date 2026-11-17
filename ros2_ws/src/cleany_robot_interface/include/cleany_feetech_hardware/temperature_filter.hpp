#pragma once
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace cleany_feetech_hardware
{
struct TemperatureMacdSettings
{
  double fast_tau_s{}, slow_tau_s{}, signal_tau_s{};

  bool valid() const
  {
    return std::isfinite(fast_tau_s) && std::isfinite(slow_tau_s) &&
           std::isfinite(signal_tau_s) && fast_tau_s > 0 &&
           slow_tau_s > fast_tau_s && signal_tau_s > 0;
  }
};

/// Time-based MACD smoothing in Celsius: slow EMA + signal EMA(fast - slow).
class TemperatureMacd
{
public:
  using Clock = std::chrono::steady_clock;

  double update(double raw, Clock::time_point now, const TemperatureMacdSettings & settings)
  {
    if (!settings.valid() || !std::isfinite(raw) || raw < 0 || raw > 255) {
      throw std::runtime_error("Invalid temperature MACD input/settings");
    }
    if (!initialized_) {
      fast_ = slow_ = raw;
      signal_ = 0;
      initialized_ = true;
    } else {
      const double dt = std::chrono::duration<double>(now - previous_).count();
      if (dt < 0) {throw std::runtime_error("Temperature MACD clock moved backwards");}
      const auto ema = [dt](double value, double input, double tau) {
          return value - std::expm1(-dt / tau) * (input - value);
        };
      fast_ = ema(fast_, raw, settings.fast_tau_s);
      slow_ = ema(slow_, raw, settings.slow_tau_s);
      signal_ = ema(signal_, fast_ - slow_, settings.signal_tau_s);
    }
    previous_ = now;
    return slow_ + signal_;
  }

private:
  bool initialized_{false};
  double fast_{}, slow_{}, signal_{};
  Clock::time_point previous_{};
};
}  // namespace cleany_feetech_hardware
