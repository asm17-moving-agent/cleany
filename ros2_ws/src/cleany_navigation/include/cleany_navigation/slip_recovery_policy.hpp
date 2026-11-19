#pragma once
#include <cmath>
#include <stdexcept>
#include <string>

namespace cleany_navigation {
/// One latched longitudinal retry per goal; standstill never clears the latch.
class SlipRecoveryPolicy {
public:
  enum class Phase {Normal, Braking, Retry, Failed};
  struct Settings {double unknown_timeout=3., stop_timeout=5., settle=.5;};
  explicit SlipRecoveryPolicy(Settings settings): settings_(settings) {
    if (!(settings.unknown_timeout>0 && settings.stop_timeout>settings.settle && settings.settle>0) ||
      !std::isfinite(settings.unknown_timeout+settings.stop_timeout+settings.settle))
      throw std::invalid_argument("Invalid slip recovery timing");
  }
  void reset(double now) {phase=Phase::Normal; phase_at_=known_at_=now; stopped_at_=-1; abort_=false; reason="monitoring";}
  void fail(double now, const std::string & why) {
    if (phase!=Phase::Braking) {phase_at_=now;stopped_at_=-1;}
    phase=Phase::Braking;abort_=true;reason=why;
  }
  void update(double now, bool reliable, bool confirmed, bool stopped) {
    if (phase==Phase::Failed) return;
    if (reliable) known_at_=now;
    if (phase==Phase::Braking) {
      if (stopped) {if(stopped_at_<0)stopped_at_=now;} else stopped_at_=-1;
      if (stopped_at_>=0 && now-stopped_at_>=settings_.settle) {
        phase=abort_?Phase::Failed:Phase::Retry;
        reason=abort_?reason:"longitudinal_retry";known_at_=now;
      } else if(now-phase_at_>settings_.stop_timeout) {phase=Phase::Failed;reason="stop_unverified";}
      return;
    }
    if(now-known_at_>settings_.unknown_timeout) {fail(now,"slip_measurement_unavailable");return;}
    if (reliable && confirmed) {
      if(phase==Phase::Retry) {fail(now,"slip_repeated_during_retry");return;}
      phase=Phase::Braking;phase_at_=now;stopped_at_=-1;reason="persistent_slip";
    }
  }
  Phase phase=Phase::Normal;
  std::string reason="monitoring";
private:
  Settings settings_;
  double phase_at_=0, known_at_=0, stopped_at_=-1;
  bool abort_=false;
};
}
