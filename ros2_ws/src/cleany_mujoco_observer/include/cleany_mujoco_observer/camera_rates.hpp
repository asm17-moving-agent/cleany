#pragma once
#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>

namespace cleany_mujoco_observer {
struct CameraRates {
  double head_active=10, head_idle=2, wrist_active=10;
  void validate() const {
    for(auto rate: {head_active,head_idle,wrist_active})
      if(!std::isfinite(rate) || rate<1 || rate>30) throw std::invalid_argument("camera rate outside [1,30]");
    if(head_idle>head_active) throw std::invalid_argument("idle head rate exceeds active rate");
  }
  static bool known(const std::string& key) {return key=="head" || key=="left" || key=="right";}
  double rate(const std::string& active, const std::string& camera,
              bool head_depth_boost=false, bool head_high_rate=true) const {
    if(!known(active) || !known(camera)) throw std::invalid_argument("unknown camera");
    return camera=="head" ? ((active=="head" && head_high_rate) || head_depth_boost ? head_active : head_idle) : (active==camera ? wrist_active : 0);
  }
};

inline bool camera_is_due(double simulation_time, double last_capture_time,
                          double next_capture_time, double rate_hz) {
  if (!std::isfinite(simulation_time) || !std::isfinite(last_capture_time) ||
      !std::isfinite(next_capture_time) || !std::isfinite(rate_hz) || rate_hz < 0) {
    throw std::invalid_argument("camera schedule values must be finite and rate nonnegative");
  }
  return rate_hz > 0 && simulation_time > last_capture_time &&
    simulation_time + 1e-9 >= next_capture_time;
}
}
