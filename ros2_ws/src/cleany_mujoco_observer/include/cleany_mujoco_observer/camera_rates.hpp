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
  double rate(const std::string& active, const std::string& camera) const {
    if(!known(active) || !known(camera)) throw std::invalid_argument("unknown camera");
    return camera=="head" ? (active=="head" ? head_active : head_idle) : (active==camera ? wrist_active : 0);
  }
};
}
