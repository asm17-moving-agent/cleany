#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace cleany_navigation::mppi {
struct PlanarVelocity {float x{}, y{}, yaw{};};

/// Interpolate the entire twist so axis and all four wheel accelerations agree.
inline PlanarVelocity boundedVelocityStep(
  const PlanarVelocity & from, const PlanarVelocity & target, float dt,
  float ax, float ay, float az, float wheel_acceleration, float radius, float lever)
{
  const float dx=target.x-from.x,dy=target.y-from.y,dz=target.yaw-from.yaw;
  float ratio=1.f;
  const auto bound=[&](float delta,float limit) {
    if(std::abs(delta)>1e-8f)ratio=std::min(ratio,limit*dt/std::abs(delta));
  };
  bound(dx,ax);bound(dy,ay);bound(dz,az);
  if(wheel_acceleration>0.f) {
    for(float delta:{dx-dy-lever*dz,dx+dy+lever*dz,dx+dy-lever*dz,dx-dy+lever*dz}) {
      bound(delta/radius,wheel_acceleration);
    }
  }
  return {from.x+ratio*dx,from.y+ratio*dy,from.yaw+ratio*dz};
}
}
