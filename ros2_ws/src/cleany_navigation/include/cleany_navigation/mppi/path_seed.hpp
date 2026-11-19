// Experiment: coherent path proposals, still evaluated by every MPPI critic.
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include "cleany_navigation/mppi/wheel_projection.hpp"
namespace cleany_navigation::mppi {
struct SeedPose {float x, y, yaw;};
struct SeedControl {float x, y, yaw;};
inline float wrapSeedAngle(float a) {return std::atan2(std::sin(a), std::cos(a));}
inline std::vector<SeedControl> pathSeed(
  SeedPose current, const std::vector<SeedPose> & path, size_t steps,
  float dt, float speed, float yaw_speed, float radius, float lever, float wheel_cap,
  size_t start_index = 0)
{
  std::vector<SeedControl> result; result.reserve(steps);
  size_t target = path.empty() ? 0 : std::min(start_index,path.size()-1);
  for (size_t t=0; t<steps; ++t) {
    while (target+1<path.size() && std::hypot(path[target].x-current.x,path[target].y-current.y)<0.012f &&
      std::abs(wrapSeedAngle(path[target].yaw-current.yaw))<0.025f) {++target;}
    if (path.empty()) {result.push_back({0,0,0}); continue;}
    const float dx=path[target].x-current.x, dy=path[target].y-current.y;
    const float da=wrapSeedAngle(path[target].yaw-current.yaw);
    // Interpolate all three coordinates with one fraction, preserving the
    // coupled connector to the lattice pose, instead of rotating in place.
    const float duration=std::max({dt,std::hypot(dx,dy)/speed,std::abs(da)/yaw_speed});
    const float wx=dx/duration, wy=dy/duration;
    SeedControl u{std::cos(current.yaw)*wx+std::sin(current.yaw)*wy,
      -std::sin(current.yaw)*wx+std::cos(current.yaw)*wy,da/duration};
    if (wheel_cap>0) {projectWheelTwist(u.x,u.y,u.yaw,radius,lever,wheel_cap);}
    current.x+=dt*(std::cos(current.yaw)*u.x-std::sin(current.yaw)*u.y);
    current.y+=dt*(std::sin(current.yaw)*u.x+std::cos(current.yaw)*u.y);
    current.yaw+=dt*u.yaw;
    result.push_back(u);
  }
  return result;
}
}
