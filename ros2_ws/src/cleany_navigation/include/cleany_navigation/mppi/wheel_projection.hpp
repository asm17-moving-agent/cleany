// Copyright (c) 2026 Cleany contributors
// Licensed under the Apache License, Version 2.0.
#ifndef CLEANY_NAVIGATION_MPPI__WHEEL_PROJECTION_HPP_
#define CLEANY_NAVIGATION_MPPI__WHEEL_PROJECTION_HPP_
#include <cmath>
#include <algorithm>
namespace cleany_navigation::mppi
{
// These four wheel signs imply max|omega|=(|vx|+|vy|+lever*|wz|)/radius.
// Scaling all axes together preserves the complete body twist direction.
inline void projectWheelTwist(float & vx, float & vy, float & wz,
  float radius, float lever, float maximum)
{
  if (maximum <= 0.0f) {return;}
  const float demand = std::abs(vx) + std::abs(vy) + lever * std::abs(wz);
  const float bound = radius * maximum;
  if (demand > bound) {
    const float scale = bound / demand;
    vx *= scale; vy *= scale; wz *= scale;
  }
}
}  // namespace cleany_navigation::mppi
#endif  // CLEANY_NAVIGATION_MPPI__WHEEL_PROJECTION_HPP_
