#pragma once
#include <algorithm>
#include <cmath>
namespace mppi {
inline float segmentPoseDistanceSquared(float x,float y,float yaw,
  float ax,float ay,float aa,float bx,float by,float ba,float radius)
{
  const auto wrap=[](float a){return std::atan2(std::sin(a),std::cos(a));};
  const float dx=bx-ax,dy=by-ay,da=wrap(ba-aa);
  const float ex=x-ax,ey=y-ay,ea=wrap(yaw-aa),r2=radius*radius;
  const float denom=dx*dx+dy*dy+r2*da*da;
  const float u=denom>1e-9f ? std::clamp((ex*dx+ey*dy+r2*ea*da)/denom,0.0f,1.0f) : 0;
  const float a=wrap(yaw-aa-u*da);
  return (ex-u*dx)*(ex-u*dx)+(ey-u*dy)*(ey-u*dy)+r2*a*a;
}
}
