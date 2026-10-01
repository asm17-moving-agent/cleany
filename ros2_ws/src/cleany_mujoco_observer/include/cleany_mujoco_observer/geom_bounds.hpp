#pragma once

#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace cleany_mujoco_observer
{
struct GeomBounds
{
  std::array<double, 3> low, high;
};

// Evaluation only. A transformed local box is not a tight bound for a tilted
// mesh/cylinder and can falsely put a resting object's bottom below the table.
inline GeomBounds geomBoundsInBase(const mjModel* model, const mjData* data, int g, int base)
{
  mjtNum delta[3], center[3], relative[9];
  mju_sub3(delta, data->geom_xpos + 3*g, data->xpos + 3*base);
  mju_mulMatTVec3(center, data->xmat + 9*base, delta);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      relative[3*row+col] = 0;
      for (int k = 0; k < 3; ++k) {
        relative[3*row+col] += data->xmat[9*base+3*k+row] * data->geom_xmat[9*g+3*k+col];
      }
    }
  }
  GeomBounds result;
  result.low.fill(std::numeric_limits<double>::infinity());
  result.high.fill(-std::numeric_limits<double>::infinity());
  if (model->geom_type[g] == mjGEOM_MESH) {
    const int mesh = model->geom_dataid[g];
    for (int v = model->mesh_vertadr[mesh]; v < model->mesh_vertadr[mesh]+model->mesh_vertnum[mesh]; ++v) {
      for (int row = 0; row < 3; ++row) {
        double value = center[row];
        for (int col = 0; col < 3; ++col) {
          value += relative[3*row+col] * model->mesh_vert[3*v+col];
        }
        result.low[row] = std::min(result.low[row], value);
        result.high[row] = std::max(result.high[row], value);
      }
    }
    return result;
  }
  const auto* size = model->geom_size + 3*g;
  for (int row = 0; row < 3; ++row) {
    const auto* r = relative + 3*row;
    double extent = 0, offset = 0;
    switch (model->geom_type[g]) {
      case mjGEOM_SPHERE: extent = size[0]; break;
      case mjGEOM_CAPSULE: extent = size[0] + std::abs(r[2])*size[1]; break;
      case mjGEOM_CYLINDER:
        extent = std::hypot(r[0], r[1])*size[0] + std::abs(r[2])*size[1]; break;
      case mjGEOM_ELLIPSOID:
        for (int i = 0; i < 3; ++i) {extent += r[i]*r[i]*size[i]*size[i];}
        extent = std::sqrt(extent); break;
      default:
        for (int i = 0; i < 3; ++i) {
          offset += r[i]*model->geom_aabb[6*g+i];
          extent += std::abs(r[i])*model->geom_aabb[6*g+3+i];
        }
    }
    result.low[row] = center[row]+offset-extent;
    result.high[row] = center[row]+offset+extent;
  }
  return result;
}
}  // namespace cleany_mujoco_observer
