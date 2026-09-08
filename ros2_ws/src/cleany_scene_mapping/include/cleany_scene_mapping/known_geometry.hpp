#pragma once

#include <geometric_shapes/bodies.h>
#include <octomap/OcTree.h>

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace cleany_scene_mapping
{
struct ClearResult
{
  std::size_t examined = 0;
  std::size_t cleared = 0;
  bool budget_exhausted = false;
};

using KnownBodies = std::map<unsigned int, std::unique_ptr<bodies::Body>>;
using GetTransform = std::function<bool(unsigned int, Eigen::Isometry3d&)>;

// All transforms must be available and finite. On failure no body is moved.
bool placeKnownBodies(KnownBodies& bodies, const Eigen::Isometry3d& map_from_cloud,
                      const GetTransform& cloud_from_body);

// Caller owns the tree write lock. Does not create cells or clear partial overlap.
// All eight corners must belong to ONE convex body, not a union across bodies.
ClearResult clearContainedOccupancy(octomap::OcTree& tree,
                                   const std::vector<const bodies::Body*>& bodies,
                                   std::size_t maximum_examined);
}  // namespace cleany_scene_mapping
