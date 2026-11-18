#pragma once

#include <octomap/OcTree.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cleany_scene_mapping
{
/// One bit per free voxel in an OctoMap-aligned 4 x 4 x 4 block.
struct FreeVoxelBlock
{
  octomap::OcTreeKey origin;
  std::uint64_t mask;
};

class RaycastWorkspace
{
public:
  /// The dense storage budget affects performance only.
  explicit RaycastWorkspace(std::size_t maximum_cache_bytes = 8 * 1024 * 1024)
    : maximum_cache_bytes_(maximum_cache_bytes) {}

  /// Return the exact union of computeRayKeys, excluding each ray's endpoint.
  void compute(const octomap::OcTree& geometry, const octomap::point3d& origin,
               const std::array<const octomap::KeySet*, 3>& endpoints, octomap::KeySet& free);

  /// Include clipped endpoints and remove occupied endpoints from the free updates.
  /// The caller resolves model/occupied conflicts before tracing the frame.
  std::vector<FreeVoxelBlock> traceFrame(const octomap::OcTree& geometry,
      const octomap::point3d& origin, const octomap::KeySet& occupied,
      const octomap::KeySet& model, const octomap::KeySet& clipped);

private:
  std::vector<FreeVoxelBlock> trace(const octomap::OcTree& geometry, const octomap::point3d& origin,
      const std::array<const octomap::KeySet*, 3>& endpoints, bool apply_endpoints);
  std::size_t maximum_cache_bytes_;
  octomap::KeyRay ray_;
  std::vector<std::uint64_t> visited_;
};

/// Apply one native miss per marked voxel. The caller owns the tree write lock.
void integrateFreeBlocks(octomap::OcTree& tree, const std::vector<FreeVoxelBlock>& blocks);
}  // namespace cleany_scene_mapping
