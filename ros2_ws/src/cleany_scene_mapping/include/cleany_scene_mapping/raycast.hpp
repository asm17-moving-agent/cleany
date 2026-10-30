#pragma once

#include <octomap/OcTree.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cleany_scene_mapping
{
/** @brief Reuse bounded membership storage while retaining OctoMap's exact rays.
 *
 * Duplicate free keys are filtered before hash insertion. Ray voxels, endpoint
 * handling and out-of-range behavior match computeRayKeys; hash iteration order
 * is unspecified.
 * This workspace is owned by the serialized frame callback, not shared by workers.
 */
class RaycastWorkspace
{
public:
  /// The cache budget affects performance only; zero uses the reference path.
  explicit RaycastWorkspace(std::size_t maximum_cache_bytes = 8 * 1024 * 1024)
    : maximum_cache_bytes_(maximum_cache_bytes) {}
  void compute(const octomap::OcTree& geometry, const octomap::point3d& origin,
               const std::array<const octomap::KeySet*, 3>& endpoints,
               octomap::KeySet& free);

private:
  std::size_t maximum_cache_bytes_;
  octomap::KeyRay ray_;
  std::vector<std::uint64_t> visited_;
};
}  // namespace cleany_scene_mapping
