#include "cleany_scene_mapping/raycast.hpp"

#include <algorithm>

namespace cleany_scene_mapping
{
void RaycastWorkspace::compute(const octomap::OcTree& geometry, const octomap::point3d& origin,
                              const std::array<const octomap::KeySet*, 3>& endpoints,
                              octomap::KeySet& free)
{
  free.clear();
  octomap::OcTreeKey low, high;
  bool cached = maximum_cache_bytes_ >= sizeof(std::uint64_t) && geometry.coordToKeyChecked(origin, low);
  std::array<std::size_t, 3> size{};
  if (cached)
  {
    high = low;
    for (const auto* keys : endpoints)
      for (const auto& key : *keys)
        for (unsigned int axis = 0; axis < 3; ++axis)
        {
          low[axis] = std::min(low[axis], key[axis]);
          high[axis] = std::max(high[axis], key[axis]);
        }
    for (unsigned int axis = 0; axis < 3; ++axis)
      size[axis] = std::size_t(high[axis]) - low[axis] + 1;
    const std::uint64_t cells = std::uint64_t(size[0]) * size[1] * size[2];
    const auto words = (cells + 63) / 64;
    cached = words <= maximum_cache_bytes_ / sizeof(std::uint64_t);
    if (cached)
      visited_.assign(static_cast<std::size_t>(words), 0);
  }
  for (const auto* keys : endpoints)
    for (const auto& endpoint : *keys)
      if (geometry.computeRayKeys(origin, geometry.keyToCoord(endpoint), ray_))
      {
        if (!cached)
        {
          free.insert(ray_.begin(), ray_.end());
          continue;
        }
        for (const auto& key : ray_)
        {
          // OctoMap DDA steps monotonically between origin and endpoint keys.
          const auto index = std::size_t(key[0] - low[0]) + size[0] *
              (std::size_t(key[1] - low[1]) + size[1] * std::size_t(key[2] - low[2]));
          auto& word = visited_[index / 64];
          const std::uint64_t bit = std::uint64_t{1} << (index % 64);
          if (word & bit) continue;
          word |= bit;
          free.insert(key);
        }
      }
}
}  // namespace cleany_scene_mapping
