#include "cleany_scene_mapping/raycast.hpp"

#include <algorithm>
#include <unordered_map>

namespace cleany_scene_mapping
{
namespace
{
constexpr unsigned int block_width = 4;

octomap::OcTreeKey blockOrigin(const octomap::OcTreeKey& key)
{
  return {static_cast<octomap::key_type>(key[0] & ~3u), static_cast<octomap::key_type>(key[1] & ~3u),
          static_cast<octomap::key_type>(key[2] & ~3u)};
}

std::uint64_t voxelBit(const octomap::OcTreeKey& key)
{
  // Group the eight siblings of each 2 x 2 x 2 child into one byte.
  const unsigned int bit = 8 * octomap::computeChildIdx(key, 1) + octomap::computeChildIdx(key, 0);
  return std::uint64_t{1} << bit;
}

octomap::OcTreeKey voxelKey(const octomap::OcTreeKey& base, unsigned int bit)
{
  auto key = base;
  for (unsigned int axis = 0; axis < 3; ++axis)
    key[axis] += 2 * ((bit >> (3 + axis)) & 1u) + ((bit >> axis) & 1u);
  return key;
}

// Mirrors OctoMap's eager update semantics but unwinds a shared ancestor once
// for the whole block. An existing childless node represents pruned evidence;
// a newly allocated one represents unknown space.
bool applyMisses(octomap::OcTree& tree, octomap::OcTreeNode* node, unsigned int remaining,
                 const octomap::OcTreeKey& base, std::uint64_t mask, bool created)
{
  if (!created && !tree.nodeHasChildren(node))
  {
    const float odds = node->getLogOdds();
    if ((tree.getProbMissLog() <= 0 && odds <= tree.getClampingThresMinLog()) ||
        (tree.getProbMissLog() >= 0 && odds >= tree.getClampingThresMaxLog()))
      return false;
    if (remaining > 0)
      tree.expandNode(node);
  }
  if (remaining == 0)
  {
    tree.updateNodeLogOdds(node, tree.getProbMissLog());
    return true;
  }

  const auto update_child = [&](unsigned int index, std::uint64_t child_mask) {
    const bool missing = !tree.nodeChildExists(node, index);
    auto* child = missing ? tree.createNodeChild(node, index) : tree.getNodeChild(node, index);
    return applyMisses(tree, child, remaining - 1, base, child_mask, missing);
  };
  bool changed = false;
  if (remaining > 2)
    changed = update_child(octomap::computeChildIdx(base, remaining - 1), mask);
  else
  {
    const unsigned int bits_per_child = remaining == 2 ? 8 : 1;
    const unsigned int child_bits = (1u << bits_per_child) - 1;
    for (unsigned int index = 0; mask; ++index, mask >>= bits_per_child)
      if (const auto child_mask = mask & child_bits)
        changed |= update_child(index, child_mask);
  }
  if (changed && !tree.pruneNode(node))
    node->updateOccupancyChildren();
  return changed;
}
}  // namespace

std::vector<FreeVoxelBlock> RaycastWorkspace::trace(const octomap::OcTree& geometry,
    const octomap::point3d& origin, const std::array<const octomap::KeySet*, 3>& endpoints,
    bool apply_endpoints)
{
  octomap::OcTreeKey origin_key;
  const bool valid_origin = geometry.coordToKeyChecked(origin, origin_key);
  octomap::OcTreeKey low(65535, 65535, 65535), high(0, 0, 0);
  const auto include = [&](const octomap::OcTreeKey& key) {
    const auto base = blockOrigin(key);
    for (unsigned int axis = 0; axis < 3; ++axis)
    {
      low[axis] = std::min(low[axis], base[axis]);
      high[axis] = std::max(high[axis], base[axis]);
    }
  };
  if (valid_origin)
    include(origin_key);
  for (const auto* keys : endpoints)
    for (const auto& key : *keys)
      include(key);
  if (low[0] > high[0])
    return {};

  const auto populate = [&](auto word_for) {
    if (valid_origin)
      for (const auto* keys : endpoints)
        for (const auto& endpoint : *keys)
          if (geometry.computeRayKeys(origin, geometry.keyToCoord(endpoint), ray_))
            for (const auto& key : ray_)
              word_for(key) |= voxelBit(key);
    if (apply_endpoints)
    {
      for (const auto& key : *endpoints[2])
        word_for(key) |= voxelBit(key);
      for (const auto& key : *endpoints[0])
        word_for(key) &= ~voxelBit(key);
    }
  };

  const std::size_t nx = (high[0] - low[0]) / block_width + 1;
  const std::size_t ny = (high[1] - low[1]) / block_width + 1;
  const std::size_t nz = (high[2] - low[2]) / block_width + 1;
  const std::uint64_t count = std::uint64_t{nx} * ny * nz;
  std::vector<FreeVoxelBlock> blocks;
  if (count > maximum_cache_bytes_ / sizeof(std::uint64_t))
  {
    std::unordered_map<octomap::OcTreeKey, std::uint64_t, octomap::OcTreeKey::KeyHash> sparse;
    populate([&](const octomap::OcTreeKey& key) -> std::uint64_t& { return sparse[blockOrigin(key)]; });
    blocks.reserve(sparse.size());
    for (const auto& entry : sparse)
      if (entry.second)
        blocks.push_back({entry.first, entry.second});
    return blocks;
  }

  visited_.assign(count, 0);
  populate([&](const octomap::OcTreeKey& key) -> std::uint64_t& {
    const auto x = (key[0] - low[0]) / block_width;
    const auto y = (key[1] - low[1]) / block_width;
    const auto z = (key[2] - low[2]) / block_width;
    return visited_[x + nx * (y + ny * z)];
  });
  std::size_t index = 0;
  for (unsigned int z = low[2]; z <= high[2]; z += block_width)
    for (unsigned int y = low[1]; y <= high[1]; y += block_width)
      for (unsigned int x = low[0]; x <= high[0]; x += block_width, ++index)
        if (visited_[index])
          blocks.push_back({octomap::OcTreeKey(x, y, z), visited_[index]});
  return blocks;
}

void RaycastWorkspace::compute(const octomap::OcTree& geometry, const octomap::point3d& origin,
    const std::array<const octomap::KeySet*, 3>& endpoints, octomap::KeySet& free)
{
  free.clear();
  for (const auto& block : trace(geometry, origin, endpoints, false))
    for (auto mask = block.mask; mask; mask &= mask - 1)
      free.insert(voxelKey(block.origin, __builtin_ctzll(mask)));
}

std::vector<FreeVoxelBlock> RaycastWorkspace::traceFrame(const octomap::OcTree& geometry,
    const octomap::point3d& origin, const octomap::KeySet& occupied,
    const octomap::KeySet& model, const octomap::KeySet& clipped)
{
  return trace(geometry, origin, {&occupied, &model, &clipped}, true);
}

void integrateFreeBlocks(octomap::OcTree& tree, const std::vector<FreeVoxelBlock>& blocks)
{
  for (const auto& block : blocks)
  {
    auto mask = block.mask;
    if (tree.isChangeDetectionEnabled())
    {
      // The native writer also records new keys and occupancy transitions.
      for (; mask; mask &= mask - 1)
        tree.updateNode(voxelKey(block.origin, __builtin_ctzll(mask)), false);
      continue;
    }
    if (mask && !tree.getRoot())
    {
      tree.updateNode(voxelKey(block.origin, __builtin_ctzll(mask)), false);
      mask &= mask - 1;
    }
    if (mask)
      applyMisses(tree, tree.getRoot(), tree.getTreeDepth(), block.origin, mask, false);
  }
}
}  // namespace cleany_scene_mapping
