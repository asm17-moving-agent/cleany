#include "cleany_scene_mapping/known_geometry.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace cleany_scene_mapping
{
bool placeKnownBodies(KnownBodies& bodies, const Eigen::Isometry3d& map_from_cloud,
                      const GetTransform& cloud_from_body)
{
  if (!map_from_cloud.matrix().allFinite())
    return false;
  std::vector<std::pair<bodies::Body*, Eigen::Isometry3d>> poses;
  poses.reserve(bodies.size());
  for (const auto& entry : bodies)
  {
    Eigen::Isometry3d pose;
    if (!entry.second || !cloud_from_body(entry.first, pose) || !pose.matrix().allFinite())
      return false;
    poses.emplace_back(entry.second.get(), map_from_cloud * pose);
  }
  for (const auto& entry : poses)
    entry.first->setPose(entry.second);
  return true;
}

ClearResult clearContainedOccupancy(octomap::OcTree& tree,
                                   const std::vector<const bodies::Body*>& bodies,
                                   std::size_t maximum_examined)
{
  struct BoundedBody
  {
    const bodies::Body* body;
    bodies::BoundingSphere sphere;
  };
  std::vector<BoundedBody> bounded;
  for (const auto* body : bodies)
  {
    if (!body)
      continue;
    BoundedBody item{body, {}};
    body->computeBoundingSphere(item.sphere);
    if (item.sphere.center.allFinite() && std::isfinite(item.sphere.radius) &&
        item.sphere.radius > 0.0)
      bounded.push_back(item);
  }
  ClearResult result;
  if (bounded.empty())
    return result;
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
  {
    if (result.examined >= maximum_examined)
    {
      result.budget_exhausted = true;
      break;
    }
    ++result.examined;
    if (!tree.isNodeOccupied(*it))
      continue;
    const Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
    const double half_size = it.getSize() * 0.5;
    std::array<Eigen::Vector3d, 8> corners;
    for (unsigned int corner = 0; corner < corners.size(); ++corner)
      corners[corner] = center + Eigen::Vector3d(
          (corner & 1U) ? half_size : -half_size,
          (corner & 2U) ? half_size : -half_size,
          (corner & 4U) ? half_size : -half_size);
    for (const auto& candidate : bounded)
    {
      if ((center - candidate.sphere.center).squaredNorm() >
          candidate.sphere.radius * candidate.sphere.radius)
        continue;
      bool contained = true;
      for (const auto& corner : corners)
      {
        if (!candidate.body->containsPoint(corner))
        {
          contained = false;
          break;
        }
      }
      if (contained)
      {
        it->setLogOdds(tree.getClampingThresMinLog());
        ++result.cleared;
        break;
      }
    }
  }
  if (result.cleared > 0)
    tree.updateInnerOccupancy();
  return result;
}
}  // namespace cleany_scene_mapping
