#include "cleany_scene_mapping/known_geometry.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace cleany_scene_mapping
{
namespace
{
struct ExclusionVolume
{
  const bodies::Body* body;
  bodies::BoundingSphere sphere;
  std::vector<Eigen::Vector4d> planes;

  explicit ExclusionVolume(const bodies::Body* source) : body(source)
  {
    body->computeBoundingSphere(sphere);
    const auto* mesh = dynamic_cast<const bodies::ConvexMesh*>(body);
    if (!mesh || mesh->getVertices().empty() || mesh->getPlanes().empty())
      return;

    // geometric_shapes pads mesh vertices radially from their mean. On a long
    // mesh that gives side faces less than padding_offset of clearance. Use
    // the configured distance along each convex face normal for voxel clearing.
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d low = mesh->getVertices().front();
    Eigen::Vector3d high = low;
    for (const auto& vertex : mesh->getVertices())
    {
      center += vertex;
      low = low.cwiseMin(vertex);
      high = high.cwiseMax(vertex);
    }
    center /= mesh->getVertices().size();
    const double scale = body->getScale();
    const double padding = body->getPadding();
    const auto& pose = body->getPose();
    const auto add_plane = [&](const Eigen::Vector3d& normal, double offset) {
      const Eigen::Vector3d world_normal = pose.linear() * normal;
      planes.emplace_back(world_normal.x(), world_normal.y(), world_normal.z(),
                          offset - world_normal.dot(pose.translation()));
    };
    planes.reserve(mesh->getPlanes().size() + 6);
    for (const auto& plane : mesh->getPlanes())
      add_plane(plane.head<3>(), scale * plane.w() -
          (1.0 - scale) * plane.head<3>().dot(center) - padding * plane.head<3>().norm());

    // Also bound the offset hull by its padded local box. This keeps the broad
    // phase conservative without using a radial mesh sphere that can be too small.
    low = center + scale * (low - center) - Eigen::Vector3d::Constant(padding);
    high = center + scale * (high - center) + Eigen::Vector3d::Constant(padding);
    for (unsigned int axis = 0; axis < 3; ++axis)
    {
      add_plane(Eigen::Vector3d::Unit(axis), -high[axis]);
      add_plane(-Eigen::Vector3d::Unit(axis), low[axis]);
    }
    sphere.center = pose * ((low + high) * 0.5);
    sphere.radius = (high - low).norm() * 0.5;
  }

  bool contains(const Eigen::Vector3d& point) const
  {
    if (planes.empty())
      return body->containsPoint(point);
    for (const auto& plane : planes)
      if (plane.head<3>().dot(point) + plane.w() > 1e-9)
        return false;
    return true;
  }
};
}  // namespace

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
  std::vector<ExclusionVolume> bounded;
  for (const auto* body : bodies)
  {
    if (!body)
      continue;
    ExclusionVolume item(body);
    if (item.sphere.center.allFinite() && std::isfinite(item.sphere.radius) &&
        item.sphere.radius > 0.0)
      bounded.push_back(std::move(item));
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
        if (!candidate.contains(corner))
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
