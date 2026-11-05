#include "cleany_scene_mapping/known_geometry.hpp"

#include <cmath>
#include <utility>

namespace cleany_scene_mapping
{
ExclusionGeometryPtr prepareExclusionGeometry(const bodies::Body& body)
{
  auto result = std::make_shared<ExclusionGeometry>();
  const auto* mesh = dynamic_cast<const bodies::ConvexMesh*>(&body);
  if (!mesh || mesh->getVertices().empty() || mesh->getPlanes().empty())
    return result;

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  auto low = mesh->getVertices().front(), high = low;
  for (const auto& vertex : mesh->getVertices())
  {
    center += vertex;
    low = low.cwiseMin(vertex);
    high = high.cwiseMax(vertex);
  }
  center /= mesh->getVertices().size();
  const double scale = body.getScale(), padding = body.getPadding();
  result->planes.reserve(mesh->getPlanes().size() + 6);
  for (const auto& plane : mesh->getPlanes())
    result->planes.emplace_back(plane.x(), plane.y(), plane.z(), scale * plane.w() -
        (1.0 - scale) * plane.head<3>().dot(center) - padding * plane.head<3>().norm());
  result->low = center + scale * (low - center) - Eigen::Vector3d::Constant(padding);
  result->high = center + scale * (high - center) + Eigen::Vector3d::Constant(padding);
  for (unsigned int axis = 0; axis < 3; ++axis)
  {
    const Eigen::Vector3d normal = Eigen::Vector3d::Unit(axis);
    result->planes.emplace_back(normal.x(), normal.y(), normal.z(), -result->high[axis]);
    result->planes.emplace_back(-normal.x(), -normal.y(), -normal.z(), result->low[axis]);
  }
  return result;
}

ExclusionVolume::ExclusionVolume(bodies::BodyConstPtr body, ExclusionGeometryPtr geometry)
  : body_(std::move(body)), geometry_(geometry ? std::move(geometry) : prepareExclusionGeometry(*body_))
{
  if (geometry_->planes.empty())
  {
    body_->computeBoundingSphere(sphere_);
    bodies::AABB box;
    body_->computeBoundingBox(box);
    low_ = box.min(); high_ = box.max();
    return;
  }
  const auto& pose = body_->getPose();
  const Eigen::Vector3d half = (geometry_->high - geometry_->low) * 0.5;
  sphere_.center = pose * ((geometry_->low + geometry_->high) * 0.5);
  sphere_.radius = half.norm();
  // The broad phase must include padded corners and the precise predicate's
  // roundoff tolerance, even with a single mesh and no overlapping spheres.
  sphere_.radius += 1e-8 * (1.0 + sphere_.center.cwiseAbs().maxCoeff() + sphere_.radius);
  const Eigen::Vector3d world_half = pose.linear().cwiseAbs() * half;
  low_ = sphere_.center - world_half; high_ = sphere_.center + world_half;
  planes_.reserve(geometry_->planes.size());
  for (const auto& plane : geometry_->planes)
  {
    const Eigen::Vector3d normal = pose.linear() * plane.head<3>();
    planes_.push_back({Eigen::Vector4d(normal.x(), normal.y(), normal.z(),
        plane.w() - normal.dot(pose.translation())), normal.cwiseAbs().sum()});
  }
}

ExclusionVolume ExclusionVolume::transformed(const Eigen::Isometry3d& transform) const
{
  return ExclusionVolume(body_->cloneAt(transform * body_->getPose()), geometry_);
}

bool ExclusionVolume::containsPoint(const Eigen::Vector3d& point) const
{
  if (planes_.empty()) return body_->containsPoint(point);
  if (!point.allFinite()) return false;
  for (const auto& plane : planes_)
    if (plane.equation.head<3>().dot(point) + plane.equation.w() > 1e-9)
      return false;
  return true;
}

bool ExclusionVolume::containsCell(const Eigen::Vector3d& center, double half_size) const
{
  if (!planes_.empty())
  {
    // Maximum of n.dot(corner) over an axis-aligned cube. This is exactly the
    // eight-corner test for a convex half-space, with one evaluation per face.
    for (const auto& plane : planes_)
      if (plane.equation.head<3>().dot(center) + plane.equation.w() +
          half_size * plane.cube_support > 1e-9)
        return false;
    return true;
  }
  for (unsigned int corner = 0; corner < 8; ++corner)
    if (!body_->containsPoint(center + Eigen::Vector3d(
        (corner & 1U) ? half_size : -half_size,
        (corner & 2U) ? half_size : -half_size,
        (corner & 4U) ? half_size : -half_size)))
      return false;
  return true;
}

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
  std::vector<ExclusionVolume> volumes;
  volumes.reserve(bodies.size());
  for (const auto* body : bodies)
    if (body)
      volumes.emplace_back(bodies::BodyConstPtr(body, [](const bodies::Body*) {}));
  return clearContainedVolumes(tree, volumes, maximum_examined);
}

ClearResult clearContainedVolumes(octomap::OcTree& tree,
                                 const std::vector<ExclusionVolume>& volumes,
                                 std::size_t maximum_examined)
{
  ClearResult result;
  std::vector<const ExclusionVolume*> bounded;
  bounded.reserve(volumes.size());
  for (const auto& volume : volumes)
  {
    const auto& sphere = volume.sphere();
    if (sphere.center.allFinite() && std::isfinite(sphere.radius) && sphere.radius > 0.0)
      bounded.push_back(&volume);
  }
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
    for (const auto* candidate : bounded)
    {
      const auto& sphere = candidate->sphere();
      if ((center - sphere.center).squaredNorm() > sphere.radius * sphere.radius)
        continue;
      if (candidate->containsCell(center, half_size))
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
