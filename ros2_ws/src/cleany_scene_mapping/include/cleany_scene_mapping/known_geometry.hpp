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

// Immutable, mesh-local face offsets. Build once per registered shape, then
// share between capture-time point masking and full-voxel containment.
struct ExclusionGeometry
{
  std::vector<Eigen::Vector4d> planes;
  Eigen::Vector3d low = Eigen::Vector3d::Zero(), high = Eigen::Vector3d::Zero();
};
using ExclusionGeometryPtr = std::shared_ptr<const ExclusionGeometry>;
ExclusionGeometryPtr prepareExclusionGeometry(const bodies::Body& body);

class ExclusionVolume
{
public:
  explicit ExclusionVolume(bodies::BodyConstPtr body, ExclusionGeometryPtr geometry = {});
  ExclusionVolume transformed(const Eigen::Isometry3d& transform) const;
  bool containsPoint(const Eigen::Vector3d& point) const;
  bool containsCell(const Eigen::Vector3d& center, double half_size) const;
  const bodies::BoundingSphere& sphere() const { return sphere_; }
  const Eigen::Vector3d& low() const { return low_; }
  const Eigen::Vector3d& high() const { return high_; }

private:
  struct Plane
  {
    Eigen::Vector4d equation;
    double cube_support;
  };
  bodies::BodyConstPtr body_;
  ExclusionGeometryPtr geometry_;
  std::vector<Plane> planes_;
  bodies::BoundingSphere sphere_;
  Eigen::Vector3d low_, high_;
};

using KnownBodies = std::map<unsigned int, std::unique_ptr<bodies::Body>>;
using GetTransform = std::function<bool(unsigned int, Eigen::Isometry3d&)>;

/// All transforms must be available and finite. On failure no body is moved.
bool placeKnownBodies(KnownBodies& bodies, const Eigen::Isometry3d& map_from_cloud,
                      const GetTransform& cloud_from_body);

/// Caller owns the tree write lock. Does not create cells or clear partial overlap.
/// All eight corners must belong to ONE convex body, not a union across bodies.
/// Mesh padding is applied along convex face normals, bounded by its padded box;
/// primitive bodies retain their native containment/padding semantics.
ClearResult clearContainedOccupancy(octomap::OcTree& tree,
                                   const std::vector<const bodies::Body*>& bodies,
                                   std::size_t maximum_examined);

// Runtime path: reuse capture-time geometry and transformed planes.
ClearResult clearContainedVolumes(octomap::OcTree& tree,
                                 const std::vector<ExclusionVolume>& volumes,
                                 std::size_t maximum_examined);
}  // namespace cleany_scene_mapping
