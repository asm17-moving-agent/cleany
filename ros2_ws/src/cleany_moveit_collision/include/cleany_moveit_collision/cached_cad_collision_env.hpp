#pragma once

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <fcl/geometry/shape/convex.h>
#include <geometric_shapes/shapes.h>
#include <map>
#include <mutex>

namespace cleany_cached
{
/** Split a closed convex-component mesh into solids without filling component gaps.
 * Coordinates are metres. Invalid, open or concave components throw; no hull is
 * inferred from an arbitrary CAD surface. Winding is normalized outwards.
 */
std::vector<std::unique_ptr<fcl::Convexd>> solid_components(const shapes::Mesh& mesh);

/** FCL queries with solid robot meshes and unchanged world/ACM/attachment handling.
 * Every component retains its original link and collision-shape transform index.
 * Self queries reject disjoint or allowed link pairs before visiting components.
 * FCL callbacks, distance queries and scene copies preserve their normal semantics.
 */
class SolidCollisionEnv : public collision_detection::CollisionEnvFCL
{
public:
  explicit SolidCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                             double padding = 0., double scale = 1., bool declared_only = false);
  SolidCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                    const collision_detection::WorldPtr& world, double padding = 0., double scale = 1.,
                    bool declared_only = false);
  SolidCollisionEnv(const SolidCollisionEnv& other, const collision_detection::WorldPtr& world);
  std::size_t solidComponentCount() const { return solid_components_; }
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state) const override;
  void checkSelfCollision(const collision_detection::CollisionRequest& req,
                          collision_detection::CollisionResult& res,
                          const moveit::core::RobotState& state,
                          const collision_detection::AllowedCollisionMatrix& acm) const override;
  using CollisionEnvFCL::checkRobotCollision;
  void distanceSelf(const collision_detection::DistanceRequest& req,
                    collision_detection::DistanceResult& res,
                    const moveit::core::RobotState& state) const override;
  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res,
                           const moveit::core::RobotState& state) const override;
  void checkRobotCollision(const collision_detection::CollisionRequest& req,
                           collision_detection::CollisionResult& res,
                           const moveit::core::RobotState& state,
                           const collision_detection::AllowedCollisionMatrix& acm) const override;

protected:
  void updatedPaddingOrScaling(const std::vector<std::string>& links) override;

private:
  void rebuild();
  void constructSelfObjects(const moveit::core::RobotState& state,
                            collision_detection::FCLObject& objects) const;
  void checkGroupedSelf(const collision_detection::CollisionRequest& req,
                        collision_detection::CollisionResult& res,
                        const moveit::core::RobotState& state,
                        const collision_detection::AllowedCollisionMatrix* acm) const;
  std::size_t solid_components_ = 0;
  bool declared_only_ = false;
  std::vector<collision_detection::FCLGeometryConstPtr> self_geoms_;
  std::vector<collision_detection::FCLCollisionObjectConstPtr> self_objects_;
  bool separate_self_geometry_ = false;
  // An adjacent pair's relative transform depends only on its joint positions.
  mutable std::mutex clear_pair_mutex_;
  mutable std::map<const moveit::core::JointModel*, std::vector<double>> clear_pairs_;
};

class SolidCollisionAllocator
  : public collision_detection::CollisionDetectorAllocatorTemplate<SolidCollisionEnv, SolidCollisionAllocator>
{
public:
  static const std::string NAME;
};

/** Inspect mixed CAD surfaces and explicitly named cad_enclosure__ solids.
 * Only named enclosures are converted to solids; malformed enclosures still
 * throw. Other CAD meshes retain standard FCL surface collision semantics.
 * cad_self__ and cad_world__ prefixes select separate representations without
 * allowing link pairs. Fixed-joint descendants are one rigid body for self
 * queries. World queries retain all fixed attachments and their geometry.
 */
class CADInspectionCollisionEnv : public SolidCollisionEnv
{
public:
  explicit CADInspectionCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                                    double padding = 0., double scale = 1.)
    : SolidCollisionEnv(model, padding, scale, true) {}
  CADInspectionCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                            const collision_detection::WorldPtr& world, double padding = 0., double scale = 1.)
    : SolidCollisionEnv(model, world, padding, scale, true) {}
  CADInspectionCollisionEnv(const CADInspectionCollisionEnv& other, const collision_detection::WorldPtr& world)
    : SolidCollisionEnv(other, world) {}
};

class CADInspectionCollisionAllocator
  : public collision_detection::CollisionDetectorAllocatorTemplate<CADInspectionCollisionEnv, CADInspectionCollisionAllocator>
{
public:
  static const std::string NAME;
};
}  // namespace cleany_cached
