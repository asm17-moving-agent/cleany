#pragma once

#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/collision_detection/collision_detector_allocator.h>
#include <fcl/geometry/shape/convex.h>
#include <geometric_shapes/shapes.h>

namespace cleany
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
                             double padding = 0., double scale = 1.);
  SolidCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                    const collision_detection::WorldPtr& world, double padding = 0., double scale = 1.);
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
  void checkGroupedSelf(const collision_detection::CollisionRequest& req,
                        collision_detection::CollisionResult& res,
                        const moveit::core::RobotState& state,
                        const collision_detection::AllowedCollisionMatrix* acm) const;
  std::size_t solid_components_ = 0;
};

class SolidCollisionAllocator
  : public collision_detection::CollisionDetectorAllocatorTemplate<SolidCollisionEnv, SolidCollisionAllocator>
{
public:
  static const std::string NAME;
};
}  // namespace cleany
