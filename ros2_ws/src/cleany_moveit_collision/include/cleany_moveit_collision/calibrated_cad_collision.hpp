#pragma once
#include <cleany_moveit_collision/cad_linkage.hpp>
#include <cleany_moveit_collision/cached_cad_collision_env.hpp>

namespace cleany
{
using cleany_cached::CADInspectionCollisionEnv;
/** Apply saved nonlinear pin closure before the existing CAD collision queries. */
class CalibratedCADCollisionEnv : public CADInspectionCollisionEnv
{
public:
  explicit CalibratedCADCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                                   double padding=0.,double scale=1.);
  CalibratedCADCollisionEnv(const moveit::core::RobotModelConstPtr& model,
                           const collision_detection::WorldPtr& world,double padding=0.,double scale=1.);
  CalibratedCADCollisionEnv(const CalibratedCADCollisionEnv& other,const collision_detection::WorldPtr& world);
  using CADInspectionCollisionEnv::checkRobotCollision;
  void checkSelfCollision(const collision_detection::CollisionRequest&,collision_detection::CollisionResult&,
                          const moveit::core::RobotState&) const override;
  void checkSelfCollision(const collision_detection::CollisionRequest&,collision_detection::CollisionResult&,
                          const moveit::core::RobotState&,const collision_detection::AllowedCollisionMatrix&) const override;
  void checkRobotCollision(const collision_detection::CollisionRequest&,collision_detection::CollisionResult&,
                           const moveit::core::RobotState&) const override;
  void checkRobotCollision(const collision_detection::CollisionRequest&,collision_detection::CollisionResult&,
                           const moveit::core::RobotState&,const collision_detection::AllowedCollisionMatrix&) const override;
  void distanceSelf(const collision_detection::DistanceRequest&,collision_detection::DistanceResult&,
                    const moveit::core::RobotState&) const override;
  void distanceRobot(const collision_detection::DistanceRequest&,collision_detection::DistanceResult&,
                     const moveit::core::RobotState&) const override;

private:
  std::shared_ptr<const CadLinkage> closure_;
};

class CalibratedCADCollisionAllocator : public collision_detection::CollisionDetectorAllocatorTemplate<
    CalibratedCADCollisionEnv,CalibratedCADCollisionAllocator>
{
public:
  static const std::string NAME;
};
}  // namespace cleany
