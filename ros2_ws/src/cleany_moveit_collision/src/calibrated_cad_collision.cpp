#include <cleany_moveit_collision/calibrated_cad_collision.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <moveit/collision_detection/collision_plugin.h>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace cleany
{
namespace
{
std::shared_ptr<const CadLinkage> load(const moveit::core::RobotModel& model)
{
  const char* config=std::getenv("CLEANY_CAD_LINKAGE_JSON");
  if (!config || !*config) throw std::invalid_argument("Calibrated CAD collision requires CLEANY_CAD_LINKAGE_JSON");
  return std::make_shared<CadLinkage>(config,model);
}
}  // namespace

const std::string CalibratedCADCollisionAllocator::NAME="cleany/CalibratedCADFCL";

CalibratedCADCollisionEnv::CalibratedCADCollisionEnv(const moveit::core::RobotModelConstPtr& model,
    double padding,double scale)
  : CADInspectionCollisionEnv(model,padding,scale),closure_(load(*model)) {}

CalibratedCADCollisionEnv::CalibratedCADCollisionEnv(const moveit::core::RobotModelConstPtr& model,
    const collision_detection::WorldPtr& world,double padding,double scale)
  : CADInspectionCollisionEnv(model,world,padding,scale),closure_(load(*model)) {}

CalibratedCADCollisionEnv::CalibratedCADCollisionEnv(const CalibratedCADCollisionEnv& other,
    const collision_detection::WorldPtr& world)
  : CADInspectionCollisionEnv(other,world),closure_(other.closure_) {}

void CalibratedCADCollisionEnv::checkSelfCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res,const moveit::core::RobotState& state) const
{
  moveit::core::RobotState resolved(state);
  if (!closure_->resolve(resolved)) { res.collision=true; return; }
  CADInspectionCollisionEnv::checkSelfCollision(req,res,resolved);
}

void CalibratedCADCollisionEnv::checkSelfCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res,const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix& acm) const
{
  moveit::core::RobotState resolved(state);
  if (!closure_->resolve(resolved)) { res.collision=true; return; }
  CADInspectionCollisionEnv::checkSelfCollision(req,res,resolved,acm);
}

void CalibratedCADCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res,const moveit::core::RobotState& state) const
{
  moveit::core::RobotState resolved(state);
  if (!closure_->resolve(resolved)) { res.collision=true; return; }
  CADInspectionCollisionEnv::checkRobotCollision(req,res,resolved);
}

void CalibratedCADCollisionEnv::checkRobotCollision(const collision_detection::CollisionRequest& req,
    collision_detection::CollisionResult& res,const moveit::core::RobotState& state,
    const collision_detection::AllowedCollisionMatrix& acm) const
{
  moveit::core::RobotState resolved(state);
  if (!closure_->resolve(resolved)) { res.collision=true; return; }
  CADInspectionCollisionEnv::checkRobotCollision(req,res,resolved,acm);
}

void CalibratedCADCollisionEnv::distanceSelf(const collision_detection::DistanceRequest& req,
    collision_detection::DistanceResult& res,const moveit::core::RobotState& state) const
{
  moveit::core::RobotState resolved(state);
  if (!closure_->resolve(resolved))
  { res.minimum_distance.distance=-std::numeric_limits<double>::infinity(); return; }
  CADInspectionCollisionEnv::distanceSelf(req,res,resolved);
}

void CalibratedCADCollisionEnv::distanceRobot(const collision_detection::DistanceRequest& req,
    collision_detection::DistanceResult& res,const moveit::core::RobotState& state) const
{
  moveit::core::RobotState resolved(state);
  if (!closure_->resolve(resolved))
  { res.minimum_distance.distance=-std::numeric_limits<double>::infinity(); return; }
  CADInspectionCollisionEnv::distanceRobot(req,res,resolved);
}

class CalibratedCADCollisionPlugin : public collision_detection::CollisionPlugin
{
public:
  bool initialize(const planning_scene::PlanningScenePtr& scene) const override
  {
    scene->allocateCollisionDetector(CalibratedCADCollisionAllocator::create());
    return true;
  }
};
}  // namespace cleany

PLUGINLIB_EXPORT_CLASS(cleany::CalibratedCADCollisionPlugin,collision_detection::CollisionPlugin)
