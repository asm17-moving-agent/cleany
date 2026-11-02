#include <cleany_moveit_collision/solid_collision_env.hpp>
#include <moveit/collision_detection/collision_plugin.h>
#include <pluginlib/class_list_macros.hpp>

namespace cleany
{
class SolidCollisionPlugin : public collision_detection::CollisionPlugin
{
public:
  bool initialize(const planning_scene::PlanningScenePtr& scene) const override
  {
    scene->allocateCollisionDetector(SolidCollisionAllocator::create());
    return true;
  }
};
}  // namespace cleany
PLUGINLIB_EXPORT_CLASS(cleany::SolidCollisionPlugin, collision_detection::CollisionPlugin)
