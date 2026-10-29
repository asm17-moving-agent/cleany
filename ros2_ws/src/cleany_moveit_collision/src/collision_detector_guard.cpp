#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>

namespace cleany
{
/** Refuse a silently substituted detector before accepting planning requests. */
class CollisionDetectorGuard : public move_group::MoveGroupCapability
{
public:
  CollisionDetectorGuard() : MoveGroupCapability("RequireCollisionDetector") {}

  void initialize() override
  {
    auto node = context_->moveit_cpp_->getNode();
    std::string requested;
    node->get_parameter("collision_detector", requested);
    planning_scene_monitor::LockedPlanningSceneRO scene(context_->planning_scene_monitor_);
    if (requested.empty() || scene->getCollisionDetectorName() != requested)
    {
      RCLCPP_FATAL(node->get_logger(), "Requested collision detector %s did not load (active: %s)",
                   requested.c_str(), scene->getCollisionDetectorName().c_str());
      rclcpp::shutdown(node->get_node_base_interface()->get_context());
      throw std::runtime_error("Collision detector mismatch; refusing planning services");
    }
    RCLCPP_INFO(node->get_logger(), "Verified collision detector: %s", requested.c_str());
  }
};
}  // namespace cleany

PLUGINLIB_EXPORT_CLASS(cleany::CollisionDetectorGuard, move_group::MoveGroupCapability)
