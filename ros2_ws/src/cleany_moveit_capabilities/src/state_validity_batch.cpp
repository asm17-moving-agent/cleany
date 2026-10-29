#include <cleany_moveit_capabilities/state_validity_batch.hpp>
#include <moveit/move_group/move_group_capability.h>
#include <moveit/move_group/move_group_context.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <set>

namespace cleany
{
void check_state_batch(const planning_scene::PlanningScene& scene,
                       const BatchValidity::Request& request,
                       BatchValidity::Response& response,
                       const builtin_interfaces::msg::Time& stamp)
{
  response.success = false;
  response.first_invalid_index = -1;
  response.valid.clear();
  response.contacts.clear();
  if (request.robot_states.empty() || request.robot_states.size() > 64)
  {
    response.message = "Expected 1..64 robot states";
    return;
  }
  if (!request.group_name.empty() && !scene.getRobotModel()->hasJointModelGroup(request.group_name))
  {
    response.message = "Unknown joint model group";
    return;
  }
  for (const auto& state : request.robot_states)
  {
    const auto& joints = state.joint_state;
    std::set<std::string> names(joints.name.begin(), joints.name.end());
    if (joints.name.size() != joints.position.size() || names.size() != joints.name.size() ||
        !std::all_of(joints.position.begin(), joints.position.end(), [](double q) { return std::isfinite(q); }) ||
        !std::all_of(names.begin(), names.end(), [&](const auto& name) {
          const auto& variables = scene.getRobotModel()->getVariableNames();
          return std::find(variables.begin(), variables.end(), name) != variables.end();
        }))
    {
      response.message = "Invalid joint state";
      return;
    }
  }
  kinematic_constraints::KinematicConstraintSet constraints(scene.getRobotModel());
  if (!constraints.add(request.constraints, scene.getTransforms()))
  {
    response.message = "Invalid constraints";
    return;
  }
  collision_detection::CollisionRequest collision_request;
  collision_request.group_name = request.group_name;
  collision_request.contacts = true;
  const auto geometry_count = scene.getWorld()->size() +
      scene.getRobotModel()->getLinkModelsWithCollisionGeometry().size();
  collision_request.max_contacts = geometry_count * geometry_count;
  for (std::size_t index = 0; index < request.robot_states.size(); ++index)
  {
    // Start every diff from the scene state: no state or attachment leaks from
    // a preceding sample. Live ACM, attached objects and OctoMap stay in scene.
    auto state = scene.getCurrentState();
    if (!moveit::core::robotStateMsgToRobotState(request.robot_states[index], state))
    {
      response.message = "Robot state conversion failed";
      response.valid.clear();
      return;
    }
    collision_detection::CollisionResult collision;
    scene.checkCollision(collision_request, collision, state);
    const bool valid = !collision.collision && constraints.decide(state).satisfied;
    response.valid.push_back(valid);
    if (!valid)
    {
      response.first_invalid_index = static_cast<int32_t>(index);
      for (const auto& entry : collision.contacts)
        for (const auto& contact : entry.second)
        {
          moveit_msgs::msg::ContactInformation message;
          collision_detection::contactToMsg(contact, message);
          message.header.frame_id = scene.getPlanningFrame();
          message.header.stamp = stamp;
          response.contacts.push_back(std::move(message));
        }
      break;
    }
  }
  response.success = true;
  response.message = "Checked against one locked planning scene";
}

class StateValidityBatchCapability : public move_group::MoveGroupCapability
{
public:
  StateValidityBatchCapability() : MoveGroupCapability("StateValidityBatch") {}
  void initialize() override
  {
    auto node = context_->moveit_cpp_->getNode();
    std::string requested;
    if (node->get_parameter("collision_detector", requested) && !requested.empty())
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(context_->planning_scene_monitor_);
      if (scene->getCollisionDetectorName() != requested)
      {
        RCLCPP_FATAL(node->get_logger(), "Requested collision detector %s did not load (active: %s)",
                     requested.c_str(), scene->getCollisionDetectorName().c_str());
        rclcpp::shutdown();
        throw std::runtime_error("Collision detector mismatch; refusing planning services");
      }
    }
    service_ = node->create_service<BatchValidity>("check_state_validity_batch",
        [this, node](const std::shared_ptr<BatchValidity::Request> request,
                     const std::shared_ptr<BatchValidity::Response> response) {
          try
          {
            planning_scene_monitor::LockedPlanningSceneRO scene(context_->planning_scene_monitor_);
            check_state_batch(*scene.operator->(), *request, *response, node->now());
          }
          catch (const std::exception& error)
          {
            response->success = false;
            response->valid.clear();
            response->first_invalid_index = -1;
            response->message = error.what();
          }
        });
  }
private:
  rclcpp::Service<BatchValidity>::SharedPtr service_;
};
}  // namespace cleany
PLUGINLIB_EXPORT_CLASS(cleany::StateValidityBatchCapability, move_group::MoveGroupCapability)
