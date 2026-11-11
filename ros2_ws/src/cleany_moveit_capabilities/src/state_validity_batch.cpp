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
  const auto& first = request.robot_states.front();
  const auto& variables = scene.getRobotModel()->getVariableNames();
  bool reusable_state = true;
  for (const auto& state : request.robot_states)
  {
    const auto& joints = state.joint_state;
    const bool same_names = joints.name == first.joint_state.name;
    if (joints.name.size() != joints.position.size() ||
        !std::all_of(joints.position.begin(), joints.position.end(), [](double q) { return std::isfinite(q); }))
    {
      response.message = "Invalid joint state";
      return;
    }
    if (&state == &first || !same_names)
    {
      std::set<std::string> names(joints.name.begin(), joints.name.end());
      if (names.size() != joints.name.size() ||
          !std::all_of(names.begin(), names.end(), [&](const auto& name) {
            return std::find(variables.begin(), variables.end(), name) != variables.end();
          }))
      {
        response.message = "Invalid joint state";
        return;
      }
    }
    reusable_state = reusable_state && same_names && state.is_diff &&
        state.attached_collision_objects.empty() &&
        state.multi_dof_joint_state.joint_names.empty() && state.multi_dof_joint_state.transforms.empty() &&
        joints.velocity == first.joint_state.velocity && joints.effort == first.joint_state.effort;
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
  std::vector<int> variable_indices;
  if (reusable_state)
    for (const auto& name : first.joint_state.name)
      variable_indices.push_back(scene.getRobotModel()->getVariableIndex(name));
  auto state = scene.getCurrentState();
  for (std::size_t index = 0; index < request.robot_states.size(); ++index)
  {
    if (reusable_state && index > 0)
    {
      // The first conversion retained scene attachments and shared metadata.
      // Indexed setters also update mimic joints and dirty transforms.
      const auto& positions = request.robot_states[index].joint_state.position;
      for (std::size_t joint = 0; joint < variable_indices.size(); ++joint)
        state.setVariablePosition(variable_indices[joint], positions[joint]);
      state.update();
    }
    else
    {
      // General diffs must start independently, including attachment changes.
      if (index > 0)
        state = scene.getCurrentState();
      if (!moveit::core::robotStateMsgToRobotState(request.robot_states[index], state))
      {
        response.message = "Robot state conversion failed";
        response.valid.clear();
        return;
      }
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
