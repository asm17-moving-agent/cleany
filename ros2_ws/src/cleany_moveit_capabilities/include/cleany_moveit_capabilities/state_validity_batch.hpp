#pragma once
#include <cleany_interfaces/srv/check_state_validity_batch.hpp>
#include <moveit/planning_scene/planning_scene.h>

namespace cleany
{
using BatchValidity = cleany_interfaces::srv::CheckStateValidityBatch;
void check_state_batch(const planning_scene::PlanningScene& scene,
                       const BatchValidity::Request& request,
                       BatchValidity::Response& response,
                       const builtin_interfaces::msg::Time& stamp);
}  // namespace cleany
