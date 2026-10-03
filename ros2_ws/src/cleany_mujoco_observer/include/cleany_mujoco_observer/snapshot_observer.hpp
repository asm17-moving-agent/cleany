#pragma once

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <memory>

namespace cleany_mujoco_observer
{
// Internal reader, NOT registered with the vendor's unsynchronized plugin loop.
// Call only with a privately owned snapshot obtained under the physics lock.
std::unique_ptr<mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase>
makeSnapshotObserver();
}
