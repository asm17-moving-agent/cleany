#pragma once

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <memory>

namespace cleany_mujoco_observer
{
/// 내부 reader이며 동기화되지 않은 vendor plugin 루프에는 등록하지 않습니다.
/// physics 잠금 아래 얻은 독점 소유 snapshot으로만 호출합니다.
std::unique_ptr<mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase>
makeSnapshotObserver();
}
