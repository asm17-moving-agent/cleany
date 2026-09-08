#pragma once
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace cleany_mujoco_observer {
// A sensor renderer only. Private snapshots never become control/GT messages.
class ScheduledCameras {
public:
  using Snapshot = std::function<void(mjData*&)>;
  ScheduledCameras(mjModel* model, Snapshot snapshot);
  ~ScheduledCameras();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
