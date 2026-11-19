#pragma once
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace cleany_mujoco_observer {
/**
 * @brief Render scheduled sensor frames from private simulator snapshots.
 *
 * @par Publishers
 * /camera/color/image_raw and /{left,right}_wrist_camera/image_raw
 * (sensor_msgs/msg/Image): RGB images in the respective optical frames.
 * /camera/color/camera_info and /{left,right}_wrist_camera/camera_info
 * (sensor_msgs/msg/CameraInfo): Matching calibration and capture timestamps.
 * /camera/aligned_depth_to_color/image_raw (sensor_msgs/msg/Image): Metric
 * 32FC1 depth in head_camera_rgb_optical_frame, aligned to the RGB image.
 * All publishers use SensorDataQoS. No TF is broadcast.
 *
 * @par Lifetime and scheduling
 * The model and snapshot callbacks must remain valid until destruction joins
 * the renderer thread. Each due frame uses snapshot simulation time; unchanged
 * timestamps do not produce another capture. Rendering never writes controls.
 */
class ScheduledCameras {
public:
  using Snapshot = std::function<void(mjData*&)>;
  using SimulationTime = std::function<double()>;
  ScheduledCameras(mjModel* model, Snapshot snapshot, SimulationTime simulation_time,
                   bool show_viewer = false);
  ~ScheduledCameras();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
