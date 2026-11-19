#pragma once
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace cleany_mujoco_observer {
/**
 * @brief 시뮬레이터의 전용 snapshot으로 예약된 센서 프레임을 렌더링합니다.
 *
 * @par Publishers
 * /camera/color/image_raw, /{left,right}_wrist_camera/image_raw
 * (sensor_msgs/msg/Image): 각 광학 좌표계의 RGB 영상.
 * /camera/color/camera_info, /{left,right}_wrist_camera/camera_info
 * (sensor_msgs/msg/CameraInfo): 대응하는 보정값과 촬영 시각.
 * /camera/aligned_depth_to_color/image_raw (sensor_msgs/msg/Image):
 * head_camera_rgb_optical_frame의 미터 단위 32FC1 깊이이며 RGB에 정렬합니다.
 * 모든 publisher는 SensorDataQoS를 사용하며 TF를 발행하지 않습니다.
 *
 * @par Lifetime and scheduling
 * 소멸자가 렌더러 스레드를 join할 때까지 모델과 snapshot 콜백이 유효해야 합니다.
 * 각 예정 프레임은 snapshot의 시뮬레이션 시각을 사용하며 시각이 같으면 다시
 * 취득하지 않습니다. 렌더링은 제어값을 쓰지 않습니다.
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
