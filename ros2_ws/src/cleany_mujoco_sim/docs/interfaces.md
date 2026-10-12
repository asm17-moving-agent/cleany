# cleany_mujoco_sim 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## mujoco_sim

실행 파일: `mujoco_sim_node`. 원본: [sim_node.py](../cleany_mujoco_sim/sim_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `~/joint_cmd` | `sensor_msgs/msg/JointState` | R10 |
| 토픽 구독 | `cmd_vel` | `geometry_msgs/msg/Twist` | R10 |
| 토픽 발행 | `joint_states` | `sensor_msgs/msg/JointState` | R10; `publish_rate_hz` 기본 60 Hz |
| 토픽 발행 | `odom` | `nav_msgs/msg/Odometry` | R10; `publish_rate_hz` 기본 60 Hz |
| 토픽 발행 | `scan` | `sensor_msgs/msg/LaserScan` | R10; `scan_rate_hz` 기본 5.5 Hz, `scan_enabled` |
| 토픽 발행 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 발행 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

`~/joint_cmd`는 private 이름이며 root namespace에서는 `/mujoco_sim/joint_cmd`다. Odom TF는 `publish_odom_tf`로 제어한다. Custom backend에는 별도 `/clock` publisher가 없다.

## mujoco_rgbd_sim

실행 파일: `mujoco_rgbd_sim_node`. 원본: [rgbd_sim_node.py](../cleany_mujoco_sim/rgbd_sim_node.py), [sim_node.py](../cleany_mujoco_sim/sim_node.py), [rgbd.py](../cleany_mujoco_sim/rgbd.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `~/joint_cmd` | `sensor_msgs/msg/JointState` | R10 |
| 토픽 구독 | `cmd_vel` | `geometry_msgs/msg/Twist` | R10 |
| 토픽 발행 | `joint_states` | `sensor_msgs/msg/JointState` | R10; `publish_rate_hz` 기본 60 Hz |
| 토픽 발행 | `odom` | `nav_msgs/msg/Odometry` | R10; `publish_rate_hz` 기본 60 Hz |
| 토픽 발행 | `scan` | `sensor_msgs/msg/LaserScan` | R10; `scan_rate_hz` 기본 5.5 Hz, `scan_enabled` |
| 토픽 발행 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 발행 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 발행 | `camera/color/image_raw` | `sensor_msgs/msg/Image` | Sensor; `rgbd_color_image_topic` |
| 토픽 발행 | `camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor; `rgbd_color_info_topic` |
| 토픽 발행 | `camera/depth/image_raw` | `sensor_msgs/msg/Image` | Sensor; `rgbd_depth_image_topic` |
| 토픽 발행 | `camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor; `rgbd_depth_info_topic` |
| 토픽 발행 | `ground_truth/objects` | `cleany_interfaces/msg/DetectedObject3DArray` | Sensor; `ground_truth_topic` |

위 이름은 `rgbd_pick_demo.launch.py`의 node name이다. 직접 실행하면 부모의 `mujoco_sim` 이름을 사용한다. RGB-D 기본 capture 주기는 `rgbd_rate_hz=5`이며 설정 이름은 생성자 기본값이다. 부모의 endpoint도 포함한다.

## left_wrist_camera_contract_adapter

실행 파일: `camera_contract_adapter`. 원본: [camera_contract_adapter.py](../cleany_mujoco_sim/camera_contract_adapter.py), [camera_contract.py](../cleany_mujoco_sim/camera_contract.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/cleany/internal/mujoco/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | Sensor |
| 토픽 구독 | `/cleany/internal/mujoco/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor |
| 토픽 발행 | `/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | Sensor; 입력 동기화와 계약 검사 통과 시 |
| 토픽 발행 | `/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor |

입출력 이름은 `manifest_path`의 camera contract로 결정한다. 위 표는 기본 contract다. Vendor topic의 remap과 ros2_control backend는 [launch](../launch/handeye_backend.launch.py)를 따른다.
