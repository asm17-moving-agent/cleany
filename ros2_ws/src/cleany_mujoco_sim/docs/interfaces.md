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

## left_wrist_camera_contract_adapter

실행 파일: `camera_contract_adapter`. 원본: [camera_contract_adapter.py](../cleany_mujoco_sim/camera_contract_adapter.py), [camera_contract.py](../cleany_mujoco_sim/camera_contract.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/cleany/internal/mujoco/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | Sensor |
| 토픽 구독 | `/cleany/internal/mujoco/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor |
| 토픽 발행 | `/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | Sensor; 입력 동기화와 계약 검사 통과 시 |
| 토픽 발행 | `/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor |

입출력 이름은 `manifest_path`의 camera contract로 결정한다. 위 표는 기본 contract다. Vendor topic의 remap과 ros2_control backend는 [launch](../launch/study_cafe_backend.launch.py)를 따른다.

## simulation_placement_verifier

실행 파일: `placement_verifier`. 원본: [placement_verifier.py](../cleany_mujoco_sim/placement_verifier.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS |
|---|---|---|---|
| 토픽 구독 | `/simulation/sorting_ground_truth` | `visualization_msgs/msg/MarkerArray` | R10 |
| 서비스 제공 | `/sorting/verify_placement` | `cleany_interfaces/srv/VerifyPlacement` | Service 기본값 |

시뮬레이션 전용 결과 판정이다. 물체 정답 pose를 조작 코드에 반환하지 않는다. 실물에는 독립 센서 기반 제공자가 필요하다.

놓기 이후의 표본을 최소 `minimum_samples`개 사용하되, 높은 발행 주기에서도
`settled_duration_sec` 전체를 포함하도록 이전 표본까지 확장한다. 부족한 시간 폭을
허용하지 않으며 해당 구간 전체에서 수거함 내부 포함과 이동량 기준을 검사한다.
Marker의 중심과 크기는 결과 평가용 실제 형상의 base_link AABB다. Isaac 어댑터는
기존 tabletop collision vertex와 컵 바닥 원기둥의 극값에서 이를 계산하며,
회전한 전체 외접 직육면체로 대신하지 않는다.
