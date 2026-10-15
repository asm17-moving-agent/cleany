# cleany_gazebo_sim 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## gazebo_command_guard

실행 파일: `gazebo_command_guard`. 원본: [command_guard.py](../cleany_gazebo_sim/command_guard.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `cmd_vel` | `geometry_msgs/msg/Twist` | R10 |
| 토픽 발행 | `gazebo_cmd_vel` | `geometry_msgs/msg/Twist` | R10; 명령 수신 또는 timeout 정지 시 |

Timeout 검사는 `timeout_check_rate_hz` 기본 20 Hz다.

## gazebo_feedback_sampler

실행 파일: `gazebo_feedback_sampler`. 원본: [feedback_sampler.py](../cleany_gazebo_sim/feedback_sampler.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/gazebo/raw_clock` | `rosgraph_msgs/msg/Clock` | Sensor |
| 토픽 구독 | `/gazebo/raw_joint_states` | `sensor_msgs/msg/JointState` | Sensor |
| 토픽 발행 | `/clock` | `rosgraph_msgs/msg/Clock` | R10 |
| 토픽 발행 | `/joint_states` | `sensor_msgs/msg/JointState` | Sensor |

Raw simulator feedback을 받아 샘플링한다. `publish_period_s` 기본 0.02초는 ROS stamp 간격 기준이며 wall-time 50 Hz를 보장하지 않는다.

## gazebo_odom_tf_publisher

실행 파일: `gazebo_odom_tf_publisher`. 원본: [odom_tf_publisher.py](../cleany_gazebo_sim/odom_tf_publisher.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `gazebo_odom` | `nav_msgs/msg/Odometry` | R10; `input_topic` |
| 토픽 발행 | `odom` | `nav_msgs/msg/Odometry` | R10; `publish_odometry=true` |
| 토픽 발행 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

입력 수신 시 `odom_frame_id → base_frame_id` TF를 발행한다. 기본 frame은 odom/base_link다.

## gazebo_pan_tf_publisher

실행 파일: `gazebo_pan_tf_publisher`. 원본: [pan_tf_publisher.py](../cleany_gazebo_sim/pan_tf_publisher.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | Sensor |
| 토픽 발행 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

Pan feedback에서 head camera TF를 발행한다. Frame 이름과 장착 geometry는 노드 parameter를 따른다.

## gazebo_sensor_tf_publisher

실행 파일: `gazebo_sensor_tf_publisher`. 원본: [sensor_tf_publisher.py](../cleany_gazebo_sim/sensor_tf_publisher.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 발행 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

시작 시 설정된 sensor의 static TF를 보낸다. `parent_frame_id`와 sensor별 frame/translation/rotation parameter가 필요하다.

## simulated_encoder

실행 파일: `simulated_encoder_node`. 원본: [simulated_encoder_node.py](../cleany_gazebo_sim/simulated_encoder_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | Sensor; `input_topic` |
| 토픽 발행 | `wheel_encoder/joint_states` | `sensor_msgs/msg/JointState` | Sensor; 입력 수신 시, `output_topic` |

## simulated_odometry_error

실행 파일: `simulated_odometry_error_node`. 원본: [simulated_odometry_error_node.py](../cleany_gazebo_sim/simulated_odometry_error_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `wheel/odom_raw` | `nav_msgs/msg/Odometry` | R10; `input_topic` |
| 토픽 발행 | `wheel/odom` | `nav_msgs/msg/Odometry` | R10; 입력 수신 시, `output_topic` |

## ground_truth_route_follower

실행 파일: `ground_truth_route_follower`. 원본: [ground_truth_route_follower.py](../cleany_gazebo_sim/ground_truth_route_follower.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `ground_truth/odom` | `nav_msgs/msg/Odometry` | R10 |
| 토픽 발행 | `cmd_vel` | `geometry_msgs/msg/Twist` | R10; `control_rate_hz` 기본 20 Hz |

별도 GT 주행 시험용이다. Nav2와 같은 cmd_vel 명령원을 동시에 실행하지 않는다.

## ros_gz_bridge

외부 실행 파일은 `ros_gz_bridge/parameter_bridge`다. [Sensor profile](../cleany_gazebo_sim/launch_helpers/sensor_profile.py)이 `gazebo_<group>_bridge`를 만들며 기본 `lidar_nav`는 core와 lidar 그룹을 사용한다. 아래는 [bridge 설정](../config/bridge)의 ROS 측 목록이다.

| 그룹 | 종류 / 방향 | ROS 기본 이름 | ROS 타입 | 조건 |
|---|---|---|---|---|
| `core` | 토픽 구독 | `/gazebo_cmd_vel` | `geometry_msgs/msg/Twist` | 선택된 그룹에서 활성화 |
| `core` | 토픽 발행 | `/gazebo_odom` | `nav_msgs/msg/Odometry` | 선택된 그룹에서 활성화 |
| `core` | 토픽 발행 | `/ground_truth/odom` | `nav_msgs/msg/Odometry` | 선택된 그룹에서 활성화 |
| `core` | 토픽 발행 | `/joint_states` | `sensor_msgs/msg/JointState` | Profile launch에서 `/gazebo/raw_joint_states`로 remap |
| `core` | 토픽 발행 | `/clock` | `rosgraph_msgs/msg/Clock` | Profile launch에서 `/gazebo/raw_clock`로 remap |
| `core` | 토픽 발행 | `/imu/data` | `sensor_msgs/msg/Imu` | 선택된 그룹에서 활성화 |
| `lidar` | 토픽 발행 | `/scan` | `sensor_msgs/msg/LaserScan` | 선택된 그룹에서 활성화 |
| `head_depth` | 토픽 발행 | `/camera/head/depth/image_raw` | `sensor_msgs/msg/Image` | `SENSOR_DATA` |
| `head_depth` | 토픽 발행 | `/camera/head/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `SENSOR_DATA` |
| `head_rgbd` | 토픽 발행 | `/camera/head/color/image_raw` | `sensor_msgs/msg/Image` | ROS 구독자가 있을 때 활성화 |
| `head_rgbd` | 토픽 발행 | `/camera/head/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | `SENSOR_DATA` |
| `head_rgbd` | 토픽 발행 | `/camera/head/depth/image_raw` | `sensor_msgs/msg/Image` | 선택된 그룹에서 활성화 |
| `left_wrist` | 토픽 발행 | `/camera/left_wrist/color/image_raw` | `sensor_msgs/msg/Image` | 선택된 그룹에서 활성화 |
| `right_wrist` | 토픽 발행 | `/camera/right_wrist/color/image_raw` | `sensor_msgs/msg/Image` | 선택된 그룹에서 활성화 |

QoS를 지정하지 않은 행은 설치된 bridge 기본값을 따른다. `/clock`과 `/joint_states`의 raw 입력은 `gazebo_feedback_sampler`가 정규 토픽으로 다시 발행한다. `bridge_config` override는 sampler와 이 remap을 사용하지 않고 지정 YAML로 단일 `gazebo_override_bridge`를 만든다.

`bridge.yaml`, `navigation_bridge.yaml`은 별도 통합 설정이며 위 profile 표와 동시에 활성화되는 것은 아니다. Gazebo wire type은 Fortress의 `ignition.msgs`를 사용한다. 서비스와 액션 bridge는 현재 설정에 없다.
