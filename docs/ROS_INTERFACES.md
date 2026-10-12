# ROS 인터페이스 색인

각 노드의 발행, 구독, 서비스 제공/호출, 액션 제공/호출을 찾는 색인이다. 상세 표는 코드 옆 `docs/interfaces.md`가 관리하고, 동작과 안전 규칙은 [INTEGRATION](INTEGRATION.md)의 계약 reference를 따른다.

표는 현재 소스와 launch의 기본 이름을 기록한다. 실행별 namespace, node name, parameter와 remap에 따라 실제 graph는 달라진다. 조건부 endpoint, 외부 server, 미구현 타입을 구분하며 모든 노드가 동시에 실행되는 것으로 해석하지 않는다.

## 표 읽기

| 표기 | 뜻 |
|---|---|
| 토픽 발행 / 구독 | 해당 노드에서 나가는 데이터 / 들어오는 데이터 |
| 서비스 제공 / 호출 | Server / client |
| 액션 제공 / 호출 | Action server / client |
| R10, R20, R1 | Reliable, volatile, keep-last depth 10 / 20 / 1 |
| B1 | Best-effort, volatile, keep-last depth 1 |
| Sensor | Humble `qos_profile_sensor_data`: best-effort, volatile, keep-last depth 5 |
| R/T1, R/T2048 | Reliable, transient-local, keep-last depth 1 / 2048 |
| Service 기본값 | Humble service QoS: reliable, volatile, keep-last depth 10 |
| Action 기본값 | rclpy 기본 request/result/cancel service와 feedback/status QoS. Status는 reliable, transient-local, depth 1 |
| TF 기본 QoS | tf2_ros broadcaster/listener 기본값. 응용 코드가 별도 QoS를 지정하지 않음 |
| 상대 이름 / `/절대 이름` / `~/private` | Node namespace 적용 / root 이름 / node namespace와 node name 적용 |
| 주기 | 소스의 기본 timer 또는 callback 조건. 실제 수신 빈도와 동일하지 않을 수 있음 |

## 구현 노드

| 노드 이름 | 패키지 | 실행 파일 | 인터페이스 표 |
|---|---|---|---|
| `cleany_base_driver` | `cleany_base_driver` | `base_driver_node` | [표](../ros2_ws/src/cleany_base_driver/docs/interfaces.md#cleany_base_driver) |
| `mock_base_mcu` | `cleany_base_driver` | `mock_mcu_node` | [표](../ros2_ws/src/cleany_base_driver/docs/interfaces.md#mock_base_mcu) |
| `base_odom_relay` | `cleany_base_driver` | `odom_relay_node` | [표](../ros2_ws/src/cleany_base_driver/docs/interfaces.md#base_odom_relay) |
| `wheel_odometry` | `cleany_base_odometry` | `wheel_odometry_node` | [표](../ros2_ws/src/cleany_base_odometry/docs/interfaces.md#wheel_odometry) |
| `control_bridge` | `cleany_control_bridge` | `control_bridge` | [표](../ros2_ws/src/cleany_control_bridge/docs/interfaces.md#control_bridge) |
| `mission_runtime` | `cleany_mission_manager` | `mission_runtime` | [표](../ros2_ws/src/cleany_mission_manager/docs/interfaces.md#mission_runtime) |
| `grasp_server` | `cleany_grasping` | `grasp_server` | [표](../ros2_ws/src/cleany_grasping/docs/interfaces.md#grasp_server) |
| `grasp_selection_server` | `cleany_skill_executor` | `grasp_selection_server` | [표](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#grasp_selection_server) |
| `grasp_execution_demo` | `cleany_skill_executor` | `grasp_execution_demo` | [표](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#grasp_execution_demo) |
| `can_grasp_execution_demo` | `cleany_skill_executor` | `can_grasp_execution_demo` | [표](../ros2_ws/src/cleany_skill_executor/docs/interfaces.md#can_grasp_execution_demo) |
| `handeye_collision_scene_applier` | `cleany_moveit_config` | `apply_handeye_collision_scene` | [표](../ros2_ws/src/cleany_moveit_config/docs/interfaces.md#handeye_collision_scene_applier) |
| `cleany_telemetry` | `cleany_telemetry` | `telemetry_node` | [표](../ros2_ws/src/cleany_telemetry/docs/interfaces.md#cleany_telemetry) |
| `depth_clearance` | `cleany_navigation` | `depth_clearance` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#depth_clearance) |
| `depth_coverage` | `cleany_navigation` | `depth_coverage` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#depth_coverage) |
| `pan_motion_gate` | `cleany_navigation` | `pan_motion_gate` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#pan_motion_gate) |
| `reactive_guard` | `cleany_navigation` | `reactive_guard` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#reactive_guard) |
| `obstacle_memory` | `cleany_navigation` | `obstacle_memory` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#obstacle_memory) |
| `voxel_guard` | `cleany_navigation` | `voxel_guard` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#voxel_guard) |
| `nav2_bt_trace` | `cleany_navigation` | `nav2_bt_trace` | [표](../ros2_ws/src/cleany_navigation/docs/interfaces.md#nav2_bt_trace) |
| `gazebo_command_guard` | `cleany_gazebo_sim` | `gazebo_command_guard` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_command_guard) |
| `gazebo_feedback_sampler` | `cleany_gazebo_sim` | `gazebo_feedback_sampler` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_feedback_sampler) |
| `gazebo_odom_tf_publisher` | `cleany_gazebo_sim` | `gazebo_odom_tf_publisher` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_odom_tf_publisher) |
| `gazebo_pan_tf_publisher` | `cleany_gazebo_sim` | `gazebo_pan_tf_publisher` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_pan_tf_publisher) |
| `gazebo_sensor_tf_publisher` | `cleany_gazebo_sim` | `gazebo_sensor_tf_publisher` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#gazebo_sensor_tf_publisher) |
| `simulated_encoder` | `cleany_gazebo_sim` | `simulated_encoder_node` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#simulated_encoder) |
| `simulated_odometry_error` | `cleany_gazebo_sim` | `simulated_odometry_error_node` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#simulated_odometry_error) |
| `ground_truth_route_follower` | `cleany_gazebo_sim` | `ground_truth_route_follower` | [표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#ground_truth_route_follower) |
| `mujoco_sim` | `cleany_mujoco_sim` | `mujoco_sim_node` | [표](../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#mujoco_sim) |
| `mujoco_rgbd_sim` | `cleany_mujoco_sim` | `mujoco_rgbd_sim_node` | [표](../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#mujoco_rgbd_sim) |
| `left_wrist_camera_contract_adapter` | `cleany_mujoco_sim` | `camera_contract_adapter` | [표](../ros2_ws/src/cleany_mujoco_sim/docs/interfaces.md#left_wrist_camera_contract_adapter) |
| `single_pose_handeye_orchestrator` | `cleany_handeye_calibration` | `single_pose_calibration` | [표](../ros2_ws/src/cleany_handeye_calibration/docs/interfaces.md#single_pose_handeye_orchestrator) |
| `multi_pose_handeye_orchestrator` | `cleany_handeye_calibration` | `multi_pose_calibration` | [표](../ros2_ws/src/cleany_handeye_calibration/docs/interfaces.md#multi_pose_handeye_orchestrator) |
| `handeye_pose_generator` | `cleany_handeye_calibration` | `generate_pose_manifest` | [표](../ros2_ws/src/cleany_handeye_calibration/docs/interfaces.md#handeye_pose_generator) |
| `cleany_dev_monitor` | `cleany_dev_monitor` | `dev_monitor` | [표](../ros2_ws/src/cleany_dev_monitor/docs/interfaces.md#cleany_dev_monitor) |
| `cleany_motor_controller` | ESP32 micro-ROS | `esp32/src/main.cpp` | [MCU 표](../esp32/README.md#ros-인터페이스) |

Single/multi pose 실행 전 설정 파일만 읽는 `single_pose_handeye_config`, `multi_pose_handeye_config`는 임시 bootstrap 노드다. 응용 endpoint가 없으며 ROS 기본 인터페이스만 생성한다. `cleany_description`, `cleany_bringup`은 모델/launch를 제공하고, `cleany_interfaces`, `cleany_base_interfaces`는 타입을 정의한다. `cleany_perception`, `cleany_planner`, `cleany_logger`에는 현재 등록된 자체 ROS 노드가 없다.

## 외부 노드와 타입만 있는 계약

| 경계 | 연결 위치와 현재 상태 |
|---|---|
| Gazebo `ros_gz_bridge` | [Bridge 표](../ros2_ws/src/cleany_gazebo_sim/docs/interfaces.md#ros_gz_bridge). ROS/Gazebo sensor와 명령 변환 |
| Nav2 / AMCL / Collision Monitor | [Navigation launch](../ros2_ws/src/cleany_navigation/launch/navigation.launch.py)와 [설정](../ros2_ws/src/cleany_navigation/config)이 조합을 선택. Mission이 호출하는 action/lifecycle 서비스는 Mission 노드 표에 포함 |
| MoveIt `move_group` | `/compute_ik`, `/compute_fk`, `/check_state_validity`, `/get_planning_scene`, `/apply_planning_scene`, `/move_action`, `/execute_trajectory`를 이용. 자체 호출 노드 표와 [Planning 계약](../ros2_ws/src/cleany_moveit_config/docs/planning.md)에서 연결 확인 |
| ros2_control | Arm/gripper의 FollowJointTrajectory와 joint_state_broadcaster. [Controller 설정](../ros2_ws/src/cleany_moveit_config/config/mock_ros2_controllers.yaml)과 [MuJoCo backend launch](../ros2_ws/src/cleany_mujoco_sim/launch/handeye_backend.launch.py)이 선택 |
| `robot_state_publisher` | `joint_states` 구독, `/tf`, `/tf_static` 발행. URDF와 [모델 계약](../ros2_ws/src/cleany_description/docs/model.md)이 frame 의미를 관리 |
| `InspectScene` | [Action 타입](../ros2_ws/src/cleany_interfaces/action/InspectScene.action)은 존재. 이 저장소에는 해당 ROS action server 구현이 없음 |
| `ExecuteManipulationSkill` | Runtime의 action client와 [타입](../ros2_ws/src/cleany_interfaces/action/ExecuteManipulationSkill.action)은 존재. 기본 mock는 process 내부 구현이며 외부 action server는 별도로 필요 |

위 표는 외부 패키지의 연결 경계다. 외부 패키지의 전체 API 목록은 해당 패키지 문서와 실행 graph를 기준으로 확인한다. RViz, rqt, controller spawner와 검증용 probe 노드는 실행 보조 도구이며 자체 제품 endpoint 목록에 포함하지 않는다.

## ROS가 자동 생성하는 인터페이스

Parameter 서비스와 이벤트, rosout, sim clock 구독, action 내부 전송은 응용 표에서 반복하지 않는다. 아래 패턴이 노드 옵션과 실행 상태에 따라 추가된다.

| 항목 | 기본 패턴 / 타입 |
|---|---|
| Parameter 서비스 | `<node>/{describe_parameters,get_parameter_types,get_parameters,list_parameters,set_parameters,set_parameters_atomically}`; `rcl_interfaces/srv/*` |
| Parameter 변경 이벤트 | `/parameter_events`; `rcl_interfaces/msg/ParameterEvent` |
| 로그 | `/rosout`; `rcl_interfaces/msg/Log`, rosout 활성화 시 |
| Simulation clock | `/clock`; `rosgraph_msgs/msg/Clock`, ROS time source 연결 시; `use_sim_time`으로 시간 적용 선택 |
| Action 요청 / 결과 / 취소 | `<action>/_action/{send_goal,get_result,cancel_goal}` 서비스 |
| Action feedback / status | `<action>/_action/{feedback,status}` 토픽 |

명시적으로 만든 cancel client와 status subscription은 호출 노드의 표에도 표시한다. ESP32 rclc 노드는 표에 적힌 wheel interface만 초기화하며 rclpy의 자동 parameter 서비스가 동일하게 생긴다고 가정하지 않는다.

## 실행 중 확인

실행한 구성과 같은 DDS domain, 같은 overlay의 터미널에서 확인한다.

```bash
ros2 node list
ros2 node info /mission_runtime
ros2 topic list -t
ros2 topic info /cmd_vel --verbose
ros2 service list -t
ros2 action list -t
ros2 action info /grasp/select_reachable
```

실행 graph는 해당 실행의 증거다. 조건부 노드를 실행하지 않은 결과를 전체 소스의 인터페이스 목록으로 대체하지 않는다. 코드를 변경할 때 해당 노드 표와 이 색인을 함께 갱신한다.
