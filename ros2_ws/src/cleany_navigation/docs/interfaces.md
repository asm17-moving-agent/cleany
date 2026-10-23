# cleany_navigation 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## depth_clearance

실행 파일: `depth_clearance`. 원본: [depth_clearance_node.py](../cleany_navigation/depth_clearance_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `camera/head/depth/image_raw` | `sensor_msgs/msg/Image` | B1; `input_topic` |
| 토픽 구독 | `camera/head/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor; `camera_info_topic` |
| 토픽 발행 | `camera/head/depth/points` | `sensor_msgs/msg/PointCloud2` | Sensor; 유효 depth 처리 시, `output_topic` |
| 토픽 발행 | `camera/head/depth/obstacle_points` | `sensor_msgs/msg/PointCloud2` | Sensor; `obstacle_topic` |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

## depth_coverage

실행 파일: `depth_coverage`. 원본: [depth_coverage_node.py](../cleany_navigation/depth_coverage_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `camera/head/depth/image_raw` | `sensor_msgs/msg/Image` | Sensor; `input_topic` |
| 토픽 구독 | `camera/head/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor; `camera_info_topic` |
| 토픽 구독 | `nav2/cmd_vel` | `geometry_msgs/msg/Twist` | R10 |
| 토픽 구독 | `odom` | `nav_msgs/msg/Odometry` | Sensor |
| 토픽 발행 | `head_pan/coverage` | `std_msgs/msg/String` | R10; `update_period_s` 기본 0.2초 |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

## pan_motion_gate

실행 파일: `pan_motion_gate`. 원본: [pan_motion_gate_node.py](../cleany_navigation/pan_motion_gate_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `nav2/cmd_vel` | `geometry_msgs/msg/Twist` | R10; `command_topic` |
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | Sensor; `joint_states_topic` |
| 토픽 구독 | `odom` | `nav_msgs/msg/Odometry` | Sensor; `odom_topic` |
| 토픽 구독 | `camera/head/depth/points` | `sensor_msgs/msg/PointCloud2` | Sensor; `depth_topic` |
| 토픽 발행 | `pan_ready/cmd_vel` | `geometry_msgs/msg/Twist` | R10; `output_topic` |
| 토픽 발행 | `head_pan/command` | `std_msgs/msg/Float64` | R10; `pan_command_topic` |
| 토픽 발행 | `head_pan/state` | `std_msgs/msg/String` | R10; `state_topic` |

`observation_only=true`에서는 속도 출력 publisher와 odom/depth 구독을 생성하지 않는다. Update 주기는 `update_period_s` 기본 0.05초다. TF listener는 없다.

## reactive_guard

실행 파일: `reactive_guard`. 원본: [reactive_guard_node.py](../cleany_navigation/reactive_guard_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `safety_2d/cmd_vel` | `geometry_msgs/msg/Twist` | R1; `command_topic` |
| 토픽 구독 | `nav2/cmd_vel` | `geometry_msgs/msg/Twist` | R1; `raw_command_topic` |
| 토픽 구독 | `odom` | `nav_msgs/msg/Odometry` | Sensor; `odom_topic` |
| 토픽 구독 | `scan` | `sensor_msgs/msg/LaserScan` | Sensor; `scan_topic` |
| 토픽 구독 | `camera/head/depth/points` | `sensor_msgs/msg/PointCloud2` | Sensor; `depth_topic` |
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | Sensor; `joint_states_topic`, `require_pan_feedback=true` |
| 토픽 발행 | `cmd_vel` | `geometry_msgs/msg/Twist` | R1; `output_topic` |
| 토픽 발행 | `reactive_guard/state` | `std_msgs/msg/String` | R10 |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

선택 실행 경로다. Full voxel 경로의 최종 publisher와 중복 실행하지 않는다. Update 주기는 `update_period_s` 기본 0.05초다.

## obstacle_memory

실행 파일: `obstacle_memory`. 원본: [obstacle_memory_node.py](../cleany_navigation/obstacle_memory_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `map` | `nav_msgs/msg/OccupancyGrid` | R/T1; `map_topic` |
| 토픽 구독 | `scan` | `sensor_msgs/msg/LaserScan` | Sensor; `scan_topic` |
| 토픽 구독 | `camera/head/depth/points` | `sensor_msgs/msg/PointCloud2` | Sensor; `depth_topic` |
| 토픽 발행 | `obstacle_memory/grid` | `nav_msgs/msg/OccupancyGrid` | R/T1; memory update 시 |
| 토픽 발행 | `obstacle_memory/lidar` | `nav_msgs/msg/OccupancyGrid` | R/T1; memory update 시 |
| 토픽 발행 | `obstacle_memory/depth` | `nav_msgs/msg/OccupancyGrid` | R/T1; memory update 시 |
| 토픽 발행 | `obstacle_memory/navigation_grid` | `nav_msgs/msg/OccupancyGrid` | R/T1; memory update 시 |
| 토픽 발행 | `obstacle_memory/voxels` | `sensor_msgs/msg/PointCloud2` | R/T1; `world_publish_period` |
| 토픽 발행 | `obstacle_memory/local_voxels` | `sensor_msgs/msg/PointCloud2` | R/T1; memory update 시 |
| 토픽 발행 | `obstacle_memory/voxel_status` | `std_msgs/msg/String` | R/T1 |
| 서비스 제공 | `obstacle_memory/save` | `std_srvs/srv/Trigger` | Service 기본값 |
| 서비스 제공 | `obstacle_memory/set_enabled` | `std_srvs/srv/SetBool` | Service 기본값 |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

Update 주기는 `update_period` 기본 0.5초이며, 출력별 실제 발행은 변경 여부와 별도 주기 설정에 따라 달라진다.

## voxel_guard

실행 파일: `voxel_guard`. 원본: [voxel_guard_node.py](../cleany_navigation/voxel_guard_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `safety_2d/cmd_vel` | `geometry_msgs/msg/Twist` | R10; `command_topic` |
| 토픽 구독 | `odom` | `nav_msgs/msg/Odometry` | Sensor; `odom_topic` |
| 토픽 구독 | `obstacle_memory/voxels` | `sensor_msgs/msg/PointCloud2` | R/T1; `voxel_topic` |
| 토픽 구독 | `obstacle_memory/voxel_status` | `std_msgs/msg/String` | R/T1; `status_topic` |
| 토픽 발행 | `cmd_vel` | `geometry_msgs/msg/Twist` | R10; `output_topic` |
| 토픽 발행 | `body_guard_3d/state` | `std_msgs/msg/String` | R/T1 |
| 토픽 발행 | `body_guard_3d/risk_voxels` | `sensor_msgs/msg/PointCloud2` | R/T1 |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

Update 주기는 `update_period_s` 기본 0.05초다. 이름 외에도 body profile과 안전 설정이 필요하다.

## nav2_bt_trace

실행 파일: `nav2_bt_trace`. 원본: [bt_trace_node.py](../cleany_navigation/bt_trace_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | R10; `log_topic` |
| 토픽 구독 | `bt_navigator/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | R10; `navigator` + suffix |
| 토픽 구독 | `navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | R/T1; `action` + suffix |
| 토픽 발행 | `navigation/bt_trace` | `std_msgs/msg/String` | R/T1; 입력 이벤트 처리 시, `output_topic` |

시작할 때 `manifest_file`과 `xml_file`을 검증한다.

## nav2_lifecycle_recovery

실행 파일: `nav2_lifecycle_recovery`. 원본: [lifecycle_recovery_node.py](../cleany_navigation/lifecycle_recovery_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 액션 제공 | `navigation/recover` | `cleany_interfaces/action/RecoverComponent` | Action 기본값; `actions` |
| 액션 제공 | `localization/recover` | `cleany_interfaces/action/RecoverComponent` | Action 기본값; `actions` |
| 서비스 호출 | `lifecycle_manager_navigation/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | Service 기본값; `lifecycle_managers`, `call_timeout_s` 10초 |
| 서비스 호출 | `lifecycle_manager_localization/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | Service 기본값; `lifecycle_managers` |

System Supervisor의 복구 요청을 받아 해당 lifecycle manager에 RESET, STARTUP 순서로 위임한다. 모든 Action을 통틀어 한 번에 하나의 goal만 수락하고, cancel은 단계 사이에서만 적용한다. 성공은 manager가 성공을 응답했다는 뜻이며 실제 복구는 Supervisor가 이후 fresh lifecycle과 localization health로 검증한다. `lifecycle_manager_safety`의 collision_monitor와 Nav2 BT recovery는 다루지 않는다. 실제 Nav2 stack에서 RESET 뒤 AMCL 초기 pose와 지도 상태는 검증하지 않았다.
