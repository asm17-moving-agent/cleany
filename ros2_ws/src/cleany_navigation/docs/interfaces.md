# cleany_navigation 노드 인터페이스

## localization_guard

실행 파일은 `localization_guard`이며 휴대폰 LiDAR 주행 launch에서 실행한다.
지도와 scan의 정합을 관측하고, Supervisor가 요청한 정지 상태의 위치 복구를 수행한다.

| 방향 | 이름 | 타입 | 계약 |
|---|---|---|---|
| 구독 | `map` | nav_msgs/OccupancyGrid | R/T1; 지도 변경 시 이전 정합 기록 무효화 |
| 구독 | `scan` | sensor_msgs/LaserScan | Sensor; scan 시각의 TF로 평가 |
| 구독 | `amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | Sensor; 신선한 map 좌표와 유한 공분산 확인 |
| 구독 | `diagnostics` | diagnostic_msgs/DiagnosticArray | R10; base_driver의 disarmed와 피드백 나이 확인 |
| 구독·발행 | `initialpose` | geometry_msgs/PoseWithCovarianceStamped | R10; 수동 입력은 진행 중인 자동 검색을 중단 |
| 발행 | `system/component_health` | cleany_interfaces/ComponentHealth | R10, 5Hz; source=`localization_alignment`; 검증된 위치의 degraded는 제한된 주행을 위해 AVAILABLE 유지 |
| 발행 | `localization/status` | std_msgs/String | R1, 5Hz; state/reason/phase, score, 재정합 사유, 속도 제한 비율, 지도 ID와 관측 시각 |
| 발행 | `navigation/localization_speed_limit` | nav2_msgs/SpeedLimit | R10, 5Hz; percentage=true; 정상 100%, degraded 30%, 미확인·실패 0% |
| Action 서버 | `localization/recover` | cleany_interfaces/RecoverComponent | 동시에 한 요청; 정지·신선도·후보·AMCL 결과 검증 |
| 서비스 호출 | `request_nomotion_update` | std_srvs/Empty | 정지 중 위치 미확인 또는 복구 검증 시 실제 AMCL 관측 갱신 요청 |

TF는 읽기만 한다. wheel 명령과 속도를 발행하지 않는다. 토픽 이름은 ROS remap으로
바꿀 수 있다. 다른 `localization/recover` owner와 동시에 실행하지 않는다.
상태와 복구 조건은 [위치 정합과 복구](planning.md#위치-정합과-복구)를 따른다.

## slip_monitor, traction_brake와 SlipRecovery BT

| 소유자 | 방향 | 기본 이름 | 타입 | 의미 |
|---|---|---|---|---|
| `slip_monitor` | 구독 | `scan_raw`, `odom` | LaserScan, Odometry | 센서 header 시각의 독립 이동 비교, sensor-data QoS |
| `slip_monitor` | 구독 | `nav2/cmd_vel`, `cmd_vel` | geometry_msgs/Twist | 분석용 요청/최종 명령, sensor-data QoS |
| `slip_monitor` | 구독 | `map` | nav_msgs/OccupancyGrid | transient-local depth 1, 지도 geometry/cells 식별 해시 |
| `slip_monitor` | 발행 | `navigation/slip` | std_msgs/String | 5Hz JSON, state, t0/t1, consecutive, reason, 이동 및 오차 |
| BT `SlipRecovery` | 구독 | `navigation/slip`, `odom` | String, Odometry | 새로운 신뢰 가능한 비교와 fresh 정지 증거 |
| BT `SlipRecovery` | 구독 | `motion/traction_cmd_vel` | geometry_msgs/Twist | fresh 0 출력 확인, 이후 0.5초 정지 유지 |
| BT `SlipRecovery` | 발행 | `navigation/traction_brake` | std_msgs/Bool | tick마다 brake heartbeat, true는 감속 정지 |
| BT `SlipRecovery` | 발행 | `navigation/traction_recovery` | std_msgs/String | JSON phase, reason, retry |
| `traction_brake` | 구독 | `motion/cmd_vel`, `navigation/traction_brake` | Twist, Bool | 기존 자동 조종 명령과 BT 상태 |
| `traction_brake` | 발행 | `motion/traction_cmd_vel` | geometry_msgs/Twist | 50Hz 충돌 감시 입력, stale heartbeat/명령은 0 |

`state`는 `unknown`, `clear`, `suspect`, `confirmed`다. `t0/t1`은 scan midpoint의
ROS 초이며 같은 비교를 heartbeat로 재발행해도 연속 횟수는 증가하지 않는다.
`clear`는 해당 비교에서 검출 증거가 없다는 의미다. 정합 품질 탈락은 `unknown`이다.
`phase`는 `warming_up`, `normal`, `braking`, `longitudinal_retry`, `failed`,
`succeeded`, `idle` 중 하나다. 감지 노드는 명령과 TF를 발행하지 않는다.

`SlipRecovery`는 일반 Nav2 subtree와 종방향 retry subtree 두 child를 갖는
control plugin이다. header age, 수신 freshness와 retry 이후 비교 시작 시각을
검사한다. 한 목표 안에서 retry latch를 유지하고 취소 시 두 child를 halt한다.
복구 정책과 시간 설정은 [실행 가이드](../../../../docs/RUNNING.md#휴대폰-omni-주행의-슬립-복구)를 따른다.

종방향 retry는 `/traction_retry/compute_path_to_pose` 액션과 `Longitudinal`
planner ID를 사용한다. retry planner server와 그 global costmap은 `traction_retry`
namespace와 별도 lifecycle manager를 가지며, 지도와 scan 입력은 `/map`, `/scan`이다.

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## phone_collision_monitor

실행 파일: `phone_collision_monitor`. 실행 노드명: `collision_monitor`. 원본: [phone_collision_monitor.cpp](../src/phone_collision_monitor.cpp).

Nav2 Collision Monitor의 lifecycle, 센서 좌표 변환, polygon 판정과 정지를 사용한다.
지정한 slowdown polygon에서는 고정 비율 대신 병진 합성 속도와 회전의 절대 상한으로
제한 비율을 계산한다. `translation_limit_m_s`, `rotation_limit_rad_s`가 모두 0이면
기존 휠 제한만 사용한다. 최종 휠 상한도 함께 검사하며, 작은 명령은 키우지 않는다.
안전 출력의 별도 발행자를 추가하지 않는다.

`localization_speed_limit_topic`을 지정하면 같은 percentage 제한을 추가로 적용한다.
`nominal_translation_m_s`, `nominal_rotation_rad_s`를 기준으로 절대 상한을 계산하므로
이미 작은 명령에 비율을 중복 곱하지 않는다. 신선한 제한이 없으면 0을 출력한다.
정상값에서 감속값으로 낮출 때는 `localization_limit_ramp_per_s`를 따르며, 0%와
입력 timeout은 ramp를 거치지 않는다.

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `motion/cmd_vel` | `geometry_msgs/msg/Twist` | R1; `cmd_vel_in_topic` |
| 토픽 구독 | `scan` | `sensor_msgs/msg/LaserScan` | Sensor; `scan.topic` |
| 토픽 발행 | `navigation/safe_cmd_vel` | `geometry_msgs/msg/Twist` | R1 lifecycle; `cmd_vel_out_topic` |
| 토픽 발행 | `safety/stop_zone` | `geometry_msgs/msg/PolygonStamped` | R1 lifecycle; `StopZone.polygon_pub_topic` |
| 토픽 발행 | `safety/slow_zone` | `geometry_msgs/msg/PolygonStamped` | R1 lifecycle; `SlowZone.polygon_pub_topic` |
| 서비스 제공 | `collision_monitor/change_state`, `collision_monitor/get_state` | lifecycle_msgs 서비스 | Nav2 lifecycle manager가 소유 |

`phone_lidar_safety.yaml`은 바퀴 외곽 +16cm의 주황 영역과 +5cm 정지 영역을 정의한다. `absolute_limit_polygon` 기본값 `SlowZone`, `wheel_limit_rad_s` 기본값 0.7, wheel radius 0.0635m, wheelbase 0.35m, wheel separation 0.59m를 사용한다. 속도와 geometry 변경은 lifecycle 재구성을 거친다. `SlowZone.slowdown_ratio`는 계산에 쓰는 고정 감속비가 아니며 각 입력에서 상한 비율로 대체된다. 상한보다 작은 명령은 그대로 통과한다.

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

실물 `phone_depth_local.yaml`은 `/camera/d435/depth/image_rect_raw`와
`/camera/d435/depth/camera_info`를 받아 `/perception/depth/nav_points`(카메라 좌표의
유효 clearing 관측), `/perception/depth/nav_obstacle_points`(`base_link`의 필터링한
marking 점)를 발행한다. 두 출력은 원본 촬영 시각을 유지한다. local VoxelLayer는
`sensor_frame=d435_depth_frame`으로 ray 시작점을 지정한다.
`spatial_filter_file` 설정 시 marking에 거리, 높이, 고립점과 voxel 간격 필터를 적용한다.
`self_filter_boxes_file`은 base 좌표 고정 차체 AABB JSON이며 기존 `self_filter_profile`과
동시에 지정할 수 없다. self-occluded 광선은 marking과 clearing 모두에서 제외한다.
실물 설정은 `max_image_age_s=0.5`, `min_valid_fraction=0.05`다. TF 누락, 오래된 영상,
유효 깊이 부족은 출력하지 않는다. `positive_infinity_is_free=false`로 무효 픽셀을
빈 공간으로 바꾸지 않는다. 카메라 방향이 바뀌어 재관측하지 못한 장애물은 지우지 않는다.

`phone_collision_monitor`는 이 marking 토픽을 SensorData QoS로 받아 기존 2D 영역에
적용한다. 실물 overlay의 `required_depth_topic`, `required_depth_frame=base_link`,
`required_depth_timeout_s=0.5`는 최초 유효 입력 전과 입력 중단, 촬영 시각 만료 시
속도 입력을 0으로 제한한다. Humble의 기본 source timeout은 오래된 센서를 무시하므로
같은 Monitor 안에서 필수 depth의 신선도를 별도로 확인한다. 유효 영상에서 장애물만
없는 빈 점군은 정상 입력이다. 기본 `required_depth_topic`은 비어 있어 LiDAR 전용은 유지된다.

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

충돌 검사와 속도 출력 주기는 `update_period_s` 기본 0.10초(10Hz)다. 기존 Collision Monitor와
ToF의 주기는 별도로 유지한다. 이름 외에도 body profile과 안전 설정이 필요하다.

## nav2_bt_trace

실행 파일: `nav2_bt_trace`. 원본: [bt_trace_node.py](../cleany_navigation/bt_trace_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | R10; `log_topic` |
| 토픽 구독 | `bt_navigator/transition_event` | `lifecycle_msgs/msg/TransitionEvent` | R10; `navigator` + suffix |
| 토픽 구독 | `navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | R/T1; `action` + suffix |
| 토픽 발행 | `navigation/bt_trace` | `std_msgs/msg/String` | R/T1; 입력 이벤트 처리 시, `output_topic` |

시작할 때 `manifest_file`과 `xml_file`을 검증한다.

## 매핑 세션 상태

`mapping/status`의 `error_code`, `error_file`은 마지막 작업 실패의 단계 코드와
진단 JSON 경로다. 오류가 없으면 빈 문자열이며 새 작업 접수 시 초기화된다.
`output`은 시작 준비 단계부터 제공되고, `generation`은 성공한 시작 횟수만 나타낸다.

## nav2_lifecycle_recovery

실행 파일: `nav2_lifecycle_recovery`. 원본: [lifecycle_recovery_node.py](../cleany_navigation/lifecycle_recovery_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 액션 제공 | `navigation/recover` | `cleany_interfaces/action/RecoverComponent` | Action 기본값; `actions` |
| 액션 제공 | `localization/recover` | `cleany_interfaces/action/RecoverComponent` | Action 기본값; `actions` |
| 서비스 호출 | `lifecycle_manager_navigation/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | Service 기본값; `lifecycle_managers`, `call_timeout_s` 10초 |
| 서비스 호출 | `lifecycle_manager_localization/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | Service 기본값; `lifecycle_managers` |

System Supervisor의 복구 요청을 받아 해당 lifecycle manager에 RESET, STARTUP 순서로 위임한다. 모든 Action을 통틀어 한 번에 하나의 goal만 수락하고, cancel은 단계 사이에서만 적용한다. 성공은 manager가 성공을 응답했다는 뜻이며 실제 복구는 Supervisor가 이후 fresh lifecycle과 localization health로 검증한다. `lifecycle_manager_safety`의 collision_monitor와 Nav2 BT recovery는 다루지 않는다. 실제 Nav2 stack에서 RESET 뒤 AMCL 초기 pose와 지도 상태는 검증하지 않았다.

## ToF를 활성화한 명령 경로

`navigation.launch.py tof_enabled:=true`는 `nav2/cmd_vel`과 기존 안전 계층 사이에
`motion/cmd_vel`을 선택하는 `desk_approach`를 둔다. 기존 최종 안전 출력은
`proximity/input`으로 옮기고 `proximity_guard`가 유일한 `cmd_vel` 발행자가 된다.
`depth_coverage`의 고정 구독도 선택된 명령으로 remap한다. 별도 Nav2 노드나 recovery는
기존과 같은 경로를 사용한다. ToF endpoint와 계약은
[ToF reference](../../cleany_tof/docs/interfaces.md)를 따른다.

기존 `depth_coverage`와 `head_pan/coverage`는 원래 observer 설정에 따른다. ToF는
RGB-D coverage 근거를 요청하지 않으며 observer를 강제로 활성화하지 않는다.

병행 지도 확장 실험은 `tools/validation/map_extension/extend.py`에서 표준 Cartographer
서비스 `start_trajectory`, `finish_trajectory`, `write_state`, `get_trajectory_states`를
`/map_extension` 아래에서 호출한다. 입력 `/scan`과 `/odom`은 기존 실물 토픽이며,
지도와 자세 출력은 `/map_extension/map`, `/map_extension/tracked_pose`다.
`publish_to_tf=false`로 기존 AMCL의 TF 소유권을 유지한다.

### 슬립 관측의 분석 문맥

`navigation/slip.context`는 새 관측 창마다 고정하며 반복 heartbeat에서 갱신하지 않는다.
`at=t1`의 `map → base_link` TF를 조회해 `pose=[x,y,yaw]`와 `frame`을 담는다.
과거 TF가 없으면 `pose=null`, `pose_reason=historical_transform_unavailable`이며 최신 TF로
대체하지 않는다. 위치는 비교 창의 끝점이지 슬립이 시작된 정확한 접촉 위치가 아니다.
`map_revision`은 문맥 생성 시 수신한 지도의 frame, geometry와 cells에 대한 SHA-256이다.

`commands.requested/output`은 각각 기본 `nav2/cmd_vel`, `cmd_vel`의
`velocity=[vx,vy,wz]`, `received_at`, `age_s`, `topic`, `reason`을 담는다.
Twist에 header가 없으므로 `command_time_basis=ros_receive_time`이다. `t1` 이전 마지막
수신값을 선택하며 기본 `command_context_max_age_s=0.3`보다 오래되거나 값이 없으면 null이다.
미래 명령을 과거 관측에 붙이지 않는다. 명령 이력은 각 4초/최대 512개로 제한한다.
기록용 topic/frame은 `requested_command_topic`, `output_command_topic`, `map_topic`,
`map_frame` 파라미터로 바꾼다. 이 문맥은 감지 판정이나 BT 제어 입력이 아니다.

로거 motion 항목은 `slip_topic`을 5Hz, `traction_recovery_topic`을 수신 이벤트마다 기록한다.
기존 raw CDR bag, 용량 상한과 보존 정책을 사용한다. 별도 무제한 파일은 만들지 않는다.


## depth_obstacles_preview

실행 파일 `depth_obstacles_preview`는 촬영 시점 pan/tilt TF로 깊이 점을
`base_link`로 변환하는 미리보기 노드다. costmap, clearing, 주행 명령은 발행하지 않는다.

| 방향 | 토픽 | 타입 | 의미 |
|---|---|---|---|
| 구독 | `/camera/d435/depth/image_rect_raw` | sensor_msgs/Image | rectified 깊이 |
| 구독 | `/camera/d435/depth/camera_info` | sensor_msgs/CameraInfo | 깊이 내부 파라미터 |
| 구독 | `/scan_raw` | sensor_msgs/LaserScan | CAD 높이의 LiDAR 표시 |
| 발행 | `/perception/depth/measured_points` | sensor_msgs/PointCloud2 | 원본 변환 점 |
| 발행 | `/perception/depth/obstacle_points` | sensor_msgs/PointCloud2 | 높이, 고정 차체, 고립점 필터 결과 |
| 발행 | `/perception/depth/ground_points` | sensor_msgs/PointCloud2 | 관측 바닥면 주변 진단용 점 |
| 발행 | `/perception/lidar/cad_points` | sensor_msgs/PointCloud2 | LiDAR 별도 표시 |
| 발행 | `/perception/depth/filter_status` | std_msgs/String | 유효성, 개수, `min_base_z` JSON |

센서와 점군 QoS는 SensorData, 상태는 reliable depth 10이다. 촬영 시점 TF를 얻지 못하거나
새 유효 데이터가 0.5초 동안 없으면 미리보기를 비운다. 원본 촬영 시각을 유지한다.
`filter_file` JSON의 `min_base_z`를 지정하면 바닥 상대 하한 `min_height` 대신
`base_link`의 고정 z 하한(m)을 쓴다. 미지정 시 바닥 상대 하한을 사용한다.
상한 `max_height`는 관측 바닥면 기준이고, floor 설정은 진단용 grid에도 사용한다.
`body_boxes_file`은 고정 CAD 부품의 ray self-mask이며 자세 미상의 가동 팔은 제외한다.
별도 static TF `d435_ground_preview`, `lidar_cad_preview`를 발행하며 기존 `lidar_link`는 변경하지 않는다.
