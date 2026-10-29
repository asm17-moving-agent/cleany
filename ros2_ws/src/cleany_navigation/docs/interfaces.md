# cleany_navigation 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## Nav2 controller와 planner

기본 설정은 `cleany_navigation::MPPIController`와 Smac Lattice를 사용한다.
Omni 설정의 `cleany_navigation::SmacLatticePlanner`는 native Smac을 상속하며,
시작/목표가 같은 XY 격자와 방향 bin일 때만 정확한 목표까지의 짧은 연결을
기존 전체 footprint/swept 충돌 검사로 확인한다. 이는 Nav2 1.1.20의 동일
탐색 노드에서 parent가 없어 역추적 실패하는 경우를 보완한다. 그 밖의 탐색과
primitive는 native Smac을 그대로 사용한다. 목표 오차나 안전 영역을 완화하지 않는다.
튜닝 소스는
`codex/nav2-tuning-recovered-20261024`의 `69b25020`에서 가져왔다.
MPPI controller/critic은 이 패키지의 `mppi/`와 `include/cleany_navigation/mppi/`에서
빌드한다. 원본 라이선스는 [LICENSE](../mppi/LICENSE.md)에 있다.

기존 Nav2 action과 속도 인터페이스는 유지한다. `visualize=true`일 때
`/trajectories`(MarkerArray, R1)와 `transformed_global_plan`(Path, R1)을 발행한다.
`mppi_replay_snapshot`은 저장된 map/pose 자료로 optimizer를 실행하는 독립
진단 CLI다. 일반 Mission launch에서는 실행하지 않는다.

`FollowPath.path_pose_yaw_weight`는 가까운 경로 시작점을 고를 때
XY 거리에 최단 각도 차이×가중치를 더한다(단위 m/rad, 시작 시 설정).
기본값0은 기존 XY 선택을 유지한다. 방향을 가진 lattice 경로가 회전 후
같은 XY 부근을 다시 지날 때, 양수 설정으로 아직 수행하지 않은 회전 구간을
잘라내는 것을 방지한다. 경로 후보와 출력의 footprint 충돌 검사는 그대로 적용된다.
같은 가중치는 현재 제어에 전달된 경로의 진행 좌표에도 적용한다. 재계획 시 좌표 기준만
갱신하고 누적 진행량과 제한 시각은 보존한다. 계획된 제자리 회전은 진행으로 평가하지만
경로 교체 자체나 멈춤, 반대 방향 회전은 기존 진행 제한 시간을 갱신하지 않는다.

`FollowPath.validated_stop_fallback=true`는 출력 검증에서 모든 이동 후보가
거부됐을 때 정지 sequence도 동일한 전체 footprint 검사로 확인한다(기본 false).
측정한 초기 속도에 의한 이동도 검사하며 무효이면 기존 오류를 반환한다.
sequence shifting 시 실제로 방출하는1번 제어부터 검사해, 방출하지 않는
0번 제어의 감속을 안전 여유로 계산하지 않는다.
유한한 양수 `path_progress_timeout`이 있어야 기동할 수 있다. 정지 출력은
진행 시간 제한을 초기화하지 않으며 각 횟수를 로그로 남긴다. 코스트맵을
지우거나 Nav2 복구를 호출하지 않는다. 경로 seed는 XY/각도 waypoint를
각각0.1mm/0.1mrad 이내로 지난 뒤 다음 구간으로 넘어가 코너 단축을 방지한다.

`GridBased.validate_swept_path=true`는 native lattice 경로 전체를 기존
`sweptPathCollisionFree`로 재검사한다(기본 false). 중간 자세가 충돌하면
기존 `checkedBodyHeadingGridPath`로 연속 footprint를 만족하는 경로를 보정한다.
검색은 `swept_repair_timeout`(기본2초)과 전체 `max_planning_time` 안에서만 수행한다.
`body_heading_grid_bins`(기본32), `body_heading_grid_heuristic_weight`(기본8)와
기존 `cost_penalty`를 사용한다. 보정에도 실패하면 빈 경로를 반환한다.
시작/목표를 native 격자 중심과 방향 bin에 맞췄을 때 충돌하면 native 검색을
호출하기 전에 같은 연속 자세 검색으로 연결한다. 실제 시작/목표의 footprint 검사도 유지한다.
이 옵션은 native 경로와 실제 시작/정확한 요청 목표 사이의 연결도 연속 검사한다.
출력은 격자 중심에서 끝나지 않으며 호출자가 지정한 도착 오차를 유지할 수 있다.
이는 계획 단계의 경로 보정이며 Nav2 복구 동작과 구별해 로그로 남긴다.

`smac_replay_snapshot SNAPSHOT OUTPUT.csv --ros-args --params-file NAVIGATION.yaml`은
`map.txt`(가로/세로/해상도/원점), `map.bin`(raw cost byte), `poses.txt`(x/y/yaw)의
첫 자세와 마지막 목표로 같은 플래너를 오프라인 실행한다. 경로의 각 footprint cost와
전체 연속 충돌 검사 결과를 남기며 제어 명령을 발행하지 않는다. MPPI 재생도 같은 입력을
사용하며 `MPPI_REPLAY_PATH_ONLY=1`일 때 경로 자세의 충돌 검사만 수행한다.

기본 lattice는 `config/lattice/omni-r10cm.json`이다. `resolve_default_lattice`가
설치 경로를 해석하며 명시한 `GridBased.lattice_filepath`는 유지한다.
5cm 격자와 10cm 곡선 반경은 primitive 생성 설정이며 실물 회전 반경 검증값이 아니다.
축 전용 DWB/StraightFirstPlanner는 `axis_motion=true`의 별도 프로파일이다.

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
