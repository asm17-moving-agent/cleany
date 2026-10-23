# cleany_dev_monitor 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## cleany_dev_monitor

실행 파일: `dev_monitor`. 원본: [node.py](../cleany_dev_monitor/node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `<runtime_namespace>/mission/debug_snapshot` | `cleany_interfaces/msg/RuntimeDebugSnapshot` | 동적 QoS; `runtime_namespace` |
| 토픽 구독 | `<runtime_namespace>/mission/runtime_events` | `cleany_interfaces/msg/RuntimeEvent` | 동적 QoS; `runtime_namespace`, depth 2048 |
| 토픽 구독 | `<runtime_namespace>/mission/result` | `cleany_interfaces/msg/MissionResult` | 동적 QoS; `runtime_namespace` |
| 토픽 구독 | `/rosout` | `rcl_interfaces/msg/Log` | 동적 QoS |
| 토픽 구독 | `/clock` | `rosgraph_msgs/msg/Clock` | 동적 QoS |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/map` | `nav_msgs/msg/OccupancyGrid` | 동적 QoS; `map_topic` |
| 토픽 구독 | `/plan` | `nav_msgs/msg/Path` | 동적 QoS; `path_topic` |
| 토픽 구독 | `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 동적 QoS; `local_costmap_topic` |
| 토픽 구독 | `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 동적 QoS; `global_costmap_topic` |
| 토픽 구독 | `/nav2/cmd_vel` | `geometry_msgs/msg/Twist` | 동적 QoS; `velocity_topics` |
| 토픽 구독 | `/safety_2d/cmd_vel` | `geometry_msgs/msg/Twist` | 동적 QoS; `velocity_topics` |
| 토픽 구독 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 동적 QoS; `velocity_topics` |
| 토픽 구독 | `/wheel/odom` | `nav_msgs/msg/Odometry` | 동적 QoS; `odometry_topic` |
| 토픽 구독 | `/ground_truth/odom` | `nav_msgs/msg/Odometry` | 동적 QoS; `ground_truth_topic` |
| 토픽 구독 | `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 동적 QoS; `amcl_pose_topic` |
| 토픽 구독 | `/head_pan/state` | `std_msgs/msg/String` | 동적 QoS; `pan_state_topic` |
| 토픽 구독 | `/reactive_guard/state` | `std_msgs/msg/String` | 동적 QoS; `reactive_guard_topic` |
| 토픽 구독 | `/body_guard_3d/state` | `std_msgs/msg/String` | 동적 QoS; `voxel_guard_topic` |
| 토픽 구독 | `/head_pan/coverage` | `std_msgs/msg/String` | 동적 QoS; `coverage_topic` |
| 토픽 구독 | `/behavior_tree_log` | `nav2_msgs/msg/BehaviorTreeLog` | 동적 QoS; `nav2_bt_topic` |
| 토픽 구독 | `/navigation/bt_trace` | `std_msgs/msg/String` | 동적 QoS; `nav2_bt_trace_topic` |
| 토픽 구독 | `/collision_monitor_state` | `discovery 타입` | 동적 QoS; `collision_state_topic` |
| 토픽 구독 | `<runtime_namespace>/mock/manipulation/execution_events` | `discovery 타입` | 동적 QoS; `manipulation_events_topic` |
| 토픽 구독 | `/evaluation` | `discovery 타입` | 선택 시 동적 구독; `controller_evaluation_topic` |
| 토픽 구독 | `/scan` | `sensor_msgs/msg/LaserScan` | 선택 시 동적 구독; `scan_topic` |
| 토픽 구독 | `/camera/head/depth/obstacle_points` | `sensor_msgs/msg/PointCloud2` | 선택 시 동적 구독; `depth_points_topic` |
| 토픽 구독 | `/body_guard_3d/risk_voxels` | `sensor_msgs/msg/PointCloud2` | 선택 시 동적 구독; `risk_points_topic` |
| 토픽 구독 | `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | 선택 시 동적 구독; `footprint_topic` |
| 토픽 구독 | `/safety/stop_zone` | `geometry_msgs/msg/PolygonStamped` | 선택 시 동적 구독; `stop_polygon_topic` |
| 토픽 구독 | `/safety/slow_zone` | `geometry_msgs/msg/PolygonStamped` | 선택 시 동적 구독; `slow_polygon_topic` |
| 토픽 구독 | `/local_plan` | `nav_msgs/msg/Path` | 선택 시 동적 구독; `local_path_topic` |
| 토픽 구독 | `/obstacle_memory/grid` | `nav_msgs/msg/OccupancyGrid` | 선택 시 동적 구독; `obstacle_memory_topic` |
| 토픽 구독 | `/obstacle_memory/lidar` | `nav_msgs/msg/OccupancyGrid` | 선택 시 동적 구독; `lidar_memory_topic` |
| 토픽 구독 | `/obstacle_memory/depth` | `nav_msgs/msg/OccupancyGrid` | 선택 시 동적 구독; `depth_memory_topic` |
| 토픽 구독 | `/submap_list` | `cartographer_ros_msgs/msg/SubmapList` | 동적 QoS; `slam_submaps_topic` |
| 토픽 구독 | `/trajectory_node_list` | `visualization_msgs/msg/MarkerArray` | 동적 QoS; `slam_trajectory_topic` |
| 토픽 구독 | `/constraint_list` | `visualization_msgs/msg/MarkerArray` | 동적 QoS; `slam_constraints_topic` |
| 토픽 구독 | `/scan_matched_points2` | `sensor_msgs/msg/PointCloud2` | 동적 QoS; `slam_matched_points_topic` |
| 서비스 요청 | `/submap_query` | `cartographer_ros_msgs/srv/SubmapQuery` | allowlist된 읽기 전용 요청; `slam_submap_query_service` |
| 토픽 구독 | `<action>/_action/status` | `action_msgs/msg/GoalStatusArray` | Discovery된 최대 32개 status/feedback 토픽 내에서 선택 |
| 토픽 구독 | `<action>/_action/feedback` | `<action package>/action/<Action>_FeedbackMessage` | Discovery 타입과 publisher QoS |
| 토픽 구독 | `<사용자 선택 / camera 토픽>` | `discovery 타입` | 설정과 웹 선택에 따라 구독 생성 / 해제 |

응용 ROS publisher, service server와 action server/client는 없다. SubmapQuery client는 서브맵을 읽고, 녹화 제어 client는 독립 logger에 요청한다. Discovery에서 해당 이름과 타입이 확인돼야 실제 구독을 생성한다. 표의 타입은 기본 입력의 기대 타입이며 구독은 discovery 타입을 사용한다. `runtime_namespace`가 비어 있으면 prefix는 없다. TF listener 구독은 별도로 생성된다.

동적 QoS는 모든 publisher와 호환되는 reliability/durability를 선택한다. Depth는 image 1, runtime_events 2048, manipulation execution events 100, 그 밖의 토픽 5다. 구독의 존재와 발행 주기는 보장하지 않는다.

HTTP/WebSocket은 `host=127.0.0.1`, `port=8768`이 기본값이다. 주요 경로는 `/api/snapshot`, `/api/camera`, `/api/recordings`, `/api/recordings/{session}/replay`, `/ws`이며 전체 route는 [server.py](../cleany_dev_monitor/server.py)를 따른다. 기록 설정 API는 독립 logger의 서비스를 호출하며 로봇 구동 명령은 보내지 않는다. [관측 데이터 계약](data.md)을 함께 확인한다.

## Logger 연결

`logger_namespace=/cleany_logger` 아래 `start`, `stop`, `configure`, `get_config`에
`cleany_interfaces/srv/RecordingControl` client를 생성한다. `status`는
`cleany_interfaces/msg/RecordingStatus`, Reliable, Transient Local, depth 1로
구독한다. `logger_timeout_s=3.0`은 상태 유효시간과 서비스 응답 제한의 기본값이다.
서비스 의미와 단위는 [Logger 인터페이스](../../cleany_logger/docs/interfaces.md)를 따른다.

Monitor와 logger의 `state_directory`는 같은 호스트의 같은 디렉터리여야 한다.
HTTP `PUT /api/recording/settings`는 정지 상태에서 설정하거나 활성 녹화를 정지한다.
`POST /api/recording/session`은 정지 후 새 세션을 시작하고, 이미 녹화 중이면 같은
세션을 반환한다. Monitor 종료는 logger 서비스 호출 없이 자체 HTTP와 구독만 닫는다.

## 승인 기반 작업 제안 (Task Proposals)

웹 대시보드의 인증된 운영자 세션에서 호출되는 작업 제안 및 승인 관련 ROS 2 인터페이스입니다.
`cleany_dev_monitor_operator_<pid>_<id>` 어댑터 노드가 클라이언트로 동작하며, 제안 및 재관찰 단계는 순수 인식(`InspectScene`)만 수행하고 로봇 모터는 구동하지 않습니다.

### 1. ROS 2 인터페이스 목록

| 종류 / 방향 | 기본 토픽 및 서비스 이름 | ROS 타입 | 설명 |
|---|---|---|---|
| **Action Client** | `/perception/inspect_scene` | `cleany_interfaces/action/InspectScene` | 동일한 `tracking_session_id`로 초기 제안 생성 및 승인 시 동일성 재관찰 수행 |
| **Service Client** | `/perception/get_scene_snapshot` | `cleany_interfaces/srv/GetSceneSnapshot` | 캐시된 스냅샷 유효성 확인 |
| **토픽 구독** | `/perception/debug_image_latched` | `sensor_msgs/msg/Image` (Reliable, Transient Local 1) | 추천 대상 물체가 오버레이된 제안 확인용 디버그 이미지 |
| **Service Client** | `/manipulation/control_state` | `std_srvs/srv/Trigger` | 실행 전후 하드웨어 제어권 및 비상 정지 상태 조회 |
| **Action Client** | `/manipulation/execute_skill` | `cleany_interfaces/action/ExecuteManipulationSkill` | 승인된 물체에 대해 단 1회 조작 액션 실행 (Execution ID = 승인 작업 UUID) |

### 2. HTTP 제어 API

- `POST /api/control/jobs`:
  - `action="propose"`: 새 장면에서 최적의 대상 물체 제안 요청
  - `action="approve", args={"proposal_id": "<UUID>"}`: 추천된 제안 승인 및 1회 실행
  - `action="skip", args={"proposal_id": "<UUID>"}`: 현재 추천된 대상을 제외하고 새 제안 요청
- `GET /api/control/status`: 현재 활성 제안(`proposal`) 및 작업 진행 단계(`phase`) 조회
- `GET /api/control/proposals/{id}/image`: 촬영 시점과 일치하는 실시간 관측 JPEG 이미지 반환

제안의 `target`은 종류명, 처리 분류, 검출 신뢰도와 위치를 포함합니다.
Gemini 분류 사유는 생성, 저장하거나 표시하지 않습니다. 제안의 `reason`은
대상 선정 규칙이나 실행 불가 상태를 설명하는 로컬 메시지입니다.

### 3. 주요 파라미터 및 승인 유효성 기준

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `proposal_max_age_sec` | 90.0초 | 촬영 후 승인 재확인 완료까지의 최대 허용 시간 |
| `proposal_position_tolerance_m` | 0.015m (1.5cm) | 승인 대상 물체의 최대 허용 위치 변동량 |
| `proposal_minimum_bbox_iou` | 0.7 | 승인 대상 물체 바운딩 박스의 최소 겹침 비율 (IoU) |
| `proposal_inspection_timeout_sec` | 180.0초 | 인식 노드 응답 대기 제한 시간 |
| `proposal_execution_timeout_sec` | 1200.0초 | 조작 액션 실행 대기 제한 시간 |

- **제안 상태 전이**: `READY` (승인 대기) → `CONSUMED` (승인 완료 또는 만료)
- **작업 진행 단계**: `PROPOSING` → `REVALIDATING` (동일성 재검증) → `EXECUTING` (동작 수행) → `PROPOSING_NEXT` (다음 작업 대기)
