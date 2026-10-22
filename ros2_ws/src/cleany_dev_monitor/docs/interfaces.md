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
| 토픽 구독 | `<사용자 선택 / camera / recording 토픽>` | `discovery 타입` | 설정과 웹 선택에 따라 구독 생성 / 해제 |

응용 ROS publisher, service server와 action server/client는 없다. SubmapQuery service client만 서브맵을 읽는다. Discovery에서 해당 이름과 타입이 확인돼야 실제 구독을 생성한다. 표의 타입은 기본 입력의 기대 타입이며 구독은 discovery 타입을 사용한다. `runtime_namespace`가 비어 있으면 prefix는 없다. TF listener 구독은 별도로 생성된다.

동적 QoS는 모든 publisher와 호환되는 reliability/durability를 선택한다. Depth는 image 1, runtime_events 2048, manipulation execution events 100, 그 밖의 토픽 5다. 구독의 존재와 발행 주기는 보장하지 않는다.

HTTP/WebSocket은 `host=127.0.0.1`, `port=8768`이 기본값이다. 주요 경로는 `/api/snapshot`, `/api/camera`, `/api/recordings`, `/api/recordings/{session}/replay`, `/ws`이며 전체 route는 [server.py](../cleany_dev_monitor/server.py)를 따른다. 기록 설정 API는 Monitor 내부 기록을 변경하며 로봇 구동 명령은 보내지 않는다. [관측 데이터 계약](data.md)을 함께 확인한다.

## 승인 기반 작업 제안

인증된 제어 경로의 `cleany_dev_monitor_operator_<pid>_<id>` adapter가 요청별 작업을
수행한다. 제안과 재관찰은 detector-only InspectScene이며 팔 동작을 요청하지 않는다.
승인 작업은 기존 `manipulation_test_client`로 정확한 Goal을 한 번 전달한다.
재관찰은 승인한 대상의 동일성, 분류, 위치, bbox, 깊이와 크기 조건을 검증한다.
주변 검출 목록의 일치는 승인 조건이 아니며, `scene_context`와 `scene_object_count`는
제안 당시 관측 기록으로 유지한다. 경로와 충돌 검사는 승인 이후 실행기가 수행한다.

| 종류 / 방향 | 기본 이름 | 타입과 조건 |
|---|---|---|
| Action Client | `/perception/inspect_scene` | `cleany_interfaces/action/InspectScene`, 같은 `tracking_session_id`로 제안과 승인 시 재관찰 |
| 서비스 Client | `/perception/get_scene_snapshot` | `cleany_interfaces/srv/GetSceneSnapshot`, 승인 관측의 잔존 여부 확인 |
| 토픽 구독 | `/perception/debug_image_latched` | `sensor_msgs/msg/Image`, reliable/transient_local/depth 1; `proposal_debug_image_topic` |
| 서비스 Client | `/manipulation/control_state` | `std_srvs/srv/Trigger`, 실행 전후 현재 소유권과 차단 조회 |
| Action Client, 기존 CLI | `/manipulation/execute_skill` | `cleany_interfaces/action/ExecuteManipulationSkill`, 승인된 snapshot/object/skill/destination; execution ID는 승인 작업 UUID |

`POST /api/control/jobs`의 `action=propose`는 빈 `args`로 제안한다. `approve`, `skip`은
`args={"proposal_id":"UUID"}`만 허용한다. 대상과 동작은 서버의 불변 제안에서 읽는다.
동일 request UUID 재전송은 이전 작업을 반환하고, 다른 UUID로 같은 제안을 재사용하면
거절한다. 별도 shell 인자나 임의 Goal은 받지 않는다. `GET /api/control/status`는
현재 `proposal`과 진행 작업의 `phase`를 반환한다. `GET /api/control/proposals/{id}/image`는
현재 인증 세션에 촬영 시각과 일치하는 JPEG만 반환하며 캐시하지 않는다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `proposal_max_age_sec` | 90.0 | 촬영 후 승인 재확인 완료까지의 유효 시간; 인식 cache TTL도 별도 적용 |
| `proposal_position_tolerance_m` | 0.015 | 같은 좌표계에서 승인 대상의 최대 위치 차이 |
| `proposal_minimum_bbox_iou` | 0.7 | 승인 대상 bbox의 최소 겹침 비율 |
| `proposal_inspection_timeout_sec` | 180.0 | 각 detector-only 관찰 대기 상한 |
| `proposal_execution_timeout_sec` | 1200.0 | 기존 CLI가 Action 결과를 기다리는 상한 |

제안 상태는 `READY`, `NO_TARGET`, 조회 시 파생되는 `EXPIRED`, `CONSUMED`다. 실제
실행 결과와 독립적이며 `NO_TARGET`은 전체 작업 완료가 아니다. 진행 단계는
`PROPOSING`, `REVALIDATING`, `EXECUTING`, `PROPOSING_NEXT`다. 승인 작업 실패 후에도
소비된 제안은 재사용하지 않는다. `propose`, 별도 인식, 준비, 직접 실행, 복귀, 복구,
종료 요청은 기존 제안을 무효화한다. 모니터 재시작은 승인 대기 시간을 연장하지 않는다.
