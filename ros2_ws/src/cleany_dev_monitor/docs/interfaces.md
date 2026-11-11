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

관측 노드에는 응용 ROS publisher, service server와 action client/server가 없다. 선택적 운영 제어는 아래 별도 adapter가 담당한다. SubmapQuery client는 서브맵을 읽고, 녹화 제어 client는 독립 logger에 요청한다. Discovery에서 해당 이름과 타입이 확인돼야 실제 구독을 생성한다. 표의 타입은 기본 입력의 기대 타입이며 구독은 discovery 타입을 사용한다. `runtime_namespace`가 비어 있으면 prefix는 없다. TF listener 구독은 별도로 생성된다.

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

## 운영 제어와 상태

`control_enabled=false`가 기본값이다. `control_enabled=true`와
`control_directory/enabled` 파일이 모두 있어야 변경 요청을 허용한다.
`control_directory`는 기본 `~/.local/state/cleany/dev-monitor/operator`이며 CLI는
`XDG_STATE_HOME`을 따른다. `control_repository_root`는 기존 실행 도구의 저장소 경로다.
제어 노드는 프로세스별 고유 이름을 사용하고 관측 collector와 분리한다.

| 경로 또는 호출 | 계약 |
|---|---|
| `POST /api/control/login`, `logout` | 이름과 0600 조작키로 단일 세션 인증. 기본 3600초(`control_session_seconds`), 명시적 제어권 가져오기, HttpOnly/SameSite 쿠키와 동일 origin 검사 |
| `GET /api/control/status` | 현재 실행기 상태, 허용 동작과 제한 사유, 작업 이력과 제안 |
| `POST /api/control/jobs` | `request_id`, 허용된 `action`, 엄격한 `args`만 수락. 같은 UUID는 기존 작업 반환 |
| `POST /api/control/cancel` | 활성 execution ID의 중단 또는 Action 이전 작업의 취소. 접수는 실물 정지 증거가 아님 |
| `POST /api/control/jobs/{id}/acknowledge` | 사라진 작업 소유자의 불명확한 결과를 확인한 메모. 실행기 차단을 해제하지 않음 |
| `/manipulation/cancel` | `CancelManipulation` Client, 정확한 실행 ID와 중단 방식 |
| `/manipulation/confirm_recovery` | `ConfirmManipulationRecovery` Client, 검토한 execution ID/revision과 정지, 빈 그리퍼 확인 |

허용 동작은 준비, 인식, 직접 실행, 제안, 승인, 건너뛰기, 팔 복귀, 복구와 종료다.
토크, 그리퍼 개방과 복구에는 명시 확인이 필요하다. 준비는 기존 runtime start와
Action server 시작, 직접 실행은 `make manipulation-run`, 복귀와 종료는 기존 CLI를
호출한다. 임의 명령을 받지 않으며 guard, 실행 소유권과 차단 검사를 유지한다.

웹 작업은 별도 systemd 사용자 scope에서 실행한다. 브라우저나 모니터 종료만으로
로봇 작업을 종료하지 않는다. 웹과 CLI는 제어 활성화 중 공통 `operation.lock`을
사용하며 시작 또는 진행 중 작업이 있으면 제어 토글을 거절한다.
`jobs/<UUID>.json`과 `.log`에 요청, 운영자와 결과를 보존한다. PID와 Linux 시작 tick으로
소유자를 확인하며 사라진 작업은 결과 확인 전까지 새 실행을 막는다.
응답 유실 시 같은 UUID만 재사용한다. 복구 revision은 확인창에서 고정하고 서버가
다시 비교한다. 로그 요약은 오류 설명이며 실행 허가나 실제 정지 판단에 쓰지 않는다.

`key`와 허용된 운영 옵션의 `runtime-settings.json`은 0600으로 저장한다.
API key 값은 웹에 내보내지 않는다. 화면에는 최근 5개 명령을 표시하고 원본은 보존한다.

Jetson 프로필은 `/manipulation/execution_events`의 서버 기록을 사용한다.
동일 기록의 heartbeat는 생존 시각만 갱신하며 revision과 완료 단계는 바꾸지 않는다.
연결 또는 heartbeat가 오래되면 진행 상태와 동작 허가를 표시하지 않는다.
준비 단계는 `manipulation_request_file`에서 읽으며, 기본 파일은
`~/.local/state/cleany/manipulation/request.json`이다. CLI와 함께
`CLEANY_MANIPULATION_PROGRESS_FILE`로 위치를 바꿀 수 있다. 준비 파일은 실시간 관측용이며
Logger 녹화에 자동 포함하지 않는다. 요청의 execution ID가 일치하기 전에는 이전
작업 결과를 현재 결과로 표시하지 않는다. `ASSUMED`와 실측 `CONFIRMED`를 구분한다.

카메라 미리보기는 지원 JPEG를 재인코딩하지 않고 사용한다. 손목 raw 영상은 가능한 경우
5 Hz preview/compressed, 헤드 컬러는 image_raw/compressed를 우선 사용한다.
원본 깊이와 compressedDepth는 미리보기에서 제외한다. 선택은 set_enabled를 호출하지 않는다.
`X-Camera-Topic`은 실제 구독 토픽이다. 리소스 수집은 Logger의 공통 sampler를 사용하며
Linux에서 PID별 stat 한 번으로 CPU, 시작 tick과 RSS를 읽고 PID 재사용 시 기준을 초기화한다.

## 승인 기반 작업 제안

인증된 제어 경로의 `cleany_dev_monitor_operator_<pid>_<id>` adapter가 요청별 작업을
수행한다. 제안은 InspectScene, 승인 재확인은 RevalidateTarget을 사용하며 팔 동작을 요청하지 않는다.
승인 작업은 기존 `manipulation_test_client`로 정확한 Goal을 한 번 전달한다.
재관찰은 승인한 대상의 동일성, 분류, 위치, bbox, 깊이와 크기 조건을 검증한다.
주변 검출 목록의 일치는 승인 조건이 아니며, `scene_context`와 `scene_object_count`는
제안 당시 관측 기록으로 유지한다. 경로와 충돌 검사는 승인 이후 실행기가 수행한다.

| 종류 / 방향 | 기본 이름 | 타입과 조건 |
|---|---|---|
| Action Client | `/perception/inspect_scene` | `cleany_interfaces/action/InspectScene`, 같은 `tracking_session_id`로 새 작업 제안과 분류 |
| 서비스 Client | `/perception/revalidate_target` | `cleany_interfaces/srv/RevalidateTarget`, 승인 대상의 최신 기하와 원본 분류 반환 |
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

제안의 `target`은 종류명, 처리 분류, 검출 신뢰도와 위치를 포함합니다.
재확인 응답은 승인 관측의 분류 사유(`sorting_reason`)도 유지합니다. 제안의 `reason`은
대상 선정 규칙이나 실행 불가 상태를 설명하는 로컬 메시지입니다.

승인 시 원본 snapshot, object ID, track ID와 session을 재확인 서비스에 전달합니다.
서비스는 Gemini를 호출하지 않고 승인한 종류명과 처리 분류를 해당 1회 작업에 유지합니다.
최신 신뢰도, 깊이, 크기와 목적지 정책은 다시 검사합니다. 위치, 박스, 좌표계,
설정, 원본 캐시 및 유효 기간 검사는 유지하며 실행 명령은 원래 승인한 snapshot/object를 사용합니다.
원본 분류의 정확성을 새로 검증하는 경로는 아니며 다음 제안은 다시 분류합니다.

마스크 대응 또는 RGB-D 수신 실패만 제한된 횟수로 새 영상을 요청합니다.
이동, 낮은 박스 겹침, 깊이, 정책 또는 설정 실패에는 재시도하지 않습니다.
`control_directory/revalidations/<proposal_id>.json`에 승인 대상, 각 응답의
촬영 시각, 위치, 박스, 신뢰도, 크기, 실패 원인과 종료 시각을 저장합니다.
오류 메시지는 이동 거리(mm), 박스 겹침 또는 후보 식별 실패를 구분합니다.

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `proposal_max_age_sec` | 90.0 | 촬영 후 승인 재확인 완료까지의 유효 시간; 인식 cache TTL도 별도 적용 |
| `proposal_position_tolerance_m` | 0.015 | 같은 좌표계에서 승인 대상의 최대 위치 차이 |
| `proposal_minimum_bbox_iou` | 0.7 | 승인 대상 bbox의 최소 겹침 비율 |
| `proposal_inspection_timeout_sec` | 180.0 | 관측 응답 대기 상한 |
| `proposal_execution_timeout_sec` | 1200.0 | 기존 CLI가 Action 결과를 기다리는 상한 |
| `proposal_reobservation_attempts` | 2 | 새 영상 재확인 최대 시도 수, 1~3의 정수 |

제안 상태는 `READY`, `NO_TARGET`, 조회 시 파생되는 `EXPIRED`, `CONSUMED`다. 실제
실행 결과와 독립적이며 `NO_TARGET`은 전체 작업 완료가 아니다. 진행 단계는
`PROPOSING`, `REVALIDATING`, `EXECUTING`, `PROPOSING_NEXT`다. 승인 작업 실패 후에도
소비된 제안은 재사용하지 않는다. `propose`, 별도 인식, 준비, 직접 실행, 복귀, 복구,
종료 요청은 기존 제안을 무효화한다. 모니터 재시작은 승인 대기 시간을 연장하지 않는다.
