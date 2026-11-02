# cleany_mission_manager 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## mission_runtime

실행 파일: `mission_runtime`. 원본: [node.py](../cleany_mission_manager/node.py), [nav2.py](../cleany_mission_manager/adapters/nav2.py), [manipulation.py](../cleany_mission_manager/adapters/manipulation.py), [supervision.py](../cleany_mission_manager/adapters/supervision.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 발행 | `mission/status` | `cleany_interfaces/msg/MissionStatus` | R/T1; 상태 sequence 변경 시; `tick_hz` 기본 10 Hz |
| 토픽 발행 | `mission/result` | `cleany_interfaces/msg/MissionResult` | R/T1; 새 최종 결과 |
| 토픽 발행 | `mission/debug_snapshot` | `cleany_interfaces/msg/RuntimeDebugSnapshot` | R/T1; 기본 2 Hz |
| 토픽 발행 | `mission/runtime_events` | `cleany_interfaces/msg/RuntimeEvent` | R/T2048; 이벤트 발생 시 |
| 토픽 구독 | `robot/safety_fault` | `std_msgs/msg/String` | R10; `supervision_mode=disabled`만 |
| 토픽 구독 | `robot/safety_released` | `std_msgs/msg/String` | R10; `supervision_mode=disabled`만 |
| 액션 제공 | `mission/stop` | `cleany_interfaces/action/StopMission` | Action 기본값; `supervision_mode=required`, `stop_action` |
| 서비스 호출 | `system/check_permission` | `cleany_interfaces/srv/CheckPermission` | Service 기본값; required; 갱신 TTL/3 |
| 토픽 구독 | `system/heartbeat` | `cleany_interfaces/msg/SystemHeartbeat` | R1; required |
| 토픽 구독 | `system/state` | `cleany_interfaces/msg/SystemState` | R/T1; required |
| 토픽 구독 | `system/capabilities` | `cleany_interfaces/msg/SystemCapabilities` | R/T1; required |
| 서비스 제공 | `mission/offer` | `cleany_interfaces/srv/OfferMission` | Service 기본값 |
| 서비스 제공 | `mission/cancel` | `cleany_interfaces/srv/CancelMission` | Service 기본값 |
| 서비스 제공 | `mission/snapshot` | `cleany_interfaces/srv/GetRuntimeSnapshot` | Service 기본값 |
| 서비스 제공 | `mission/reset_error` | `std_srvs/srv/Trigger` | Service 기본값 |
| 액션 호출 | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Action 기본값; `navigation_backend=nav2` |
| 서비스 호출 | `navigate_to_pose/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | Service 기본값; Nav2 adapter |
| 서비스 호출 | `<lifecycle_node>/get_state` | `lifecycle_msgs/srv/GetState` | Service 기본값; `required_navigation_lifecycle_nodes`; `supervision_mode=disabled`만 (감독 시 Supervisor가 확인) |
| 토픽 구독 | `odom` | `nav_msgs/msg/Odometry` | Sensor; `odom_topic`, Nav2 adapter |
| 토픽 구독 | `navigate_to_pose/_action/status` | `action_msgs/msg/GoalStatusArray` | Action status 기본 QoS; Nav2 adapter |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 액션 호출 | `mock/manipulation/execute_skill` | `cleany_interfaces/action/ExecuteManipulationSkill` | Action 기본값; `manipulation_action` |
| 서비스 호출 | `mock/manipulation/execute_skill/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | Service 기본값; `manipulation_action` + suffix |
| 서비스 호출 | `mock/manipulation/get_execution` | `cleany_interfaces/srv/GetManipulationExecution` | Service 기본값; `manipulation_query` |

Nav2 adapter의 endpoint와 TF 구독은 `navigation_backend=nav2`에서 생성한다. 조작 endpoint는 `manipulation_backend=action_mock`일 때만 생성하며 기본 `mock`는 process 내부 실행이다. 기본 lifecycle 목록은 `map_server`, `amcl`, `planner_server`, `controller_server`, `bt_navigator`, `collision_monitor`다. `supervision_mode` 기본값은 `required`이며 `supervision_capabilities`(기본 `[NAVIGATION]`) 하나의 lease를 미션마다 요청한다. `disabled`는 mock과 simulation 단독 실행용이고 Runtime snapshot의 `supervision`에 기록된다. [Mission 계약](../../cleany_interfaces/docs/mission.md)을 따른다.

## 책상 정밀 접근을 사용하는 navigation port

`desk_approach_enabled=true`이면 좌석 이동의 Nav2 성공, 요청 pose 검증과 정지 확인 뒤
`desk/approach` (`cleany_interfaces/action/DeskApproach`, Action 기본 QoS)를 호출한다.
목표 `desk_distance_m`는 base 원점에서 평면까지의 거리다. 기존 Supervisor lease의
식별자를 전달하며 접근 중 lease 상실과 `mission/stop`도 같은 navigation port의 취소로
연결한다. home에는 호출하지 않는다. 후속 책상 작업은 접근 성공과 신선한 정지 확인
이후에만 시작한다. journal의 `desk_goal_uuid`는 재시작 시 정확한 goal의 취소에 사용하며,
취소 수락만으로 정지가 확인된 것으로 처리하지 않는다.
