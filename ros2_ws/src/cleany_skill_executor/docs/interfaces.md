# cleany_skill_executor 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## grasp_selection_server

실행 파일: `grasp_selection_server`. 원본: [grasp_selection_node.py](../cleany_skill_executor/grasp_selection_node.py), [moveit_adapter.py](../cleany_skill_executor/moveit_adapter.py), [planning_scene.py](../cleany_skill_executor/planning_scene.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | R20; `joint_state_topic` |
| 액션 제공 | `grasp/select_reachable` | `cleany_interfaces/action/SelectReachableGrasp` | Action 기본값 |
| 서비스 호출 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Service 기본값 |
| 서비스 호출 | `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | Service 기본값 |
| 서비스 호출 | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Service 기본값 |
| 서비스 호출 | `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Service 기본값 |
| 액션 호출 | `/move_action` | `moveit_msgs/action/MoveGroup` | Action 기본값; plan-only |

도달성 검사 action이며 controller 실행을 제공하지 않는다.

현재 조작 실행은 [nearest-pregrasp와 sorting 계약](manipulation.md)을 따른다. 이전 demo coordinator는 제거됐다.

## manipulation_server

실행 파일: `manipulation_server`. 원본: [manipulation_node.py](../cleany_skill_executor/manipulation_node.py). Mock launch는 `/mock` namespace를 적용한다.

| 종류 / 방향 | 기본 상대 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 액션 제공 | `manipulation/execute_skill` | `cleany_interfaces/action/ExecuteManipulationSkill` | Action 기본값 |
| 서비스 제공 | `manipulation/get_execution` | `cleany_interfaces/srv/GetManipulationExecution` | Service 기본값 |
| 서비스 제공 | `manipulation/cancel` | `cleany_interfaces/srv/CancelManipulation` | Service 기본값 |
| 토픽 발행 | `manipulation/execution_events` | `cleany_interfaces/msg/ManipulationExecutionRecord` | Reliable, transient-local depth 100; record 변경 시 |
| 서비스 호출 | `manipulation/execute_skill/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | 정확한 Goal의 cancel 응답 경로 |

Tick 기본 주기는 steady clock 0.05초다. 물리 명령 publisher는 없다. Mission은 취소 service의 별도 모드 대신 ROS Action cancel의 CHECKPOINT를 사용한다.

Action 이름을 remap하면 서버와 내부 CancelGoal client가 같은 해결된 이름을 사용한다. 예를 들어 `manipulation/execute_skill:=/custom/execute`에서는 내부 취소 요청도 `/custom/execute/_action/cancel_goal`로 전달한다.

수락, 기록과 취소의 의미는 [공통 Action 계약](../../cleany_interfaces/docs/observation-contracts.md#단일-물체-manipulation-action)을 따른다.

## manipulation_monitor

실행 파일: `manipulation_monitor`. 원본: [manipulation_monitor_node.py](../cleany_skill_executor/manipulation_monitor_node.py).
`event_topic` 기본값 `/mock/manipulation/execution_events`를 Reliable, transient-local,
depth 100으로 구독한다. `monitor_port`의 localhost ZeroMQ REP socket은 읽기 전용
트리 상태를 제공하며 ROS Action을 실행하거나 취소하지 않는다.

## manipulation_test_client

실행 파일: `manipulation_test_client`. 원본: [manipulation_client.py](../cleany_skill_executor/manipulation_client.py).
`namespace` 옵션으로 선택한 서버의 `manipulation/execute_skill` Action과
`manipulation/get_execution`, `manipulation/cancel` 서비스를 호출한다.
요청, 취소와 결과 확인 예시는 [실행 안내](manipulation_mock_usage.md)를 따른다.

## physical_grasp_selection_server

실물 전용 파지 후보 선택 서버입니다. (노드명: `grasp_selection_server`)
시뮬레이션용 서버와 동일한 액션 인터페이스(`grasp/select_reachable`)를 제공하되, GPU cuRobo IK 및 실물 씬 매핑 검증을 거칩니다.

| 종류 | 기본 이름 | ROS 타입 | 설명 |
|---|---|---|---|
| 서비스 제공 | `/grasp/warmup_status` | `std_srvs/srv/Trigger` | cuRobo IK 엔진 예열 상태 확인 |
| 서비스 제공 | `/grasp/vlm_warmup` | `std_srvs/srv/SetBool` | VLM 감지 시 제한 시간 내 백그라운드 예열 트리거 |

## 실물 준비, 검증 및 모니터링 노드

| 실행 파일 | 제공 및 호출 인터페이스 | 역할 및 동작 상세 |
|---|---|---|
| `validate_pick_server` | `grasp/validate_pick` (`cleany_interfaces/srv/ValidatePick`) [제공]<br/>`grasp/validate_pick_debug` (`sensor_msgs/msg/Image`) [발행] | 촬영, 파지 후보 생성 및 도달성 검증을 실제 모터 이동 없이 확인 (Plan-only) |
| 〃 | `perception/inspect_scene`, `grasp/plan`, `grasp/select_reachable` [호출] | 하위 인식 및 파지 서버로 요청을 위임하고 취소/타임아웃 전달 |
| `manipulation_evidence` | `/manipulation/check_evidence` (`CheckManipulation`) [제공] | 모터 텔레메트리, 손목 비전 추적, 안전 가드 상태를 종합하여 작업 성공 증거 판정 |
| `collection_pipeline`, `pregrasp_pipeline`, `hardware_pregrasp` | `/grasp/validate_pick`, `/joint_states`, `/robot_description` [구독] | 단계별 준비 상태 확인 및 네이티브 제어기 액션 호출 준비 |
| `pregrasp_sim_ready` | `/joint_states`, 카메라 `camera_info`, TF, `controller_manager/list_controllers` [구독] | 런타임 시작 전 센서 및 제어기의 준비 신선도 대기 |

실물 trajectory는 `/{arm}_{arm,gripper}_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectory`)로 보낸다. 실행기는 검증 Action과 센서 관측의 성공, 모터 command 수락과 물리 도착을 구분한다. 외부 limits와 contact 판단 조건은 [physical-manipulation](physical-manipulation.md)을 따른다.
