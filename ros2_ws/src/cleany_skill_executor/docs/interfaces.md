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

실행 파일은 `physical_grasp_selection_server`이며 ROS node 이름은 `grasp_selection_server`다. 기존 선택 Action과 메시지 계약을 사용하되 [실물 경로](physical-manipulation.md)로 명시 선택한다. 기존 서버와 같은 namespace에서 동시에 사용하지 않는다.

| 종류 | 기본 이름 | ROS 타입 | 조건 |
|---|---|---|---|
| 서비스 제공 | `/grasp/warmup_status` | `std_srvs/Trigger` | IK 예열 상태 조회 |
| 서비스 제공 | `/grasp/vlm_warmup` | `std_srvs/SetBool` | 활성화된 hybrid profile의 제한 시간 내 예열 요청 |

예열 실패는 파지 성공을 의미하지 않는다. 유효 상태, 경로와 실제 실행은 각각 검사한다.


## 실물 준비, 검증과 증거 노드

| 실행 파일 | 기본 ROS 통신 | 역할 |
|---|---|---|
| `validate_pick_server` | `grasp/validate_pick` (`ValidatePick`) 제공 | 촬영, 후보 생성과 도달성의 plan-only 검증 |
| 같은 서버 | `perception/inspect_scene` (`InspectScene`), `grasp/plan` (`PlanGrasp`), `grasp/select_reachable` (`SelectReachableGrasp`) 호출 | 제한 시간과 취소를 downstream에 전달 |
| 같은 서버 | `/perception/observe_object_reference` (`ObserveObjectReference`) 호출 | 선택적으로 대상 reference 고정과 정리 |
| 같은 서버 | `grasp/validate_pick_debug` (`sensor_msgs/Image`) 발행 | reliable/transient-local depth 1, 검증 overlay |
| `manipulation_evidence` | `/manipulation/check_evidence` (`CheckManipulation`) 제공 | motor, wrist tracking, guard와 outcome의 신선한 증거 결합 |
| 같은 노드 | `/joint_state_broadcaster/dynamic_joint_states`, `/perception/wrist_tracking_status`, `/real_control/status`, `/perception/manipulation_outcome` 구독 | 관측만 결합하고 성공을 가정하지 않음 |
| `pregrasp_pipeline`, `hardware_pregrasp`, `collection_pipeline` | `/grasp/validate_pick`, `/joint_states`, `/robot_description` | 준비와 검증 입력; 실행 모드는 native controller Action을 명시적으로 사용 |
| `pregrasp_sim_ready` | `/joint_states`, head와 양손목 `camera_info`, TF, `/controller_manager/list_controllers` | 시뮬레이션 입력의 신선도와 controller 준비 대기 |

실물 trajectory는 `/{arm}_{arm,gripper}_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectory`)로 보낸다. 실행기는 검증 Action과 센서 관측의 성공, 모터 command 수락과 물리 도착을 구분한다. 외부 limits와 contact 판단 조건은 [physical-manipulation](physical-manipulation.md)을 따른다.
