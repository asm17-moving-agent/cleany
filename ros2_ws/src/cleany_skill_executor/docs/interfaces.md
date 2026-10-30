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

실물 후보를 검증하고 준비 자세, 파지 자세의 시간 궤적을 반환한다. 실행 명령은 보내지 않는다.
기존 `grasp_selection_server`와 같은 Action 이름을 사용하므로 함께 실행하지 않는다.

| 방향 | 기본 이름 | 타입과 의미 |
|---|---|---|
| Action 제공 | `grasp/select_reachable` | `SelectReachableGrasp`; `action_name`으로 변경 |
| 서비스 제공 | `/grasp/warmup_status` | `std_srvs/srv/Trigger`; 예열 준비 상태 |
| 서비스 제공 | `/grasp/vlm_warmup` | `std_srvs/srv/SetBool`; VLM 대기 구간 알림 |
| 서비스 호출 | `/check_state_validity_batch` | `CheckStateValidityBatch`; 전체 상태의 배치 검사 |
| 구독 | `joint_states` | `sensor_msgs/msg/JointState`, depth 20; `joint_state_topic` 기본값 |
| 구독 | `/robot_description` | `std_msgs/msg/String`; transient-local |

MoveIt IK, FK, 장면 조회, 변경과 `/move_action`도 사용한다.
상세 실행 조건은 [실물 파지 계획](physical-pregrasp.md)을 따른다.

## validate_pick_server

`grasp/validate_pick` Action(`cleany_interfaces/action/ValidatePick`)은 인식, 후보 생성,
도달성 검사를 연결한다. 기본 downstream은 `perception/inspect_scene`, `grasp/plan`,
`grasp/select_reachable`이며 각각 이름 파라미터로 바꿀 수 있다.
`pin_object_reference=true`이면 `/perception/observe_object_reference`로 기준 물체를 등록한다.
`grasp/validate_pick_debug`는 `sensor_msgs/msg/Image`, reliable, transient-local depth 1이다.
Action 결과는 계획 근거이며 실물 수거 성공을 뜻하지 않는다.

## 준비 자세 실행 도구

`pregrasp_pipeline`은 기본 plan-only이며 `mujoco` 모드를 선택할 수 있다.
`hardware_pregrasp`는 `/grasp/validate_pick`, 기준 물체 관찰과 controller 상태 조회를
사용하고 `/{arm}_{arm|gripper}_controller/follow_joint_trajectory` Action으로 준비 자세를
실행한다. `/joint_states`와 `/robot_description`을 읽으며 controller를 자동 활성화하지 않는다.
`pregrasp_sim_ready`는 시뮬레이션 시작 상태를 확인하는 명시 실행 도구다.
