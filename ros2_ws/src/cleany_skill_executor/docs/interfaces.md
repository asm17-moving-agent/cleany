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

`visibility_sensor_link`가 설정되면 optical 원점을 해당 카메라 하우징 링크의
고정 TF로 표현한다. MoveIt은 시야 원뿔 검사에서 센서 자체의 하우징만 제외한다.
동적 mount TF는 거부하며, 다른 로봇 링크의 가림과 일반 물리 충돌 검사는 유지한다.
기본 빈 문자열은 기존 base_link 표현을 유지한다. 전체 머리 모델을 사용하는
Isaac 설정은 실제 카메라 충돌 형상을 소유한 `head_tilt_link`를 지정한다.
열림 위치가 설정된 sorting 모드의 접근 자세 시야 검사는 현재 그리퍼 값 대신
실제 재인식 시점의 열림 위치를 사용한다. 팔 궤적에는 그리퍼 관절을 추가하지 않는다.

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

`cartesian_orientation_weight_m`는 local Cartesian refinement에서 목표 SLERP 회전 행렬의 잔차에 적용하는 미터 단위 가중치다. 기본값 0은 기존 위치 중심 보간을 유지하고 Isaac cleanup은 0.002를 사용한다. 이 값은 계획 목적 함수만 바꾸며, 실행 전 1mm 위치 corridor, 5도 자세 검사와 MoveIt 충돌 검사를 완화하지 않는다.

`sorting_lift_observation_source`는 `auto`(기존 동작) 또는 `head_rgbd`다. 후자는 손목 HANDOFF를 사용하는 경우에도 리프트 확인만 기존 head RGB-D 재검출/높이/held geometry 연관 검사로 수행한다. 카메라 전환 전과 확인 후 새 연속 gripper feedback을 요구한다. 시각 확인 실패를 motor-only 성공으로 바꾸지 않으며, 재관측 자세 이동은 기존 `sorting_reobserve_after_lift` 설정을 따른다.

Head 재관측 후보는 기존 중앙 행으로 유효 후보를 얻지 못할 때 화면 위/아래 행도 검사한다. 전체 관측 bounding sphere의 inset frustum 포함, 리프트 높이, 최대 이동량은 그대로 적용하며, 추가 후보는 짧은 이동 순으로 정렬한다. 이 기하 후보도 기존 IK와 MoveIt 충돌 검사, 이동 후 RGB-D 검증을 통과해야 한다.

`sorting_reobserve_fixed_wrist_pitch`(기본 false)를 켜면 재관측 후보를 `sorting_reobserve_wrist_pitch_rad`의 손목 pitch로 계산한다. head 카메라에서 손목 뒤로 가려지는 물체의 관측 자세를 설정하는 옵션이다. 기존 carry roll 범위 안에서 관측 물체 중심의 ±1mm 영역 IK와 충돌 검사를 수행하며, 관절 한계 밖이거나 도달할 수 없으면 후보를 거부한다. 자세 변경 자체는 수거 확인 근거가 아니며 이동 후 RGB-D 재검출과 접촉 유지 검사가 그대로 필요하다.
`sorting_reobserve_wrist_roll_offset_rad`(기본 0)은 fixed-pitch 재관측에서 파지 시 roll 기준에 대한 추가 회전이다. 비영 값은 fixed joint IK 조건으로 적용하며 현재 설정된 carry roll 범위를 넘으면 거부한다. 관측 옵션이 carry 범위를 자동 확장하지 않는다.
`sorting_reobserve_max_raising_m`(기본 0)은 원래 높이와 낮춘 높이 후보에 추가하는 최대 상승량이다. 양수이면 절반/전체 상승 높이도 검사한다. 전체 이동량 제한과 물체 전체의 화면 포함 검사는 그대로 적용한다. 낮은 리프트 높이에서는 불가능한 카메라 관측 자세를 탐색할 때 사용하며, 추가 상승도 실제 물체 높이 확인을 대체하지 않는다.

`sorting_artifact_directory`를 설정하면 Cartesian 경로뿐 아니라 MoveIt joint plan-and-execute의 Goal과 Result도 `joint_motion_request`/`joint_motion_result` CDR로 보존한다. 환경 갱신으로 실행이 중단된 경우에도 반환된 계획/실행 경로와 오류 코드를 재생 진단에 사용할 수 있다.

`attachment_uncertainty_padding_m`(기본 0, 허용 0–0.05m)은 접촉 후 MoveIt attachment에 추가하는 관측 OBB의 면당 여유다. 양수이면 원래 관측 mesh를 유지하면서 OBB 각 변을 두 배 여유만큼 확장한 box를 추가한다. 접근 전 형상과 허용 접촉 링크는 유지한다. 이 추가 형상을 운반 충돌 검사와 self-mask가 함께 사용하므로 지도에서만 물체 주변을 비우지 않는다. 관측/접촉 위치 불확실성에 대한 설정 가정이며, 새 센서 관측이나 실제 물체 이동을 뜻하지 않는다. 별도의 파지/시각/안착 검사는 그대로 필요하다.


Head RGB-D lift association uses the bounding center of the selected mask's measured `target_cloud`, with the same capture stamp and frame as `objects`. The desk-supported OBB extends down to the support plane and is not an airborne-object center. The original perception result remains unchanged. The existing association tolerance and observed minimum lift height apply to the measured cloud center; missing or mismatched cloud evidence fails closed. Tabletop grasp planning continues to use its existing support-plane geometry.

Head lift verification requests `InspectScene.Goal.free_space=true` only for its selected-object reconstruction. This preserves the default support-plane path for tabletop grasp planning and avoids requiring a visible table beneath an airborne object.

`sorting_reobserve_restore_pose` defaults false. When enabled, a successful optional head reobservation is followed by a MoveIt return to the measured pre-reobservation arm joints, before fixed-wrist bin transport. Failed visual association never triggers this return. Path constraints, collision checks, joint feedback, sustained gripper contact and minimum kinematic lift clearance remain required. The returned inspection retains its original capture stamp and pose; it is not relabeled as a post-return image.

`sorting_release_gripper_position_rad` defaults to the configured `gripper_open_position_rad`. It may specify a smaller opening at a receptacle without changing the grasp approach opening. Before bin transport, both cached and new fixed-release IK solutions must pass the existing full-robot gripper sweep check from measured contact position to this opening. The sweep is checked again from fresh joint feedback immediately before the opening command. `sorting_release_gripper_sweep_step_rad` defaults0.05rad and uses the adapter's0.005–0.2rad bounds. Opening must exceed the measured gripper position and satisfy the original tool limits. Failed checks prevent motion; collision permissions are unchanged. Actual independent placement confirmation is still required after the opening.
