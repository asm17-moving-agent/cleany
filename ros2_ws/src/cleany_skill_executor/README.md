# cleany_skill_executor

### 임시 SAM2 tracking 중단 비교

시뮬레이션 sorting 실행에 `sam2_tracking_enabled:=false`를 전달하면 손목 사용,
연속 추적, reference 관측, 비동기 carry 감시의 기본값을 함께 false로 설정한다.
개별 고급 인수는 명시적으로 재정의할 수 있다. 이 설정은 손목 스트리밍과
head reference batch tracking을 모두 끈다. 초기 SAM2 단일 이미지 분할은
유지하며 파지 후 확인은 기존 head 재검출/검사 경로를 사용한다. 비동기 이동 중
시각 감시는 중단되므로 실로봇 운용 설정이 아닌 시뮬레이션 비교 시험용이다.
GUI에서도 실행할 수 있으나 headless 결과와 비교할 때 GUI 부하 차이를 고려한다.
재검출 비용과 카메라 전환 차이도 포함하므로 순수 tracking 비용만의 A/B는 아니다.
기본 tracking 설정은 변경하지 않는다.

```bash
make sim-mujoco-sorting SORTING_ARGS='sam2_tracking_enabled:=false sorting_test_only_label:="lego brick"'
```

`start_perception:=false`는 호스트에서 inspection node를 실행하거나 모델/키를
검사하지 않고 외부 GPU perception을 사용한다. 외부 노드는 동일한 프로필,
ROS domain, 시계, RGB-D/TF 토픽과 tracking 설정을 사용해야 한다.
통신·관측 실패는 기존 검사에서 중단하며 로컬 perception으로 자동 대체하지 않는다.
외부 모델/설정의 동일성을 자동 협상하지는 않는다. Jetson 연결 명령은
`containers/vision/README.md`를 따른다.

## Sorting 속도 설정 (2026-09-08 변경)

Simulation sorting launch의 감속 기본값을 상향했다. 일반 demo와 실제 로봇의
하드웨어 한계는 변경하지 않는다. 각 이름은 launch argument 및 ROS parameter다.

| 설정 | 이전 | 현재 |
|---|---:|---:|
| `velocity_scaling` / `acceleration_scaling` | 0.08 / 0.08 | 0.30 / 0.50 |
| `sorting_payload_velocity_scaling` / `sorting_payload_acceleration_scaling` | 0.04 / 0.02 | 0.24 / 0.20 |
| `cartesian_translation_speed_m_s` | 0.10 | 0.30 |
| `approach_velocity_scaling` / `retreat_velocity_scaling` | 0.7 / 0.6 | 1.0 / 0.8 |
| `cartesian_translation_acceleration_m_s2` | 0.20 | 0.80 |
| `cartesian_joint_acceleration_rad_s2` | 1.0 | 4.0 |
| `cartesian_rotation_speed_rad_s` | 0.50 | 1.50 |
| `lin_acceleration_scaling` | 0.4 | 0.8 |
| `corridor_time_margin` | 1.25 | 1.05 |
| `gripper_motion_sec` (별도 닫기/놓기 명령) | 8.0 | 2.0 |

접근 가속도 배율 `sorting_approach_acceleration_scaling=1.0`은 유지한다.
pregrasp와 그리퍼 열기는 기존 6관절 동시 계획을 유지하며, 별도 2초 대기를
추가하지 않는다. 운반은 기본 이동 배율과 payload 배율의 최솟값을 적용한다.
위 Cartesian 속도는 retiming 상한이지 실제 일정 속도를 보장하는 값이 아니다.
충돌/FK/관절 위치·속도 한계, 추적·접촉 감시, timeout 취소는 유지한다.
시험 runner의 강제 0.5배속도 제거해 launch 기본과 동일한 1.0배속을 사용한다.
CPU 시뮬레이션의 실측 RTF는 여전히 1 미만일 수 있다. 새 설정의 실제 3배
단축과 파지 유지 성공은 별도 실행으로 검증해야 하며 하드웨어 검증값이 아니다.

sorting launch의 `use_observed_collision_geometry=true`는 grasp 서버가 발행한
`/grasp/collision_geometry`를 target collision/attachment에 사용한다. 일반
기본 false는 기존 OBB 경로다. 64개 bounded cache에서 snapshot/object ID,
capture header와 OBB pose가 정확히 일치하는 mesh만 사용한다. finite/closed/
outward/convex 검사와 vertex/triangle 상한을 통과해야 하며, 활성화 상태에서
mesh가 없으면 timeout 후 실패하고 OBB로 조용히 대체하지 않는다. support patch,
전체 로봇 open/closure 검사는 유지한다. 운반/재관측/놓기 반경은 OBB 반대각선과
관측 mesh의 최대 local vertex 거리 중 큰 값이다. trimmed OBB 바깥 관측점도
포함하며 mesh 사용 모드에서 정확히 일치하는 geometry가 없으면 실패한다.
관측 프리즘은 숨은 실제 형상의 보장이 아니며, geometry cache는 identity 검사다.
시간 신선도 검사는 기존 sensor-scene/wrist 경로가 별도로 수행한다.
sorting artifact에는 수신한 `collision_geometry` CDR도 함께 기록한다.
헤드 재관측 후에는 기존 중심/방향 연속성 제한을 만족하는 후보만 동일 팔
reachability selector에 전달한다. 새 후보의 점수 순위 변화로 방향이 크게
다른 후보를 먼저 선택한 뒤 전체 작업을 중단하지 않도록 한다. 제한 자체는
완화하지 않으며 호환 후보가 없으면 이동 전에 실패한다.
Launch argument `sorting_head_reference_refresh_age_sec`는 기존 기본 30초
(simulation clock)를 유지한다. 더 짧은 양수 값으로 설정하면 단독 진단에서도
헤드 재관측 경로를 검증할 수 있다. 새 관측+재계획도 같은 신선도 상한을
만족해야 하며, 기존 노드 검증대로 30초를 초과할 수 없다.

`planning_scene_timeout_sec`는 응답 대기 상한이다. 일반 selector/executor는
각각 1/2초, CPU sorting launch는 5초다. 고정 대기를 추가하지 않으며 timeout
발생 시 해당 작업은 실패한다. 충돌 조건이나 관측 신선도 상한과는 별개다.
selector의 `state_validity_timeout_sec`도 CPU sorting launch에서 5초로 설정한다
(일반 기본값 1초). 이는 `/check_state_validity` 응답 상한이며 충돌 검사를
생략하거나 실패 결과를 허용하지 않는다.
`fk_timeout_sec`도 sorting에서 5초다. `ik_response_margin_sec`와
`planning_response_margin_sec`는 각각 IK/MoveGroup 계산 예산 뒤에 더하는
응답 여유이며 일반 1초, sorting 5초다. aim IK에도 같은 IK 응답 여유를 쓴다.
IK 계산 예산 0.15초, aim IK 1초, MoveGroup 계획 예산 4초 자체는 늘리지 않는다.
Cartesian 실행 wall deadline은 `max(minimum_sec, trajectory_duration * factor +
margin_sec)`이다. `cartesian_execution_wall_timeout_` 접두사의 `minimum_sec`는
60초, `margin_sec`는 10초, `factor`는 일반 2/sorting CPU simulation 10이다.
이는 고정 대기나 동작 감속이 아니며 완료 결과가 도착하면 즉시 진행한다.
기존 접촉/추적 감시와 timeout 시 취소·종료 확인은 유지한다. 느린 시뮬레이션의
13.9초 궤적이 wall time 60초를 넘는 경우에도 고정 60초로 잘리지 않게 한다.

Sorting은 초기 양손 그리퍼 열기 단계를 실행하지 않는다. 선택된 팔만
`*_pregrasp_open` 6관절 MoveIt group으로 pregrasp 이동과 열기를 함께 계획·실행한다.
팔과 그리퍼를 별도 비동기 명령으로 겹치지 않고, 열린 jaw를 포함한 전체 경로를
충돌 검사한다. Target contact 허용은 여전히 grasp 접근 단계에서만 적용한다.
MoveIt이 기존 팔/그리퍼 controller에 궤적을 분배하며, 양쪽 action의 완료와
선택한 6관절 feedback를 확인하기 전에는 손목 인계/접근으로 넘어가지 않는다.
반대 팔 그리퍼는 움직이지 않는다. Grasp/retreat IK는 기존 5관절 계약을 유지한다.
투입 완료 후에는 `*_return_close` 6관절 group으로 저장된 원래 팔 자세로 복귀하면서
선택 그리퍼를 `sorting_return_gripper_position_rad=-0.30` rad로 함께 닫는다.
의도한 release가 완료되고 attachment가 해제된 빈 팔에서만 허용하며,
이 경로도 jaw를 포함해 충돌 검사하고 두 controller의 완료를 확인한다.
`gripper_open_position_rad`는 동시 이동의 최종 열림 위치다. 기존
`gripper_motion_sec=8`은 별도 파지/놓기 명령에만 적용하며 동시 이동 시간은
MoveIt의 속도·가속도 제한 및 경로가 결정한다. 시작 상태 표시는 `search`다.

2026-09-07 headless attempt56에서는 왼팔/왼쪽 그리퍼 controller의 첫 goal 수신
간격이 2.06 ms였고 두 controller가 같은 시점에 성공했다(동시 이동 약 9.65초).
오른쪽 그리퍼 goal과 초기 `prepare_grippers` 단계는 없었다. 컵 투입·복귀 확인은
coordinator `starting` 기준 145.53초, `pick` 기준 114.71초였다. 가속도 설정은
변경하지 않았다. 이전 headless attempt54의 152.44초보다 총 6.91초 짧았으나
인식/계획 시간 변동이 포함된 단회 비교이며 제거된 초기 열기 대기 시간과 같지는 않다.
컵 안착은 검증됐지만 전체 미션은 기존처럼 `lost_item` 미검증으로 종료했다.
실행 로그는 `artifacts/sorting_20260907/launch_attempt56_parallel.log`에 있다.

sorting launch는 `sorting_use_wrist_camera=true`를 기본으로 사용한다.
`wrist_continuous_tracking=true`가 기본이며 HANDOFF 이후 perception worker가
선택 손목을 계속 추적한다. `sorting_async_carry_monitor` 기본값은 손목 사용 여부를 따른다.
인계한 reference/arm/source/object ID와 일치하는 `/perception/wrist_tracking_status`만
사용한다. 파지 접촉 안정화 후 최신 정상 결과가 있어야 이동 중 감시를 활성화한다.
상승 후 `lift_hold_sec` 대기와 동기 CHECK RPC는 생략하고, 관절 기반 clearance 확인 후
다음 운반을 계획·실행한다. 궤적 구간 사이 계획/피드백 처리는 남으므로
속도가 끊기지 않는 trajectory blending이나 visual servo를 구현한 것은 아니다.

감시 중 mask 소실/면적 이상, 추론·카메라 오류, 결과 stale 또는 그리퍼 접촉 추정 소실은
**파지 이상 의심**으로 latch하고 MoveGroup/ExecuteTrajectory를 취소한다.
취소 응답뿐 아니라 action terminal 결과까지 기다리며 실패하면 다음 동작/놓기를 실행하지 않는다.
Humble 상위 ExecuteTrajectory 취소만으로 controller 정지가 전달되지 않는 경로를 고려해
선택 팔 JTC의 status에서 확인한 **특정 goal UUID**도 직접 취소한다.
반대 팔이나 파지 중인 그리퍼를 취소하지 않으며 JTC terminal 상태와 새 정지 feedback까지 확인한다.
감시 중 payload MoveGroup goal은 내부 자동 replan도 끄므로 취소한 controller를 재시작하지 않는다.
근거: [MoveIt Humble ExecuteTrajectory 구현](https://github.com/moveit/moveit2/blob/humble/moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp)
및 아래 시험 로그의 실제 controller 취소/정지 응답. 이 우회는 시뮬레이션 backend에 한정된다.
이동 중 compliant jaw의 일시적 속도 변동은 `sorting_contact_loss_grace_sec=0.3`초로
연속 소실 여부를 판별한다(대기 동작이 아님). 영상 소실은 이 debounce를 거치지 않는다.
그리퍼 feedback 최대 수신 age `sorting_joint_feedback_max_age_sec=1`초도 별도 검사한다.
이는 확정 낙하 판정이나 안전 인증된 정지 기능이 아니다. 가림/시야 이탈/오추적과
실제 낙하는 RGB mask만으로 구별할 수 없다. 현재 CPU 추론은 약 0.2 Hz이므로
소실 감지에 수초가 걸릴 수 있고, 시야에 남은 떨어진 물체를 놓칠 수도 있다.
`sorting_tracking_max_capture_age_sec=12`는 원본 촬영 시각(ROS clock),
`sorting_tracking_max_update_age_sec=8`은 새 결과 수신 간격(monotonic wall clock) 제한이다.
CPU의 추론 중 이전 결과를 사용하는 상황을 고려한 **시뮬레이션 시험값**이지 실기 안전값이 아니다.
중복/역순 결과는 heartbeat를 갱신하지 않는다. 수거함 개구부 도달/접촉 확인 후
의도한 release 직전에 감시를 해제하여 정상 놓기를 낙하로 처리하지 않는다.
그리퍼 파지 안정화, 충돌/장면 갱신 barrier와 개구부 확인은 유지한다.

2026-09-07 GUI 없는 정상 컵 시험 65/68에서 접근 18.886→12.531초,
상승 controller 종료→운반 시작 7.688→0.224초, pick→컵 안착·복귀 확인
101.766→92.286초(각 2회 평균)를 기록했다. 최종 코드 단회 68은 88.823초였다.
상승 명령 시작/운반 중 소실 신호 주입 시험에서도 JTC 취소와 새 정지 feedback,
후속 투입 금지를 확인했다. 실제 낙하 검출 정확도 시험은 아니며 전체 미션은 `lost_item` 미검증이다.
revision/측정 경계 및 한계: `artifacts/sorting_20260907/async_carry_report.md`.

`sorting_async_carry_monitor:=false`는 기존 상승 후 CHECK+3초 hold 비교 모드다.
이 모드에서만 CHECK는 상승 완료 이후 frame과 최대 결과 age 6초를 요구한다.
batch 비교는 여기에 `wrist_continuous_tracking:=false`를 함께 지정한다.
아래 수치는 이번 이동 중 감시 변경 **이전**의 연속 추적 효과다.
2026-09-07 headless 대조군 attempt60과 attempt61/62 평균 비교에서
손목 CHECK RPC는 19.232→4.499초, pick부터 컵 수거·복귀 확인은
115.952→101.766초였다. 최종 2회 모두 컵 안착 성공이나 전체 미션은
`lost_item` 미검증이다. 초기화 포함 시간은 147.680→127.195초였으며 초기
인식/계획·경로 변동도 포함한다. 상세: `artifacts/sorting_20260907/continuous_tracking_report.md`.
head RGB-D로 물체/3D grasp를 선택하고 pregrasp에 도착하면 해당 손목 RGB로
인계한다. 이때 head renderer는 10→2 Hz, 선택 손목은 10 Hz가 된다.
기존 3D 추정과 경로를 보존하고 손목 RGB로 대상 일관성을 검사하며,
이 분기에서 head 재검출/3D 재계산과 head 시야 복구 이동을 수행하지 않는다.
RGB 인계가 실패하면 중단하며 이전 3D 위치를 새 측정값으로 포장하지 않는다.
lift는 손목 mask·그리퍼 관절 feedback 기반 접촉 추정·kinematic clearance로 확인한다.
접촉 추정은 닫힘 명령 대비 정지한 관절 상태를 사용하며 독립 힘 센서 측정이 아니다. 이것은 독립적인
depth 상승량 검증이 아니다. 수거함은 `robot_top_bins.yaml`의 초기 base_link 기준 위치를
사용하고 운반 전후 접촉과 충돌·개구부 검사는 유지한다. 복귀 후 head 10 Hz로 복원한다.
시뮬레이션 손목 TF는 nominal CAD mount이며 실제 로봇에서는 측정한 양팔 hand-eye
calibration이 필요하다. `sorting_wrist_cameras_config`로 simulation nominal YAML을
지정하며 기본은 `cleany_mujoco_sim/config/sorting_wrist_cameras.yaml`이다.
외부 로봇 실행은 아직 지원하지 않는다. 손목 영상 기반 연속 위치 보정(visual servo)은
구현하지 않았으며 접근/운반 경로는 기존 head 3D 추정 및 관절 feedback에 의존한다.
기존 head-only 비교 시험은 `sorting_use_wrist_camera:=false`로 실행한다.

연속 추적 적용 전인 2026-09-07 headless attempt54에서는 batch 손목 확인으로 컵을 `trash_left`에 투입하고
복귀했다(pick 시작부터 안착 확인까지 110.60초, CPU 손목 확인 RPC 19.16초 포함).
현재 전체 미션은 `lost_item` 미검증으로 종료한다. 성공 1회는 연속 수거/반복 성공률
검증이 아니며 손목 영상 기반 실시간 위치 보정이 완성됐다는 의미도 아니다.

## 상태

점수순 grasp 후보를 양팔 MoveIt plan-only 검증으로 평가한다. 운영 action은 실제
trajectory와 gripper 명령을 실행하지 않는다. 별도 시뮬레이션 데모에서만 선택 결과를
MuJoCo arm controller로 실행할 수 있다. `nearest_pregrasp_coordinator`는 perception이
제공한 거리 유효 객체를 가까운 순서로 시도한다. 기본
`execute_grasp_and_lift=false`에서는 첫 reachable grasp의 pre-grasp까지만 실행하고,
옵션을 켜면 재인식과 Pilz LIN 기반 접촉 파지·후퇴·상승까지 수행한다.

## Reachable grasp action

`grasp/select_reachable` (`SelectReachableGrasp`)는 가까운 팔부터 IK, state validity,
current→pre-grasp와 pre-grasp→grasp plan을 검사한다. pre-grasp는 접근 벡터 반대 방향
0.14 m다. 5축 position-only IK의 방향 손실을 막기 위해 먼저 TCP 위치 IK로 seed를 만든
뒤, TCP의 실제 `-Y` 접근축 앞 0.14 m에 있는 가상 aim tip을 grasp point에 맞춘다. 이로써
물리 gripper가 선택 객체를 바라보는 것은 강제된다. 위치 seed가 실패하면 현재 joint
state에서도 계속 시도하며, 전체 arm joint limit 안에 분산한 팔당 8개 seed에서 허용
오차를 만족하는 해는 현재 상태 대비 joint-limit 정규화 이동량으로 정렬하고 wrist
roll에는 2배 가중치를 둔다. 자세 오차는 동률 해의 tie-break로만 사용한다. 5축 기구가 정확한
후보 방향을 만들 수 없는 경우에는 접근축 최대 15도, parallel-jaw 대칭을 고려한 closing
축 최대 30도 안에서 가장 가까운 실행 가능 방향을 허용한다. 그보다 큰 오차는 다음 arm
또는 후보로 fallback한다. grasp 위치에서도 TCP 위치·접근축·closing 축을 FK로 다시
검사한다. 한 해의 validity 또는 planning이 실패하면 같은 팔의 다음 grasp/pre-grasp
해를 먼저 시도한 뒤 반대 팔과 다음 후보로 넘어간다. 선택 오차는 selector 로그에 남긴다.

## 제공 계약

실행 결과는 Mission Manager가 상태 전이에 사용할 수 있는 성공·실패·차단 결과로 반환한다.
Skill Executor는 Mission Manager의 FSM 상태를 직접 변경하지 않는다.

## 설정 및 검증

입력 후보는 snapshot/object/frame/target OBB가 같고 frame은 설정된 `planning_frame`과
일치해야 한다. 현재 12개 arm/gripper joint는 완전하고 0.5초 이내여야 한다. action
동안 target OBB와 ACM은 임시 변경되고, 복원에 성공한 뒤에만 최종 성공·실패·취소
상태를 확정한다. 복원 실패는 planning-scene 오류로 반환한다. 한 번에 goal 하나만
처리한다.

```bash
ros2 launch cleany_moveit_config mock_planning.launch.py
ros2 launch cleany_skill_executor grasp_selection.launch.py
pytest -q ros2_ws/src/cleany_skill_executor/test
```

planning frame, timeout, planning attempt/scaling, 최대 후보 수는
`config/grasp_selection.yaml`의 ROS parameter로 설정한다. timeout/cancel은 각 IK,
state-validity와 planning 단계 전후에 확인한다.

simulation 실행은 planning attempts 3, velocity/acceleration scaling 0.08,
MoveGroup replan 2회와 0.25초 delay를 사용한다. controller 실패 시 joint별 tracking
error를 기록하고 현재 상태에서 한 번만 같은 joint goal로 안전 재계획한다.

## 가까운 객체 자동 pre-grasp

`nearest_pregrasp_coordinator`는 다음 순서로 한 번의 작업을 수행한다.

```text
InspectScene 1차 detection
→ distance_valid 후보를 base_link 거리순 정렬
→ selected-object inspection
→ PlanGrasp
→ SelectReachableGrasp
→ target OBB를 충돌물로 유지하며 pre-grasp 실행
→ (execute_grasp_and_lift=true) 새 RGB-D 재인식과 동일 arm 재선택
→ Pilz LIN 접근 → gripper 접촉 → LIN 역접근 → LIN 수직 상승
```

양쪽 gripper는 object evaluation 전에 open한다. 따라서 MoveIt 후보 검증과 실제 실행이
동일한 gripper joint state 및 collision geometry를 사용한다.

가장 가까운 객체가 segmentation, grasp 생성 또는 양팔 도달성 검사에서 실패하면 다음
가까운 객체로 fallback한다. depth가 불충분해 `distance_valid=false`인 detection은
자동 조작에서 제외한다. infrastructure, MoveIt 또는 controller 실패는 다른 객체로
넘기지 않고 전체 작업을 실패시킨다.

RGB-D camera, `perception/inspect_scene`, `grasp/plan`,
`grasp/select_reachable`, MoveGroup과 arm/gripper controller를 먼저 실행한 뒤 coordinator를
시작한다.

```bash
ros2 launch cleany_skill_executor nearest_pregrasp.launch.py
```

query, timeout, 속도/가속도 scaling과 gripper open 위치는
`config/nearest_pregrasp.yaml`에서 설정한다. pre-grasp joint target은 운영 selector가
aim-tip IK와 FK 방향 검증까지 통과한 결과를 사용한다. 성공한 target OBB는 완료 자세에서
Planning Scene에 유지하며, 실패하거나 coordinator가 종료될 때 기존 ACM과 함께 복원한다.

가까운 객체 선택을 실제 MuJoCo RGB-D 입력부터 확인하는 GUI 데모는 다음과 같이 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_skill_executor nearest_rgbd_pregrasp_demo.launch.py
```

MuJoCo가 렌더링한 빨간 can과 파란 box의 실제 RGB 픽셀로 bbox/mask를 만들고, depth와
CameraInfo를 역투영한 뒤 capture-time TF로 `base_link` 거리를 계산한다. 유효 거리순으로
첫 객체를 선택해 geometric grasp, 양팔 MoveIt 검증 및 실제 controller pre-grasp 실행까지
이어진다. MuJoCo, RViz와 `/perception/debug_image_latched` Image View가 기본으로 열린다.
색상 detector/segmenter는 이 결정적 simulation 회귀 데모만을 위한 adapter이며 물체 pose나
가상 point cloud를 사용하지 않는다.

Study-cafe의 네 물체를 대상으로 인식부터 접촉 집기와 후퇴까지 실행하려면 다음 launch를
사용한다.

```bash
ros2 launch cleany_skill_executor study_cafe_nearest_grasp_demo.launch.py
```

기본값은 아래 센서 전용 절의 Gemini 3.1 Flash-Lite + SAM2.1-tiny 설정이다. 기존 color adapter는
과거 머그컵/휴대폰/지우개 fixture 전용이며 현재 종이컵/레고/휴지에 대응하지 않는다.
다른 YOLOE checkpoint와 SAM2.1 tiny mask 실행은 다음처럼 선택한다. Headless에서도 MuJoCo
camera에는 유효한 X11/Xvfb context가 필요하다.

```bash
export DISPLAY=:0
export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4
ros2 launch cleany_skill_executor study_cafe_nearest_grasp_demo.launch.py \
  headless:=true use_rviz:=false use_image_view:=false \
  perception_detector_type:=yoloe perception_segmenter_type:=sam2 \
  perception_minimum_detection_confidence:=0.05 \
  yoloe_model_path:=/absolute/path/to/yoloe-26n-seg.pt \
  yoloe_text_encoder_directory:=/absolute/path/to/yoloe-model-directory \
  yoloe_classes:="[cup, wallet, crumpled tissue, lego brick]" \
  perception_device:=cpu \
  sam2_model_config:=configs/sam2.1/sam2.1_hiera_t.yaml \
  sam2_checkpoint:=/absolute/path/to/sam2.1_hiera_tiny.pt
```

머리 RGB-D의 실제 렌더 픽셀에서 종이컵·지갑·휴지뭉치·레고를 검출하고 `base_link` 거리순으로
시도한다. 선택기는 작은 물체용 closing 축 오차와
TCP 보정을 적용하고, coordinator는 pre-grasp, contact-enabled grasp, gripper 닫힘,
pre-grasp 후퇴를 실행한다. 그리퍼는 닫힌 상태로 pre-grasp까지 이동한 뒤 grasp 진입 전에
선택한 팔만 연다. pre-grasp에서 정지한 뒤 새 RGB-D snapshot으로 같은 label의 최근접 OBB를
연결하고 중심 30 mm, 접근·closing 축 15도, 후보 모호성 10 mm gate를 통과한 경우에만 기존
팔로 grasp를 다시 선택한다. 현재 TCP 직선이 갱신된 접근축과 10도 이상 다르면 중단한다.
최종 접근은 Pilz `LIN` plan-only 결과를 `/execute_trajectory`로 실행한다. controller
reference-feedback 오차와 실제 속도가 TCP 목표 20 mm 안에서 각각 0.10 rad 이상,
0.05 rad/s 이하인 상태가 5 sample 이어지면 trajectory를 취소하고 접촉 정지로 인정한다.
effort threshold는 기본 비활성화이며 backend가 신뢰 가능한 값을 제공할 때 parameter로
추가할 수 있다. 접촉이 없더라도 LIN endpoint 도착은 정상이다. gripper는 목표 각도 전에
저속 정지한 경우에만 물리 접촉으로 판정한다. 성공하면 최신 OBB를 selected gripper에
attached collision object로 옮기고, 실제 물체는 weld하지 않는다. 이어서 접근 시작 TCP로
Pilz LIN 역접근하고 같은 자세를 유지한 채 `base_link +Z` 60 mm를 LIN 상승한다.
단, 현재 position-only IK는 Pilz 요청의 방향을 보장하지 않는다. 실행 전 반환
trajectory endpoint의 FK가 요청 위치 1 mm / 전체 회전 0.01 rad 이내인지 검사하며,
위치만 맞는 잘못된 방향의 계획은 실행하지 않는다. Sorting의 접근/역접근은 아래의
joint-goal corridor 방식을 사용한다.
Sorting은 `count_retreat_as_lift=true`로 후퇴 중 실제 TCP 상승량을 포함해
`lift_distance_m`(기본 60 mm)의 최소 상승량을 확보한다. 닫기/settle 후 실제 TCP와
후퇴 후 feedback FK를 비교해 부족한 높이만 추가 LIN 상승한다. 이미 확보됐다면
추가 상승 명령 없이 hold한 뒤 RGB-D로 물체 중심이 **파지 전 관측 중심 +60 mm**
이상 올라왔는지 확인한다. TCP가 올라온 것만으로 물체를 들었다고 판정하지 않는다.
이때 기존 절대 높이 기준도 함께 적용한다. 추가 LIN이 필요한 경우 원래 방향/
위치/충돌 검사로 검증하며, 동작 한계나 방향 허용치를 늘리지 않는다.
Generic 기본값은 false라 기존의 후퇴 뒤 추가 상승 동작을 유지한다.
`lift_min_center_z_m`이 양수이면 후퇴 뒤 RGB-D로 같은 label을 다시
검출·재구성해 OBB 중심 높이가 임계값 이상일 때만 `NEAREST GRASP COMPLETE`를 출력한다.
이 launch의 rqt Image View는 `/grasp/debug_image_latched`를 열며 검출 bbox/mask 위에
grasp 후보를 겹쳐 표시한다. 사각형 `PGn`은 pre-grasp, 원 `Gn`은 grasp TCP이고,
cyan은 생성된 후보, green은 MoveIt이 최종 선택한 후보다. 이 좌표에는 selector와 동일한
approach/lateral TCP 보정이 적용된다.
Coordinator는 service와 controller가 준비된 뒤 `demo_start_delay_sec`만큼 기다려 첫
RGB-D 동기화 쌍이 생성된 다음 검출을 요청한다. 이 demo에서는 파지 실패 후에도
coordinator를 유지해 마지막 `/grasp/debug_image_latched` publisher와 rqt 표시가
사라지지 않게 한다. GUI 렌더 지연을 고려해 snapshot 대기는 10초이며, rqt는 overlay
publisher가 생성된 뒤 시작하고 해당 토픽을 새 설정으로 선택해 항상 위에 표시한다.
rqt의 volatile 구독이 단발성 메시지를 놓치지 않도록 마지막 grasp overlay는 0.5초마다
steady wall clock 기준으로 다시 발행한다. 따라서 MuJoCo simulation clock이 멈춰도
마지막 이미지는 계속 표시된다.
Study-cafe grasp 생성기는 `base_link` 원점에서 물체로 향하는 수평 방향과 반대인
approach를 제거하고, box처럼 5축 손목 방향 제약이 큰 물체를 위해 closing yaw를
`-80..80 deg` 범위에서 생성한다. 정확한 도달성과 방향은 이후 양팔 IK/FK 검증이
최종 판정한다. Study-cafe에서는 5축 팔의 잔여 자유도 오차를 수용하되 과도한 기울기는
거부하도록 approach 허용값을 `18 deg`로 제한한다. 손목 roll seed는 48개로 촘촘히
검사해 `8 deg` closing 허용 범위 사이의 유효 자세를 건너뛰지 않도록 한다.

Gemini detection부터 SAM2 segmentation 및 pre-grasp 실행까지 확인하려면 키를 현재
shell의 환경변수로만 주입하고 다음처럼 실행한다. 키를 저장소 파일이나 launch 인자에
기록하지 않는다.

```bash
export GEMINI_API_KEY="<your-api-key>"
ros2 launch cleany_skill_executor nearest_rgbd_pregrasp_demo.launch.py \
  detector_type:=gemini segmenter_type:=sam2 \
  gemini_model:=gemini-robotics-er-2-preview \
  sam2_model_config:=configs/sam2.1/sam2.1_hiera_t.yaml \
  sam2_checkpoint:=/home/ubuntu/models/sam2/sam2.1_t.pt \
  sam2_device:=cpu
```

Gemini는 렌더 RGB에서 bbox와 label만 반환한다. 이후 mask는 SAM2, 거리와 3D geometry는
MuJoCo depth·CameraInfo·capture-time TF에서 계산하며, 이동은 동일한 MoveIt 및
`mujoco_ros2_control` 경로를 사용한다.

## MuJoCo 육안 확인 데모

아래 단일 launch는 실제 `mujoco_ros2_control` backend, MoveIt, RViz, grasp selector와
demo coordinator를 함께 시작한다. 기본값은 MuJoCo native viewer와 RViz를 모두
표시한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py
```

기본 synthetic reachable candidate는 같은 grasp point를 공유하는 실제 5축
pre-grasp/grasp FK 해에서 얻은 quaternion을 사용한다. 따라서 local `-Y` 접근축과
local `+X` closing 축을 selector가 보존하는지 데모 자체에서도 검증한다.

데모는 의도적으로 최고 점수의 도달 불가 후보를 먼저 검사한 뒤, 초록색 MuJoCo box와
정렬된 두 번째 후보를 왼팔로 선택한다. 선택 action에서 direction-aware pre-grasp IK,
두 endpoint의 collision/state validity, 두 구간 OMPL plan-only를 통과해야만 coordinator가
`left_arm_controller`로 pre-grasp와 grasp trajectory를 차례로 실행한다. 마지막에는
실제 `/joint_states`가 선택 결과에 수렴했는지도 검사한다. Gripper close, attach, lift는
아직 실행하지 않는다.

RViz의 `Grasp Candidates` display에서 구/화살표/상태 문구를 보고,
`MotionPlanning` display에서는 계획 궤적과 실제 joint state를 확인한다. MuJoCo 창의
초록색 box가 target이며 왼팔이 먼저 pre-grasp에서 멈춘 다음 box까지 접근한다. 창이
준비될 시간을 위해 평가 전 5초, 각 실행 구간 사이 3초를 기본 대기한다. 빠른 headless
회귀 검증은 다음처럼 실행한다.

```bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py \
  headless:=true use_rviz:=false \
  demo_start_delay_sec:=0.1 stage_hold_sec:=0.1
```

로그의 `Selected candidate=1 arm=left`, 두 개의
`MoveIt execution succeeded`, `DEMO COMPLETE`가 전체 성공 기준이다. 데모는 완료
자세와 marker를 유지하므로 종료는 `Ctrl-C`, 다시 보기는 launch 재실행으로 한다.

### 실제 RGB-D can 검출·잡기·들기 데모

다음 launch는 table, 파란 box, 빨간 can과 고정 RGB-D 카메라가 있는
`mujoco_ros2_control` 장면을 연다. 시뮬레이터가 렌더링한 RGB-D에서 빨간 can을
분할하고 `base_link` 점군으로 투영한 뒤, geometric grasp 후보 생성과 MoveIt
양팔 검증을 거쳐 선택된 pre-grasp 자세까지 실제 controller로 이동한다. 실제 joint
feedback으로 도착을 확인한 뒤 그리퍼를 열고, selector가 이미 검증한 grasp joint goal로
전진해 그리퍼를 닫는다. 비대칭 gripper 때문에 원본 후보와 pre-grasp pose는 보존하되
최종 grasp IK에만 local 접근축 `+10 mm`와 물체 폭으로 계산한 closing `+X`축
보정을 적용한다. 범용 selector의 closing 축 최대 오차는 30도지만, 원통형 캔이
jaw 사이로 빠져나가지 않도록 이 demo에서만 15도로 제한한다. 범용 parallel-jaw
계약은 `+X/-X`를 같은 축으로 보지만, 고정 jaw 보정에는 방향이 있으므로 이
demo의 최종 grasp만 부호까지 일치해야 한다. 접근 중 controller가
멈추면 FK로 목표까지 남은 거리가 10 mm
이내인지 확인한 경우에만 물체 접촉으로 받아들이고 endpoint 재시도를 생략한다.
MoveIt에는 can을 선택 gripper의 attached collision object로 전환하고, 같은 pre-grasp
goal까지 되돌아가 약 14 cm 후퇴·상승한다. 실행 직전 IK를 다시
계산하지 않는다. MuJoCo can은 60 g free body이며 weld 없이 jaw 접촉과 마찰만으로 들어
올린다. 마지막에는 새 RGB-D frame에서 can 높이가 5 cm 이상 증가했는지 확인한다.
Can demo는 기계적 limit에서 최소 0.02 rad 여유를 둔 접힌 초기 자세로 양팔을 소환한다.
이 초기값은 launch에서 ROS control description에 전달되며 다른 MuJoCo workflow의 기본
0 rad 초기 자세는 변경하지 않는다.
운영 `SelectReachableGrasp` action에서도 같은 보정과 FK 방향 검증을 수행한다. can 데모는
실행 직전에 한 번 더 FK 결과를 표시해 육안 확인용 marker와 로그를 제공한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_skill_executor can_grasp_execution_demo.launch.py
```

기본으로 세 창이 열린다.

- MuJoCo: 갈색 table 위 빨간 can으로 이동하는 실제 simulation 상태
- RViz: Planning Scene table과 검출 can 반투명 원통, 후보 구,
  선택 후보 초록 구,
  접근 방향 파란 화살표
- Image View: `/grasp/can_grasp_image`의 실제 RGB 영상 위 후보별 TCP, 접근 화살표,
  score, 접근 azimuth/elevation, 요구 opening과 MoveIt 선택 결과

로그에서 `RGB-D can segmented`, `GEOMETRIC GRASP COMPLETE`, `Selected generated
candidate`, `Direction-aware pre-grasp verified`,
`MoveIt execution succeeded: collision-checked aimed pre-grasp`, gripper open,
`MoveIt execution succeeded: contact-enabled grasp`, gripper close,
`MoveIt execution succeeded: attached-can lift retreat`,
`Physical can lift verified from RGB-D`, `CAN GRASP DEMO COMPLETE`가 차례대로
나오면 전체 경로가 성공한 것이다. RGB-D 렌더링에는 OpenGL context가 필요하다.
`headless:=true`로 native viewer를 숨길 수는 있지만 `DISPLAY`가 없으면 카메라가
비활성화되므로 X11/Xvfb context가 필요하다. table과 can은 MuJoCo 물리 충돌체이며
같은 크기와 pose로 MoveIt Planning Scene에도 등록된다. 후보 선택 중에는 검출 can
OBB를 검사한다. 실제 최종 접근부터는 선택 팔의 fixed/moving jaw에만 can 접촉을
허용하고, 닫기 뒤 MoveIt collision cylinder를 선택 gripper에 attach한다. 이 attach는
RViz와 lift 충돌 계획용이며 MuJoCo 물체를 강제로 고정하지 않는다. 닫힘 명령은 후보의
`required_opening_m`을 jaw 각도로 환산한다. 50 mm aperture의 기준점은 `0.30 rad`,
기울기는 `0.10 m/rad`이며 후보 opening보다 10 mm 작게 명령해 접촉력을 만든다.
후보 opening이 없을 때만 `0.30 rad`를 fallback으로 쓴다. 따라서 비스듬한 자세에서
약 70 mm로 보이는 박스를 50 mm용 각도까지 억지로 조여 밀어내지 않는다. 실제 접촉 시
목표까지 닫히지 않으면 0.10 rad 이상 닫힌 뒤 명령과 0.05 rad 이상 차이가 난 저속
정지 상태를 contact로 판정한다. 안정화
시간은 1초다. arm path tolerance는 실제 추종 오차 `0.100364 rad`가 기존
`0.10 rad` 경계를 넘은 측정 결과를 반영해 이 demo controller에만 `0.12 rad`를
적용한다. 최종 성공 판정은 이 관절 정지가 아니라 lift 뒤 RGB-D 높이 변화다.
`gripper_force_full_close:=true`는 후보 폭 환산을 우회해
`gripper_close_position_rad`를 직접 명령하는 simulation 진단 옵션이며 기본값은
`false`다. 이 모드에서는 물체 접촉으로 jaw가 목표 전에 정지해야만 파지로 인정한다.
설정된 완전 닫힘 위치에 도달하면 물체가 jaw 사이를 벗어났거나 contact가 관통한
것이므로 lift 전에 실패 처리한다. 실제 물체에 쓰기 전에는 충돌·토크 한계를 별도로
검증해야 한다. 접촉이 검출되면 해당 close trajectory의 위치 tolerance만 지워
토크 제한이 적용된 닫힘 목표를 lift 동안 계속 유지한다. lift 직후 높이 검사에 더해
기본 3초 유지 뒤 RGB-D 높이를 다시 검사하므로 잠깐 들렸다 떨어지는 경우는 성공으로
판정하지 않는다.
느린 simulation controller가 짧은 최종 접근 trajectory의 기본 MoveIt 시간 상한에
걸리지 않도록 이 launch에만 execution-duration scaling `2.0`, goal margin `1.0초`를
적용한다. 다른 MoveIt workflow의 기본값 `1.2`/`0.5초`는 유지한다.

selector는 물체 위치로 먼저 구한 어깨·팔꿈치 자세를 우선 IK seed로 쓰고, 같은 자세의
wrist roll을 물체가 놓인 좌우 방향에 맞춰 한 번 더 검사한 다음 범용 seed로 넘어간다.
pre-grasp 거리는 기존과 동일하게 14 cm 하나만 사용하며 selector 전체 작업 재시도는
하지 않는다. `grasp_approach_offset_m`과 `grasp_lateral_offset_m`의 공통 기본값은
모두 0이다. 이 simulation launch는 접근 방향으로 10 mm를 적용하고, 비대칭 jaw의
좌우 중심 보정은 `max(target_width_m, target_depth_m) / 2 - 8 mm`로 계산한다.
따라서 70 mm 캔과 50×70 mm 박스는 모두 27 mm를 사용한다. 박스 후보가 회전해 긴
변을 물더라도 고정 손가락이 물체 안에서 출발하지 않게 하는 보수적인 값이다. 같은
값을 selector와 실행 시 접촉 검증에 전달하므로
계획 목표와 검증 목표가 어긋나지 않는다.
같은 좌우 보정을 pre-grasp에도 적용한다. 따라서 마지막 14 cm 접근에서 TCP가
옆으로 이동하며 박스를 쓸지 않고, pre-grasp와 grasp 사이 변위가 접근축과 평행하다.

### 무작위 headless pre-grasp 스트레스 검증

DISPLAY가 없는 환경에서는 `mujoco_ros2_control`의 GLFW RGB-D renderer를 사용할 수
없다. 다음 검증은 RGB-D detection/segmentation을 제외하고 headless MuJoCo controller,
MoveIt selector, FK 자세 오차, 충돌 검사와 실제 pre-grasp 실행을 반복한다. 각 반복에서
table 위 box와 can 위치를 seed 기반으로 독립 생성하고, 하나를 target으로 선택하는 동안
다른 하나도 Planning Scene 충돌체로 유지한다. target은 box와 can을 번갈아 사용하며 성공
후 선택된 팔을 초기 자세로 복귀시켜 각 반복의 시작 조건을 맞춘다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
CLEANY_RANDOM_STRESS_ITERATIONS=100 \
CLEANY_RANDOM_STRESS_SEED=20260826 \
CLEANY_RANDOM_STRESS_RESULT=/tmp/randomized_pregrasp_stress.json \
python3 -m pytest -q -s \
  ros2_ws/src/cleany_skill_executor/test/test_randomized_pregrasp_stress.py
```

반복 수를 지정하지 않으면 장시간 스트레스 테스트는 일반 pytest에서 skip된다. 결과 JSON은
반복별 물체 위치, 선택·실행·전체 시간, 선택 arm/candidate와 실패 stage/error code를 담는다.

## 센서 전용 study-cafe 검증

`study_cafe_nearest_grasp_demo.launch.py`는 **Gemini 3.1 Flash-Lite + SAM2.1-tiny**,
`sensor_scene:=true`, `plan_only:=true`가 기본이다. 검출 설정은
`cleany_perception/config/gemini_flash_lite_sam2_tiny.yaml`을 사용한다. 기존
`yoloe_s_sam2_tiny.yaml`은 SAM2 및 명시적인 YOLOE 재선택의 기본 자산 설정으로만
병합되며 Gemini 실행에서는 YOLOE checkpoint/encoder를 로드하지 않는다. 사전 정의한
컵/지갑/휴지/책상/칸막이/
모니터 충돌 박스 loader를 실행하지 않고 전체 depth로 OctoMap을 구성한다.
MuJoCo scene은 영상과 물리 시뮬레이션 생성에만 쓰며 그 환경 형상이나 pose를
perception/MoveIt 환경 입력으로 전달하지 않는다. 로봇 URDF, 관절 상태와
카메라 외부 보정은 계속 필요하다. 현재 demo의 고정 카메라 TF는 고정 head
자세용이므로 실제 가동 head에는 촬영 시점의 동적 TF/보정이 필요하다.

```bash
make sim-mujoco-pipeline
```

실행 프로세스에 `GEMINI_API_KEY`가 필요하다. 키가 없으면 simulator를 띄우기 전
preflight에서 중단한다. 키는 명령 인수나 레포에 넣지 않고 환경 변수로 전달한다.
`gemini_model:=gemini-3.1-flash-lite`, `gemini_api_key_environment:=GEMINI_API_KEY`
인수로 모델/환경 변수 이름을 지정할 수 있다. 이는 **클라우드 API 검출**로 RGB PNG와
검출 prompt가 Google API로 전송된다. 네트워크 지연·할당량·API 오류가 발생할 수 있으며
YOLOE/color fallback 없이 실패한다. SAM2 mask와 손목 추적은 기존 로컬 모델을 유지한다.
초기 prepare는 키 존재와 클라이언트 생성만 검사하므로 API 접근/모델 사용 권한은 첫
추론에서 확인된다. confidence는 Gemini가 응답한 값이며 YOLOE 점수와 동등하게 보정된
확률이 아니다. 값 0.25를 유지하지만 정확도 비교는 실제 bbox 정답 기준으로 해야 한다.

기존 YOLOE-s 검출을 명시적으로 선택하려면 sorting 실행에서
`SORTING_ARGS='perception_detector_type:=yoloe'`를 사용한다. 이 경우 API 키는 필요 없다.

로컬 SAM2는 기본 `~/models` 또는 `CLEANY_MODEL_DIR` 아래의 `sam2/sam2.1_t.pt`를
읽는다. YOLOE를 명시적으로 선택할 때만 `yoloe/yoloe-26s-seg.pt`와
`yoloe/mobileclip2_b.ts`도 필요하다. 필요한 자산 경로가 없으면 backend 실행 전 실패하며 다운로드하지 않는다.
모델/입력 크기/클래스/신뢰도는 launch 인자로 명시적으로 변경할 수 있다.
기본 YOLOE 입력 크기는 640, confidence는 0.25다. 이전 시뮬레이션 진단의
0.05를 자동으로 적용하지 않는다. `perception_device:=auto`는 CUDA 사용
가능 여부에 따라 cuda:0 또는 CPU를 선택하고 실제 선택을 로그/ROS parameter에
기록한다. 명시한 CUDA가 없으면 CPU로 조용히 전환하지 않고 실패한다.

`preload_models:=true`는 action server 공개 전에 로컬 SAM2를 로딩하고 Gemini client를
준비한다. `PERCEPTION MODELS READY`는 로컬 초기화 완료이지 Gemini API 사용 권한이나
첫 추론까지 검증한 결과가 아니다. 모델 오류 시 색상 인식으로 대체하지
않으며 perception 또는 MoveIt process 종료 시 launch도 종료한다.
실제 입력 검출 결과가 없으면 실패로 보고한다. grasp 생성은 여전히
기하학 predictor이며 AnyGrasp 학습 모델로 바뀐 것은 아니다.

카메라와 알고리즘의 경계는 설정된 RGB/정합 depth/CameraInfo/TF 토픽이다.
기본은 `/camera/color/image_raw`, `/camera/color/camera_info`,
`/camera/aligned_depth_to_color/image_raw`; 정합 depth에는 동일한 color
CameraInfo를 사용한다. MuJoCo backend도 이 토픽에 직접 remap한다.
RGB와 depth는 같은 optical frame/해상도/intrinsics 및 동일 timestamp가
필요하고, depth는 정류되어 있어야 한다. 실 D435의 namespace, 정합,
CameraInfo와 timestamp 계약은 실제 드라이버에서 확인해야 한다.

이미 실제 로봇의 센서/관절/TF driver가 동작할 때에는 같은 구성에서
`start_simulator:=false use_sim_time:=false`를 선택하고 camera 토픽 인자만
드라이버에 맞춘다. 이 경우 MuJoCo 및 demo 고정 camera TF를 시작하지 않는다.
센서 지도와 plan-only가 아닌 외부 로봇 실행은 preflight에서 거부한다.
현재 URDF/calibration/controller 계약과 실제 하드웨어 호환성은 미검증이다.
gripper 강제 완전 닫힘도 기본 경로에서 제거했다.

이 launch는 비대칭 fixed/moving jaw 보정 때문에
`grasp_closing_sign_invariant=false`를 사용한다. 이 값은 **pregrasp와 grasp
양쪽** FK closing 축 검사에 적용된다. 접근축만 같고 closing 축이 뒤집힌
pregrasp를 허용하면, 접근 중 손목을 반 바퀴 돌리면서 측면 보정의 방향도
반전된다. 대칭 gripper용 범용 기본값 true의 unsigned 축 검사는 유지한다.

`depth_octomap_plugin`은 MoveIt launch로 전달되며 기본은 표준 updater다.
화면 없는 시뮬레이션 진단에서만 명시적으로
`cleany_scene_mapping/KnownGeometryOctomapUpdater`를 선택해 알려진 형상
내부의 가려진 점유 잔상을 정리할 수 있다. 형상 밖/부분 겹침 셀을 비우지
않는 실험 기능이며 `cleany_scene_mapping/README.md`의 한계가 적용된다.
실제 로봇의 안전 기능으로 검증한 것은 아니다.

기존 static-scene launch를 종료한 뒤 새 MoveIt 프로세스로 실행한다.
`require_sensor_scene`가 켜지면 인식 시작 및 후보 선택 직전에
`/monitored_planning_scene`에서 받은 비어 있지 않은 OctoMap과 최근 2초
이내의 `/perception/scene_cloud_filtered`를 확인한다. 지도 메시지도 2초
이내에 수신되어야 하며, map이 없는 state-only diff는 지도 시각을 갱신하지
않는다. 없거나 오래되면 실패한다. 초기에는 startup timeout 내에서 기다린다.
센서 전용 launch에서는 전용 `scene_cloud_receipt_node`가 전체 필터링
점군을 받고 원본 capture Header를 RELIABLE depth 1로 전달한다. coordinator는
`sensor_scene_receipt_topic`을 구독해 이 촬영 시각을 검사하며, 큰 점군을
추가 복사하지 않는다. 원본 점군이 없으면 receipt를 만들지 않고 timestamp를
현재 시각으로 덮어쓰지도 않는다. 직접 node 실행에서 해당 parameter가
빈 문자열이면 기존 PointCloud2 구독을 유지한다. 실패 시 cloud capture age와
map receipt age를 따로 기록하며 2초 기준은 두 경로에서 동일하다.
Humble에서 live OctoMap을 service로 반복 직렬화하는 동안 관측한 crash를
피하기 위해 readiness 검사에 `/get_planning_scene` polling을 사용하지 않는다.
`plan_only`에서는 후보가 선택되어도 pregrasp/닫기/lift를 실행하지 않고,
MoveIt trajectory execution도 꺼진다. 센서가 보지 못한 공간의 안전 정책,
동적 장애물 정지, target-contact map 처리는 아직 완료되지 않았으므로
이 모드를 실환경 자율 집기 완료로 해석하지 않는다.

`sensor_scene:=false plan_only:=false`는 이전의 사전 환경 박스 기반
시뮬레이션 실행 경로를 명시적으로 선택한다. 실환경 검증용이 아니다.
색상 fixture가 필요한 경우에만 detector/segmenter를 `simulation_color`,
`preload_models:=false`로 명시한다. 다른 legacy demo/test launch는 이
실로봇 지향 기본 실행 명령과 별개다.

## 시뮬레이션 분리 수거 (통합 검증 진행 중)

분리 수거 모드는 selector의 `require_pregrasp_visibility=true`를 사용한다.
관측한 OBB 8개 모서리를 현재 camera TF 기준으로 원근 투영하고, 이를 포함하는
16면 visibility cone을 구성한다. MoveIt `GetStateValidity`의
`VisibilityConstraint`가 준비 자세의 로봇 CAD와 이 영역의 겹침을 검사한다.
가리는 자세는 움직이기 전에 제외하고 다른 IK/후보를 검토한다. 반경은 물체
정답 크기나 고정 컵 치수가 아니라 RGB-D OBB에서 계산하며 기본 padding은
3 mm다. 동적 camera TF는 기본 0.5초 이내여야 하고, 검사 결과가 누락되면
통과로 취급하지 않는다. 일반 selector의 기본값은 false다.

이 검사는 base에 고정된 카메라를 사용하고 선택 중 base/camera가 움직이지
않는 조건의 **pregrasp 끝점 자기 가림 검사**다. 이동 경로 전체, 외부 물체에
의한 가림, 실제 영상의 detector 성공 또는 FOV 포함을 보장하지 않는다.
그리퍼가 물체에 접근하면 가림이 불가피하므로 최종 grasp에는 이 제약을
적용하지 않는다. 기존 충돌 검사와 이동 후 fresh YOLOE/SAM2 재검증은 유지한다.
([사용 버전 MoveIt 구현](https://github.com/moveit/moveit2/blob/2.5.9/moveit_core/kinematic_constraints/src/kinematic_constraint.cpp))

`sorting_contact_diagnostics:=true`는 MuJoCo observer의 접촉 부위/힘
출력만 켠다. 기본은 false이며, controller 오차와 실제 접촉을 대조하는
진단용이다. coordinator나 grasp selector는 이 GT 토픽을 읽지 않는다.

```bash
make sim-mujoco-sorting
# 같은 실행에서 GUI 창만 생략 (vendor 카메라 렌더링에는 DISPLAY 필요)
make sim-mujoco-sorting SORTING_ARGS='headless:=true use_rviz:=false use_image_view:=false'
```

`study_cafe_sorting.launch.py`는 Gemini Flash-Lite bbox + SAM2 RGB-D 인식/MoveIt
경로에 `sorting_coordinator`를 연결한다. 현재 구현은 시뮬레이션 전용이며
실제 두 분류의 집기·놓기 통합 성공은 아직 검증 중이다. 기존
`make sim-mujoco-pipeline`의 plan-only 기본값은 유지한다.
2026-09-07 headless 검증 attempt 33에서는 corrected moving-jaw 모델,
known-geometry updater와 명시 offset 0.030 m로 컵을 잡은 채 역접근에 성공했다.
실제 컵 중심이 약 168 mm 상승했지만 이후 추가 수직 LIN의 방향 불일치로
중단됐다. 상대 상승량 검증을 추가한 attempt 34는 지도 최신성 검사에서
막혔다. 4-worker self-mask를 적용한 35는 최신성/후퇴를 통과했고 실제 컵이
약 167 mm 상승했지만 held-view YOLOE 재검출 실패로 높이 검증에서 중단됐다.
640/960/1280과 cup/mug 표현의 저장 영상 시험도 이 재검출을 해결하지 못했다.
36에서는 현재 높이의 재관측 후보가 전체 물체를 화면에 담지 못했다.
낮은 관측 높이를 추가한 37은 실제 재관측 이동을 실행했으나 컵이 미끄러져
테이블로 떨어졌고 contact 검사에서 중단됐다. payload 속도를 낮춘 38/39는
각각 초기 IK 응답 초과/attachment 후 지도 갱신 공백으로 재관측에 미도달했다.
따라서 낮춘 속도의 지속 파지 효과도 아직 검증하지 못했다.
보유 검증을 생략하거나 confidence를 낮추지 않았다. 두 수거함으로 운반/놓기와
물리적 정착은 여전히 미검증이다. 관련 단위 테스트 통과를 수거 성공률로
해석하지 않는다. 실행별 기록은 `artifacts/sorting_20260907/README.md`에 있다.

손목 관측을 끄고 `sorting_use_reference_observation=true`를 선택하면 grasp 직전 검출기의
snapshot/object를 `/perception/observe_object_reference`에 PIN한다. lift 후에는
새 RGB-D/capture TF와 SAM2 reference mask로 **관측 표면** 높이를 확인한다.
새 의미 검출로 가장하지 않으며 원본 label/confidence/source identity와 요청 이후
capture stamp를 검증한다. 탁자 평면을 채우는 supported OBB 복원은 이 단계에서 쓰지
않는다. 상대 상승량 기준과 gripper contact, 중심 association 허용치는 유지한다.
기본 service timeout은 `sorting_reference_timeout_sec=30.0`초이고 만료·깊이·TF·출처
검증 실패는 중단한다. release 후 reference를 CLEAR한다. 비활성화 설정은 기존
현재 검출기의 재검출 경로를 명시적으로 선택하는 진단 옵션이며 자동 fallback이 아니다.

41 실행에서는 PIN 이후 새 SAM2 RGB-D 관측이 실제로 성공해 탁자에 남은 컵의
표면 높이 약 0.396m를 반환했다. 후퇴 초기에 실제 파지가 풀려 접촉 gate에서
중단했으며, 양쪽 수거함 운반·놓기 성공을 증명한 실행은 아직 없다.

MoveIt adapter의 `First ranked grasp/pregrasp` 로그는 실제 반환 순서의 첫 IK
해의 오차를 기록한다. 예전 `Accepted grasp` 로그는 joint-motion 우선 순서와 달리
최소 방향 오차만 출력하므로 선택된 해의 자세로 해석하면 안 된다. 첫 해도 이후
충돌/계획 검사에서 탈락할 수 있어 최종 선택은 action result의 joint state가 근거다.

Study-cafe launch의 `gripper_open_position_rad`(기본 1.2rad)로 실행 전 개방 폭을
명시적으로 설정할 수 있다. Sorting은 홈에서 해당 개방까지의 sweep을 검사한 뒤,
그 상태로 pregrasp와 접근을 계획한다. 큰 물체 진입 실험용이며 close command의
보정식, 접촉 gate, joint/controller 허용치는 변경하지 않는다.

Sorting selector는 `joint_limit_margin_rad=0.005`로 arm IK endpoint에 관절 상한/하한
여유를 둔다(일반 selector 기본 0). 경계에 포화된 해는 순위 목록과 state validity
앞에서 제외한다. 43의 pregrasp가 손목 상한에 정확히 놓인 뒤 feedback가 12.44µrad
넘어 Cartesian 시작 검증이 실패한 것을 방지하기 위한 선택 조건이다. 실제 feedback나
trajectory를 clipping하지 않으며 hard joint bounds/controller 허용치는 그대로다.

Study-cafe launch는 모든 child process 시작 전에 `fastdds_profiles_file`을
`FASTRTPS_DEFAULT_PROFILES_FILE`로 전달한다. 이미 지정된 환경변수를 우선 보존하고,
없으면 `cleany_perception/config/fastdds_rgbd.xml`(UDP + participant별 16 MiB SHM)을
기본으로 쓴다. 빈 launch 값은 사용자 기본 middleware 설정으로 되돌리는 진단 옵션이다.
다른 RMW의 동작을 바꾸는 설정이 아니며 QoS·freshness 기준은 변경하지 않는다.
41의 실제 집기 구간에서 지도 receipt 2초 초과가 없었으나 모든 부하/장치에서의 보장은
아니다. 별도로 띄우는 기록기/카메라는 같은 DDS 환경을 설정해야 비교할 수 있다.

`sorting_reobserve_after_lift=true`는 reference mask가 없거나 잘린 경우(기존 경로에서는
검출 대상이 없을 경우) 한 번의 카메라 재관측 이동을 허용한다. 정착한 실제 TCP와 관측 OBB로 파지
offset을 잡고, CameraInfo/TF의 중앙 영상 ray 3개와 관측 높이 평면들의 교점을
제안한다. 관측 OBB의 bounding sphere 전체가 화면 안에 들어오는 후보만 쓰며
`sorting_reobserve_margin_px=24`, `sorting_reobserve_geometry_padding_m=0.01`,
`sorting_reobserve_max_translation_m=0.20`으로 여백·최대 이동을 제한한다.
현재 높이에서 맞지 않으면 `sorting_reobserve_max_lowering_m=0.10` 범위에서
높이를 최대 3단계로 검사한다. 낮추더라도 필수 상대 상승량에
`sorting_reobserve_height_clearance_m=0.02`를 더한 높이 아래로 내려가지 않는다.
attached geometry를 보존하는 collision-aware IK와
일반 MoveIt 경로 계획·실행을 통과해야 한다. 이동 전후 gripper contact를 확인하고
이후 같은 선택된 관측 경로의 상대 상승량 검증을 다시 수행한다. 관측 재실패·재구성 실패·
관측 높이 부족은 성공으로 처리하지 않는다. `sorting_held_association_tolerance_m=0.03`
내에서 관측 중심과 파지 추정 중심이 일치해야 운반으로 넘어간다. 이 기능은
센서 기반 재관측 동작이며 MuJoCo GT나 자동 물체 부착을 사용하지 않는다.
실제 재관측 성공 여부는 실행 기록으로 별도 검증해야 한다.
재관측/수거함 운반의 일반 joint-goal 이동에는 파지 중에만
sorting launch에서는 `sorting_payload_velocity_scaling=0.24`,
`sorting_payload_acceleration_scaling=0.20`를
적용한다. 기존 속도·가속도보다 높이는 설정은 적용하지 않으며 열린 그리퍼로
복귀할 때는 원래 값을 사용한다. 접근/역접근의 Cartesian retiming은 별도로
유지한다. 노드 단독 기본값은 둘 다 0.01이며 launch 인수로 이전 값을 재현할 수 있다.
이는 시뮬레이션 시험 설정이며, 지속 파지 성공이나 실제 하드웨어 적합성을 뜻하지 않는다.

`config/sorting_policy.yaml`은 시뮬레이션 allowlist다. 종이컵/캔/휴지뭉치 등은 쓰레기,
지갑/레고 등은 분실물 후보로 분류한다(과거 휴대폰/지우개 label도 지원). 위험·미등록·저신뢰
물체는 review로 남기고 조작하지 않는다. 이 분류는 실제 컵의 소유권이나
일회용 여부를 판정하는 정책이 아니며 KB의 미정 정책을 확정하지 않는다.

현재 장면의 YOLOE label은 `cup`, `wallet`, `crumpled tissue`, `lego brick`이며
시뮬레이션 placement oracle의 body mapping도 이에 맞춰져 있다. 레고는 실제 2×4 크기
(몸체 31.8×15.8×9.6 mm, 돌기 포함 높이 11.4 mm)로, 기존 sorting의 50 mm 최소
파지 후보 폭 기준보다 작다. 크기를 부풀리거나 후보 기준을 자동으로 완화하지 않았다.
따라서 인식/분류 대상이어도 자동 집기는 제외될 수 있으며 별도의 소형 물체 파지 검증이
필요하다. 이 문서의 기존 성공·속도 측정은 교체 전 머그컵 장면 결과로, 새 종이컵과
휴지/레고의 수거 성공을 뜻하지 않는다. 이번 형상 교체에서는 모델 재학습이나 집기 성능
재측정을 하지 않았다.

수거 위치는 로봇 뒤쪽 +Y(좌측)의 `lost_items_left`, -Y(우측)의
`trash_right`다. `cleany_mujoco_sim/config/robot_top_bins.yaml`은
수거 위치의 알려진 보정값/형상만 공유한다. 책상과 집을 물체의 위치/형상은
카메라에서 얻으며 scene의 target 좌표를 planner에 전달하지 않는다.
수거함은 바닥과 4개 벽으로 구성되고 MoveIt에도 같은 목적지 형상을 등록한다.
후면 선반의 상판·하단판·다리 4개도 같은 설정으로 MoveIt 충돌 형상에 등록한다.
이 좌표는 현재 고정 베이스 시뮬레이션 전용이며 이동 베이스 운용을 보장하지 않는다.

분류 → 목적지까지 도달 가능한 팔로 후보 선택 → 재인식/집기/후퇴/들기 →
부착 물체를 포함한 MoveIt 운반 → 수거함 개구부 확인 → 열기 → 복귀 →
놓기 검증 순서다. 운반 실패 시 그리퍼를 열지 않는다.
sorting 모드에서는 다각도 grasp 후보와 전체 팔 IK seed를 추가로 탐색하되
기존 pose/충돌 허용 기준은 유지한다. 운반 IK에는 전체 joint feedback과
`RobotState.is_diff=true`를 보내 부착한 OBB가 검사에서 빠지지 않도록 한다.
CPU 시뮬레이터에서 열기 중 제어 주기 지연으로 추종 오차가 발생한 사례에
대응해 gripper motion은 8초로 설정하며, 경로 오차 기준은 그대로 유지한다.
action 응답 대기 시간도 설정한 motion duration에 비례한다.
첫 물체를 고르기 전에 초기 자세에서 그리퍼 열기 구간을 0.05 rad 이하
간격으로 충돌 검사하고 연다. sorting의 pregrasp 도착 후에는
열기 명령을 반복하지 않고 정지 확인·최신 인식·집기 재계산으로 진행한다.
따라서 기존의 중복 열기 8초가 제거되며, 놓기 시 개방은 유지된다.
이후 후보 선택은 열린 jaw의 실제 feedback을
사용한다. 이 이산 검사는 연속 충돌 검사의 대체물은 아니다.
재검출한 후보를 selector에 넘기기 전에 이전 snapshot의 target OBB를
제거한다. 같은 물체가 서로 다른 ID의 충돌체 두 개로 중복 등록되어
접촉 허용이 한쪽에만 적용되는 것을 방지한다.
닫기 직후 접촉 신호가 있더라도 settle 후 잔여 각도/속도 조건을 다시
확인하며, 유지되지 않으면 부착 OBB를 만들거나 lift로 진행하지 않는다.
재선택한 grasp가 현재 팔 위치에서 LIN 방향 기준을 넘으면, 반환된 새
pregrasp로 충돌 검사 이동한다. 이 이동 중 target 접촉은 허용하지 않는다.
이후 새 RGB-D로 동일 물체의 OBB 모서리 집합이 기본 5 mm 이내에서 유지되는지
확인해야 접근한다. 없거나, 오래됐거나, 여러 물체가 일치하면 중단한다.
기존 LIN 10도 기준은 그대로이며, 보정 이동 후에도 다시 검사한다.

Sorting은 `use_joint_corridor_grasp=true`, `use_seeded_cartesian_grasp=true`로
선택된 grasp **관절값**을 보존하는 접근을 사용한다. TCP는 실제 시작점과
목표 사이의 반경 1 mm 원통 안으로 제한하고, 역접근은 실제 접근 시작 관절값을
목표로 동일하게 계획한다. 두 끝 관절값의 보간을 seed로 기본 10 mm 간격
collision-aware IK를 구한 뒤 단조 cubic 곡선을 구성한다. 모든 중간 검사는
전체 관절 피드백과 현재 attached body를 보존한다.
이는 5-DOF 팔에서 임의의 6-DOF 방향을 정확히 보간하는 Pilz LIN과 동일하지 않다.
반환된 모든 waypoint를 FK로 검사해 원통 이탈/역행을 거부하며, 시작·끝 방향은
0.01 rad 이내, 중간 방향은 두 끝 자세의 SLERP에서 기본 5도 이내여야 한다.
충돌·목표 접촉 허용 정책과 controller/contact 한계는 변경하지 않는다.
관절 속도와 가속도의 곡선 전체 상한을 분석해 시간을 정하고, 실제 JTC cubic
보간 곡선을 기본 1 mm 분할 및 관절별 0.005 rad 이하 간격으로 표본화해
MoveIt 충돌/constraint/FK 검사를 거친다. 최대 2000점 초과 시 중단한다.
JTC에 position+velocity를 전달하며 acceleration 필드는 비워 동일한 cubic
보간을 유지한다. 모델 관절 한계는 `cartesian_joint_limits_file`(기본 MoveIt
`joint_limits.yaml`)에서 읽는다. 가속도 미정 관절은 명시 파라미터
`cartesian_joint_acceleration_rad_s2=1.0`에 실행 scaling을 적용한다. 이는
MoveIt 기본 가속도와 같은 시험용 상한이지 하드웨어 검증값은 아니다.

추가로 FK sample의 병진/회전 속도와 병진 가속도를 기준으로 trajectory
시간만 균일하게 늘린다. 기본 Cartesian 상한은
0.10 m/s, 0.50 rad/s, 0.20 m/s²에 실행 scaling을 적용하고, 추가 시간 여유는
`corridor_time_margin=2.0`이다. 위치/경로는 바꾸지 않고 velocity/acceleration도
같은 비율로 줄인다. 이 FK 표본 검사는 연속 시간 속도/충돌 또는 실제 하드웨어
안전 인증을 대신하지 않는다. `use_seeded_cartesian_grasp=false`이고 joint
corridor만 켜면 기존 OMPL 계획+FK 검사 경로를 사용한다. 일반 데모의 두
모드는 opt-in이며 수직 lift는
여전히 endpoint 검사를 통과한 Pilz LIN을 사용한다.

sorting launch의 현재 기본값은 `approach_velocity_scaling=1.0`,
`retreat_velocity_scaling=0.8`, `corridor_time_margin=1.05`이다. 같은 이름의
launch 인수로 조절하며 일반 데모는 기존 0.2/0.4/2.0을 유지한다.
관절 속도·가속도 한계, 충돌/경로 검사, controller tolerance는 완화하지 않는다.
sorting joint-corridor의 grasp 접근에만 `sorting_approach_acceleration_scaling=1.0`을
적용한다(기존 0.4). 계획과 FK sample 시간 배율 검사에 같은 값을 사용한다.
관절 자체 한계는 제거하지 않는다. sorting Cartesian fallback 가속도는 4 rad/s²,
별도 gripper motion은 2초, LIN 가속 배율은 0.8이다. MoveIt TOTG의 URDF 미정
관절 fallback과 Cartesian fallback은 서로 다른 경로다. 현재값 전체는 상단
속도 표를 따른다. 실기 검증값은 아니다.

sensor-scene 모드에서는 OBB attach 성공 뒤 **그 시점보다 새로운 촬영 시각**의
처리 완료 depth receipt를 기다린다. `attachment_scene_timeout_sec=5.0` 안에
오지 않으면 후퇴를 시작하지 않는다. 기존 capture/map age 2초 조건도 유지한다.
이 barrier는 새 frame 통합 전 계획을 막지만 지도 잔상이 모두 없어졌음을
보증하지는 않는다.

Sorting selector는 `align_grasp_wrist_roll=true`도 사용한다. 현재 Cleany
URDF의 wrist-roll 축과 TCP offset이 local -Y로 일치하므로, position IK가
찾은 해의 closing 방향을 이 축 둘레로 직접 맞춘 후보를 만든다. 관절 범위
안의 동치 각도만 검사하며 비대칭 jaw의 180도 반전을 허용하지 않는다.
회전한 후보는 전체 robot state 충돌 검사와 기존 FK 위치/approach/closing
허용 기준을 다시 통과해야 한다. 회전은 **계획 후보**에만 적용하며 실제
로봇/물체를 강제로 회전하지 않는다. 모델-parity 테스트가 축/TCP 가정을
검사하며 일반 selector 기본값은 false다.
부착 해제 시에는 MoveIt이 world에 다시 만든 임시 OBB까지 순차적으로
제거한다. `/sorting/status`의 실패 메시지에는 별도 `error`가 포함된다.
`sorting_artifact_directory:=/absolute/path`를 launch에 지정하면 실행마다
새 `run-<uuid>` 하위 폴더에 `inspection`(InspectScene.Result),
`grasps`(PlanGrasp.Response), `selection`(SelectReachableGrasp.Result)을
CDR로 저장한다. fresh depth/3D 추정과 실제 사용한 후보를 오프라인에서
비교하기 위한 출력 전용 옵션이며, 저장 데이터를 제어 입력으로 재사용하지 않는다.
기본값은 빈 문자열로 저장하지 않는다.
접근/후퇴/lift의 `motion_request`(MoveGroup.Goal)와 `motion_result`
(MoveGroup.Result)는 계획 실패/실행 전 거부 시에도 저장한다.
seeded 방식에서는 요청의 pipeline/planner ID가 `cleany_seeded_cartesian` /
`monotone_cubic`이며, 이는 계획 입력을 같은 형식으로 기록한 것이다.
해당 가상 pipeline ID를 MoveGroup 서버에 전송하지 않는다. 결과의 trajectory와
planning_time은 실제 IK/검증 계산으로 작성하며 실패도 기록한다.
실행 전 FK 검증을 통과한 trajectory는 `motion_plan` 이름의
`moveit_msgs/RobotTrajectory` CDR로 저장한다. 이 출력에는 corridor 시간 보정이
반영되며, 저장 자체가 물리 실행/파지 성공을 뜻하지는 않는다.
`grasp_approach_offset_m` launch 인수(기본 0.016 m)는 selector의 IK 목표와
coordinator의 실행/표시 보정에 동일하게 전달한다. 회전식 비대칭 jaw의
벌림에 따른 실제 접촉 깊이를 보정하는 시험용 입력이며, 현재 기본값은
실측 하드웨어 보정이나 지속 파지 성공으로 검증된 값이 아니다.
같은 옵션은 selector의 출력 전용 service trace도 활성화한다. 별도
`services-<uuid>` 폴더에 각 MoveIt 서비스 요청/응답 CDR과 완료 시간,
취소 여부 JSON을 저장한다. 실패한 IK 요청의 seed/목표를 재현하기 위한
진단이며 응답이나 계획을 수정하지 않는다. 이 옵션의 파일 I/O가 포함된
시간을 저장 기능을 끈 운용 성능으로 해석하지 않는다.
`/sorting/status`는 단계, label, 분류, 목적지, 완료 항목을 JSON String으로
latched 발행한다. 기본 완료 조건은 쓰레기/분실물 경로를 각각 검증하는 것이다.
두 경로가 검증되지 않았으면 `mission_complete`를 발행하지 않는다.

`/sorting/verify_placement`는 독립된 결과 검증 port다. 현재는 시뮬레이션
oracle이 release 이후의 새 물체 AABB가 올바른 수거함 내부에서 안정적으로
정지했는지 확인한다. 이 oracle은 인식·분류·grasp·경로 계산에 사용하지
않는다. 실제 로봇에는 별도의 센서 기반 결과 검증 구현이 필요하다.

## 관련 KB

### 책상 4분할 정리 파이프라인 (2026-09-08, 검증 진행 중)

`sim_speed_factor`는 기본 1.0이다. CPU SAM2가 실시간을 따라가지 못하는 VM의
기능 검증에는 `sim_speed_factor:=0.5`로 물리 시간만 감속할 수 있다. 추적 freshness
한도는 유지한다. 감속 시험의 wall time은 실제 로봇 처리 성능으로 해석하지 않는다.
`tcp_fk_timeout_sec=5.0`은 깊이 지도 갱신과 경합하는 MoveIt FK 조회의 wall timeout이다.
Seeded Cartesian FK/IK/충돌 조회와 sorting transport adapter 응답 여유에도 같은 값을
사용한다. IK 알고리즘 자체의 계산 제한은 늘리지 않고 응답 전달 대기만 분리한다.
기존 2초 제한에서 2.17초 지도 callback 때문에 실패한 사례를 반영하며 위치 오차,
파지/추적 조건을 완화하거나 성공 응답을 생략하지 않는다.
Sorting의 `cartesian_local_refinement_iterations=40`은 runtime URDF를 MoveIt FK와
대조한 뒤, Cartesian 중간점을 관절 endpoint 보간 seed 주변에서 수치 보정한다.
위치 오차에 비해 약한 관절 regularization으로 불필요한 redundant-joint 변화와
branch jump를 줄인다. 각 knot 및 실행할 cubic 보간 샘플의 MoveIt 충돌·corridor·FK
검사는 유지하며 실패 경로는 실행하지 않는다. 기본 node 값 0은 기존 서비스 IK다.

`study_cafe_sorting.launch.py`와 일반 파이프라인은 `robot_top_bins.yaml`의
후면 선반·수거함을 기본 배치로 사용한다. 책상 양끝 분류 영역과 구분선은
없으며 이전 배치 설정은 삭제했다. 후면 배치로의 운반 동작은 아직 검증하지
않았으므로 배치 변경을 수거 성공으로 해석하지 않는다.
`table_sorting_policy.yaml`의 destination으로 Gemini의 category/reason을
연결한다. label allowlist로 category를 대체하지 않으며 위험 label은 보수적으로
거부한다. RGB-D 거리순으로 시도하고 인식된 수거 구역 물체는 다시 집지 않는다.
미처리/미확인 물체가 남으면 성공으로 보고하지 않는다. 관측한 작업 물체 수와
검증된 최종 배치 수를 대조하므로, 이전 물체가 인식에서 사라졌다고 완료되지 않는다.
임시 handoff 물체도 최종 배치 전까지 미완료로 유지한다. 빈 작업 영역은 두 번
재확인하며 최대 search/action 횟수 12에 도달하면 실패한다.

배치 후보는 구역 안에서 가까운 위치부터 IK/충돌 검사한다. 물체 크기와 이미
배치한 footprint를 이용해 슬롯 중복을 피하며 GT pose로 경로를 만들지 않는다.
MuJoCo oracle은 배치 후 전체 AABB가 수거함 안에 들어가 정지했는지 검증한다.
기본 `sorting_exit_on_finish=true`; coordinator 종료 시 launch가 다른 노드도
종료한다. GUI를 유지해야 할 별도 디버그 실행은 이 파라미터를 false로 설정한다.
현재 전체 물리 정리 성공은 아직 검증 중이다. 기록: `artifacts/table_sorting_20260908/`.
sorting은 얇고 긴 객체의 RGB-D 점군에서 중심 외의 longitudinal contact 후보도
생성한다. 자세/충돌 한계는 유지하며, 세부 범위는 cleany_grasping README를 따른다.
sorting selector/executor는 `support_patch_margin_m=0.02`로 perception OBB
바닥의 지지면을 유한 collision patch로 유지한다(일반 node 기본 0, 비활성).
patch는 OBB XY 범위에 각 방향 2cm를 더하고, OBB 바닥 아래 1cm 두께로 만든다.
2cm는 현재 target mask padding 1.5cm를 포함하는 로컬 범위다. 8cm 확장은
가까운 물체에서 로봇 본체까지 가상의 면을 확장하는 문제가 있어 사용하지 않는다.
이는 perception이 지지면 법선을 OBB Z축으로, 지지면을 바닥으로 쓰는 계약에
의존한다. 수평에서 20°를 넘는 면/잘못된 pose는 거부한다. 새 테이블 정답은
읽지 않으며, 관측 밖 연장 부분은 보수적인 로컬 평면 가정이다.
물체/support pair만 접촉 허용하고 robot/support는 허용하지 않는다. attachment
이후에도 patch가 남아 target OctoMap masking 아래의 지지면이 사라지지 않으며,
transaction 종료 시 target과 함께 제거하고 원래 ACM으로 복원한다.
`require_gripper_closure_clearance=true`는 실제 full-robot 형상으로 open→close
구간을 `gripper_sweep_step_rad=0.05` 이하 간격에서 검사한다. target 접촉만
허용하고 환경/지지면 충돌은 거부한다. 이 검사는 기하학적 샘플 검사이며 접촉
동역학/실제 파지 유지 성공을 보장하지 않는다. grasping의 support-plane 간이
검사 위임은 이 support patch 및 open/closure 검사와 함께만 사용한다.
seeded cubic 경로가 충돌로 거부되면 최대 8개 contact의 body pair, 위치/frame,
penetration depth를 오류에 기록해 주변 물체/지지면/잔상 원인 분석에 사용한다.
손목 전환 직전 head reference가 `sorting_head_reference_refresh_age_sec`(30초,
simulation/ROS clock 기준)를 넘으면 head를 활성화하고 기존 재인식·위치 연관·
동일 팔 재계획 경로로 새 reference를 얻는다. 새 계획도 30초를 넘겼거나 분류가
바뀌어 review/다른 목적지가 되면 중단한다. 미래/누락 stamp는 거부하며 기존
wrist service의 60초 상한을 늘리거나 과거 관측의 stamp만 갱신하지 않는다.
신선한 reference는 추가 head 재인식 없이 기존처럼 wrist로 바로 전환한다.

sorting selector의 `pose_refinement_iterations=30`은 runtime `/robot_description`
URDF의 로컬 FK로 위치와 접근/closing 방향의 잔차를 함께 줄이는 bounded numerical
refinement를 사용한다. 매 후보 탐색 전 MoveIt FK와 일치하는지 대조한다.
초기 자세 예산은 pregrasp에서 `pregrasp_aim_attempts`, grasp에서 0이 아닌
`grasp_pose_seed_attempts`를 따른다(0이면 pregrasp 예산 재사용). 수치 보정도
설정 예산 전체를 사용하며, 첫 유효 해에서 종료한다. seed 수 증가는 실패
후보의 계획 시간을 늘릴 수 있으나 유효성 허용값을 변경하지 않는다.
기본 adapter 값 0은 기존 탐색이다. 각도 오차 기준, 관절 여유, 충돌검사 및
실행 전 경로 검증은 유지한다. 후보 보정 자체는 로봇에 명령을 보내지 않는다.
실제 grasp는 `grasp_position_tolerance_m=0.0015`로 별도 검사한다. pregrasp의
20mm 허용값을 grasp에도 공유하던 오류를 수정했으며, 관절 자세가 맞더라도
TCP 위치 오차가 1.5mm를 넘는 후보는 거부한다. `pose_refinement_position_weight=10`
으로 위치 잔차를 우선 줄인다. 각도/충돌/관절 한계는 완화하지 않는다.

현재 후면 배치에는 중앙 인계 구역(`handoff_center`)이 없다. 과거의 반대 팔 →
중앙 임시 배치 → 목적지 쪽 팔 인계 경로는 현재 활성화되지 않는다. 팔 선택은
아직 수거함의 Y 부호로 고정하므로 반대편 물체가 `unreachable`로 남을 수 있다.
후면 배치의 양팔 선택/운반 전략과 전체 수거 성공은 미검증이며 GPU 이식만으로
해결되지 않는다. `gripper_wall_timeout_factor=6`은 느린 시뮬레이터의 실제시간
timeout 여유이며 gripper 궤적 시간을 늘리는 대기 설정은 아니다.

`grasp_use_aperture_centering=true`는 검출 폭에서 opening margin 8mm를 제외한
물체 폭을 사용해 `폭/2 - fixed_jaw_inner_x(8mm)`로 TCP 옆방향 보정량을 정한다.
계획과 실행이 같은 함수를 사용한다. 기존 30mm 고정 보정은 작은 물체가 두
손가락 사이에서 벗어나는 문제가 있었다. 시뮬레이션 jaw mesh 기준으로 닫힘
근방 aperture 기울기 0.065m/rad, command opening reduction 18mm를 사용한다.
관절 정지로 파지를 확인하지 못하면 `gripper_contact_retry_steps`(기본 node 0,
sorting launch 5; 허용 0–5)만큼 `gripper_contact_retry_step_rad=0.10`씩 더 닫는다.
각 재시도는 `gripper_contact_retry_motion_sec=1.2`이며 닫힘 하한과 기존 토크 제한을
넘지 않는다. 최초 열린 위치 기준의 접촉 판정을 유지하고, 실제 마지막 명령을
운반 중 감시에도 사용한다. 한도 내에서 파지를 확인하지 못하면 lift 전에 실패한다.
이는 실제 하드웨어 캘리브레이션 값이 아니며 물리 파지 성공은 별도 검사한다.

연속 sorting에서는 `gripper_force_full_close=true`로 기존 닫힘 하한을 목표로
유지한다. 접촉 때문에 실제 관절은 그 전에 멈출 수 있으며, 기존 토크 제한과
관절 잔차/속도 기반 접촉 감시는 그대로 적용한다. 추정 폭의 중간 목표에서 멈춰
물체 자세 변화 후 추가로 조여주지 못하는 현상을 피하기 위한 설정이다.
목표까지 완전히 닫혀 잔차가 사라지면 파지 성공으로 간주하지 않는다.
초기 파지에는 정지 속도 조건을 적용한다. 이미 확인된 파지의 비동기 운반 중에는
닫히는 방향의 추가 조임을 허용하되, 목표 대비 최소 잔차/열리는 속도 상한,
관절 피드백 신선도, 손목 추적과 접촉 상실 debounce는 계속 검사한다.
이는 힘 센서 확인이 아니며 잔차가 사라지기 전의 미끄러짐을 완전히 판별하지 못한다.
sorting의 attachment 후 지도 갱신 대기 상한은 10초다. CPU 저속 시뮬레이션에서
새 지도 발행을 기다릴 시간을 확보하되, 신선도 2초와 attachment 이후 새 depth
capture 조건은 완화하지 않는다. 새 데이터가 준비되면 즉시 진행한다.
변화 없는 OctoMap 전체 메시지가 재발행되지 않아도, tree 쓰기 뒤 발행되는
processed-cloud receipt의 새로운 capture stamp로 지도 활동을 확인한다.
기존 populated-map 확인과 capture/receipt 신선도 기준은 유지하고, 원본 cloud
수신이나 중복 receipt만으로 갱신하지 않는다. 빈 지도도 통과시키지 않는다.
책상 구역에는 `sorting_table_release_clearance_m=0.015`를 따로 사용한다.
기존 bin의 60mm 여유 대신 회전 안전 bounding sphere 아래 15mm 여유로 배치해
낙하 거리를 줄인다. 이 값은 center IK 허용오차 10mm보다 커야 한다.
구 자체가 보수적인 경계이므로 실제 물체 바닥은 더 높을 수 있으며, 이 설정만으로
직립 자세나 무낙하 배치를 보장하지 않는다. 최종 안착은 별도 검증한다.
sorting selector는 `require_open_grasp_clearance=true`로 실제 열림 각도의 grasp
끝점도 검사한다. 센서 OBB에 대한 jaw 접촉 허용을 잠시 해제하고, 선택된 gripper
관절만 열림 각도로 바꾼 전체 로봇 MoveIt validity를 확인한다. 기존 피드백/계획은
변경하지 않고 ACM은 성공/실패/예외 모두 복구한다. 이 검사는 시작/중간 경로의
추가 안전 검증을 대체하지 않으며, 보수적인 OBB 때문에 유효한 후보도 거절할 수 있다.
고정 jaw에는 sorting에서 `grasp_fixed_jaw_clearance_m=0.003`을 적용해 경계를
맞닿게 두지 않는다. selector와 coordinator에 동일하게 적용하며, 설정 여유는
opening margin의 절반 이하여야 한다. 기존 비-sorting 기본값은 0이다.
`direct_vertical_lift:=true`는 기본 false인 동작 진단 옵션이다. 접근 경로를
역으로 물러나는 단계 대신 현재 TCP 위치에서 수직 lift를 계획한다.
sorting에서는 끝점 position IK/FK를 확인하고 기존 seeded 위치 corridor로 계획한다.
기존 corridor 자세 한도(5°)를 넘는 끝점은 거절하고, 전체 경로의 FK/충돌/자세
변화를 검사한다. 정확한 자세 고정 LIN으로 표현하지 않는다. 파지/손목 감시는
유지하며 기본 전체 sorting 동작은 아직 역방향 retreat를 유지한다.

- [System Concept](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Concept.md)
- [Rule-based VLA Architecture](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Rule-based%20VLA%20Architecture.md)
