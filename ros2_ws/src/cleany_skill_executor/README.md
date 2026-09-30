# cleany_skill_executor

파지 후보 선택, MoveIt 계획, 시뮬레이션 집기·분류·놓기와 모의 Manipulation Action을 담당한다.
실물 명령을 지원하는 통합 운용 단계는 아니며 외부 로봇 경로는 plan-only다.

- [분류 실행](#시뮬레이션-분리-수거-통합-검증-진행-중)
- [분류·운반 실행 계약](#분류운반-실행-계약)
- [설정 및 검증](#설정-및-검증)
- [모의 Manipulation Action](#모의-manipulation-action)

## 모의 Manipulation Action

`manipulation_server`는 승인된 물체 하나의 `collect_trash`를 실행한다.
ROS wrapper와 순수 Python core, 동작 port, Mock adapter, SQLite 저장소를 분리했다.
20Hz steady-clock timer가 상태를 진행하고 Reentrant callback과 4-thread executor를
사용한다. 수락, 취소, 진행 변경과 최종 결과는 같은 lock으로 직렬화한다.
이번 `execution_profile`은 `mock`이며 다른 backend 설정은 시작 시 거절한다.
실제 팔과 그리퍼 명령, Perception 호출, Mission Manager·Backend 연결과 BT.CPP 실행은
후속 구현 대상이다. 모의 성공은 실물 수거나 안전 정지 검증 결과가 아니다.

| 기본 ROS 이름 | 타입 | 용도 |
|---|---|---|
| `/mock/manipulation/execute_skill` | `ExecuteManipulationSkill` Action | 요청, Feedback, Result와 취소 |
| `/mock/manipulation/get_execution` | `GetManipulationExecution` Service | execution_id로 최신 기록 조회 |
| `/mock/manipulation/execution_events` | `ManipulationExecutionRecord` Topic | 수락, 단계, 물리 근거, 종료와 복구 이벤트 |

Topic QoS는 Reliable, Transient Local, depth 100이다. 최신 100개 이벤트의
재수신을 제공하며 전체 이력은 SQLite `execution_events`에 보존한다.
`SUCCESS/BLOCKED`는 ROS `SUCCEEDED`, `FAILED/FATAL`은 `ABORTED`,
정지 확인된 취소는 `CANCELED`로 반환한다. 수락 전 ID·skill 오류, 동시 실행,
중복 execution_id와 기존 fault는 Result 없이 거절한다.

레포 루트에서 빌드 후 서버를 실행한다. 테스트 시 독립 임시 DB를 사용한다.

```bash
make build-manipulation
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
mock_db_dir="$(mktemp -d)"
ros2 launch cleany_skill_executor manipulation_mock.launch.py \
  database_path:="$mock_db_dir/executions.sqlite3"
```

다른 터미널에서 같은 setup을 읽고 클라이언트를 실행한다. 클라이언트는 UUID4를
발급하고 Feedback, Result 및 조회 결과를 출력한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run cleany_skill_executor manipulation_test_client
ros2 run cleany_skill_executor manipulation_test_client --cancel-stage TRANSPORTING
ros2 run cleany_skill_executor manipulation_test_client \
  --query --execution-id '클라이언트가 출력한 UUID'
```

launch 인자는 `namespace`(기본 `mock`), `database_path`, `mock_config`, `scenario`다.
실행 중 설정 변경은 지원하지 않으며 재시작 때 반영한다.
[`config/manipulation_mock.yaml`](config/manipulation_mock.yaml)은 기본 snapshot의 물체
1·2·3, 목적지 `mock_trash_bin`, 팔 `left`와 오류 시나리오를 관리한다.
기본 단계는 0.5초, 정지 확인은 0.1초, 준비·확인 제한은 각각 5초,
동작 제한은 단계별 10초, 정지 제한은 1초다. 모두 모의 검증용 수치다.
자동 재시도는 0회이며 snapshot age도 모의 fixture 값이다.

| `scenario` | 재현 상황 |
|---|---|
| `success`, `release_unobserved` | 정상 수거, 그리퍼 이탈 근거 없이 독립 확인까지 대기 |
| `backend_not_ready`, `verification_unavailable` | 시작 준비 차단 |
| `grasp_failure`, `grasp_lost` | 집기 실패, 운반 중 물체 상태 유실 |
| `placement_failure`, `verification_timeout` | 확인 실패와 판단 불가 |
| `return_failure` | 팔 복귀 실패, 독립 수거 근거 보존 |
| `stop_failure`, `stop_timeout` | 취소·실패 정지 확인 실패와 timeout |
| `hardware_fault`, `e_stop` | 로컬 fault와 신규 실행 차단 |

없는 snapshot/object, `mock-stale-snapshot`과 잘못된 destination은 Goal 인자로
재현한다. 오류 시나리오는 새 DB로 서버를 실행하며 `scenario:=grasp_lost`처럼 선택한다.
취소를 받으면 다음 동작을 시작하지 않는다. 집기·들기·놓기는 현재 원자 구간을
완료하거나 deadline에서 차단하고 정지를 확인한다. 물체를 들고 있어도 자동 놓기나
팔 복귀를 실행하지 않는다. 실제 fault는 취소보다 우선하며 정지 확인 실패는
`FATAL/STOP_UNCONFIRMED`다.

기본 DB는 `${XDG_STATE_HOME:-~/.local/state}/cleany/manipulation_mock/executions.sqlite3`다.
실행 ID UNIQUE 제약, DB별 파일 lock, SQLite WAL/FULL 동기화 및 snapshot·이력의
동일 트랜잭션으로 수락·단계 시작·확인된 물리 변화·Result를 보존한다.
프로세스 종료 후 같은 DB로 재시작하면 미완료 기록은 `INTERRUPTED`,
`human_confirmation_required=true`, `has_result=false`로 조회·이벤트에 제공한다.
마지막 확인 근거는 보존하고 현재 정지를 추정하지 않는다.
fault, 중단, 물체 보유 또는 상태 불명이 남으면 신규 Goal을 차단한다.
이번 버전에는 reset과 기록 만료가 없다. 기존 DB를 지우는 것으로 물리 복구를
완료했다고 간주하지 않으며 모의 테스트는 처음부터 독립 DB로 분리한다.

저장 실패는 `FATAL/INTERNAL_ERROR`로 처리하고 신규 실행을 차단한다.
함께 발생한 hardware fault·e-stop·정지 확인 실패의 `FATAL` 원인은 보존한다.
`RECORDING_FAILED` 조회·이벤트는 프로세스 메모리의 진단이며 영속 저장을 주장하지 않는다.
디스크에 남은 마지막 활성 기록은 재시작 때 중단으로 복구된다.
응답 유실·클라이언트 timeout 뒤에는 기존 execution_id를 조회한다.

```bash
make test-manipulation-core
make test-manipulation
```

첫 명령은 가짜 시계 core 검사다. 두 번째는 관련 패키지 빌드 후 실제 ActionClient로
정상·차단·실패, 모든 단계의 취소, fault, 동시·중복 요청, 저장 실패,
프로세스 SIGKILL 후 재시작 조회와 복구 이벤트를 검사한다.
기존 Skill Executor 회귀 검사는 native 환경에서
`python3 -m pytest ros2_ws/src/cleany_skill_executor/test`로 실행한다.

## 책상 정리 모듈 연동 설계

최신 KB의 [Mission Lifecycle](../../../docs/cleany-docs/20_TECHNICAL/09%20-%20Mission%20Lifecycle.md)과
[Task Planning and Robot Capabilities](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Task%20Planning%20and%20Robot%20Capabilities.md)를
기준으로 본다. Mission Manager가 행동 하나를 승인하고 실행 결과를 재관찰한다.
아래 명세는 승인된 `collect_trash` 행동 하나의 Action 계약과 후속 통합 설계를 정리한다.
접근, 집기, 들기, 운반, 놓기와 확인은 Skill 내부에서 처리한다.

읽는 순서:

1. [01. 설계 안내](docs/01_manipulation_action_design.md)
2. [02. Action 인터페이스 명세](docs/02_execute_manipulation_skill_action_spec.md)
3. [03. 서버 동작 명세](docs/03_manipulation_server_behavior_spec.md)
4. [04. Mission 결과 매핑](docs/04_mission_result_mapping.md)
5. [05. 관측 데이터 명세](docs/05_initial_table_observation_spec.md)
6. [06. BT와 Groot2 설계](docs/06_manipulation_behavior_tree_design.md)

현재 `ExecuteManipulationSkill.action`과 모의 서버·기록 조회는 구현되어 있다.
Mission Manager ROS adapter는 후속 구현 대상이다.
Groot2 XML은 `CollectTrashSkill`의 정적 미리보기이며 BT.CPP 실행기는 없다.
기존 simulation sorting coordinator는 전체 물체 반복을 포함하는 demo다.
모의 서버의 실행·검증은 위 절을 따르며 실물 확인 계약은 위 명세의 미결정 범위를 유지한다.

## 작업자 관찰 모드와 파지 깊이

시뮬레이터를 시작하는 수거 launch는 `sorting_verify_placement:=false`가
기본이다. 물체를 놓고 팔이 복귀하면 `VerifyPlacement` 서비스 호출/대기 없이
다음 물체 탐색으로 넘어간다. 시작 시 해당 서비스 준비도 기다리지 않는다.
결과는 `complete_unverified`, `placement_verified: false`로 기록하며 자동
안착 검증 성공으로 취급하지 않는다. 정상 종료 단계도 `mission_complete_unverified`
이다. 실제 안착 여부는 작업자가 GUI로 확인한다.
기존 자동 검증은 `sorting_verify_placement:=true`로 복원한다.
이 변경은 운반 중 접촉 이탈 감시, 안전 정지, 충돌 검사, 다음 물체를 찾는
카메라 인식 및 최종 작업 영역 비움 판정을 제거하지 않는다.

휴지(`crumpled tissue`, `tissue`, `crumpled paper`)는 시뮬레이션 수거에서만
`tissue_grasp_extra_depth_m:=0.004`를 더해 총 접근 보정량 22mm를 사용한다.
시뮬레이션 수거의 공통 `grasp_approach_offset_m`는 18mm로, 이전 16mm보다
접근 방향으로 2mm 더 깊게 잡는다. Selector IK와 실행 목표는 launch의 공통 보정 설정을 공유한다.
pregrasp 기준 위치와 물체별 추가량(마우스 +14mm, 레고/휴지 +4mm)은 유지한다.
공통 깊이만 원복하려면 `grasp_approach_offset_m:=0.016`을 사용한다.
휴지 변경만 원복하려면 `tissue_grasp_extra_depth_m:=0.0`으로 실행한다.
새 기본값은 다음 launch부터 적용되며 이미 떠 있는 GUI의 파라미터를 소급 변경하지 않는다.

## 선택형 시뮬레이션 최적화와 원복

수거 launch에 `sim_performance_profile:=tabletop_fast`를 추가하면 먼 정적
배경 충돌을 비활성화하고 그림자 맵만 2048로 낮춘다. 기본값 `baseline`으로
재시작하면 기존 설정으로 돌아온다. 그림자 4096을 유지하는
`tabletop_collision`도 제공한다. 파지 깊이·물체 물리·관절 속도·YOLOE-seg·
MoveIt 충돌 검사와 완료 판정은 바꾸지 않는다.
적용 범위, 실행 예제와 비교 측정은
[`cleany_mujoco_sim/README.md`](../cleany_mujoco_sim/README.md#되돌릴-수-있는-고정-책상-최적화)를 따른다.

MuJoCo sorting의 마우스는 `mouse_grasp_extra_depth_m=0.014`를 기본 적용한다.
`mouse`, `computer mouse`, `wireless mouse` 라벨에만 기본 접근 0.018m에
14mm를 더해 옆면을 잡는다. Selector와 executor가 같은 보정을 사용하며
pregrasp 위치, 다른 물체의 깊이, 충돌·접촉 유지 검사는 유지한다.
레고는 `lego_grasp_extra_depth_m=0.004`로 4mm만 추가한다(총 접근 22mm).
추가량은 설정된 라벨과 정확히 일치할 때만 적용된다. 예를 들어
`red building block`은 현재 레고 추가량 매칭 대상이 아니므로 공통 18mm를 사용한다.
실물/비sorting 기본 보정은 0이다. 일반 ROS 설정에서는 `deeper_grasp_labels`와
같은 길이의 `deeper_grasp_offsets_m` 배열을 selector/executor 양쪽에 동일하게 설정한다.
마우스 크기와 모양이 달라지면 재검증해야 하는 시뮬레이션 보정값이다.

그리퍼 닫기 명령 성공 후 접촉 판정은 새 joint-state 촬영 시각으로 확인한다.
기존 접촉 속도/잔차 기준을 연속 3개 이상, ROS 시간 기준 0.10초 이상 만족하면
즉시 진행한다. 튀는 속도는 안정 구간을 초기화한다. 실제 시간 최대 5초
(`gripper_contact_feedback_timeout_sec`) 동안 확인하며, 새 피드백이 없거나
안정되지 않으면 실패한다. `gripper_contact_stable_duration_sec`로 안정 구간을
조절한다. 확인 중 추가 닫기 명령을 보내지 않으며 controller 실패도 통과시키지 않는다.

현재 스터디카페의 분실물 대상은 레고와 마우스다. Gemini/YOLOE 프로필은
`computer mouse` 라벨을 사용하며 분류 정책은 `mouse`, `computer mouse`,
`wireless mouse`를 분실물로 처리한다. 시뮬레이션 배치 검증도 같은 별칭을 지원한다.

Top-down 비교 시험은 `make sim-mujoco-sorting SORTING_ARGS="topdown_only:=true"`로
실행한다. support plane 법선 아래 방향의 후보만 만들며 기울어진 접근 후보 탐색을
끄고, 수평 성분 잡음에 민감한 로봇 반대쪽 접근 필터도 이 모드에서만 끈다.
기존 IK/충돌 허용 기준은 유지한다. 평면 추정과 IK에는 기존 오차 허용이 있으므로
수학적으로 완벽한 수직 고정 제어는 아니다. 기본은 false이며 자동으로 사선 접근으로
되돌아가지 않는다. GUI 유지가 필요하면 `shutdown_on_sorting_exit:=false`를 추가한다.

Study-cafe 파이프라인의 초기 손목 roll은 양팔 모두 `+1.58 rad`이다.
오른손 카메라 장착부가 왼손과 같은 방향으로 시작하도록 하며, MuJoCo 초기
keyframe의 관절값과 actuator 유지 목표에 함께 적용된다. 카메라 TF/장착 형상이나
실물 모터 캘리브레이션은 변경하지 않는다. 실행 중 파지 이후 손목 제한과는 별개다.

### 파지 후 Depth 최신성

MuJoCo sorting은 `cleany_scene_mapping/KnownGeometryOctomapUpdater`를 기본으로
사용한다. 가려진 파지 대상 내부의 이전 OctoMap 점유를 정리하여, 들기 시작점에서
잡은 물체와 자기 잔상 사이의 충돌로 중단되는 것을 줄인다. 관측에서 등록한 형상
내부에 완전히 포함된 셀만 정리하며, 주변 장애물과 부분 겹침 셀은 유지한다.
실물 및 sorting 이외 실행은 기존 updater를 유지한다. `depth_octomap_plugin`으로
명시 변경할 수 있다. 로봇/물체 형상 오차가 있는 환경의 안전성을 보증하지 않는다.

손목 카메라 사용 시 접근·그리퍼 닫기·들어올리기 구간에
`/sorting_cameras`의 `head_depth_boost=true`를 설정한다. 손목 선택과 YOLOE-seg
추적은 유지하며, 구간 종료(실패 포함)에 false로 복원한다. 물체 attachment 이후
촬영된 Depth 및 최신 OctoMap 확인은 생략하지 않는다. 최신성 허용값 2초와
실물 sorting의 attachment 대기 상한 10초는 유지한다. MuJoCo sorting은
GUI·추적 부하로 시뮬레이션 진행이 느려지는 경우를 위해 실제 시간 기준 상한을
30초로 둔다. `attachment_scene_timeout_sec` 실행 인수로 조절한다.
최신 데이터가 도착하면 즉시 진행하므로 고정 30초 대기가 아니다.
MuJoCo 장애물 점군은 `scene_cloud_pixel_stride=4`로 샘플링하여 기존 stride 2
대비 최대 점 수를 1/4로 줄인다(640×480 기준 76,800 → 19,200).
인식·YOLOE-seg 입력 해상도는 유지한다. 작은 장애물의 점 밀도가 줄어드는 절충이
있으며, 비교 시 `scene_cloud_pixel_stride:=2`로 복원할 수 있다. 실물 기본은 2다.
카메라 노드도 함께 빌드해야 하며 boost 설정 거부 시 이동을 시작하지 않는다.

스터디카페 grasp 후보 생성의 어깨 기준점은 migrated CAD URDF의 좌우
shoulder origin을 사용한다. 기존 모델의 기준점을 혼용하지 않는다.

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
Launch argument `sorting_head_reference_refresh_age_sec` 기본값은 0초(비활성)다.
양수 값으로 설정하면 단독 진단에서도 헤드 재관측 경로를 검증할 수 있다.
활성화 시 simulation clock 기준 새 관측+재계획도 같은 신선도 상한을
만족해야 하며, 설정값은 30초를 초과할 수 없다.

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

head RGB-D로 물체/3D grasp를 선택하고 pregrasp에 도착하면 해당 손목 RGB로
인계한다. 이때 head renderer는 10→2 Hz, 선택 손목은 10 Hz가 된다.
기존 3D 추정과 경로를 보존하고 손목 RGB로 대상 일관성을 검사하며,
이 분기에서 head 재검출/3D 재계산과 head 시야 복구 이동을 수행하지 않는다.
RGB 인계가 실패하면 중단하며 이전 3D 위치를 새 측정값으로 포장하지 않는다.
lift는 손목 mask·그리퍼 관절 feedback 기반 접촉 추정·kinematic clearance로 확인한다.
접촉 추정은 닫힘 명령 대비 정지한 관절 상태를 사용하며 독립 힘 센서 측정이 아니다. 이것은 독립적인
depth 상승량 검증이 아니다. 수거함은 `robot_top_bins.yaml`의 초기 base_link 기준 위치를
사용하고 운반 전후 접촉과 충돌·개구부 검사는 유지한다. 복귀 시작 전 head 활성 빈도로 복원한다.
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

이전 캔·박스·무작위 pregrasp 데모는 제거했다. 스터디카페가 재사용하던
MoveIt 실행·joint feedback helper는 `grasp_execution.py`의 `GraspExecutionNode`,
카메라 투영·시각화 helper는 `core/rgbd_projection.py`로 분리했다.
현재 launch와 테스트는 이 공통 코드를 사용한다.

## 상태

점수순 grasp 후보를 양팔 MoveIt plan-only 검증으로 평가한다. 운영 action은 실제
trajectory와 gripper 명령을 실행하지 않는다. 스터디카페 수거 시뮬레이션에서 선택 결과를
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

테스트는 기능별로 묶는다. `test_seeded_cartesian.py`는 경로 보간·자세 보정·URDF FK,
`test_gripper_geometry.py`는 폭 보정·접촉 판정·피드백 안정화,


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

Study-cafe의 네 물체를 대상으로 인식부터 접촉 집기와 후퇴까지 실행하려면 다음 launch를
사용한다.

```bash
ros2 launch cleany_skill_executor study_cafe_nearest_grasp_demo.launch.py
```

기본값은 YOLOE-seg instance mask + Gemini 3.1 Flash-Lite 상세 분류 설정이다. 기존 color adapter는
과거 머그컵/휴대폰/지우개 fixture 전용이며 현재 종이컵/레고/휴지에 대응하지 않는다.
이 study-cafe MuJoCo launch의 YOLOE 기본 체크포인트는
`~/models/yoloe/study_cafe_sim_yoloe26s_seg.pt`다. 현재 시뮬레이션 head 영상으로
fine-tune한 파일이며, 생성·학습·평가 절차는
[`cleany_perception/README.md`](../cleany_perception/README.md#스터디카페-mujoco-전용-yoloe-seg-학습)에 있다.
손목 HANDOFF는 별도 `~/models/yoloe/study_cafe_sim_all_views_yoloe26s_seg.pt`를
`wrist_yoloe_model_path`로 로드한다. 오른손목 HANDOFF는 head 체크포인트를
`wrist_right_yoloe_model_path`로 재사용한다. 실제 컵 pregrasp 오른손목 프레임에서
head 모델은 컵을 confidence 0.726으로 검출했고, 양손목 바닥 학습 모델은 검출하지
못했다. 파지 후 CHECK는
`~/models/yoloe/study_cafe_sim_held_yoloe26s_seg.pt`를
`wrist_check_yoloe_model_path`로 로드한다. Head 모델은 기존 학습본을 유지한다. 왼손목
시점 holdout의 마스크 IoU 0.5 일치율은 52/80→68/80으로 개선됐지만, 새 모델을
head에 쓸 때는 71/80→64/80으로 하락했기 때문이다. 오른손목에서 실제 라벨이
있는 컵·휴지의 일치율은 38/40→36/40이었다. 이 수치는 고정 시뮬레이션 자세와
별도 생성 영상의 평가이며 실제 집기 성공률을 뜻하지 않는다. 학습 결과의 `best.pt`를
각 손목 경로로 복사해 사용한다. 파지 후 레고 holdout은 CHECK 모델에서 18/20으로
개선됐으나, 같은 모델의 파지 후 컵·마우스·휴지 검출은 아직 0/10이다. 실제 카메라 영상에 대한 성능은
검증하지 않았으며, 실물 실행에는 별도 검증된 checkpoint를 `yoloe_model_path`로 지정한다.
스터디카페 기본 실행은 왼손목 마우스 CHECK에
`wrist_mouse_check_yoloe_model_path`의 전용 모델을 쓴다. 실제 MuJoCo 실패
프레임에서 기존 모델은 마우스 검출 0건, 전용 모델은 신뢰도 0.25였고, 마우스
단독 및 전체 실행에서 파지 후 CHECK를 통과했다. 오른손목 CHECK의 컵 전용
모델이 물체를 놓치면 기존 파지 후 통합 모델로 같은 RGB를 재검사한다. 실제
MuJoCo 휴지 CHECK 프레임에서 컵 전용 모델은 검출 0건, 통합 모델은 휴지를
신뢰도 0.364로 검출했다. 두 추가 검사 모두 예상 물체의 투영 위치와 mask
검증을 통과해야 한다.
YOLOE 추론의 최소 confidence는 0.08이고, 클래스별로 컵·마우스·휴지는 0.25,
레고는 0.08을 적용한다. 다른 클래스 목록이나 모델로 바꿀 때는
`yoloe_class_confidence_thresholds`도 같은 순서로 조정해야 한다.
시뮬레이션 수거 정책 `table_sorting_policy.yaml`의 최종 최소 confidence도 0.08로
맞췄다. 레고 외 물체는 YOLOE 단계에서 먼저 0.25 미만을 제거한다.
다른 YOLOE checkpoint는 `yoloe_model_path`로 지정할 수 있다.
전체 파이프라인은 아래 스터디카페 검증 명령으로 실행한다.

## 센서 전용 study-cafe 검증

`study_cafe_nearest_grasp_demo.launch.py`는 YOLOE-seg + Gemini,
aligned RGB-D/TF, AnyGrasp 후보와 MoveIt 검증을 연결한다.
`cleany_perception/config/yoloe_seg_gemini.yaml`을 바탕으로 스터디카페
전용 head/손목 체크포인트를 설정한다. `GEMINI_API_KEY`와 로컬 YOLOE 모델이 필요하다.
Depth obstacle scene은 객체 검출과 별도로 유지하며 target 위치를 simulator에서 받지 않는다.

```bash
make sim-mujoco-pipeline
make sim-mujoco-sorting
```

## 시뮬레이션 분리 수거 (통합 검증 진행 중)

분리 수거 모드는 selector의 `require_pregrasp_visibility=true`를 사용한다.
관측한 OBB 8개 모서리를 현재 camera TF 기준으로 원근 투영하고, 이를 포함하는
16면 visibility cone을 구성한다. MoveIt `GetStateValidity`의
`VisibilityConstraint`가 준비 자세의 로봇 CAD와 이 영역의 겹침을 검사한다.
가리는 자세는 움직이기 전에 제외하고 다른 IK/후보를 검토한다. 반경은 물체
정답 크기나 고정 컵 치수가 아니라 RGB-D OBB에서 계산하며 기본 padding은
3 mm다. 동적 camera TF는 기본 0.5초 이내여야 하고, 검사 결과가 누락되면
통과로 취급하지 않는다. 일반 selector의 기본값은 false다.

`sorting_contact_diagnostics:=true`는 MuJoCo observer의 접촉 부위/힘
출력만 켠다. 기본은 false이며, controller 오차와 실제 접촉을 대조하는
진단용이다. coordinator나 grasp selector는 이 GT 토픽을 읽지 않는다.

```bash
make sim-mujoco-sorting

# 기본 sorting 실행은 headless이며 RViz/image_view를 시작하지 않는다.

# 수거 완료/실패 후에도 시뮬레이터와 GUI 유지 (수거 동작을 자동 재시도하지 않음)
make sim-mujoco-sorting SORTING_ARGS='headless:=false use_rviz:=true use_image_view:=true shutdown_on_sorting_exit:=false'
# 같은 실행에서 GUI 창만 생략 (vendor 카메라 렌더링에는 DISPLAY 필요)
make sim-mujoco-sorting SORTING_ARGS='headless:=true use_rviz:=false use_image_view:=false'
```

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

`sorting_reobserve_after_lift=true`는 head 재검출에 실패한 경우(기본 관측 경로에서는
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

`config/table_sorting_policy.yaml`은 Gemini의 category/reason을 받아
쓰레기를 `trash_right`, 분실물을 `lost_items_left`로 보낸다. 위험 라벨과 저신뢰,
category/reason 미제공 결과는 review로 남기고 조작하지 않는다.

현재 장면은 종이컵·마우스·휴지뭉치·확대한 레고를 사용한다. YOLOE의 명시 클래스는
`cup`, `computer mouse`, `crumpled tissue`, `lego brick`이며 Gemini는 고정 목록 없이
외형에 따른 자유 라벨을 생성한다. 모델 출력 category/reason으로 분류하고 위험·저신뢰
결과는 review로 남긴다. 장면의 형상·크기와 평가용 body mapping은
[MuJoCo README](../cleany_mujoco_sim/README.md)를 따른다.

수거 위치는 로봇 뒤쪽 +Y(좌측)의 `lost_items_left`, -Y(우측)의
`trash_right`다. `cleany_mujoco_sim/config/robot_top_bins.yaml`은
수거 위치의 알려진 보정값/형상만 공유한다. 책상과 집을 물체의 위치/형상은
카메라에서 얻으며 scene의 target 좌표를 planner에 전달하지 않는다.
수거함은 바닥과 4개 벽으로 구성되고 MoveIt에도 같은 목적지 형상을 등록한다.
내부 후면 받침판도 같은 설정으로 MoveIt 충돌 형상에 등록한다.
이 좌표는 현재 고정 베이스 시뮬레이션 전용이며 이동 베이스 운용을 보장하지 않는다.

분류 → 물체에 가까운 팔의 후보를 먼저 검사하고 실패하면 반대 팔 검사 → 재인식/집기/후퇴/들기 →
부착 물체를 포함한 MoveIt 운반 → 수거함 개구부 확인 → 열기 → 복귀 →
선택적 놓기 검증 순서다. 운반 실패 또는 비정상 물체 중심·반경이면 그리퍼를 열지 않는다.
sorting 모드에서는 다각도 grasp 후보와 전체 팔 IK seed를 추가로 탐색하되
기존 pose/충돌 허용 기준은 유지한다. 운반 IK에는 전체 joint feedback과
`RobotState.is_diff=true`를 보내 부착한 OBB가 검사에서 빠지지 않도록 한다.
Sorting은 초기 양손 개방을 따로 명령하지 않는다. 선택된 팔의 pregrasp와
그리퍼 개방을 6관절 경로로 함께 계획하고, 목표 도착 시 모든 관절 피드백을 검사한다.
별도 닫기·놓기의 `gripper_motion_sec` 기본값은 2초이며 반환 시에도 팔 복귀와
턱 닫기를 같은 경로로 계획한다. 속도는 상단 설정 표를 따른다.
재검출 후보를 selector에 전달하기 전 이전 snapshot의 target OBB를 제거해 중복
충돌체를 방지한다. 닫기 후 잔여 각도·속도의 안정 구간이 확인돼야 attachment/lift로 진행한다.
동작 실패 뒤 depth boost 복원까지 실패하면 두 오류를 기록하고 최초 동작 오류를 유지한다.

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
`grasp_approach_offset_m` launch 인수(시뮬레이션 수거 0.018 m, 그 외 0.016 m)는 selector의 IK 목표와
coordinator의 실행/표시 보정에 동일하게 전달한다. 회전식 비대칭 jaw의
벌림에 따른 실제 접촉 깊이를 보정하는 시험용 입력이며, 현재 기본값은
실측 하드웨어 보정이나 지속 파지 성공으로 검증된 값이 아니다.
같은 옵션은 selector의 출력 전용 service trace도 활성화한다. 별도
`services-<uuid>` 폴더에 각 MoveIt 서비스 요청/응답 CDR과 완료 시간,
취소 여부 JSON을 저장한다. 실패한 IK 요청의 seed/목표를 재현하기 위한
진단이며 응답이나 계획을 수정하지 않는다. 이 옵션의 파일 I/O가 포함된
시간을 저장 기능을 끈 운용 성능으로 해석하지 않는다.
`/sorting/status`는 단계, label, 분류, 목적지, 완료 항목을 JSON String으로
latched 발행한다. 기본은 관찰 모드이며 물체별 `placement_verified=false`와
`mission_complete_unverified`를 기록한다. 자동 검증을 켜면 물체별 안착 확인 후
`mission_complete`를 발행한다. 빈 작업영역은 연속 두 번 확인해야 한다.
`sorting_require_category_coverage=true`일 때만 `sorting_required_categories`의
모든 종류가 완료 항목에 포함됐는지 추가 검사한다.

`/sorting/verify_placement`는 독립된 결과 검증 port다. 현재는 시뮬레이션
oracle이 release 이후의 새 물체 AABB가 올바른 수거함 내부에서 안정적으로
정지했는지 확인한다. 이 oracle은 인식·분류·grasp·경로 계산에 사용하지
않는다. 실제 로봇에는 별도의 센서 기반 결과 검증 구현이 필요하다.

## 분류·운반 실행 계약

`study_cafe_sorting.launch.py`와 일반 파이프라인은 `robot_top_bins.yaml`의
내부 후면 받침판·수거함을 기본 배치로 사용한다. 책상 양끝 분류 영역과 구분선은
없으며 수거함의 고정 투하 위치만 사용한다.
개별 물체 배치 성공 기록이 있지만 전체 정리 성공은 검증 중이다.
`table_sorting_policy.yaml`의 destination으로 Gemini의 category/reason을
연결한다. label allowlist로 category를 대체하지 않으며 위험 label은 보수적으로
거부한다. RGB-D 거리순으로 시도하고 인식된 수거 구역 물체는 다시 집지 않는다.
미처리/미확인 물체가 남으면 성공으로 보고하지 않는다. 관측한 작업 물체 수와
완료 항목 수를 대조하므로 이전에 관측한 수량보다 완료 수가 적으면 중단한다.
이는 수량 검사이며 객체 identity 추적이나 관찰 모드의 독립 안착 검증을 대신하지 않는다. 빈 작업 영역은 두 번
재확인하며 최대 search/action 횟수 12에 도달하면 실패한다.

MuJoCo oracle은 배치 후 전체 AABB가 수거함 안에 들어가 정지했는지 검증한다.
기본 `sorting_exit_on_finish=true`; coordinator 종료 시 launch가 다른 노드도
종료한다. GUI를 유지하려면 launch 인수 `shutdown_on_sorting_exit:=false`를 사용한다.
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
`sorting_head_reference_refresh_age_sec` 기본값은 `0.0`으로, pregrasp 도착 후
시간 경과만을 이유로 하는 Gemini 재인식을 비활성화한다. `(0, 30]`초로 설정하면
기존 ROS clock 기준 만료 검사와 재인식·위치 연관·동일 팔 재계획을 다시 활성화한다.
손목 서비스 자체의 관측 유효성 검사는 유지하며 관측 timestamp를 갱신하지 않는다.

고정 수거함 모드에서는 YOLOE-seg bbox/depth 거리로 정렬한 후보를 하나씩 처리한다.
`fixed_jaw_clearance_m`은 고정 손가락 여유거리(수거 기본 0.003 m),
`grasp_opening_margin_m`은 후보 벌림 여유폭(기본 0.008 m)이다. 여유거리는
여유폭의 절반 이하여야 한다. 5 mm 시험은 각각 `0.005`, `0.010`으로 설정한다.
벌림 여유폭은 후보 생성기와 두 TCP 보정 경로에 동일하게 전달된다.
현재 후보의 YOLOE-seg mask에만 3D 복원을 수행하고 파지 가능한 팔을 찾으면 탐색을 끝낸다.
복원·파지 계획·양팔 도달성 검사가 실패하면 다음 후보를 복원하며, 같은 주기의
복원 결과는 캐시한다. review 분류 후보는 정밀 복원 없이 미해결 대상으로 남긴다.
선택되지 않은 물체도 전체 depth 기반 OctoMap의 충돌 검사에는 계속 포함된다.
다음 후보 처리 전 snapshot TTL이 만료되면 기존 cache 오류로 처리되며 과거 영상의
시각을 갱신하지 않는다. 미해결 물체와 기존 관측 물체의 누락 검사는 유지한다.
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
`direct_vertical_lift_extra_m`(기본 0, 허용 범위 0~0.05m)은 이 진단 경로의
TCP 상승 거리에만 추가된다. 물체 중심의 기존 최소 상승 높이 검사와 경로
충돌 검사는 그대로 적용되며, 높이가 부족하면 예측/요구 중심 높이를 오류에 남긴다.
`sorting_release_edge_margin_m`(기본 0.005m)은 수거함 입구 안쪽의 추가
가장자리 여유를 조절하는 진단용 launch 인자다. 물체의 관측 bounding sphere가
실제 입구 안에 들어가는 검사와 MoveIt 충돌 검사는 그대로 유지된다.
sorting에서는 끝점 position IK/FK를 확인하고 기존 seeded 위치 corridor로 계획한다.
기존 corridor 자세 한도(5°)를 넘는 끝점은 거절하고, 전체 경로의 FK/충돌/자세
변화를 검사한다. 정확한 자세 고정 LIN으로 표현하지 않는다. 파지/손목 감시는
유지하며 기본 전체 sorting 동작은 아직 역방향 retreat를 유지한다.

- [System Context](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Context.md)
- [Task Planning and Robot Capabilities](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Task%20Planning%20and%20Robot%20Capabilities.md)
# 수거함 운반

## 분류된 수거함으로 직접 이동 / 손목 유지 (기본 모드)

현재 수거 시뮬레이션은 공유 `robot_top_bins.yaml`의
`simulation_ignore_mast_collision: true`로 고정 기둥 접촉을 MuJoCo에서 끄고
MoveIt의 기둥–로봇 링크 충돌을 제외한다. 외형 및 self-filter용 TF/형상은 유지한다.
이 편의 설정의 성공 결과는 실제 기둥을 피하는 실로봇 경로 검증이 아니다.

빈손 복귀와 그리퍼 닫기는 계속 동시에 실행하되,
`sorting_return_velocity_scaling: 0.12`, `sorting_return_acceleration_scaling: 0.15`로
복귀 동작만 별도 상한을 적용한다(더 낮은 전역 설정은 유지).
현 MJCF STS3215의 토크 한계 2.648Nm 및 감쇠 합 3.342Nm·s/rad에서는
무부하 정상 속도조차 약 0.78rad/s 이하라 기존 30% 어깨 yaw 상한 1.274rad/s를
지속 추종할 수 없다. 새 복귀 yaw 상한은 약 0.509rad/s이며 충돌 검사,
모터 토크 한계 및 0.12rad 궤적 추종 허용치는 변경하지 않는다.
이는 현재 시뮬레이터 모델용 설정이며 실제 모터 성능 인증값이 아니다.

운반은 고정 투하 방식이다. `config/nearest_pregrasp.yaml`의
고정 **물체 중심** 위치(base_link, m)는 다음과 같다. 수거함은 로봇 내부 후면에 부착된다.

- `sorting_fixed_release_lost_items_left_m`: `[-0.075, 0.105, 0.50]`
- `sorting_fixed_release_trash_right_m`: `[-0.075, -0.105, 0.50]`

분류 → 파지 → 안전한 들어 올리기 → 선택된 수거함 위 고정 투하점 → 놓기 → 복귀 순서다.
기본 모드에서는 파지 전 공통 경유점 IK 검사, 공동 손목 endpoint 검사 및 경유점 이동을
모두 생략한다. 물체 크기/그리퍼 안의 실제 추정 offset을 반영한 최종 투하 자세만 구한다.
도달 가능한 자세가 없으면 물체를 놓지 않고 중단한다. 직접 이동도 MoveIt의 현재
부착 물체/장애물 충돌 검사를 거친 경로로 실행하며 무검증 직선 명령이 아니다.

들어 올리기를 마치면 실제 손목 roll 각도를 캡처한다. 최종 투하 IK는 손목 roll만 변수에서
제외하고 어깨 yaw/pitch·팔꿈치·손목 pitch의 네 관절을 푼다. 손목을 위아래로 꺾는
pitch는 고정하지 않는다. 실행 경로의 roll tracking 허용
오차는 `sorting_fixed_release_wrist_tolerance_deg`(기본 0.5도)이며 자동 확대하지 않는다.
고정 투하점에서 물체 중심·손목 feedback·함 입구를 확인한 뒤에만 그리퍼를 연다.
물체 중심 도착 허용 오차는 1cm이고, IK의 위치 허용 오차는 축별 0.5mm다.

검증된 최종 자세는 프로세스 내 캐시에 저장하며 동일 팔/손목 각도/물체 offset/투하점일 때
재사용한다. 캐시 hit도 현재 부착 물체를 포함한 충돌 및 FK를 다시 확인한다.
센서 feedback이 달라 정확히 같은 키가 아니면 재계산하므로 캐시 hit나 시간 단축을
항상 보장하지 않는다. 직접 운반 중에는 넓은 영역 탐색이나 손목 완화로 우회하지 않는다.
이 설정은 시뮬레이션용이며 실제 로봇의 고정 투하 자세가 검증됐다는 뜻은 아니다.

## 파지 후 후퇴/들어 올리기의 손목 제약

수거 중 손목 roll만 안정된 파지 접촉 시점의 실제 관절각을 기준으로 제한한다.
pitch는 위아래 동작을 위해 자유롭게 사용하되 기존 물리 관절 한계/충돌 검사는 유지한다.
`config/nearest_pregrasp.yaml`의 ROS parameter `sorting_carry_wrist_tolerances_deg`
기본값은 `[2.0, 5.0, 10.0, 20.0, 30.0]`이다. 파지 후 후퇴/수직 lift는 빈 팔의
기존 pregrasp 관절 목표를 재사용하지 않고, 현재 허용 범위에서 새 endpoint IK와
전체 Cartesian 경로를 계획한다. endpoint나 검증된 경로를 찾지 못할 때 다음 범위로
넓힌다.
기준각은 구간마다 바꾸지 않으며, 한 번 확대한 범위는 놓기까지 유지한다.
관절 한계/FK/충돌 검사는 유지한다. 최대 범위에서도 실패하면 중단한다.

MoveIt 이동에는 같은 관절 path constraint를 전달하고, 직접 생성하는 Cartesian
궤적은 local IK/refinement 경계부터 손목 제약을 반영하고 실행 전 모든 waypoint를
검사한다. 계획/검증과 실행을 분리해 계획 실패만 재시도하며, 서비스/URDF 오류나
제어기 실패/접촉 상실 후에는 제한을 완화해 재실행하지 않는다. 일반 MoveIt 운반
구간은 여전히 IK 성공 후 경로 계획 실패 시 중단한다.

손목 유지 시 어깨/팔꿈치 움직임으로 그리퍼의 세계좌표 방향은 변할 수 있다.
후퇴/lift의 새 도착 방향은 기존 빈 팔 방향을 강제하지 않고 구간 시작 방향 대비
`sorting_carry_cartesian_rotation_limit_deg`(기본 30도)로 제한한다. 이 값은 시뮬레이션
운용 설정이며 실제 로봇의 안전한 기울기 한계가 검증됐다는 뜻은 아니다. 새 방향을
잇는 전체 FK corridor의 기존 자세 오차/속도/가속도/충돌 검사는 유지한다.
그리퍼를 열고 scene attachment를 해제한 뒤에는 손목 제한을 해제해 복귀한다.
손목 변화 최소화의 전역 최적해나 물체의 수평 자세 유지는 보장하지 않는다.

고정 투하 위치도 물체의 bounding sphere가 수거함 입구 내부에 들어가는지 검사한다.
벽 두께와 `sorting_release_edge_margin_m`(기본 0.005m)을 제외하고,
물체 아래쪽 높이는 테두리보다 `sorting_release_clearance_m`(기본 0.06m)에서
`sorting_release_maximum_clearance_m`(기본 0.21m) 사이여야 한다.
위치가 이 범위에 맞지 않으면 이동하지 않고 중단한다. 놓기 직전에도 실제
feedback으로 물체 중심·손목·함 입구를 다시 검사한다.

SAM 분할·기준 mask 추적·중앙 인계 상태는 제거했다. 현재 손목 검증은 YOLOE-seg의
HANDOFF/CHECK 재검출이며 이동 전후 접촉·충돌·수거함 개구부 검사는 유지한다.

예전 라벨 allowlist 정책, 테이블 배치 구역, 경유점/영역 탐색 운반은 제거했다.
