# cleany_manipulation_bt

승인된 `snapshot_id/object_id`의 쓰레기 하나를 MuJoCo에서 수거하는 ROS Action 서버다.
실제 BehaviorTree.CPP 4 실행기가 동작 순서를 소유하고, Python은 기존 인식, MoveIt,
그리퍼 동작과 SQLite 기록을 연결한다. Planner와 Mission Manager는 포함하지 않는다.
기존 mock 실행에는 이 패키지의 C++ 모듈을 import하지 않는다.

| 하려는 일 | 안내 |
|---|---|
| 빌드·자동 검사 | [빌드와 검증](#빌드와-검증) |
| 장면 실행·관찰·Goal 요청 | [실행](#실행) |
| 터미널 출력 조정 | [로그 설정](#터미널-로그) |
| 실행 검증·취소 | [계약과 증거](#계약과-증거), [취소 방식](#취소) |
| 결과 조회·모니터 이해 | [기록과 모니터](#기록과-모니터) |

## 빌드와 검증

Ubuntu 22.04 / ROS 2 Humble의 `behaviortree_cpp` 4.x, `pybind11_vendor`, Python 개발
header가 필요하다. 설치는 [개발환경 안내](../../../docs/DEVELOPMENT_SETUP.md)를 따른다.

```bash
make build-manipulation-bt
make test-manipulation-bt
make test-manipulation
```

첫 검사는 실제 C++ 트리와 가짜 backend, DDS Action/취소/조회, snapshot TTL,
수거 코드 추출 회귀를 검증한다. 두 번째 mock 검사는 기존 성공/실패/취소/재시작을
보존하는지 확인한다. 가짜 backend의 성공은 물리 수거 성공 근거가 아니다.

## 실행

기존 스터디카페의 YOLOE head/손목 모델, text encoder, PyTorch/Ultralytics와
`GEMINI_API_KEY`가 필요하다. 모델 경로와 인식 준비 조건은
[기존 수거 실행 안내](../cleany_skill_executor/docs/study_cafe_sorting_usage.md)를 따른다.
키는 코드/ROS parameter에 넣지 않고 실행 환경에서 제공한다.

```bash
# 키를 로컬 파일에 설정한 경우에만 사용. 키 자체를 로그에 출력하지 않는다.
source ~/.config/cleany/gemini.env
make sim-mujoco-manipulation
# GUI가 필요한 환경: SORTING_ARGS='headless:=false use_rviz:=true'
```

headless에서도 기존 GLFW 카메라 renderer에는 유효한 `DISPLAY`가 필요하다.
GUI 로그인 터미널에서 실행하거나 Xvfb가 설치된 CI에서
`xvfb-run --auto-servernum make sim-mujoco-manipulation`을 사용한다.

[manipulation_bins.yaml](config/manipulation_bins.yaml)은 기존 tray 좌표를 유지하고
시뮬레이션 trash bin의 Y 외측 치수를 170 mm에서 180 mm로 넓힌다. 기존 컵의 센서 OBB를
회전 독립 bounding sphere와 5 mm 여유로 검사하면 기존 170 mm opening에 맞지 않는 경우가
있어 BT 경로에만 별도 설정을 사용한다. 시뮬레이터, MoveIt 충돌 형상과 평가기가 같은 설정을 읽는다.
관측 payload의 수거함 적합성을 팔 이동 전에 검사하고 접근 전 재관측에서도 다시 검사한다.
실물 수거함/장착 사양의 확정은 아니다. 기존 자동 sorting의 bin 설정은 유지한다.

이 launch는 MuJoCo, 카메라, Perception, grasp/selection 서버, MoveIt, 지도와 결과 평가기,
BT Action 서버를 시작한다. 자동으로 물체를 고르는 `SortingCoordinator.run()`은 실행하지 않는다.
launch 직후에는 BT가 Goal을 기다리므로 Live에 연결돼도 모든 노드가 IDLE이다.
대기 중에도 카메라와 깊이 지도는 갱신되지만 정상 프레임 진단은 기본 로그에 출력하지 않는다.
BT의 RUNNING 전환을 보려면
아래의 관찰 요청 후 지정 물체의 `manipulation_test_client --namespace /sim` Goal을 보내야 한다.
기존 sorting 프로필의 알려진 형상 OctoMap updater, jaw clearance와 좌표 보정을
재사용한다. PTP 이동은 그리퍼 추종 오차를 줄이기 위해 속도/가속도 scaling을 각각 0.08로
설정하며 같은 이름의 launch 인자로 조정한다. controller 실패 후 자동 재실행하지 않는다.
이는 시뮬레이션 편의 설정이며 실제 로봇의 안전 검증으로 해석하지 않는다.
평가기의 MuJoCo 정답 좌표를 grasp, 물체 연결 또는 경로 계획에 사용하지 않는다.

다른 터미널에서 detector-only 관찰을 요청한다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 action send_goal /perception/inspect_scene cleany_interfaces/action/InspectScene \
  "{query: 'Detect objects on the table.', snapshot_id: '', selected_object_id: 0}"
```

반환된 실제 snapshot과 쓰레기 object ID를 지정한다. snapshot cache의 기본 TTL은
120초이고 최근 두 장만 남는다. 만료됐으면 새 관찰을 요청해야 한다.

```bash
ros2 run cleany_skill_executor manipulation_test_client \
  --namespace /sim --snapshot-id '<실제 snapshot_id>' --object-id 1 \
  --destination-id trash_right --timeout 1200
# 출력된 execution_id로 조회
ros2 run cleany_skill_executor manipulation_test_client \
  --namespace /sim --execution-id '<execution_id>' --query
# 취소 재현: 위 실행 명령에 --cancel-stage GRASPING 추가
```

## 터미널 로그

| launch 인자 | 기본값 | 상세 출력 |
|---|---|---|
| `log_level` | `info` | `debug`: 후보 평가, 좌표·오차·타이밍, 깊이 지도 프레임 진단 |
| `backend_log_level` | `warn` | `info`: MuJoCo·MoveIt·TF 초기화와 계획 상세 |

기본 출력에는 준비 완료, 주요 작업 진행, 경고·오류와 컨트롤러 spawner의
준비 완료가 남는다.

```bash
make sim-mujoco-manipulation SORTING_ARGS="headless:=false log_level:=debug"
# 외부 backend의 초기화·계획 상세까지 확인
make sim-mujoco-manipulation SORTING_ARGS="headless:=false log_level:=debug backend_log_level:=info"
```

`log_level`은 끌리니 로거와 `move_group.scene_mapping`에 적용한다.
`debug`에서도 ROS 내부 대기 루프의 로그는 함께 켜지지 않는다. 두 옵션은
`debug`, `info`, `warn`, `error`, `fatal` 중에서 선택한다.

MoveIt 2.5.9의 종료 시 controller plugin unload 문제를 피하기 위해
`keep_controller_plugin_loaded:=true`를 기본으로 전달한다. 적용 범위와 종료
회귀 검사는 [MoveIt README](../cleany_moveit_config/README.md#humble-controller-plugin-teardown)를 따른다.

## 계약과 증거

| ROS 경로 | 용도 |
|---|---|
| `/sim/manipulation/execute_skill` | 기존 `ExecuteManipulationSkill` 계약, `execution_profile=mujoco` |
| `/sim/manipulation/get_execution` | 기존 `GetManipulationExecution` 조회 계약 |
| `/sim/manipulation/cancel` | `CancelManipulation`: execution_id와 취소 mode 요청 |
| `/sim/manipulation/execution_events` | Reliable/Transient Local 기록 이벤트 |
| `/sim/manipulation/bt_transitions` | 실제 UID, 노드 ID, 이전/현재 상태 JSON |
| `/perception/get_scene_snapshot` | 읽기 전용 `GetSceneSnapshot`; TTL/촬영 시각 유지 |

[collect_trash_mujoco.xml](trees/collect_trash_mujoco.xml)이 준비, 잡기, 놓기, 확인과 종료를
정한다. C++ 노드는 `start(node_id, execution_id)` 후 RUNNING을 반환하고 후속 tick에서
`poll(operation_id)`를 호출한다. `halt()`는 `cancel(operation_id)`를 전달한다.
Python worker 하나가 개별 동작을 수행하므로 ROS 응답/모델 추론 대기는 BT tick을 막지 않는다.
서버는 시뮬레이션 시계와 독립적인 steady clock으로 20 Hz tick한다.
이 방식은 [BehaviorTree.CPP 비동기 노드 계약](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_04_sequence/)을 따른다.
큰 영상/점군과 파지/경로 결과는 실행별 Python context에 두고 Blackboard에는 execution ID만 둔다.

움직이기 전에 지정 snapshot, 대상 깊이, 모델 분류, 정책과 목적지를 검사한다.
기본 지원 쓰레기는 `cup`, `crumpled tissue`다. `paper cup`, `disposable paper cup`은
같은 컵 종류 alias로 연결하되 원래 인식 label과 분류를 보존한다. 쓰레기 분류·사유와
목적지 정책을 검증한다. `collect_lost_item`은 `mouse`, `computer mouse`, `wireless mouse`,
`lego brick`을 지원하며 모델의 `lost_item` 분류와 사유, 정책의 `lost_items_left` 목적지가
일치해야 한다. 두 skill은 기존 `collect_trash_mujoco.xml`의 집기·이동·놓기·확인 절차를 공유한다.
종류와 목적지는 정책 및 `bt_supported_trash_labels`, `bt_supported_lost_item_labels` 설정에서 읽는다.
같은 종류 여러 개는 유효한 session/epoch/track이 있을 때 허용한다.
빈 세션의 기존 단일 대상 경로는 종류가 유일해야 한다. 선택 객체 복원 뒤 이동 전에
관측 OBB로 `/sorting/register_placement_target`에 등록하고 verification ID를 고정한다.
등록이 불확실하면 이동 전에 차단하며, 놓기 확인은 그 개체에만 수행한다.
첫 팔 이동 전과 접근 전 재관찰은 승인된 track과 기하 연속성, 분류–skill–목적지를 모두
확인한다. 마우스 별칭은 같은 종류로 연결하고 다른 개체를 대신 선택하지 않는다.
미지원 대상, review 분류, 분류와 불일치하는 skill 및 다른 목적지는 차단한다.

파지 확인이 실패하면 들기를 시작하지 않는다. 보유 상실, controller 실패, 동작 timeout과
관절/시뮬레이션 시계 중단은 정지 평가로 연결된다. 기본 `CHECKPOINT` 취소는 잡기(닫기+확인), 들기(들기+보유 확인),
놓기(위치 검사+열기+이탈 확인)의 현재 checkpoint까지 제한 시간 안에서 수행한 뒤 다음 단계를 차단한다.
실패한 정상 경로 뒤에는 `StopAndAssess → FinalizeFailure`를 실행한다.
Action 결과의 BLOCKED/FAILED/CANCELED/FATAL과 BT의 FAILURE는 별도 의미다.

동작 결과가 없는 상태에서 제한 시간이 지나면 `TIMEOUT`을 기록하고 중단을 요청한다.
취소한 동작의 최종 결과는 정지 확인 중에 수집하며, 정지 평가가 먼저 끝나도 동작 종료 결과를
확인하기 전에는 최종 Result를 확정하지 않는다. 늦게 확인된 장치 오류·비상정지·내부 오류는
`FATAL`로 반환하고 후속 Goal을 차단한다. 취소 완료나 늦은 성공은 `TIMEOUT` 결과를 유지하며,
성공 관측의 물체 상태·놓기·팔 복귀 근거는 보존하되 다음 정상 동작이나 취소 후 복귀를 시작하지 않는다.
이 대기는 기존 정지 제한 시간 안에서만 수행하며, 동작 종료나 정지를 확인하지 못하면
`FATAL / STOP_UNCONFIRMED`로 종료한다. 움직임 시작 전의 시간 초과 취소에도 정지 확인을 요구한다.
시간 초과 취소를 시작하기 전에 성공 결과가 준비되어 있으면 경과 시간과 관계없이 기존처럼
성공을 처리한다. 최종 Result 확정 후 도착한 관측은 결과를 변경하지 않는다.

### 취소

`manipulation/cancel` 서비스의 mode는 다음 세 가지다. 일반 Action cancel은
`CHECKPOINT`를 사용한다. Result와 실행 기록의 `cancel_mode`에 적용한 모드를 남긴다.

| mode | 동작 |
|---|---|
| `IMMEDIATE` | 현재 노드와 원자 구간의 완료를 기다리지 않고 중단 요청, 정지 확인 후 종료 |
| `CHECKPOINT` | 현재 노드를 완료하고 정지. 잡기·들기·놓기는 같은 단계의 확인까지 완료 |
| `RETURN_ARM` | 현재 작업 중단 → 정지 확인 → 물체를 잡고 있으면 현재 위치에서 열어 놓기 → 팔 복귀 → 최종 정지 확인 |

`RETURN_ARM`은 `StopAndAssess → ReleaseInPlace → ReturnArmAfterCancel → StopAfterRecovery`
경로를 실행한다. 팔을 선택하기 전에는 복귀할 실행 대상이 없으므로 정지 후 종료한다.
복귀는 현재 관절 상태에서 실행 시작 때 저장한 팔 자세로 계획하며 수거함 이동과 목적지
놓기 검증은 수행하지 않는다. 현재 위치 놓기는 그리퍼 열기 완료에 근거한 초기 정책이다.
놓기 위치 선택과 별도 물체 이탈 관찰은 후속 검토 대상으로 둔다. `placement_state`에는
기존 확인값만 보존하고 취소 후 놓기를 수거 성공으로 만들지 않는다.
놓기·복귀 실패는 다시 정지 평가 후 실패로 반환한다. 복귀 중 `IMMEDIATE` 요청은
선점하며 이후 약한 요청으로 완화되지 않는다. 기존 feedback·controller·timeout 검사는 유지한다.

`IMMEDIATE/RETURN_ARM`은 중단 요청을 즉시 보내고 취소한 동작의 최종 결과를 정지 확인 중에 수집한다.
정지 평가가 먼저 끝나도 이전 동작의 결과가 확인될 때까지 놓기·복귀와 최종 Result 확정을 기다린다.
늦게 도착한 실제 실패는 취소보다 우선하고, 장치 오류·비상정지·내부 오류는 `FATAL`로 실행을 차단한다.
늦은 성공은 확인된 물체 상태, 놓기, 팔 복귀와 완료 단계만 보존하고 전체 수거 성공으로 바꾸지 않는다.
이 대기는 기존 정지 제한 시간 안에서만 수행한다. 취소한 동작의 종료나 정지를 기한 내 확인하지 못하면
`FATAL / STOP_UNCONFIRMED`로 끝내며, 움직임 시작 전 취소에도 정지 확인을 요구한다.

테스트 클라이언트에서 `--cancel-stage TRANSPORTING --cancel-mode RETURN_ARM`으로
재현한다. 이미 실행 중인 Goal에는 다음 서비스를 호출한다.

```bash
ros2 service call /sim/manipulation/cancel cleany_interfaces/srv/CancelManipulation \
  "{execution_id: '실행-UUID', mode: 'RETURN_ARM'}"
```

### 정상 수거의 놓기·정지 확인

그리퍼 열림은 controller와 관절 피드백으로 확인한다. `ConfirmRelease`는 방출 시각 이후의
독립 안착 검증을 기다리고, 팔 복귀 뒤 `VerifyPlacedObject`에서 새 시각을 기준으로 다시 검증한다.
평가기의 false는 확인 대기다. 제한 시간 내 안착 근거를 못 얻으면
`placement_state=UNKNOWN / VERIFICATION_TIMEOUT`을 반환한다.
SUCCESS는 독립 방출/안착, 팔 복귀와 최종 정지 근거가 모두 있을 때만 저장한다.
정지는 모든 ROS Action의 terminal 결과와 정지 확인 시작 이후의 새 관절 샘플을 요구한다.
양팔과 양쪽 그리퍼의 각 관절에 sensor stamp와 단조 시계 수신 시각을 따로 기록한다.
부분 메시지는 포함된 관절만 갱신하며, 속도 누락·비정상 값·중복 이름·반복 또는 역행 stamp는
정지 근거로 사용하지 않는다. 실행 중 feedback 감시도 관절별 freshness를 검사한다.
각 확인 표본은 모든 필수 관절이 이전 표본 이후 새 정보를 제공해야 인정한다.
양팔이 별도 메시지로 들어와도 합쳐서 확인하며, 같은 정보를 반복해서 세지 않는다.
모든 관절이 설정한 속도 이내이고 Action이 종료된 상태가 연속 표본 수와 유지 시간을
함께 만족해야 한다. 오래된 정보, 움직임 또는 미종료 Action이 있으면 확인 구간을 다시 시작한다.
취소 응답만으로 정지를 인정하지 않으며, 움직임 시작 후 정지가 확인되지 않으면
`FATAL / STOP_UNCONFIRMED`와 사람 확인 필요 상태를 기록한다.

[manipulation_mujoco.yaml](config/manipulation_mujoco.yaml)의 정지 판정 설정은 다음과 같다.

| ROS parameter | 모의 기본값 | 용도 |
|---|---|---|
| `bt_stop_timeout_sec` | 10초 | 정지 확인 제한 시간 |
| `bt_stop_feedback_max_age_sec` | 0.5초 | 관절별 수신 경과 시간과 sensor stamp 허용 오차 |
| `bt_stationary_duration_sec` | 0.25초 | 정지 상태의 최소 유지 시간, 단조 시계 기준 |
| `arm_stationary_samples` | 5 | 모든 필수 관절의 새 정보를 사용한 연속 확인 횟수 |
| `arm_stationary_velocity_rad_s` | 0.02 rad/s | 팔과 그리퍼의 정지 속도 기준 |

이 값은 MuJoCo 구현의 조정 가능한 초기값이며 실물 로봇의 안전 기준은 아니다.

장치 오류(`HARDWARE_ERROR`), 비상정지(`E_STOP`), 내부 오류(`INTERNAL_ERROR`)와
정지 미확인(`STOP_UNCONFIRMED`)을 동작 실패로 관측하면 즉시 실행을 차단하고 사람 확인 필요를 기록한다.
최종 정지 확인에 성공해도 `FATAL`과 원래 오류 코드를 유지한다. 움직임 시작 후 정지 확인에 실패하면
최종 오류 코드는 `STOP_UNCONFIRMED`를 우선하고 원래 원인은 Result message에 남긴다.

### 예상하지 못한 실행 예외

ROS 동작 호출은 명시적인 `OperationError`의 오류 코드를 유지한다. 기존 하위 동작이
예상 실패에 사용하는 정확한 `RuntimeError`·`ValueError`와 `InfrastructureError`,
`CartesianPlanningError`, `LiftRedetectionError`는 기존 준비·대상·이동·파지 실패 분류로 전달한다.
그 밖의 예외는 발생 노드와 예외 종류·내용을 담은 `INTERNAL_ERROR`로 전달하며,
정지 확인 후 `FATAL`로 종료하고 후속 Goal을 차단한다. `AttributeError`, `TypeError`,
`KeyError`와 `NotImplementedError`·`RecursionError` 같은 프로그램 오류를 일반 동작 실패로 바꾸지 않는다.
보유 접촉 검사도 정확한 `RuntimeError`만 보유 상실로 바꾸며, 다른 내부 예외는 그대로 전달한다.

작업 시작·상태 조회·취소와 BT tick·전환 조회에서 예상하지 못한 예외가 발생하면
즉시 신규 실행을 차단하고 `INTERNAL_ERROR`와 발생 단계를 기록한다.
작업 번호를 받기 전에 시작 요청이 실패해도 `Backend.abort()`가 현재 context에
중단 신호를 전달한다. 개별 중단 요청이나 BT halt의 추가 오류는 원인과 함께 보존하며
정지 평가를 막지 않는다. native halt callback의 오류는 노드 reset 후 core에서 처리한다.

이후에는 BT를 다시 tick하지 않고 core의 별도 종료 경로가 기존 `StopAndAssess`를
실행·조회한다. 이미 정지 평가가 진행 중이면 그 평가와 원래 제한 시간을 유지한다.
이 경로는 놓기나 팔 복귀를 시작하지 않는다. 이전 작업의 최종 관측을 같은 기한 안에서
수집하고, 확인된 물체 상태와 놓기·팔 복귀 진행을 보존한다.
정지와 이전 작업 종료가 확인되면 `FATAL / INTERNAL_ERROR`로 종료한다.
이미 관측한 장치 오류·비상정지 등 FATAL 원인은 유지하며, 정지 시작·조회 실패 또는
미종료 작업 때문에 확인 근거를 얻지 못하면 `FATAL / STOP_UNCONFIRMED`로 종료한다.
최종 Result와 종료 이벤트는 한 번만 확정하고, 원래 예외와 추가 정지 오류를 message에 남긴다.
BT snapshot 조회 실패는 해당 실행의 모니터 갱신만 중단하고 정지 처리와 Action 결과 전달은 계속한다.

실행 준비 확인 또는 XML 구성 실패는 Goal을 거절한다. 실행 ID를 예약한 뒤 backend
context 초기화가 실패하면 해당 실행을 수락한 뒤 위 오류 종료 경로로 최종 결과를 반환한다.
이 처리는 서버 프로세스가 살아 있는 동안의 실행 예외에 적용한다. 프로세스 종료 후의
처리는 아래 재시작 복구 계약을 따른다.

## 기록과 모니터

기본 DB는 `$XDG_STATE_HOME/cleany/manipulation_mujoco/executions.sqlite3`이다.
`XDG_STATE_HOME`이 없으면 `~/.local/state`를 사용한다. mock DB와 분리되어 있으며
`bt_database_path:=/절대/경로.sqlite3`로 바꿀 수 있다. 각 실제 동작의 시작 checkpoint를
먼저 메모리에 반영하고 SQLite 저장을 시도한다. 완료 관측과 최종 Result도 같은 방식으로 처리한다.
`bt_transitions` 테이블에는 C++ 실행기의 실제 UID/상태 전환을 추가 저장한다.
실행 기록이나 BT 상세 로그 저장 실패는 경고로 남기며 동작 중단, Result의 status·error_code 변경,
후속 Goal 차단의 원인으로 사용하지 않는다. 조회는 프로세스 메모리의 최신 기록을 반환한다.
프로세스 중단 기록은 재시작 시 INTERRUPTED/사람 확인 필요로 바꾸고 자동 재개하지 않는다.
HELD/UNKNOWN 또는 FATAL이 남으면 후속 Goal을 차단한다.
재시작 복구와 재시작 이후 중복 검사는 디스크에 저장된 기록만 사용하며 미저장 메모리 기록은 유실된다.
기동 시 DB 초기화와 기존 기록 읽기는 여전히 필수이며, 실패하면 서버를 기동하지 않는다.
새 실행은 이전 worker와 ROS 요청이 종료되어야 허용하며 늦은 응답은 기존 context에만 적용된다.

VS Code BehaviorTree Viewer 0.1.2의 Monitor에서 기본 `127.0.0.1:1667`로 연결한다.
레포 루트를 VS Code workspace로 열면 [.vscode/settings.json](../../../.vscode/settings.json)이
이 주소를 설정한다. 다른 폴더를 workspace로 열었으면 VS Code 설정에
`behaviortreeViewer.monitorHost=127.0.0.1`, `behaviortreeViewer.monitorPort=1667`을 지정한다.
기존 Monitor 연결은 끈 뒤 다시 켜야 변경된 주소를 사용한다. 연결되면 파일명에 `(live)`와
`Monitoring active`가 표시된다. Goal을 보내기 전에는 실제 노드가 모두 IDLE인 것이 정상이다.
설치된 확장의 기본 포트는 1666이므로 설정 없이 MuJoCo 서버에 연결할 수 없다.
`bt_monitor_port`로 변경할 수 있다. FULLTREE/STATUS만 제공하며 실제 C++ 트리의 UID/상태를
사용한다. 실행이 끝나며 BT가 IDLE로 reset한 자식은 그대로 표시하고 전환 이력은 ROS/SQLite에 남긴다.
mock 모니터의 `1666`과 모의 XML은 유지한다. 모니터는 로봇 명령/blackboard 변경을 지원하지 않는다.
실행 종료 뒤 native BT가 root를 포함한 노드를 IDLE로 reset하면 뷰어의 진행 색도 정리된다.
뷰어 0.1.2는 전체 IDLE 상태가 1.5초 이어지면 표시를 흐리게 한다. 이는 Live 연결 종료나
Result 기록 삭제를 뜻하지 않는다. 종료 결과와 실패 단계는 조회 서비스/클라이언트로 확인한다.

[config/manipulation_mujoco.yaml](config/manipulation_mujoco.yaml)의 ROS parameter로
동작 상한(180초), 단계별 안착 확인(10초), 정지 평가(10초), 피드백/시계 신선도(2초)와
지원 label을 조정한다. 이 수치는 첫 시뮬레이션 구현의 조정 가능한 기본값이다.
전체 물체 수거 성공률이나 실제 하드웨어에 대한 보장을 뜻하지 않는다.

## 검증 기록 (2026-10-01)

Ubuntu 22.04 arm64 / ROS Humble, BehaviorTree.CPP 4.10.0, 기존 head/손목 YOLOE 모델의
CPU 추론과 실제 Gemini API를 사용했다. 독립 시뮬레이션 실행별로 별도 `/tmp` DB를 지정했다.
실패/취소 기록을 지우거나 보유 중인 물체의 자동 재개를 허용하지 않았다.

| 실제 MuJoCo 시험 | 결과 |
|---|---|
| 승인된 컵 한 개 수거 | `SUCCESS / NONE`, `LEFT_GRIPPER / CONFIRMED`, `arm_recovered=true`, `stop_confirmed=true` |
| `GRASPING/GraspObject`에서 취소 | 닫기·파지 확인 후 정지, 들기 시작 없음. `CANCELED / HELD`, `stop_confirmed=true` |
| 취소 후 후속 Goal | 사람 확인 필요 기록을 유지하며 Goal 거절 |
| controller 추종 실패 | 정상 경로 중단, 자동 재실행 없음, `FAILED / MOTION_FAILED`, 실제 정지 확인 후 조회 |
| 이동 시작 시 시뮬레이터/controller 프로세스 일시 정지 | 시계·관절 수신 중단 감지, `FATAL / STOP_UNCONFIRMED`, `stop_confirmed=false`. Result 조회 후 프로세스를 복원하여 시험 종료 |

성공 Goal의 snapshot은 `rgbd-0000000001042000000-000003`, `object_id=2`, 관측 label은
`cup`, 목적지는 `trash_right`였다. 실행 ID `7c1918eb5b4a48a587997a05860d5152`의
DB(`/tmp/cleany-bt-physical-clean.sqlite3`)에는 checkpoint 39개와 실제 BT 전환 72개가 남았다.
방출 뒤와 팔 복귀 뒤 평가가 각각 성공했고, Action Result와 조회 서비스 결과가 일치했다.
실제 ROS 이동 중 모니터의 UID 10(`MoveToPregrasp`)이 `RUNNING`인 것도 확인했다.
취소 실행 ID는 `d123a39d4c5643d19f8d8188adedf8ec`이며 별도 취소 DB에 기록했다.
일시 정지 실행 ID는 `8c9b82db607848d5b019ec6c61c63e33`이며
`/tmp/cleany-bt-physical-paused.sqlite3`에 FATAL 기록을 보존했다.

`make test-manipulation-bt`의 272개 검사와 `make test-manipulation`의 기존 mock 102개 검사가
통과했다. `make test-grasp-pregrasp`로 관련 C++ 지도/관측과 기존 인식·파지·수거·MoveIt·MuJoCo
회귀 검사도 통과했다. native BT 실패 경로, 원자적 취소, stale/잘못된 대상 차단, false 평가 응답의
확인 대기, 늦은 응답 격리, stop 근거 누락과 기록 실패, 재시작 자동 재개 금지를 포함한다.
이 기록은 첫 컵 한 개의 연동 검증이며 다른 대상의 수거 성공률 측정은 아니다.
