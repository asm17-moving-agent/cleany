# 모의 Manipulation Action 사용법

[패키지 안내로 돌아가기](../README.md)

**물체 하나의 `collect_trash` 또는 `collect_lost_item`을 요청하고, 진행·결과·실행 기록을 터미널에서 확인한다.**
현재 실행 profile은 `mock`이다. 팔과 그리퍼 동작 및 확인 근거를 모의로 생성하며,
팔 애니메이션이나 실제 로봇 명령은 제공하지 않는다.

| 하려는 일 | 안내 |
|---|---|
| 처음 실행해 보기 | [빠른 시작](#빠른-시작) |
| 진행 중 취소하거나 결과 다시 조회하기 | [취소와 기록 조회](#취소와-기록-조회) |
| 집기 실패·정지 실패 등을 재현하기 | [실패 상황 재현](#실패-상황-재현) |
| namespace·대상·제한 시간 바꾸기 | [설정](#설정) |
| SQLite와 재시작 동작 이해하기 | [SQLite 기록과 재시작](#sqlite-기록과-재시작) |
| 자동 검사 실행하기 | [검증](#검증) |
| Action·Service·Topic 계약 확인하기 | [ROS 인터페이스와 실행 구조](#ros-인터페이스와-실행-구조) |

## 빠른 시작

아래 명령은 **bash 터미널 두 개**, **레포 루트**에서 실행한다.
처음 개발환경을 준비한다면 [설치 가이드](../../../../docs/DEVELOPMENT_SETUP.md)를 따른다.

**1. 터미널 A: 빌드하고 서버 실행**

```bash
cd /home/ubuntu/Documents/cleany
make build-manipulation
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

mock_db_dir="$(mktemp -d)"
ros2 launch cleany_skill_executor manipulation_mock.launch.py \
  database_path:="$mock_db_dir/executions.sqlite3"
```

`Mock manipulation ready`가 나오면 요청을 받을 준비가 된 것이다.
`mock_db_dir`에는 이번 테스트의 독립 DB 경로가 저장된다. 서버 종료는 `Ctrl+C`다.

**2. 터미널 B: 정상 수거 요청**

```bash
cd /home/ubuntu/Documents/cleany
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run cleany_skill_executor manipulation_test_client
```

기본 요청은 `mock-snapshot-001`의 **1번 물체**, 목적지 `mock_trash_bin`, 팔 `left`다.
약 5초 동안 다음 순서의 Feedback을 출력한다.

```text
VALIDATING → PREPARING_TARGET → APPROACHING → GRASPING → LIFTING
→ TRANSPORTING → PLACING → RETURNING_ARM → VERIFYING_PLACEMENT → FINALIZING
```

**3. 출력에서 확인할 값**

| 출력 | 의미 |
|---|---|
| 첫 줄의 `execution_id` | 클라이언트가 발급한 UUID4. 기록 조회 때 사용 |
| 단계별 Feedback | 지금 진행 중인 단계와 설명 |
| 첫 번째 JSON | Action의 최종 Result |
| 두 번째 JSON | 조회 Service로 읽은 최신 실행 기록 |

정상 종료 시 Result는 `status=SUCCESS`, `execution_profile=mock`,
`error_code=NONE`, `object_state=LEFT_GRIPPER`, `placement_state=CONFIRMED`,
`arm_recovered=true`, `stop_confirmed=true`다.
조회 결과는 `found=true`, `record_state=FINISHED`, `has_result=true`다.

같은 단계에서도 시작, 관측 결과 수신, 단계 완료를 각각 Feedback으로 알린다.
클라이언트는 `stage/substage`로 세부 동작을 표시한다. 예를 들어 집기는 다음과 같다.

```text
GRASPING/GraspObject Starting GRASPING
GRASPING/ConfirmGrasp Substage completed: GraspObject; starting ConfirmGrasp
GRASPING/ConfirmGrasp Observation received: Mock contact observation
GRASPING/ConfirmGrasp Stage completed: GRASPING
FINALIZING Finalizing result: Mock collection verified
FINALIZING Execution finished: SUCCESS; Mock collection verified
```

`Finalizing result`는 결과 확정 시작, `Execution finished`는 실행 종료를 알린다.
`revision`도 작업 횟수가 아니라 기록 갱신 번호다.

기록의 `completed_substages`는 실제 완료한 모의 세부 단계이며, 실패 시 Result의
`failed_substage`로 그리퍼 명령 실패와 파지 확인 실패 등을 구분한다.
그리퍼 닫기 완료만으로 물체 보유를 확정하지 않는다.

**세부 단계를 천천히 관찰하기**

기존 서버를 종료하고 터미널 A에서 아래 명령을 실행한다. 별도 임시 설정 파일 없이
각 세부 단계를 약 3초, 전체 정상 흐름을 약 51초 동안 확인할 수 있다.

```bash
mock_db_dir="$(mktemp -d)"
ros2 launch cleany_skill_executor manipulation_mock.launch.py \
  database_path:="$mock_db_dir/executions.sqlite3" monitor:=true \
  mock_config:="$PWD/ros2_ws/src/cleany_skill_executor/config/manipulation_mock_slow.yaml"
```

이 설정의 시나리오는 `success`만 제공한다. 원래 설정의 `stage_duration_sec`는 큰 단계
전체 시간이며 세부 단계 수에 나눠 적용한다. 느린 설정은 단계별 시간과 timeout을 함께 늘렸다.

## VS Code에서 로컬 트리 모니터링

설치된 **BehaviorTree Viewer** 확장(NicholasJamesBell, 0.1.2)의 Monitor 기능으로
모의 Action의 단계별 진행을 표시한다. Python 이벤트 브리지가 `FULLTREE`·`STATUS`
ZMQ 요청에 응답하는 로컬 테스트 기능이며 BT.CPP 실행기나 Groot2 전체 프로토콜 구현은 아니다.

1. 빠른 시작의 터미널 A에서 launch에 `monitor:=true`를 추가한다.

   ```bash
   mock_db_dir="$(mktemp -d)"
   ros2 launch cleany_skill_executor manipulation_mock.launch.py \
     database_path:="$mock_db_dir/executions.sqlite3" monitor:=true
   ```

2. VS Code에서 이 패키지의 `docs/groot2_table_cleanup.xml`을 열고
   `BehaviorTree: Open Behavior Tree Viewer`를 실행한다. 뷰어에서 **Monitor**를 누른다.
   확장 설정은 `behaviortreeViewer.monitorHost=127.0.0.1`,
   `behaviortreeViewer.monitorPort=1666`을 사용한다. 레포 workspace 설정은 MuJoCo용
   1667이므로 mock을 모니터링할 때는 1666으로 바꾼다.
3. 터미널 B에서 `ros2 run cleany_skill_executor manipulation_test_client`를 실행한다.
   실행 중 파란색, 완료 초록색, 실패 빨간색으로 바뀐다. 끝난 상태는 다음 Goal까지 유지된다.
   같은 명령을 다시 실행해 새로운 실행을 볼 수 있다.
4. 실패를 보려면 서버를 종료한 뒤 새 임시 DB와 `scenario:=grasp_failure`로 시작한다.
   뷰어를 다시 연결하고 클라이언트를 실행하면 집기 노드가 실패하고 뒤의 동작은 미실행으로 남는다.

서버가 별도로 실행 중이면 같은 namespace에서 브리지만 시작할 수도 있다.

```bash
ros2 run cleany_skill_executor manipulation_monitor --ros-args -r __ns:=/mock
```

| 항목 | 동작 |
|---|---|
| 입력 | namespace 내 `manipulation/execution_events`, Reliable·Transient Local |
| 연결 | loopback `127.0.0.1:1666`, launch의 `monitor_port`로 변경 가능 |
| 표시 범위 | 최신 `execution_profile=mock` 실행 하나. revision으로 중복·오래된 기록 제외 |
| 표시 근거 | `stage`, `last_completed_stage`, Result. 단계 시작만으로 성공 처리하지 않음 |
| 실패·취소 | BT 상태는 FAILURE로 투영. `BLOCKED`·`CANCELED`·`FATAL` 구분은 Action Result 확인 |
| 미지원 | blackboard 실제 값, breakpoint, fault injection, 로봇 제어, 실행 이력 replay |

XML은 서버가 실행하는 코드가 아니라 이벤트를 표시하는 도식이다. 특히 종료 경로는
Result를 화면에 투영한 것으로, XML의 각 노드를 실제로 tick했다는 의미가 아니다.
서버 재시작으로 `INTERRUPTED`가 된 기록은 성공한 정지·종료 처리로 표시하지 않는다.
연결 직후 최근 보존 이벤트를 읽으며 전체 과거 기록을 재생하지는 않는다.

브리지는 `python3-zmq`가 필요하다([개발환경 안내](../../../../docs/DEVELOPMENT_SETUP.md#로컬-action-트리-모니터-의존성)).
확장에 **native zeromq binary unavailable**이 나오면 모니터링용 네이티브 의존성이
설치되지 않은 것이다. 정적 미리보기와 별개이며 해당 플랫폼의 zeromq 바이너리를 준비한 뒤
VS Code의 `Developer: Reload Window`를 실행한다. Remote SSH에서는 확장도 원격 VM에 설치한다.
PC의 로컬 확장에서 VM 브리지에 연결한다면 SSH로 1666 포트를 전달해야 한다.

## 취소와 기록 조회

아래 명령은 **빠른 시작의 터미널 B**에서 실행한다.

**운반 중 취소**

```bash
ros2 run cleany_skill_executor manipulation_test_client \
  --cancel-stage TRANSPORTING
```

`STOPPING` 이후 `status=CANCELED`, `stop_confirmed=true`, `object_state=HELD`를
확인한다. 기본 `CHECKPOINT`에서는 물체를 들고 있어도 놓기나 팔 복귀를 실행하지 않는다.
물체 보유 상태가 남으므로 이후 새 Goal은 거절된다.
다른 독립 테스트를 하려면 터미널 A에서 서버를 종료하고 새 임시 DB로 실행한다.

집기·들기·놓기 중 취소는 현재의 짧은 원자 구간을 완료하거나 제한 시간에서 차단한
뒤 정지를 확인한다. 취소 후 다음 동작은 시작하지 않는다. fault는 취소보다 우선하며
정지를 확인하지 못하면 `FATAL/STOP_UNCONFIRMED`다.

**현재 작업을 중단하고, 잡고 있는 물체를 그 자리에서 놓은 뒤 팔 복귀**

```bash
ros2 run cleany_skill_executor manipulation_test_client \
  --cancel-stage TRANSPORTING --cancel-mode RETURN_ARM
```

`STOPPING → RELEASING_IN_PLACE → RECOVERING_ARM → STOPPING → FINALIZING`으로
종료한다. `CANCELED`, `cancel_mode=RETURN_ARM`, `object_state=LEFT_GRIPPER`,
`arm_recovered=true`, `stop_confirmed=true`이며 수거함 확인은 실행하지 않는다.
빈 팔에서는 놓기를 건너뛴다. 현재 위치 놓기의 추가 위치 정책은 후속 검토 대상이다.
`--cancel-mode IMMEDIATE`는 현재 완료를 기다리지 않고 정지를 요청한다.

이미 실행 중인 Goal에는 `CancelManipulation` 서비스를 사용한다. `/sim`에서도 같은
서비스와 모드를 제공한다. 모드 설정과 기본 Action 취소를 따로 호출할 필요는 없다.

```bash
ros2 service call /mock/manipulation/cancel cleany_interfaces/srv/CancelManipulation \
  "{execution_id: '실행-UUID', mode: 'RETURN_ARM'}"
```

놓기·복귀 중 `IMMEDIATE`로 선점할 수 있으며 이미 즉시 정지를 요청했으면 완화되지 않는다.

**기존 실행 기록 조회**

첫 줄에 출력된 UUID를 `execution_id`에 넣는다. 조회는 새 Goal을 보내지 않는다.

```bash
execution_id='클라이언트가 출력한 UUID'
ros2 run cleany_skill_executor manipulation_test_client \
  --query --execution-id "$execution_id"
```

응답 유실이나 클라이언트 timeout 뒤에도 같은 ID로 먼저 조회한다.
같은 execution_id로 Goal을 다시 보내면 중복 요청으로 거절된다.

## 실패 상황 재현

**서버 시나리오를 바꾸는 경우:** 터미널 A에서 기존 서버를 `Ctrl+C`로 종료하고,
독립 임시 DB와 `scenario`를 지정해 다시 실행한다.

```bash
mock_db_dir="$(mktemp -d)"
ros2 launch cleany_skill_executor manipulation_mock.launch.py \
  database_path:="$mock_db_dir/executions.sqlite3" \
  scenario:=grasp_lost
```

터미널 B에서는 정상 수거 명령을 다시 실행한다.
`grasp_lost`는 운반 중 `FAILED/GRASP_LOST`, `object_state=UNKNOWN`으로 끝난다.

| `scenario` | 재현 상황 | 예상 결과 |
|---|---|---|
| `success` | 정상 수거 | `SUCCESS` |
| `release_unobserved` | 그리퍼 이탈 근거 없이 독립 확인까지 대기 | 확인 후 `SUCCESS` |
| `backend_not_ready` | 시작 준비 불가 | `BLOCKED/BACKEND_NOT_READY` |
| `verification_unavailable` | 확인 기능 준비 불가 | `BLOCKED/VERIFICATION_UNAVAILABLE` |
| `grasp_failure` | 집기 실패 | `FAILED/GRASP_FAILED` |
| `grasp_lost` | 운반 중 물체 상태 유실 | `FAILED/GRASP_LOST` |
| `placement_failure` | 수거함 내부 확인 실패 | `FAILED/PLACEMENT_NOT_CONFIRMED` |
| `verification_timeout` | 확인 제한 시간 초과 | `FAILED/VERIFICATION_TIMEOUT` |
| `return_failure` | 팔 복귀 실패, 독립 수거 근거 보존 | `FAILED/MOTION_FAILED`, `placement_state=CONFIRMED` |
| `stop_failure` | 취소·실패 후 정지 확인 실패 | `FATAL/STOP_UNCONFIRMED` |
| `stop_timeout` | 취소·실패 후 정지 확인 제한 시간 초과 | `FATAL/STOP_UNCONFIRMED` |
| `hardware_fault` | 운반 중 모의 hardware fault | `FATAL/HARDWARE_ERROR` |
| `e_stop` | 들기 중 모의 e-stop | `FATAL/E_STOP` |

`stop_failure`와 `stop_timeout`에서는 클라이언트에 `--cancel-stage TRANSPORTING`을
추가해 정지 경로를 실행한다. 정상 수거 요청만으로는 이 오류가 발생하지 않는다.

**Goal 인자로 준비 실패를 재현하는 경우:** `success` 서버에서 아래 명령 중 하나를 실행한다.
모두 움직임 전에 `BLOCKED`로 끝난다.

```bash
# 없는 대상: TARGET_UNAVAILABLE
ros2 run cleany_skill_executor manipulation_test_client --object-id 99

# 오래된 대상: STALE_TARGET
ros2 run cleany_skill_executor manipulation_test_client --snapshot-id mock-stale-snapshot

# 잘못된 목적지: DESTINATION_UNAVAILABLE
ros2 run cleany_skill_executor manipulation_test_client --destination-id missing
```

없는 snapshot도 `--snapshot-id missing`으로 재현할 수 있다.

## 설정

**서버 launch 인자**

| 인자 | 기본값 | 용도 |
|---|---|---|
| `namespace` | `mock` | ROS 이름의 namespace |
| `database_path` | [기본 DB 경로](#sqlite-기록과-재시작) | 실행 기록 파일 |
| `scenario` | `success` | 위 표의 모의 시나리오 |
| `mock_config` | package의 `config/manipulation_mock.yaml` | 대상 fixture와 모의 시간 설정 |

예를 들어 서버를 `namespace:=demo`로 실행하면 클라이언트에도 `--namespace /demo`를
전달한다. 설정 변경은 서버 재시작 때 반영한다. 노드의 `backend` parameter는
`mock`만 지원하며 다른 값은 시작 시 거절한다.

**테스트 클라이언트 옵션**

| 옵션 | 기본값 또는 동작 |
|---|---|
| `--namespace` | `/mock` |
| `--snapshot-id`, `--object-id` | `mock-snapshot-001`, `1` |
| `--destination-id` | `mock_trash_bin` |
| `--skill-name` | `collect_trash`. 분실물은 `collect_lost_item`과 `--destination-id mock_lost_item_bin` 지정 |
| `--mission-id`, `--task-id` | `mock-mission`, `mock-task` |
| `--execution-id` | 생략하면 UUID4 발급 |
| `--cancel-stage` | 지정 단계에서 취소 요청 |
| `--cancel-mode` | `CHECKPOINT`(기본), `IMMEDIATE`, `RETURN_ARM` |
| `--query` | `--execution-id`의 기록만 조회 |
| `--timeout` | Result 응답 대기 120초. 서버의 단계 제한 시간과 별개 |

전체 옵션은 `ros2 run cleany_skill_executor manipulation_test_client --help`로 확인한다.

**모의 YAML 설정**

[`config/manipulation_mock.yaml`](../config/manipulation_mock.yaml)에서 관리한다.
아래 수치는 모의 검증용이며 실물 안전 기준으로 사용하지 않는다.

| 항목 | 기본값 |
|---|---|
| 대상 | `mock-snapshot-001`의 물체 1·2·3 |
| 목적지 / 선택 팔 | `mock_trash_bin` / `left` |
| 단계 실행 / 정지 확인 시간 | 0.5초 / 0.1초 |
| 준비 / 확인 제한 시간 | 각각 5초 |
| 동작 / 정지 제한 시간 | 단계별 10초 / 1초 |
| 관찰 최대 age | 30초. snapshot age는 모의 fixture 값 |
| 자동 재시도 | 0회 |

## SQLite 기록과 재시작

현재 서버는 최신 실행 기록을 메모리에 유지하고 SQLite에도 저장을 시도한다.
같은 프로세스의 중복 검사와 결과 조회는 메모리 기록을 사용하며, 재시작 복구와
재시작 이후 중복 검사는 디스크에 저장된 기록을 사용한다.
Python 표준 라이브러리의 `sqlite3`를 사용하므로 별도 DB 서버나
`sqlite3` 명령어 설치는 필요 없다. 서버가 DB 파일을 자동으로 만든다.

기본 경로는 `${XDG_STATE_HOME:-~/.local/state}/cleany/manipulation_mock/executions.sqlite3`다.
빠른 시작에서는 `database_path`를 지정해 테스트별 임시 DB를 사용한다.

| `record_state` | 의미 |
|---|---|
| `ACTIVE` | 수락한 실행이 진행 중 |
| `FINISHED` | 실행 종료와 Result 확정. 디스크 저장 완료를 보장하지 않음 |
| `INTERRUPTED` | 재시작 시 발견한 미완료 실행. `has_result=false`, 사람 확인 필요 |
| `RECORDING_FAILED` | 이전 구현의 저장 실패 진단. 현재 서버는 새 기록에 사용하지 않음 |

**같은 DB로 재시작할 때**

- 미완료 기록을 `INTERRUPTED`, `human_confirmation_required=true`로 표시한다.
- 마지막 물리 근거는 보존하고 정지를 추정하거나 가짜 Action Result를 생성하지 않는다.
- 중단 기록을 조회 Service와 복구 이벤트로 제공한다. 이전 Goal을 자동 재개하지 않는다.
- fault·중단·물체 보유·상태 불명이 남으면 신규 Goal을 차단한다.

이번 버전에는 reset과 기록 자동 만료가 없다. 다른 모의 테스트는 새 임시 DB로 분리한다.
기존 DB 삭제를 물리 복구 완료로 간주하지 않는다.

저장 실패는 경고 로그로 남기고 실제 실행의 status와 error_code를 유지한다.
저장 실패만으로 동작이나 신규 실행을 차단하지 않는다. 최신 진행과 결과는 메모리에 유지하고
이후 갱신 시 저장을 다시 시도한다. 최초 저장이 실패했으면 다음 갱신에서 기록 생성을 다시 시도한다.
함께 발생한 hardware fault·e-stop·정지 확인 실패와 물체 보유·상태 불명에 따른 차단은 유지한다.
디스크에 남은 마지막 활성 기록은 재시작 때 중단으로 복구되지만, 미저장 메모리 기록은 유실된다.
기동 시 DB 초기화와 기존 이력 읽기가 실패하면 서버는 기동하지 않는다.

## 검증

레포 루트에서 필요한 검사를 선택한다.

| 명령 | 검사 범위 |
|---|---|
| `make test-manipulation-core` | 가짜 시계 기반 core: 단계 진행·실패·취소·기록·복구 |
| `make test-manipulation` | 관련 패키지 빌드, 실제 ActionClient 통신, 동시·중복 요청, 저장 실패, SIGKILL 후 복구 |

기존 Skill Executor 전체 회귀 검사는 ROS setup을 읽은 터미널에서 실행한다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ros2_ws/src/cleany_skill_executor/test
```

## ROS 인터페이스와 실행 구조

| 기본 ROS 이름 | 타입 | 용도 |
|---|---|---|
| `/mock/manipulation/execute_skill` | `ExecuteManipulationSkill` Action | 요청·Feedback·Result·취소 |
| `/mock/manipulation/cancel` | `CancelManipulation` Service | execution_id와 취소 방식 지정 |
| `/mock/manipulation/get_execution` | `GetManipulationExecution` Service | execution_id로 최신 기록 조회 |
| `/mock/manipulation/execution_events` | `ManipulationExecutionRecord` Topic | 수락·단계·물리 근거·종료·복구 이벤트 |

| Result `status` | ROS 종료 |
|---|---|
| `SUCCESS`, `BLOCKED` | `SUCCEEDED`. Result payload의 status도 확인 |
| `FAILED`, `FATAL` | `ABORTED` |
| `CANCELED` | 정지 확인 후 `CANCELED` |

ID·skill 오류, 동시 실행, 중복 execution_id와 기존 fault는 수락 전에 거절하며
일반 Result payload가 없다. 상세 필드·단계·오류는
[Action 명세](02_execute_manipulation_skill_action_spec.md)를 따른다.

ROS wrapper, 순수 Python core, 동작 port, Mock adapter와 SQLite 저장소를 분리했다.
20Hz steady-clock timer와 Reentrant callback, 4-thread executor를 사용하고,
수락·취소·진행 변경·최종 결과는 같은 lock으로 직렬화한다.
SQLite는 실행 ID UNIQUE 제약, DB별 파일 lock, WAL/FULL 동기화를 사용한다.
수락·단계 시작·확인된 물리 변화·Result의 최신 기록과 이력을 동일 트랜잭션으로 저장한다.

이벤트 QoS는 Reliable·Transient Local, depth 100이다. 최신 100개 이벤트를 재수신할 수
있으며 SQLite 저장에 성공한 이력은 `execution_events`에 보존한다.
이 mock 경로는 실제 Perception·팔 backend를 사용하지 않는다. 실제 MuJoCo BT 실행은
[cleany_manipulation_bt](../../cleany_manipulation_bt/README.md)를 따르며, Mission Manager·Backend 연결은 후속 단계다.
