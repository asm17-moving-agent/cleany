# 모의 Manipulation Action 사용법

[패키지 안내로 돌아가기](../README.md)


**물체 하나의 `collect_trash`를 요청하고, 진행·결과·저장 기록을 터미널에서 확인한다.**
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
| 두 번째 JSON | 조회 Service로 읽은 저장 기록 |

정상 종료 시 Result는 `status=SUCCESS`, `execution_profile=mock`,
`error_code=NONE`, `object_state=LEFT_GRIPPER`, `placement_state=CONFIRMED`,
`arm_recovered=true`, `stop_confirmed=true`다.
조회 결과는 `found=true`, `record_state=FINISHED`, `has_result=true`다.

같은 단계에서도 시작, 관측 결과 수신, 단계 완료를 각각 Feedback으로 알린다.
예를 들어 집기는 다음과 같이 출력된다. 동작을 세 번 실행한 뜻은 아니다.

```text
GRASPING Starting GRASPING
GRASPING Observation received: Mock contact observation
GRASPING Stage completed: GRASPING
FINALIZING Finalizing result: Mock collection verified
FINALIZING Execution finished: SUCCESS; Mock collection verified
```

`Finalizing result`는 결과 확정 시작, `Execution finished`는 실행 종료를 알린다.
`revision`도 작업 횟수가 아니라 기록 갱신 번호다.

## 취소와 기록 조회

아래 명령은 **빠른 시작의 터미널 B**에서 실행한다.

**운반 중 취소**

```bash
ros2 run cleany_skill_executor manipulation_test_client \
  --cancel-stage TRANSPORTING
```

`STOPPING` 이후 `status=CANCELED`, `stop_confirmed=true`, `object_state=HELD`를
확인한다. 물체를 들고 있어도 자동 놓기나 팔 복귀를 실행하지 않는다.
물체 보유 상태가 남으므로 이후 새 Goal은 거절된다.
다른 독립 테스트를 하려면 터미널 A에서 서버를 종료하고 새 임시 DB로 실행한다.

집기·들기·놓기 중 취소는 현재의 짧은 원자 구간을 완료하거나 제한 시간에서 차단한
뒤 정지를 확인한다. 취소 후 다음 동작은 시작하지 않는다. fault는 취소보다 우선하며
정지를 확인하지 못하면 `FATAL/STOP_UNCONFIRMED`다.

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
| `--mission-id`, `--task-id` | `mock-mission`, `mock-task` |
| `--execution-id` | 생략하면 UUID4 발급 |
| `--cancel-stage` | 지정 단계에서 취소 요청 |
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

현재 서버는 SQLite에 **중복 실행 방지, 진행 보존, 결과 조회와 재시작 복구**를 위한
기록을 저장한다. Python 표준 라이브러리의 `sqlite3`를 사용하므로 별도 DB 서버나
`sqlite3` 명령어 설치는 필요 없다. 서버가 DB 파일을 자동으로 만든다.

기본 경로는 `${XDG_STATE_HOME:-~/.local/state}/cleany/manipulation_mock/executions.sqlite3`다.
빠른 시작에서는 `database_path`를 지정해 테스트별 임시 DB를 사용한다.

| `record_state` | 의미 |
|---|---|
| `ACTIVE` | 수락한 실행이 진행 중 |
| `FINISHED` | 실행 종료와 Result 저장 완료 |
| `INTERRUPTED` | 재시작 시 발견한 미완료 실행. `has_result=false`, 사람 확인 필요 |
| `RECORDING_FAILED` | 저장 실패. 프로세스 메모리의 진단이며 영속 저장을 주장하지 않음 |

**같은 DB로 재시작할 때**

- 미완료 기록을 `INTERRUPTED`, `human_confirmation_required=true`로 표시한다.
- 마지막 물리 근거는 보존하고 정지를 추정하거나 가짜 Action Result를 생성하지 않는다.
- 중단 기록을 조회 Service와 복구 이벤트로 제공한다. 이전 Goal을 자동 재개하지 않는다.
- fault·중단·물체 보유·상태 불명이 남으면 신규 Goal을 차단한다.

이번 버전에는 reset과 기록 자동 만료가 없다. 다른 모의 테스트는 새 임시 DB로 분리한다.
기존 DB 삭제를 물리 복구 완료로 간주하지 않는다.

저장 실패는 `FATAL/INTERNAL_ERROR`로 처리하고 신규 실행을 차단한다.
함께 발생한 hardware fault·e-stop·정지 확인 실패의 치명 원인은 보존한다.
디스크에 남은 마지막 활성 기록은 재시작 때 중단으로 복구된다.

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
있으며 전체 이력은 SQLite `execution_events`에 보존한다.
실제 Perception·팔 backend, Mission Manager·Backend 연결과 BT.CPP 실행기는 후속 단계다.
