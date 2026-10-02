# 02. Manipulation Action 인터페이스 명세

> **모의 1차 구현 계약.** `ExecuteManipulationSkill.action`과 `manipulation/execute_skill`을 구현했다.
> 기본 namespace는 `/mock`이며 Result의 `execution_profile`은 `mock`이다.
> KB는 책임의 의미를 정한다. 실물 확인·정지와 외부 승인 계약은 후속 검토 대상이다.

## 1. Goal 하나가 하는 일

**현재 제안의 지원 행동은 `collect_trash` 하나다.** Mission Manager가 승인한 물체
하나를 지정 수거함에 옮기고 결과를 확인한다. 접근, 집기와 운반은 서버 내부 단계다.
다음 물체를 제안하려면 이번 행동 결과와 새 Scene이 필요하다.

| 범위 | 처리 |
|---|---|
| 승인된 쓰레기 후보 하나 수거 | 이번 Action 제안의 범위 |
| 분실물 보관 또는 인계 | KB에서 미정. 제품 Action의 지원 행동으로 등록하지 않음 |
| 책상 전체 반복 | Mission Manager와 Planner의 책임 |
| 인식만 수행 | 독립 Perception interface 사용 |

시뮬레이션 sorting의 분실물 경로, staging과 handoff는 기존 데모 기능이다.
이를 쓰레기 수거 완료나 제품 분실물 정책으로 자동 변환하지 않는다.

## 2. Goal: 무엇을 실행할지

| 필드 | 타입 | 쉽게 설명하면 | 예시 |
|---|---|---|---|
| `mission_id` | `string` | 어떤 미션인가 | `mission-001` |
| `task_id` | `string` | 어떤 승인된 행동인가 | `task-003` |
| `execution_id` | `string` | 이번 실행 시도의 고유 번호 | UUID |
| `skill_name` | `string` | 승인된 행동 이름 | `collect_trash` |
| `snapshot_id` | `string` | 대상이 확인된 관찰 | `snapshot-010` |
| `object_id` | `uint32` | 그 관찰 안의 물체 번호 | `3` |
| `destination_id` | `string` | 설정에 등록된 수거 목적지 | 예: `onboard_trash_bin` |

`object_id`는 snapshot마다 다시 붙을 수 있다. `(snapshot_id, object_id)`로 조회하며
다음 관찰의 3번이 같은 물체라고 가정하지 않는다. 목적지 예시는 실제 설정 ID가 아니다.
`work_area_id`는 이번 Goal에 넣지 않는다. 좌석과 접근 위치는 미션 및 Navigator 계약,
물리 workspace는 서버 설정과 backend가 검사한다.

### 식별자와 승인

- 미션 식별자는 기존 값을 전달하고 task는 승인된 proposal에 연결한다.
- task ID의 재관찰 간 유지 및 발급 규칙은 Planner adapter에서 정해야 한다.
- 호출 adapter가 `execution_id`를 생성하고 실제 움직임 전에 기록한다.
- 새 실행 ID로 재시도할 때도 이전 시도의 물체 상태를 먼저 확인한다.
- Goal 필드만으로 권한이나 의미 분류를 증명하지 않는다. Mission Manager가 상태,
  allowlist와 인자를 승인하고, 허용 client의 ROS 접근 경계는 구현 시 정의한다.

## 3. Goal 수락과 중복 요청

| 상황 | 제안하는 처리 |
|---|---|
| ID 누락, `object_id=0`, 지원하지 않는 skill | Goal 거절 |
| 다른 Goal 실행 중 | Goal 거절. 활성 행동 선점 금지 |
| fault 또는 정지 확인이 필요한 상태 | Goal 거절 |
| 같은 실행 ID가 실행, 종료 또는 중단 기록에 있음 | 중복 Goal 거절. 다시 움직이지 않음 |
| 같은 실행 ID에 다른 인자 | 충돌로 거절 |
| 구조상 유효한 새 Goal | 수락 후 준비 및 물리 검증 |
| snapshot 조회 실패, 대상 이동 또는 verifier 준비 실패 | 수락 후 `BLOCKED`, 움직임 시작하지 않음 |

ROS Goal 거절에는 일반 Result payload가 없다. 호출 adapter는 거절을 수거 실패나
취소 완료로 만들지 않고 준비 상태와 실행 기록을 확인한다. 기록 조회는 `manipulation/get_execution`이다.

완료 Result를 새 Goal에 자동 재생하는 계약은 사용하지 않는다. 응답을 잃으면 실행 ID로
기록을 조회한다. 이번 구현은 SQLite와 `GetManipulationExecution.srv`를 사용하며 기록을 자동 만료하지 않는다.
조회할 수 없다면 새 ID로 무조건 재실행하지 않는다. 프로세스 재시작 후 이전 활성
실행은 중단으로 기록하고 자동 재개하지 않는다.

### 작업 중 프로그램이나 장치가 다시 켜진 경우

예를 들어 물체를 집던 중 프로그램이 종료되고 다시 켜졌다면, 기존 기록만으로
물체가 그리퍼에 있는지 또는 팔이 멈췄는지 확정할 수 없다.

1. 이전 활성 실행을 중단으로 표시하고 마지막 물리 진행 기록을 보존한다.
2. 중단 사실과 확인 가능한 상태를 Mission Manager의 복구 처리에 전달한다.
3. Mission Manager와 외부 상태 adapter는 **미션 중단과 사람 확인 필요**를 Backend에 알린다.
4. 실제 팔, 그리퍼와 물체 상태 및 안전 조건을 확인하기 전 새 물리 Goal을 허용하지 않는다.
5. 이전 Goal을 자동 재개하지 않는다. 후속 미션이나 행동은 별도 검증과 승인 절차를 따른다.

재시작 자체를 `CANCELED` Result나 `stop_confirmed=true`의 근거로 사용하지 않는다.
기존 Goal에 응답할 수 없으면 실행 기록 조회 및 복구 이벤트로 중단을 전달해야 한다.
이번 구현의 중단 이벤트는 `manipulation/execution_events`의
`ManipulationExecutionRecord`이며 Reliable·Transient Local, depth 100이다.
SQLite 기록과 함께 제공하고 Backend 전달은 후속 Mission Manager adapter가 맡는다.
외부 보고 연결은 [결과 매핑 명세](04_mission_result_mapping.md)를 따르며,
제품 요구는 [KB 재시작 규칙](../../../../docs/cleany-docs/20_TECHNICAL/02%20-%20Robot%20Operations%20and%20Mission%20Dispatch.md)에 있다.

## 4. Feedback: 지금 어디까지 했는지

| 필드 | 타입 | 의미 |
|---|---|---|
| `execution_id` | `string` | 이번 실행 시도 |
| `stage` | `string` | 아래 단계 |
| `selected_arm` | `string` | 선택 전 빈 값, 선택 후 등록된 팔 ID |
| `message` | `string` | 사람이 읽는 설명 |

| 단계 | 화면에서 읽을 뜻 |
|---|---|
| `VALIDATING` | 입력과 준비 상태 확인 |
| `PREPARING_TARGET` | 선택 물체의 좌표, grasp와 경로 준비 |
| `APPROACHING` | 빈 팔을 물체 쪽으로 이동 |
| `GRASPING` | 그리퍼를 닫아 집기 |
| `LIFTING` | 들어 올리고 잡은 상태 확인 |
| `TRANSPORTING` | 물체를 든 채 수거함으로 이동 |
| `PLACING` | 목적지에서 그리퍼를 열어 놓기 |
| `RETURNING_ARM` | 팔을 지정된 안전 위치로 복귀 |
| `VERIFYING_PLACEMENT` | 수거함 내부 위치 확인 |
| `STOPPING` | 취소 또는 실패 후 동작 종료와 정지 확인 |
| `FINALIZING` | 실제 진행과 종료 결과 기록 |

Feedback은 진행 안내다. `PLACING`을 받았다고 물체가 수거된 것은 아니다.
첫 이미지 전달, 책상 재관찰과 베이스 복귀는 이 Action의 단계가 아니다.

모의 구현의 Feedback은 `substage`도 제공한다. 예를 들어 `GRASPING/GraspObject`는
그리퍼 닫기, `GRASPING/ConfirmGrasp`는 파지 확인이다. Result의 `failed_substage`는
실패 세부 동작을 식별하고, 조회 기록의 `completed_substages`는 완료한 세부 동작만
보존한다. 큰 단계의 취소 및 timeout 계약은 유지한다.

## 5. Result: 이번 행동이 어떻게 끝났는지

### 구현 필드

| 필드 | 타입 | 확인하려는 것 |
|---|---|---|
| `execution_id` | `string` | 어떤 시도 결과인가 |
| `execution_profile` | `string` | 이번 구현은 `mock`. 모의 근거와 실물 근거 구분 |
| `status` | `string` | 성공, 차단, 실패, 취소 또는 치명 오류 |
| `error_code` | `string` | 원인. 정상 성공은 `NONE` |
| `failed_stage` | `string` | 문제가 난 단계. 성공은 빈 값 |
| `last_completed_stage` | `string` | 근거를 확인하고 끝낸 마지막 단계. 없으면 빈 값 |
| `object_state` | `string` | 물체를 건드리지 않았는지, 들고 있는지 등 |
| `placement_state` | `string` | 수거함 내부 위치를 확인했는지 |
| `selected_arm` | `string` | 사용한 팔. 선택 전 종료면 빈 값 |
| `stop_confirmed` | `bool` | 종료 시 팔과 그리퍼의 정지를 확인했는가 |
| `arm_recovered` | `bool` | 팔을 지정 안전 위치로 복귀했는가 |
| `retryable` | `bool` | 같은 행동을 다시 검토할 여지가 있는가 |
| `message` | `string` | 사람이 읽는 설명 |

`retryable=true`는 재실행 명령이나 책상 전체 재시작 허가가 아니다. Mission Manager가
재관찰, 승인과 retry budget을 확인한다. false인 Goal을 그대로 자동 반복하지 않는다.

### 상태와 ROS 종료

| `status` | 뜻 | ROS 종료 |
|---|---|---|
| `SUCCESS` | 수거, 놓은 결과 확인과 팔 복귀 완료 | `SUCCEEDED` |
| `BLOCKED` | 움직임 전에 필요한 조건을 통과하지 못함 | `SUCCEEDED`, payload 확인 필요 |
| `FAILED` | 실행을 시작했으나 요구한 결과를 얻지 못함 | `ABORTED` |
| `CANCELED` | 취소 후 checkpoint에서 종료하고 정지를 확인 | `CANCELED` |
| `FATAL` | e-stop, hardware fault 또는 정지 확인 불가 | `ABORTED` |

취소 수락과 취소 완료는 다르다. 정지가 확인되지 않으면 `FATAL`을 반환한다.
전송 연결 유실이나 client timeout은 위 Result를 실제로 받았다는 뜻이 아니다.
정상 종료와 취소 요청이 동시에 오면 종료 상태 변경을 직렬화한다. Result가 이미
확정된 Goal의 취소는 수락하지 않고 기존 결과를 유지한다. 취소를 먼저 수락했다면
최종 물리 진행을 보존하면서 취소 경로를 완료한다. 실제 fault는 취소보다 우선한다.

### 물체 상태와 놓은 결과

| `object_state` | 뜻 |
|---|---|
| `NOT_TOUCHED` | 잡거나 움직이지 않았다는 근거가 있음 |
| `HELD` | 마지막 확인에서 그리퍼로 들고 있음 |
| `LEFT_GRIPPER` | 그리퍼에서 떠난 것을 확인. 목적지 내부인지는 별도 |
| `UNKNOWN` | 접촉, 낙하 또는 센서 유실로 상태를 확정할 수 없음 |

| `placement_state` | 뜻 |
|---|---|
| `NOT_CHECKED` | 수거함 내부 확인을 아직 수행하지 않음 |
| `CONFIRMED` | 놓은 뒤 검증 근거로 수거함 내부 위치를 확인 |
| `NOT_CONFIRMED` | 확인 기능의 성공 조건을 만족하지 못함 |
| `UNKNOWN` | 확인 timeout 또는 센서 문제로 판단 불가 |

`LEFT_GRIPPER`는 그리퍼 열기 명령 성공만으로 기록하지 않는다. 추가 근거가 없으면
`UNKNOWN`이다. 수거함 내부 검증이 성공하면 `LEFT_GRIPPER/CONFIRMED`로 갱신할 수 있다.
기존 `VerifyPlacement.srv`의 false는 명시적인 수거함 밖 판정이 아니다.
false는 `NOT_CONFIRMED`, 호출 불가와 timeout은 `UNKNOWN`으로 구분하는 adapter 제안이다.

### 예시로 읽기

| 상황 | 상태 | 물체 / 놓은 결과 | 중요한 점 |
|---|---|---|---|
| 수거와 확인, 팔 복귀 완료 | `SUCCESS` | `LEFT_GRIPPER / CONFIRMED` | 정지와 복귀도 확인 |
| 시작 전에 snapshot이 없음 | `BLOCKED` | `NOT_TOUCHED / NOT_CHECKED` | 새 관찰 필요 |
| 운반 중 물체 상태를 잃음 | `FAILED` | `UNKNOWN / NOT_CHECKED` | 정지와 재관찰 필요 |
| 확인 응답 false | `FAILED` | 확인된 상태 / `NOT_CONFIRMED` | 밖에 있다고 단정하지 않음 |
| 놓기는 확인됐지만 팔 복귀 실패 | `FAILED` | `LEFT_GRIPPER / CONFIRMED` | 실제 수거 증거 보존 |
| 취소 후 물체를 든 채 정지 | `CANCELED` | `HELD / NOT_CHECKED` | 베이스 자동 복귀 금지 |
| 취소 후 정지 확인 불가 | `FATAL` | 마지막 확인 상태 / 기존 확인 상태 | 신규 Goal 차단 |

성공 Result의 필수 조건은 `error_code=NONE`, `failed_stage=""`,
`last_completed_stage=VERIFYING_PLACEMENT`, `object_state=LEFT_GRIPPER`,
`placement_state=CONFIRMED`, 팔 ID 존재, `stop_confirmed=true`,
`arm_recovered=true`, `retryable=false`다.
`FINALIZING`은 보관 단계이며 마지막 물리 완료 단계로 쓰지 않는다.

실패와 취소에서도 실제로 확인된 물체 및 놓은 결과를 초기값으로 되돌리지 않는다.
`CANCELED`는 `error_code=CANCELED`, `retryable=false`로 반환하고 취소를 관찰한 실행
단계를 `failed_stage`에 남긴다. 정지 실패는 그 단계와 별도로 `STOP_UNCONFIRMED` 원인을
보존한다. 실행 후 실패 및 치명 오류의 재시도 기본 판단은 false다. 준비 중 일시 오류만
정지 및 미접촉 근거가 있는 경우 true 후보로 검토한다. 확인하지 못한 물리 상태는
false 또는 UNKNOWN으로 표시하고 추정해서 성공 값으로 채우지 않는다.

## 6. ROS 정의

값 목록은 위 표를 따르며 Python core는 문자열 Enum을 사용한다.
아래 필드는 [실제 Action 정의](../../cleany_interfaces/action/ExecuteManipulationSkill.action)와 같다.

```text
# Goal
string mission_id
string task_id
string execution_id
string skill_name
string snapshot_id
uint32 object_id
string destination_id
---
# Result
string execution_id
string execution_profile
string status
string error_code
string failed_stage
string last_completed_stage
string object_state
string placement_state
string selected_arm
bool stop_confirmed
bool arm_recovered
bool retryable
string message
string failed_substage
---
# Feedback
string execution_id
string stage
string substage
string selected_arm
string message
```

## 7. 오류와 후속 결정

구현 오류 목록: `NONE`, `INVALID_ARGUMENT`, `TARGET_UNAVAILABLE`, `STALE_TARGET`,
`DESTINATION_UNAVAILABLE`, `BACKEND_NOT_READY`, `VERIFICATION_UNAVAILABLE`,
`GRASP_FAILED`, `GRASP_LOST`, `MOTION_FAILED`, `PLACEMENT_NOT_CONFIRMED`,
`VERIFICATION_TIMEOUT`, `TIMEOUT`, `CANCELED`, `E_STOP`, `HARDWARE_ERROR`,
`STOP_UNCONFIRMED`, `INTERNAL_ERROR`.

반환 단계와 adapter 대응은 [서버 명세](03_manipulation_server_behavior_spec.md)와
[Mission 결과 매핑](04_mission_result_mapping.md)에 정리한다.
클라이언트는 execution_id를 발급하며 테스트 클라이언트는 UUID4를 사용한다.
저장 실패는 `FATAL/INTERNAL_ERROR`, 신규 Goal 차단과 `RECORDING_FAILED` 메모리 진단으로 전달한다.
함께 발생한 hardware fault·e-stop·정지 미확인의 `FATAL` 원인은 보존한다.
`ManipulationExecutionRecord`는 Goal과 최신 진행·Result, `record_state`, `has_result`,
`human_confirmation_required`, revision과 Unix nanoseconds 시각을 제공한다.
`has_result=false`인 중단 기록에는 가짜 status/error_code를 채우지 않는다.
제품 snapshot 보존, 실물 정지 판정·verifier와 허용 client는 후속 결정 대상이다.
