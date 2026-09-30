# 04. Action 결과를 Mission Manager에 연결하는 명세

> **개발 제안.** 아래 enum과 retry 횟수는 현재 Python core의 구현 사실이다.
> KB의 고정 계약으로 취급하지 않는다. 새 ROS adapter와 취소 경로는 구현이 필요하다.

## 1. 결과는 세 단계로 읽는다

```mermaid
flowchart LR
    A["ROS 종료 상태"] --> B["행동 Result<br/>물체, 확인과 정지"]
    B --> C["Mission Manager<br/>기록, 재관찰과 다음 판단"]
    C --> D["최종 MissionReport<br/>전후 관찰과 전체 결과"]
```

물체 하나의 수거 성공은 전체 미션 성공과 다르다. 이후 실패, 남은 대상과 복귀 결과를
함께 보존한다. `SUCCEEDED`만 읽지 말고 Result의 상태와 근거를 확인한다.

## 2. 현재 core가 표현하는 것

| 타입 | 실제 값 또는 필드 | 코드 |
|---|---|---|
| `ModuleResult.status` | `OK`, `BLOCKED`, `FAILED`, `FATAL` | [result.py](../../cleany_mission_manager/cleany_mission_manager/core/result.py) |
| `ModuleResult` | `ok`, `failure_code`, `retryable`, `message`, `data` | 같은 파일 |
| `MissionReport.status` | manager가 만드는 `SUCCESS`, `PARTIAL_SUCCESS`, `HUMAN_REVIEW_REQUIRED`, `FAILED`, `BLOCKED` | [manager.py](../../cleany_mission_manager/cleany_mission_manager/core/manager.py) |
| `MissionReport` | 완료, skip, 실패 지점과 review flag | [models.py](../../cleany_mission_manager/cleany_mission_manager/core/models.py) |
| retry 기본값 | 상태 1회, skill 2회 | `RetryPolicy`, 현재 core 기본값 |

현재 core에는 취소 결과와 행동별 재관찰이 없다. `completed_tasks`에는 실제로
`completed_skills`의 이름을 복사한다. 이를 물체 수나 완료 task ID로 해석하면 안 된다.

## 3. Action 결과 변환 제안

| Action Result | core envelope 후보 | 이어서 할 일 |
|---|---|---|
| `SUCCESS`, 모든 성공 조건 충족 | `OK`, `ok=true`, `retryable=false` | 행동 기록, 재관찰, Planner 재호출 |
| `BLOCKED` | `BLOCKED`, `ok=false`, `retryable=false` | 이유 기록, 안전하면 재관찰. 새 제안 또는 종료 |
| `FAILED`, 정지 확인됨 | `FAILED`, `ok=false` | 물리 진행 보존, 재관찰. 이전 Goal 자동 반복 금지 |
| `FATAL` 또는 정지 확인 불가 | `FATAL`, `ok=false`, `retryable=false` | 물리 정지와 신규 실행 차단, 치명 결과 보고 |
| `CANCELED`, 정지 확인됨 | 기존 envelope로 무손실 변환 불가 | 별도 취소 이벤트로 미션 종료 경로 연결 |
| 응답 유실 또는 client timeout | 실제 Result 없음 | cancel 및 기록 조회. 새 Goal 금지, 종료 상태 미확인 |
| Goal 거절 | 실제 Result 없음 | 인자, busy와 fault 확인. 물리 실행 실패로 단정하지 않음 |

이는 목표 adapter 동작이다. 현재 manager는 `BLOCKED`면 보고로 이동하고
`FAILED`가 retryable이면 같은 skill을 바로 다시 호출한다. checkpoint와 재관찰을
구현할 때 이 부분도 바꿔야 한다. 표를 연결하는 것만으로 목표 폐루프가 완성되지 않는다.

취소를 `FAILED/UNKNOWN_ERROR`로 숨기지 않는다. 취소 종료 이벤트나 결과 타입
확장이 필요하다. fault와 e-stop도 정상 취소로 바꾸지 않는다.

## 4. 오류 원인 대응 후보

| Action `error_code` | 기존 `FailureCode` 후보 | 규칙 |
|---|---|---|
| `NONE` | 없음 | 성공만 사용, core에서는 `None` |
| `TARGET_UNAVAILABLE`, `STALE_TARGET` | `PERCEPTION_FAIL` | snapshot, 선택 물체 또는 위치 검증 실패 |
| `GRASP_FAILED`, `GRASP_LOST` | `GRASP_FAIL` | 잡기 실패와 잡은 상태 유실 |
| `PLACEMENT_NOT_CONFIRMED` | `PLACE_FAIL` | false 응답을 수거함 밖 판정으로 확대하지 않음 |
| `VERIFICATION_TIMEOUT`, `TIMEOUT` | `TIMEOUT` | 실패 단계와 물체 상태도 보존 |
| `MOTION_FAILED` | `SKILL_FAIL` | motion 계획과 실행 실패. hardware fault와 구분 |
| `HARDWARE_ERROR` | `HARDWARE_ERROR` | backend가 hardware fault를 보고한 경우 |
| `E_STOP` | `E_STOP` | 실제 e-stop 경로 활성화 |
| `INVALID_ARGUMENT`, `DESTINATION_UNAVAILABLE` | `PLAN_BLOCKED` 후보 | 수락 뒤 인자 검증에서 발견 |
| `BACKEND_NOT_READY`, `VERIFICATION_UNAVAILABLE` | `SKILL_FAIL` 후보 | 준비 불가 상세와 `BLOCKED` 보존 |
| `INTERNAL_ERROR` | `UNKNOWN_ERROR` | 예외와 단계 보존 |
| `STOP_UNCONFIRMED` | 전용 코드 또는 기존 코드 확장 검토 | hardware fault가 입증됐다고 단정하지 않음 |
| `CANCELED` | 기존 코드 없음 | 취소를 일반 실패로 강제 변환하지 않음 |

기존 코드가 원인을 충분히 담지 못하면 typed `data`에 원래 Result를 유지한다.
새 FailureCode는 구현 검토 후 추가하고 KB에 상수 목록을 복제하지 않는다.
Perception의 장면 관찰 실패는 Perception adapter가 변환한다. 대상 준비 중 실패는
해당 행동의 `PREPARING_TARGET` 지점도 함께 기록한다.

## 5. 행동 완료와 최종 미션 결과

| 상황 | 기록과 판단 |
|---|---|
| 대상 하나 수거 성공 | task와 실행 ID, 확인 근거 기록. 미션은 계속 판단 |
| 앞선 물체 성공, 다음 물체 실패 | 성공 기록과 실패 원인 보존. 전체 성공으로 축약하지 않음 |
| review 또는 닿지 않는 대상 남음 | 미처리 이유와 최신 관찰 유지. Planner 제안과 Mission Manager가 판단 |
| 완료 제안과 최종 관찰 불일치 | 재판단 또는 실패 종료 |
| 첫 유효 관찰이 빈 장면 | 사진과 빈 목록 기록. Action Goal 불필요. 성공 또는 `NO_OBJECTS` 정책은 별도 검토 |
| 놓기는 확인됐지만 팔 복귀 실패 | 수거 증거와 실행 실패, 복귀 제한을 함께 보고 |
| 미션 취소 | 진행 보존, 자동 재개 금지, 취소 사실과 정지 및 물체 상태 보고 |
| 활성 미션 도중 프로세스 또는 장치 재시작 | 진행 보존, 자동 재개 금지, 외부에 **중단과 사람 확인 필요** 보고 |

현재 manager는 review가 있으면 `HUMAN_REVIEW_REQUIRED`, review 없이 skip이 있으면
`PARTIAL_SUCCESS`, 그 외 정상 종료면 `SUCCESS`를 만든다. 후속 실패는 `FAILED`,
차단은 `BLOCKED`, 치명 오류는 즉시 `FAILED` 보고 후 `ERROR`로 간다.
이는 현재 구현 설명이다. 부분 결과, 외부 취소와 중단까지 표현하는 완성 계약은 아니다.

새 보고는 행동별 성공과 실패, 미처리 대상, 전후 observation 참조와 복귀 결과를
보존한다. 대표 상태와 후속 실패의 우선순위는 제품 및 adapter 계약에서 정해야 한다.
그 전에는 실패를 감추는 매핑을 구현하지 않는다.

### 재시작에 따른 중단을 외부 결과에 연결하기

재시작 후에는 기존 Action Goal의 Result를 받지 못할 수 있다. 따라서 실행 기록과
복구 이벤트로 이전 활성 미션을 찾아 아래 의미를 함께 전달해야 한다.

| 보고 내용 | 연결 규칙 |
|---|---|
| 미션 종료 이유 | 작업 중 프로그램 또는 장치 재시작으로 중단됨 |
| 사람 확인 필요 | 기존 `MissionReport.needs_human_review` 필드를 사용하는 adapter라면 `true`로 반영 |
| 마지막 진행 | 실행 ID, 마지막 확인 단계, 물체 상태와 정지 확인 근거 보존. 현재 상태 미확인은 별도 표시 |
| 이미 완료된 행동 | 기록과 근거를 보존. 중단 때문에 성공 기록을 삭제하지 않음 |
| Backend 연결 없음 | 중단 보고를 보관하고 재연결 뒤 상태와 결과를 맞춤 |

**외부에는 중단 여부와 사람 확인 필요가 모두 보여야 한다.** 중단을 일반 실패나
정상 취소로만 바꾸거나 `HUMAN_REVIEW_REQUIRED` 한 값에 중단 이유를 감추지 않는다.
재시작 후의 실제 정지와 물체 상태 확인이 필요하며 이전 행동은 자동 재개하지 않는다.

기존 `needs_human_review` 필드는 있지만 재시작 감지, 복구 이벤트, 중단 상태와 외부
보고 연결은 현재 core에 구현되어 있지 않다. 정확한 외부 status 값과 payload 및
저장소는 adapter 계약에서 정해야 한다.
이 요구의 출처는 [KB Robot Operations](../../../../docs/cleany-docs/20_TECHNICAL/02%20-%20Robot%20Operations%20and%20Mission%20Dispatch.md)다.

## 6. 재시도 규칙

**재시도 가능한 원인과 지금 다시 움직여도 되는 상태를 함께 확인한다.**

1. 실제 실행 종료와 팔 및 그리퍼 정지를 확인한다.
2. 물체를 들었거나 상태가 불명확하면 동일 수거를 자동 반복하지 않는다.
3. 최신 Scene과 기록을 Planner에 전달한다.
4. Mission Manager가 제안과 상태, allowlist 및 retry budget을 검증한다.
5. 허용된 새 시도에 새 `execution_id`를 발급한다.

서버는 물체 접촉 이후 전체 행동을 자동 retry하지 않는 제안이다. 준비 조회의 제한된
내부 retry는 [서버 명세](03_manipulation_server_behavior_spec.md)를 따른다.
현재 core의 skill retry 2회를 새 서버에 그대로 적용하지 않는다.

## 7. 남은 adapter 작업

| 작업 | 이유 |
|---|---|
| typed 실행 payload | 물체, 놓은 결과와 정지 상태 보존 |
| 행동 checkpoint와 재관찰 | KB의 Planner 폐루프 연결 |
| 취소 및 중단 표현 | 현재 core에 없는 결과 연결. 재시작 시 중단 이유와 사람 확인 필요를 함께 보고 |
| task 기록 | skill 이름과 완료 물체를 구분 |
| 거절, 응답 유실과 기록 조회 | 중복 움직임 방지 |
| 외부 상태 adapter | 내부 stage와 Backend 운영 lifecycle 구분 |

관련 문서: [Action 명세](02_execute_manipulation_skill_action_spec.md),
[관측 명세](05_initial_table_observation_spec.md),
[KB Mission Lifecycle](../../../../docs/cleany-docs/20_TECHNICAL/09%20-%20Mission%20Lifecycle.md).
