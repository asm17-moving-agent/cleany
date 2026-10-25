# MuJoCo Manipulation 실행 계약

승인된 물체 하나를 실행하는 `execution_profile=mujoco` 서버의 상세 계약이다.
실행 명령과 ROS 경로는 [패키지 README](../README.md)를 따른다.
Goal·Feedback·Result 필드는 [공통 Action 명세](../../cleany_skill_executor/docs/02_execute_manipulation_skill_action_spec.md)에 있다.

| 확인할 내용 | 위치 |
|---|---|
| 실행 순서, 대상 검사와 timeout | [실행 순서와 결과](#실행-순서와-결과) |
| 취소 모드와 늦은 결과 처리 | [취소](#취소) |
| 놓기 검증과 관절별 정지 근거 | [정상 수거의 놓기·정지 확인](#정상-수거의-놓기정지-확인) |
| 내부 예외와 오류 종료 | [예상하지 못한 실행 예외](#예상하지-못한-실행-예외) |

## 실행 순서와 결과

[collect_trash_mujoco.xml](../trees/collect_trash_mujoco.xml)이 준비, 잡기, 놓기, 확인과 종료를
정한다. C++ 노드는 `start(node_id, execution_id)` 후 RUNNING을 반환하고 후속 tick에서
`poll(operation_id)`를 호출한다. `halt()`는 `cancel(operation_id)`를 전달한다.
Python worker 하나가 개별 동작을 수행하므로 ROS 응답/모델 추론 대기는 BT tick을 막지 않는다.
서버는 시뮬레이션 시계와 독립적인 steady clock으로 20 Hz tick한다.
이 방식은 [BehaviorTree.CPP 비동기 노드 계약](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_04_sequence/)을 따른다.
큰 영상/점군과 파지/경로 결과는 실행별 Python context에 두고 Blackboard에는 execution ID만 둔다.

### 대상 검증

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

### 실패와 timeout

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

## 취소

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

## 정상 수거의 놓기·정지 확인

그리퍼 열림은 controller와 관절 피드백으로 확인한다. `ConfirmRelease`는 방출 시각 이후의
독립 안착 검증을 기다리고, 팔 복귀 뒤 `VerifyPlacedObject`에서 새 시각을 기준으로 다시 검증한다.
평가기의 false는 확인 대기다. 제한 시간 내 안착 근거를 못 얻으면
`placement_state=UNKNOWN / VERIFICATION_TIMEOUT`을 반환한다.
SUCCESS는 독립 방출/안착, 팔 복귀와 최종 정지 근거가 모두 있을 때만 저장한다.

### 정지 근거

중단 요청 시 Action cancel과 함께 MoveIt의 `trajectory_execution_event`에 `stop`을
발행한다. `bt_moveit_stop_topic` 기본값은 `/trajectory_execution_event`이며 Reliable,
Volatile이다. 미종료 Action이나 늦은 수락이 남아 있으면 정지 평가 기한 안에서
0.2초 간격으로 반복한다. 이 경로는 같은 MoveIt 실행기를 독점 사용하는 단일 작업용이다.
정상 완료 후 `cancel=false` 정지 평가는 이벤트를 발행하지 않는다.
[MoveIt 2.5.10의 MoveGroupInterface::stop](https://github.com/moveit/moveit2/blob/2.5.10/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp#L929)과
같은 TEM 정지 경로를 사용한다. 해당 버전 ExecuteTrajectory의 실행 callback은
동기 대기하며 Action 취소 callback은 실행 중단을 직접 호출하지 않는다.
이벤트 발행이나 cancel 응답 자체를 정지 성공으로 간주하지 않는다.

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

[manipulation_mujoco.yaml](../config/manipulation_mujoco.yaml)의 정지 판정 설정은 다음과 같다.

| ROS parameter | MuJoCo 기본값 | 용도 |
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

## 예상하지 못한 실행 예외

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
처리는 [README의 재시작 복구 계약](../README.md#기록과-모니터)을 따른다.

## Isaac 실행 프로필

`manipulation_isaac_server`는 동일한 C++ 트리와 sensor/MoveIt 동작을 사용하고
기록과 결과를 `execution_profile=isaac`으로 구별한다. `ASSUMED` 안착을 지원하지 않는다.
`manipulation/ready_to_drive`는 명시적으로 설정한 주행 자세에 있는 새 관절 피드백,
정지 구간, 비어 있는 물체 상태, 종료된 Action과 비차단 BT를 함께 검사하는 읽기 전용
Trigger 서비스다. 서비스 호출은 팔을 움직이거나 오류를 초기화하지 않는다.

정지 확인 timeout 로그는 Action 종료 여부, 아직 응답하지 않은 수락/결과 수, 연속 정지 샘플 수, 유효하지 않은 관절 피드백과 속도 초과 관절을 구분한다. 이 진단은 정지 기준을 바꾸지 않으며, 별도 관측에서 로봇이 정지해 보인다는 이유로 `STOP_UNCONFIRMED`를 성공으로 변경하지 않는다.
