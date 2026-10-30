# 실물 수거 실행과 복귀

실물 BT는 선택된 물체의 접근, 파지, 들기, 운반과 팔 복귀를 실행한다.
경로 계획과 파지 전 준비 자세는 [physical-pregrasp.md](physical-pregrasp.md)를 따른다.
MoveIt은 경로만 계산하고 실제 명령은 하드웨어 실행기가 보낸다.

## 실행 경로

`manipulation_jetson_server`는 기존 수거 트리를 실행하며 실행 기록을 별도 SQLite에 보존한다.
관측과 파지 후보 선택 뒤 실행 준비 상태, 명령 경로와 피드백을 확인한다.
정지 확인이 끝나지 않은 작업, 진행 중인 복귀 또는 해제하지 않은 복구 차단이 있으면
새 작업을 시작하지 않는다. ROS 경로는 [BT 인터페이스](../../cleany_manipulation_bt/docs/interfaces.md)를 따른다.

| 경로 | 동작과 완료 근거 |
|---|---|
| `full_collection` | 설정된 종류와 수거함을 사용하고 파지, 보유와 놓기 근거를 확인 |
| `grasp_lift_hold`, `grasp_lift_return_hold` | 파지와 들기 이후 물체를 보유하는 단일 시험 |
| `pick_return_release` | 원래 집은 위치로 돌아와 놓는 단일 시험. 놓기 상태는 명시적인 `assume_success` 설정이며 센서로 확인한 성공과 구분 |

실물 프로필은 로봇별로 준비한 JSON과 모션 감시 설정을 명시적으로 입력한다.
저장소의 `supervised_collection_fixture.json`은 가짜 동작 포트를 위한 테스트 입력이다.
프로필 검사 통과나 `commissioning_verified` 값만으로 실물 검증을 대신하지 않는다.

## 복귀와 복구

| 동작 | 적용 조건 |
|---|---|
| 기록 경로 역재생 | `return_to_pick_reverse_enabled`로 선택한다. 목표 높이에 해당하는 구간을 찾고 명령 범위, 속도와 가속도를 검사 |
| 단일 높이 보정 | `lift_height_correction_enabled`일 때 부족한 높이를 설정 단위로 올림해 한 번만 보정. XY, 위쪽 초과와 방향 검사는 유지 |
| 빈 팔 home 복귀 | 저장된 home와 중간 자세를 사용하며 팔과 그리퍼의 수락 응답을 공통 시작 시각 전에 확인 |
| 복구 차단 해제 | 현재 정지와 그리퍼 비움에 대한 명시 확인을 받고 최신 피드백으로 정지를 다시 검사. 과거 결과는 수정하지 않음 |

현재 Jetson 프로필의 고정 중간 자세 복귀는 `bt_arm_return_collision_check_enabled=false`다.
이 경로는 충돌 검사를 생략하지만 최종 명령의 관절 범위, 속도, 가속도와 도착 검사는 유지한다.
충돌 검사 생략 여부를 실행 로그에 기록한다. 중간 자세가 없는 일반 복귀 경로는 충돌 검사와
직접 연결 또는 RRT 계획을 사용한다. 이 구현 설정은 실물 안전성 검증 결과가 아니다.

물체 복귀의 역재생 경로는 완료 후 추가 하강 보정이나 도착 오차 재검사를 수행하지 않는다.
`hardware_pickup_height_search_tolerance_m`은 경로 절단 시점을 찾는 수치 오차다.
취소, 통신 실패와 모터 fault는 계속 실패로 처리한다.

## 명시 실행

1. [개발환경](../../../../docs/DEVELOPMENT_SETUP.md)과 [실행 안내](../../../../docs/RUNNING.md)에 따라 설치를 준비한다.
2. 로봇별 프로필 경로를 지정한다.

   ```bash
   export CLEANY_MANIPULATION_COLLECTION_CONFIG=/absolute/path/collection.json
   export CLEANY_FEETECH_CONTROL_CONFIG=/absolute/path/control.json
   export CLEANY_HARDWARE_MONITOR_CONFIG=/absolute/path/motion-monitor.yaml
   ```

3. `make collection-start`로 런타임을 시작하고 `make manipulation-start`로 대기 중인 BT 서버를 연결한다.
   `make collection-plan`은 계획만 확인하고 `make manipulation-run`은 실제 단일 수거를 요청한다.
4. 진행 상태는 `make collection-status`와 `make collection-logs`로 확인한다.
   `make manipulation-recover`는 복구 상태를 조회하며 확인 인자는
   `tools/collection-runtime action-recover --help`를 따른다.
   `make arm-return`은 실제 팔 복귀를 요청한다.
5. `make collection-stop`은 실행기 종료와 정지 확인을 기다린 뒤 의존 프로세스를 종료하고 토크를 해제한다.
   종료를 확인하지 못한 프로세스의 PID는 보존하고 성공으로 보고하지 않는다.

## 접촉과 대상 근거

`manipulation_evidence`는 원시 모터 피드백의 접촉 판정과 손목 관측의 대상 식별을 결합한다.
전류나 부하를 힘으로 해석하지 않으며, 관측의 대상 ID, 신선도와 시간 차이를 검사한다.
구형 부하 단독 판정은 사용하지 않는다. 같은 근거를 재전송해도 새 관측이 되지 않는다.
서비스와 메시지 정의는 [인터페이스](interfaces.md#manipulation_evidence)를 따른다.
