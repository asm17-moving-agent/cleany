# 실물 수거 실행과 복귀

실물 BT는 선택된 물체의 접근, 파지, 들기, 운반과 팔 복귀를 실행한다.
경로 계획과 파지 전 준비 자세는 [physical-pregrasp.md](physical-pregrasp.md)를 따른다.
MoveIt은 경로만 계산하고 실제 명령은 하드웨어 실행기가 보낸다.

새 cuRobo 실물 준비 세션은 모터 보호 노드의 실시간 준비 상태와 MoveIt 계획 장면 서비스의 관절 상태 응답을 확인한 뒤 GPU IK 초기화를 시작한다. GPU 기동 전과 전체 준비 검사에서 MoveIt의 궤적 실행 설정이 불리언 false인지 확인한다. 탐색이 끝난 뒤에도 설정 조회 응답이 도착하지 않으면 10초 검사 기한 안에 한 번 재조회한다.

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

실측 TCP의 FK는 `execution_reuse_measured_fk` ROS 파라미터로 재사용하며 기본값은 `true`다.
요청 좌표계에서 TCP까지의 URDF 관절 위치가 정확히 같을 때 마지막 성공 결과 하나만 재사용한다.
관절 위치, 모델, 좌표계, 요청 링크나 통신 복구 횟수가 바뀌면 다시 계산한다.
해당 TCP 위치에 영향을 주지 않는 그리퍼 관절과 다른 팔의 변경은 재사용을 막지 않는다.
URDF 의존관계를 해석할 수 없거나 관절 정보가 불완전하면 기존 FK 서비스를 호출한다.
재사용할 때도 최신 피드백, 취소와 모터 fault 검사를 먼저 수행하며 충돌과 도착 판정은 저장하지 않는다.
실행 시간 기록을 켜면 `pickup_pose_captured`, `pickup_departure_recorded`, `return_planned`에
누적 재사용 횟수 `measured_fk_cache_hits`와 성공한 서비스 호출 횟수 `measured_fk_service_calls`를 남긴다.

## 명령 준비와 대기 재사용

Jetson 운용값은 [manipulation_jetson.yaml](../../cleany_manipulation_bt/config/manipulation_jetson.yaml)에서 읽는다.

| 파라미터 | Jetson 값 | 적용 내용 |
|---|---|---|
| `hardware_trajectory_limit_scale` | 1.0 | 현재 관절별 속도와 가속도 상한을 명령 시간 계산에 반영. 코드 기본값은 0.8 |
| `hardware_path_step_rad` | 0.02 | 궤적 충돌 검사 표본 간 관절 변화 상한. 명령점 수와 별개 |
| `hardware_start_delay_sec`, `hardware_return_start_delay_sec` | 0.2초 | 팔과 그리퍼의 공통 시작 여유. 시작 전 수락 조건 유지 |
| `hardware_retime_retreat` | true | 기록 후퇴 곡선의 속도와 가속도 극값으로 실행 시간을 다시 계산 |
| `hardware_retreat_min_duration_sec`, `hardware_lift_min_duration_sec` | 0.3초 | 후퇴 역재생과 일반 들어올리기의 최소 시간. 실제 제한이 요구하면 더 길어짐 |
| `hardware_prepare_models_on_startup` | true | 시작 시 정적 모델과 FK 준비. 전송 전 현재 모델과 controller 상태 조회 유지 |
| `hardware_prefetch_release_retreat` | true | 놓은 뒤 후퇴 계획과 독립적인 모델 조회를 겹침 |

팔의 요청 상한은 2rad/s와 8rad/s²이며 그리퍼는 별도 하드웨어 상한을 따른다.
시간 계산의 5% 수치 여유와 관절 범위 검사는 유지한다. 곡선 표본은 벡터 계산하고
URDF 회전축은 한 번 정규화한다. 측정 메시지의 필드를 직접 복사하고 하나의 응답
대기 반복에서는 같은 측정을 두 번 만들지 않는다.

파지 접근에서 선택된 궤적의 첫 점을 정착한 실측 관절각과 현재 유지 명령에 각각
비교합니다. 모두 `hardware_start_tolerance_rad`(기본 0.005 rad) 이내이면 원본 관절
위치, 시간, 속도와 가속도를 사용합니다. 전송 직전에 실측 자세를 다시 비교합니다.
저장된 접근 경로가 비었거나 시작 자세가 허용치를 넘으면 실측 자세에서 같은 파지
TCP로 다시 계획합니다. 새 명령은 이전 유지 명령의 지지량을 한 번 보정한 뒤,
물리 TCP 목표에 대해 확인합니다.

원본 궤적도 관절 범위, 속도, 가속도와 운영 충돌 정책에 따라 전송 전에 검사합니다.
전송 전에 경로 검사가 실패하면 해당 명령을 거절합니다. 전송 이후에 제어기 오류나
실측 도착점 오차가 검출되면 실행 오류의 정지 절차를 따릅니다. 후퇴용 기록에는 실제
보낸 가속도도 저장합니다.

복귀 조회는 현재 팔, 명령 횟수, 통신 복구 세대와 신선도가 일치할 때 한 번 소비한다.
후퇴 직후의 도착 확인은 전체 관절 위치와 모델이 정확히 같을 때만 재사용한다.
빈 팔 복귀 시작에서도 정지 이력을 별도로 검사하고, 조건이 달라지면 새 조회와 대기를 한다.

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

1. [개발환경](../../../../docs/implementation/development-setup.md)과 [실행 안내](../../../../docs/RUNNING.md)에 따라 설치를 준비한다.
2. 로봇별 프로필 경로를 지정한다.

   ```bash
   export CLEANY_MANIPULATION_COLLECTION_CONFIG=/absolute/path/collection.json
   export CLEANY_FEETECH_CONTROL_CONFIG=/absolute/path/control.json
   export CLEANY_HARDWARE_MONITOR_CONFIG=/absolute/path/motion-monitor.yaml
   ```

3. `make collection-start`로 런타임을 시작하고 `make manipulation-start`로 대기 중인 BT 서버를 연결한다.
   시작 시 설치된 ROS 패키지로 보정 파일과 제어 모델을 먼저 읽는다.
   파일을 열거나 모델을 생성할 수 없으면 웹 작업 이력에 원인을 기록하고 장치 기동 전에 종료한다.
   제어기가 첫 관절 피드백을 기다리는 `waiting` 상태에서는 기존 상태 조회 기한 안에 `ready`로 전환되는지 확인한다.
   `make collection-plan`은 계획만 확인하고 `make manipulation-run`은 실제 단일 수거를 요청한다.
4. 진행 상태는 `make collection-status`와 `make collection-logs`로 확인한다.
   `make manipulation-recover`는 복구 상태를 조회하며 확인 인자는
   `tools/collection-runtime action-recover --help`를 따른다.
   `make arm-return`은 실제 팔 복귀를 요청한다.
5. `make collection-stop`은 실행기 종료와 정지 확인을 기다린 뒤 의존 프로세스를 종료하고 토크를 해제한다.
   종료를 확인하지 못한 프로세스의 PID는 보존하고 성공으로 보고하지 않는다.

## 접촉과 대상 근거

`manipulation_evidence`는 원시 모터 피드백의 접촉 판정과 손목 관측의 대상 식별을 결합한다.
Jetson 운영 프로파일은 양쪽 그리퍼에서 토크 한도 읽기값 400과 정상 정지 피드백을 확인한다.
0.4초 창의 전류 원시값 중앙값이 15 이상이고 부하 원시값 최솟값이 200 이상이면 접촉으로 판정한다.
전류 중앙값과 부하 최댓값이 각각 기준 미만이면 빈 상태로 판정한다.
닫힘 목표까지 남은 인코더 틱은 진단에 기록한다.
손목 관측은 대상 ID, 신선도와 시간 차이를 검사한다.
구형 부하 단독 판정은 사용하지 않는다. 같은 근거를 재전송해도 새 관측이 되지 않는다.
서비스와 메시지 정의는 [인터페이스](interfaces.md#manipulation_evidence)를 따른다.

## 실행 준비와 시간 최적화

동일한 관절 상태와 모델의 FK, IK client와 불변 모델 데이터는 재사용한다.
최신 피드백 감시와 충돌 검사는 요청마다 유지하며 충돌 장면의 결과는 캐시하지 않는다.
기록 경로는 `hardware_retime_retreat`로 역재생 시간 재계산을 선택할 수 있다.

| Jetson 설정 | 값 | 적용 |
|---|---|---|
| `hardware_trajectory_limit_scale` | `1.0` | 요청 속도와 가속도를 물리 관절 한도로 제한 |
| `hardware_path_step_rad` | `0.02` | 충돌 검사용 관절 보간 간격 |
| `hardware_lift_min_duration_sec` | `0.3` | 들기 최소 시간. retimer의 속도와 가속도 제한 유지 |
| `hardware_retreat_min_duration_sec` | `0.3` | 복귀 최소 시간 |
| `hardware_retime_retreat` | `true` | 현재 한도로 기록 경로 재타이밍 |
| `hardware_return_start_delay_sec` | `0.2` | 팔과 그리퍼의 공통 시작 대기 |

위 값은 `cleany_manipulation_bt/config/manipulation_jetson.yaml`의 실행 프로필이며
실행기 자체의 기본 여유율은 `0.8`이다. 실물 안전성과 전체 수거 시간의 개선은 별도 검증한다.
