# 실물 매니퓰레이션 조작 경로 및 파라미터 규격

본 문서는 실물 로봇(Jetson Orin NX)에서 수거 파이프라인(`physical_grasp_selection_server`)을 운영할 때 적용되는 기구학적 파라미터, 접근 보정, 다단계 접근 준비 자세 및 안전 계약을 설명합니다.

> [!IMPORTANT]
> 실물 파지 선택기는 `physical_grasp_selection_server`로 실행되며, 기존 시뮬레이션용 `grasp_selection_server`와 동일한 네임스페이스에서 동시에 기동하지 않습니다.

## 1. 파지 목표 추가 삽입

수거 공용 설정 `collection_motion.yaml`의 `grasp_approach_offset_m`은 `0.0`이다.
최종 TCP 목표를 후보의 접근 방향으로 추가 이동하지 않는다.
`make manipulation-run`과 dev-monitor의 수거 실행은 같은 설정을 사용한다.
TCP의 기구학적 정의와 기존 닫힘 방향의 비대칭 보정은 유지한다.

추가 삽입이 0이므로 접근 방향의 준비 거리는 기본 8cm, 대체 6cm와 4cm다.
준비 위치와 최종 파지 목표는 기존 IK, 상태 유효성, 그리퍼 닫힘 공간과
경로 검사를 따른다. 변경한 설정은 planning runtime을 다시 시작할 때 적용한다.

## 접근 준비 자세의 거리 대체

`grasp_ik_first=true`에서는 파지 자세를 먼저 구하고 같은 해를 시작점으로 접근 준비
자세를 검사한다. `pregrasp_offset_m`의 기본 거리는 0.08m다. 이 준비 자세의 IK가
없거나 자세 허용 오차를 넘으면 `pregrasp_fallback_offsets_m`의 거리를 순서대로
검사한다. 수거 프로필은 `[0.06, 0.04]`를 사용하므로 8cm, 6cm, 4cm 순서가 된다.
앞선 거리에서 통과하면 이후 거리는 검사하지 않는다. 대체 거리는 양수이며 기본
거리보다 짧고 순서대로 감소해야 한다. 목록이 비어 있으면 기존 단일 설정인
`pregrasp_fallback_offset_m`을 사용하며, 이 값도 0이면 대체 검사를 끈다.

대체 준비 자세도 기존 위치와 방향 허용 오차, 전체 장면 충돌 검사, 두 구간의 경로
계획과 경로 자세 검사를 통과해야 한다. 접근 통로 검사가 켜져 있으면 실제 선택한
준비 자세를 시작점으로 사용한다. 이 설정은 파지 후보, 목표 파지 자세와 controller
실행 계약을 바꾸지 않는다. 피드백의 `offset`은 추가 삽입을 포함한 준비 위치와
최종 목표 사이의 거리이고, `fallback`은 대체 준비 자세 사용 여부다.

## 들어올리기 단일 높이 보정

들어올리기 후 `lift_height_correction_enabled=true`이면 목표 높이와 실측 관절 FK 높이를 비교한다. 부족분이 `lift_height_tolerance_m` 이하면 추가 상승을 생략하고, 초과하면 전체 부족분을 `lift_height_correction_step_m` 단위로 올림해 한 번만 추가 상승한다. 허용오차가 0.010m이면 0.527mm와 10mm 부족은 보정하지 않는다. 허용오차를 생략하면 0m이며 음수나 비유한 값은 거부한다. 기본 단위는 0.005m이며, 0.011m 부족하면 0.015m를 추가한다. 물리 경로는 실측 TCP에서 시작하고, 유지 중인 모터 명령의 지지량은 별도로 보존한다. 보정 후 남은 높이 오차는 기록만 하며 재보정이나 높이 합격 판정 없이 보유 확인과 복귀 흐름을 계속한다. 기존 `lift_height_correction_max_attempts`, `lift_height_correction_min_progress_m`, `lift_height_correction_max_total_m` 값은 사용하지 않는다. 모터 fault, 최신 피드백, 접촉, 취소, 명령 경로와 XY 및 방향의 도착 검사는 계속 적용한다. 추가 상승을 생략해도 기존 검사를 수행하며 보정 결과에 적용한 `tolerance_m`을 기록한다.

## 놓기 후 후퇴 및 동기 복귀 충돌 검사

- 측정된 관절 FK 위치에서 후퇴 경로를 계획하며 첫 명령은 유지 중인 목표값을 이어받는다.

팔 복귀와 그리퍼 닫기의 동기 경로는 충돌 검사가 활성화된 경우 실행 직전 전체 로봇 상태로 검사한다. 거부하면 `hardware_collision_validation_failed`에 `trajectory_stage=synchronized_return`, 표본 번호, 관절값, 접촉한 링크·장애물과 지도 고정 여부를 남긴다. `hold_bridge`는 측정 위치에서 유지 명령까지의 검사 구간이고 `command_path`는 실제 명령 경로다. 검사가 활성화된 상태에서 통과하지 못한 경로는 전송하지 않는다. Jetson의 고정 중간 자세 복귀는 `bt_arm_return_collision_check_enabled=false`로 계획 단계와 동기 실행 단계의 충돌 검사를 생략하며, 충돌 지도 갱신을 기다리지 않는다. 이 경우에도 최종 명령의 관절 범위, 속도와 가속도 제한, 피드백 및 도착 확인은 유지한다. 생략 여부는 `collision_checks_enabled`와 `hardware_collision_validation_skipped`에 기록한다.

## 5. GPU 자원 조율 및 우선순위 (GPU Work Lock)

AnyGrasp(비전)와 cuRobo(모션) 간의 GPU 충돌을 방지하기 위해 파일 락(`gpu-work.lock`)을 운용합니다.
- **실제 요청 최우선**: 사용자의 파지 요청(Foreground Demand)이 백그라운드 모델 예열보다 항상 우선합니다.
- **양보 메커니즘**: 진행 중인 cuRobo 예열 작업은 실제 요청이 감지되면 다음 CUDA 단계 경계에서 즉시 GPU를 양보합니다.

## 기록 역재생 운반의 완료 기준

`pick_return_release`에서 `return_to_pick_reverse_enabled=true`인 운반은 현재 측정 자세를 기준으로 목표 높이에 해당하는 기록 경로 구간을 한 번만 역재생한다. 이동 완료 뒤 추가 하강 보정, 관절 및 TCP의 도착 허용오차 판정과 놓기 직전 위치 재검사를 생략하고 다음 놓기 단계로 진행한다. 컨트롤러의 정상 완료 응답을 기다리며 취소, 통신 실패와 모터 fault는 계속 실패로 처리한다. 명령의 관절 범위와 속도 및 가속도 제한도 적용한다. 기존 `return_to_pick_height_correction_max_attempts`, `return_to_pick_height_correction_max_step_m`과 도착 판정용 `return_to_pick_height_tolerance_m`은 이 역재생 경로에서 사용하지 않는다. `hardware_pickup_height_search_tolerance_m`은 이동 전 경로의 절단 시점을 찾는 수치 오차이며 도착 합격 기준이 아니다.


## 역재생 후퇴의 손목 방향

들어올리기 시작 시 `retreat_reverse_approach_enabled=true`이면 기록한 접근 명령을 역재생하되 손목 roll은 잡은 뒤 측정값으로 고정한다. 복귀 목표 위치는 저장한 pregrasp 위치를 유지하며, 목표 방향은 현재 측정 자세에 고정 roll 경로의 회전을 적용해 계산한다. 접근 전 손목 방향으로 되돌리는 검사는 하지 않는다. 계산한 목표 방향은 이동 전에 고정하여 컨트롤러 실행과 이동 후 확인에 동일하게 사용한다. 높이 보정이 활성화된 경우에도 XY, 위쪽 초과와 이 목표 방향에 대한 허용오차 검사는 유지한다. `retreat_measured_start`에는 검사 목표 위치와 방향 및 방향의 출처를 기록한다.

## 6. 실행 FK와 시간 계측

실행기는 `execution_use_verified_local_fk=true`일 때 현재 URDF의 로컬 FK를
사용합니다. 최초 사용과 관절 상태, 그리퍼 설정 또는 URDF 변경 뒤에는
MoveIt FK와 위치 및 회전 행렬을 대조합니다. 불일치와 비유한 참조 값은
실패 처리하며, 충돌 검사 결과는 캐시하지 않습니다. Jetson Action 프로파일은
이 기능을 켜고, 별도 프로파일을 사용하는 실행기의 기본값은 `false`입니다.

물체 원위치 복귀는 기록 궤적의 역재생과 높이 탐색을 사용합니다. 빈 팔의
home 복귀는 Jetson에서 두 개의 고정 중간 자세를 거칩니다. 중간 자세 파일이 없는 경로는 직선 관절 경로를 먼저 검사하고 막히면 RRT를 사용하며 충돌 검사를 유지합니다.
두 경로는 별개이며 FK 전환은 명령 궤적이나 속도 제한을 변경하지 않습니다.

`execution_timing_enabled=true`는 `execution_timing` JSON 로그를 추가합니다.
Jetson 프로파일에서 기본 활성화하며 다른 실행기의 기본값은 `false`입니다.

| 필드 | 의미 |
|---|---|
| `execution_id`, `bt_node` | 현재 작업과 BT 단계. BT 밖 호출은 빈 문자열 |
| `span_id`, `parent_span_id` | 호출과 부모 호출의 식별자. 최상위 부모는 `null` |
| `phase` | 서비스, Cartesian 계획, 높이 탐색, 모델 검증, 충돌 검사, 안정화, 도착 확인, 궤적 실행 |
| `started_wall_ns` | 로그와 연결하는 Unix 벽시계 시작 시각, ns |
| `wall_sec` | 단조 시계로 측정한 자식 호출 포함 경과시간, 초 |
| `outcome` | 정상 반환 `returned` 또는 예외 `raised`. 반환 자체가 작업 성공을 뜻하지 않음 |
| `service` | `service_call`일 때 서비스명. 시간에는 준비 및 응답 대기 포함 |

부모 span에 자식 시간을 다시 더하지 않습니다. `duration_sec`는 명령 궤적의
시간이며 순수 연산 시간이나 실측 모터 이동 시간으로 해석하지 않습니다.
동기 궤적의 `scheduled_start_delay_sec`는 팔과 그리퍼의 예약 시작 여유입니다.
계측 로깅 실패는 제어 결과와 취소 예외를 바꾸지 않습니다.



## 실물 어깨 yaw의 전방 범위

Jetson 운영 control profile의 `joint_position_bounds`는 다음 범위를 사용합니다.
각도는 영점과 방향이 보정된 ROS 관절각이며 모터의 원시 엔코더 값이 아닙니다.

| 모터 | 관절 | 허용 범위, rad | 허용 범위, 도 |
|---|---|---|---|
| 1 | `left_shoulder_yaw_joint` | −2.16 ~ 0 | −123.758884 ~ 0 |
| 7 | `right_shoulder_yaw_joint` | 0 ~ 2.0033789089786476 | 0 ~ 114.78515625 |

control profile을 읽어 생성한 `robot_control.urdf`의 관절 한계를 CPU 보정 IK,
GPU IK와 MoveIt이 공유합니다. 직접 연결, Cartesian과 RRT의 최종 궤적도
이 범위를 검사합니다. 실행기는 시작점 연결과 시간 배분 후의 명령 궤적을
다시 검사하며, 기록 궤적은 점 사이 cubic 극값까지 확인합니다.
복귀의 충돌 검사를 생략해도 어깨 관절 범위 검사는 생략하지 않습니다.
Feetech 드라이버는 매 제어 주기의 최종 명령에도 같은 한계를 적용합니다.

범위를 벗어난 시작 자세나 경로는 실행 전에 거절하며 자동으로 각도를 잘라
이동시키지 않습니다. 이 설정은 어깨 yaw의 뒤쪽 회전 구간을 제한합니다.
다른 관절의 접힘이나 실제 모터의 추종 오차까지 없애는 공간 제약은 아닙니다.
설정 변경은 실물 제어기와 계획 노드의 다음 기동부터 적용됩니다.


## Pregrasp와 그리퍼의 대기 및 명령 간소화

| 설정 | 일반 기본값 / Jetson 설정 | 동작과 검증 조건 |
|---|---|---|
| `hardware_start_delay_sec` | `0.8` / `0.3` | pregrasp의 팔과 그리퍼 공통 시작 여유, 초. 시작 전 양쪽 수락 확인 조건은 복귀와 동일 |
| `hardware_state_validity_batch_size` | `32` / `64` | 최종 실행 경로의 한 요청당 충돌 표본 수. 표본 간격과 전체 표본 수는 유지하며 서비스가 없으면 단일 상태 검사로 대체 |

센서 barrier를 재사용해도 관측 시각, 수신 시각, OctoMap 존재와 최대 나이
검사를 모두 통과해야 합니다. 기존 freeze 모드에서는 재사용하지 않습니다.


물리 pregrasp에서 예비 검사를 최종 검사로 통합한 경우 하드웨어와 모델 조회도
최종 전송 검사에서 한 번 수행합니다. 그리퍼 닫힘 목표를 계산할 때는 직전의
검증된 캘리브레이션을 재사용하지만 전송 직전에 다시 읽은 한계와 목표가
일치해야 합니다. 모델이나 캘리브레이션 변경을 캐시만으로 승인하지 않습니다.

단일 구간 그리퍼 명령은 위치와 속도를 가진 두 끝점으로 같은 cubic을
전달합니다. 관절 한계와 필요한 충돌 검사는 조밀한 표본으로 유지하며, 중간
자세가 있는 명령은 압축하지 않습니다. 한 구간의 retiming은 구간 간 시간
재분배를 생략하고 기존 속도, 가속도 한계로 계산한 시간을 사용합니다.
`command_samples`는 검사 표본 수, `controller_points`는 실제 전송점 수,
`two_point_gripper_command`는 두 끝점 전달 여부입니다.
`gripper_contact_wait`는 닫힘 명령 후 접촉 판정에 추가로 걸린 시간입니다.
접촉 판정의 원시 피드백 창과 완전 열림 목표는 유지합니다.


### 검사 상태 준비와 제어 연결 재사용

최종 충돌 검사의 전체 관절 상태는 공통 피드백 메타데이터와 각 표본의
명령 관절값으로 구성합니다. 모든 검사 표본과 접근 clearance 검사는
유지합니다. 아직 전송하지 않은 상태 메시지를 만드는 동안에는
`hardware_state_preparation_monitor_period_sec` 간격 또는 64개 표본마다
취소와 하드웨어 상태를 확인하고 마지막에도 확인합니다. 기본 간격은
0.01초이며 0보다 크고 0.01초 이하여야 합니다. 충돌 RPC와 실제 실행의
감시 주기는 변경하지 않습니다. 이 준비 시간은 `collision_state_preparation`
span으로 기록하며 `collision_validation`에 포함됩니다.

Jetson BT 실행기는 제어기별 Action 연결을 유지하되 실행마다 별도의
명령 추적 객체를 사용합니다. 이전 명령의 수락 응답과 최종 결과가 모두
확인되고 사용이 끝나야 같은 연결을 다시 빌릴 수 있습니다. 늦게 도착한
수락 응답도 원래 실행의 취소 상태를 따르며, 노드 종료 시 연결을 해제합니다.
첫 사용은 연결을 생성하므로 재사용에 따른 단축은 이후 사용에 적용됩니다.
`synchronized_goal_accepted`의 `transport_reused`는 재사용 여부,
`client_acquisition_wall_sec`는 두 제어 연결을 준비한 시간입니다.
팔과 그리퍼 로그에 같은 합계가 기록되므로 중복 합산하지 않습니다.

물리 pregrasp의 예비 검사를 생략하는 경로는 예비 센서 대기도 생략합니다.
전송 직전의 정지 자세 기준 센서 갱신, 지도 최대 나이와 최종 충돌 검사는
유지합니다. 예비 검사를 수행하는 경로와 시뮬레이션은 기존 대기를 유지합니다.
센서가 이미 최신이면 대기 통합의 시간 효과는 작으며, 센서 한 프레임의
대기 시간이 무조건 사라지는 것은 아닙니다.


## 독립 모델 조회와 cubic 명령 전송

`hardware_parallel_model_queries`와 `hardware_sparse_cubic_commands`의 일반
기본값은 `false`이며 Jetson의 `manipulation_jetson.yaml`에서 `true`로 설정한다.
첫 옵션은 하드웨어 구성, 활성 제어기, 제어기 모델과 MoveIt 모델의 읽기 요청을
모두 보낸 뒤 응답을 기다린다. 응답 대기는 전체 30초 한도를 공유하며 실행
소유자의 취소, BT deadline과 물체 보유 중 감시를 따른다. 요청 실패 시 남은
로컬 future와 pending request를 정리하고 수집한 응답을 폐기한다. 네 응답의
검증 조건과 매 실행의 최신 조회는 유지하며 지난 응답을 대신 사용하지 않는다.

두 번째 옵션은 한 번 준비한 최종 retimed cubic의 경계 위치, 속도와 시각만
새 궤적 명령에 사용한다. 충돌 검사에는 기존 조밀한 표본과 측정 시작점의
hold bridge를 사용한다. 속도, 가속도, 관절 범위와 접근 통로의 검사 조건,
중간 자세, 궤적 시간과 팔/그리퍼의 공통 시작 시각은 유지한다. acceleration
필드는 비워 JTC가 같은 cubic을 보간하도록 한다. 나노초 반올림 후 시각이
중복되는 동기 명령은 전송 전에 거부한다. 기존 기록 궤적 재생에는 새 점
간소화를 적용하지 않고 저장된 위치, 속도와 시각을 그대로 보낸다.

`command_samples`는 기존 검사 표본 수이고 `controller_points`는 전송 점
수다. 단독 명령의 후자는 정수, 동기 명령의 후자는 arm과 gripper별 객체다.
`sparse_cubic_command`와 `parallel_model_queries`로 적용 여부를 확인한다.
`model_queries_wall_sec`는 모델 확인 내부의 조회 구간이며 전체
`model_verification_wall_sec`에 포함된다. `parallel_service_calls` span도
같은 포함 시간이다. 서로 겹치는 span이나 팔/그리퍼의 시간을 더하지 않는다.


## 후보별 빠른 경로 우선 선택

Hybrid의 `fast_candidate_paths_first`가 켜져 있으면 후보 순서대로 직접 관절
연결과 Cartesian 접근을 먼저 시도합니다. 빠른 경로로 연결되는 후보가 없을
때만 경로 실패 후보를 원래 순서로 다시 방문해 RRT를 허용합니다. 뒤 후보의
빠른 경로가 앞 후보의 RRT 경로보다 우선될 수 있습니다. 모든 빠른 경로가
실패하면 추가 후보 검사 비용이 생길 수 있으므로 항상 빨라지는 것은 아닙니다.
같은 선택 요청 안에서만 성공한 끝점 IK 쌍을 재사용하고, 재방문 시 현재 장면의
끝점 충돌과 그리퍼 sweep, 통로, 방향, 최종 경로를 다시 검사합니다. IK 또는
끝점 충돌에서 탈락한 후보는 경로만 바꿔 재시도하지 않습니다. 이 옵션에는
`grasp_ik_first=true`와 `stop_after_first_feasible=true`가 필요합니다.
RRT를 보류한 시도는 `rrt_deferred`로 기록하며 RRT 실행 시간으로 합산하지 않습니다.
