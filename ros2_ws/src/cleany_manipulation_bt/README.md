# cleany_manipulation_bt

승인된 `snapshot_id/object_id`의 쓰레기 또는 분실물 하나를 MuJoCo에서 수거하는 ROS Action 서버다.
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
| 최근 물리 연동 결과 | [2026-10-14 검증 기록](#검증-기록-2026-10-14) |
| 물리 연동의 기존 검증 범위 | [2026-10-01 검증 기록](#검증-기록-2026-10-01) |

## 빌드와 검증

아래 Make 명령은 레포 루트에서 실행한다.

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
첫 팔 이동 전 재관찰의 임시 충돌 객체를 정리하고, 새 pregrasp 작업에서 대상 접촉을
금지한 뒤 팔과 그리퍼를 함께 이동한다. 이후 접근 단계에서는 다시 재관찰하고 접촉을 허용한다.
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
  "{query: 'Detect objects on the table.', tracking_session_id: 'desk-validation-1', snapshot_id: '', selected_object_id: 0}"
```

반환된 실제 snapshot과 쓰레기 object ID를 지정한다. snapshot cache의 기본 TTL은
120초이고 최근 두 장만 남는다. 만료됐으면 새 관찰을 요청해야 한다.

같은 고정 베이스 책상 작업에는 같은 `tracking_session_id`를 사용한다. BT는 승인된
snapshot의 session/epoch/track ID를 재관측에서도 확인한다. 베이스 이동 후에는 새 세션을
시작한다. 자세한 조건은 [인식 패키지의 물체 추적](../cleany_perception/README.md#고정-베이스-책상-물체-추적)을 따른다.
빈 세션은 추적을 끄는 호환 경로이며, 재관측에서 같은 종류가 여러 개면 수거를 중단한다.

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

정상 수거의 `SUCCESS`에는 물체 이탈·안착, 팔 복귀와 최종 정지 근거가 모두 필요하다.
관절 메시지의 시각과 내용이 모두 같은 연속 중복 수신은 새 관측으로 세지 않는다.
기존 수신 시각을 유지하므로 멈춘 스트림은 만료되고, 같은 시각에 내용이 달라지거나
시각이 역행하는 메시지는 기존 무효 판정을 따른다. 중복만으로 정지를 확인하지 않는다.
스터디카페 backend의 로컬 제어 실행기는 한 주기의 `read/update/write`에 같은 시각을
사용하고 새 시뮬레이션 시각에서만 다음 주기를 실행한다. 무효 피드백 로그의
`invalid_reasons`는 `repeated_stamp`, `backward_stamp`, `missing_velocity`,
`stale_receipt`, `clock_skew` 등을 구분하며, `clock_stopped`는 시계 정지를 표시한다.
피드백 만료와 정지 확인 기준은 변경하지 않는다.
실패·취소에서도 이미 확인한 물체 상태와 놓기 결과를 보존한다.
정지 미확인, 장치 오류, 비상정지와 내부 오류는 `FATAL`로 종료하고 후속 Goal을 차단한다.
실제 트리 순서, 대상 검사, timeout과 늦은 응답 처리는 [상세 실행 계약](docs/execution_contract.md)을 따른다.

### 취소

일반 Action cancel은 `CHECKPOINT`다. `manipulation/cancel` 서비스는
`IMMEDIATE`, `CHECKPOINT`, `RETURN_ARM`을 제공한다.
취소 수락 후에도 이전 동작의 종료와 정지 확인을 기다린다.
`RETURN_ARM`은 정지 확인 뒤 필요하면 현재 위치에서 그리퍼를 열고, 실행 시작 때 저장한 팔 자세로 복귀한다.
모드별 절차와 제한은 [상세 취소 계약](docs/execution_contract.md#취소)을 따른다.

### 정상 수거의 놓기·정지 확인

독립 안착 평가와 새 관절 샘플로 확인하며, 명령 성공이나 취소 응답만으로 정지를 인정하지 않는다.
평가기의 `false`는 제한 시간 안의 확인 대기이며, 확인 기한을 넘기면 `UNKNOWN / VERIFICATION_TIMEOUT`이다.
`arm_recovered=true`는 실행 시작 자세로의 복귀 근거다. 모바일 베이스의 주행 허가는 별도 readiness 계약이 필요하다.
판정 조건과 설정은 [놓기·정지 확인](docs/execution_contract.md#정상-수거의-놓기정지-확인)을 따른다.

### 예상하지 못한 실행 예외

프로그램 오류는 `INTERNAL_ERROR`로 기록하고 별도 종료 경로에서 정지를 확인한다.
정지 처리와 최종 결과는 [실행 예외 계약](docs/execution_contract.md#예상하지-못한-실행-예외)을 따른다.

## 기록과 모니터

### 실행 기록

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

### Live 모니터

VS Code BehaviorTree Viewer 0.1.2의 **Monitor**를 사용한다.

| 서버 | 기본 주소 | 표시 근거 |
|---|---|---|
| MuJoCo BT | `127.0.0.1:1667` | 실제 C++ 트리의 UID와 노드 상태 |
| Mock | `127.0.0.1:1666` | 모의 실행 이벤트를 설계 XML에 투영 |

1. VS Code workspace 설정에 `behaviortreeViewer.monitorHost=127.0.0.1`,
   `behaviortreeViewer.monitorPort=1667`을 지정한다.
2. 기존 Monitor 연결을 끈 뒤 다시 켠다. 파일명에 `(live)`와 `Monitoring active`가 표시되면 연결된 상태다.
3. [실행](#실행)의 관찰 요청과 Goal 전송을 수행한다. Goal 전송 전에는 모든 노드가 IDLE이다.

MuJoCo 포트는 `bt_monitor_port`로 변경한다. 뷰어의 기본 포트는 1666이므로 MuJoCo 주소를 지정해야 한다.
모니터는 FULLTREE/STATUS를 제공한다. 트리 구조 XML은 실행기 생성 시 한 번 보관하고,
모니터 조회마다 실제 노드 상태를 읽는다. 로봇 명령이나 blackboard 변경 기능은 제공하지 않는다.

실행 종료 후 native BT가 노드를 IDLE로 reset하면 진행 색도 정리된다.
뷰어 0.1.2는 전체 IDLE 상태가 1.5초 이어지면 표시를 흐리게 한다.
종료 결과는 조회 서비스에, 상태 전환 이력은 ROS/SQLite에 남는다.

### 실행 설정

[config/manipulation_mujoco.yaml](config/manipulation_mujoco.yaml)의 ROS parameter로
동작 상한(180초), 단계별 안착 확인(10초), 정지 평가(10초), 피드백/시계 신선도(2초)와
지원 label을 조정한다. 이 수치는 첫 시뮬레이션 구현의 조정 가능한 기본값이다.
전체 물체 수거 성공률이나 실제 하드웨어에 대한 보장을 뜻하지 않는다.

## 검증 기록 (2026-10-14)

최신 main을 기반으로 정리한 브랜치에서 실제 MuJoCo/MoveIt, CPU YOLOE와 Gemini API로
컵 한 개 수거를 다시 실행했다. 실행마다 새 장면과 별도 SQLite DB를 사용했다.
정상 수거 완료는 확인하지 못했으며, 2026-10-01 성공 기록을 현재 브랜치의 성공 근거로
사용하지 않는다.

| 관찰 결과 | 상태와 조치 |
|---|---|
| 첫 팔 이동에서 충돌 장면을 이중 등록 | `MOTION_FAILED`. 재관찰 장면을 정리한 뒤 pregrasp 장면을 등록하도록 수정하고 회귀 검사 추가 |
| 관절 피드백 시각 중복 | 내용까지 같은 메시지는 수신 시각·정지 확인 횟수를 갱신하지 않도록 수정. 같은 시각에 값이 달라지는 수신은 여전히 `HARDWARE_ERROR`로 중단되며 해결 필요 |
| 추적 없는 경로의 컵 재관측 | 대상 종류가 없거나 두 개 검출되면 수거 중단. 마지막 두 개 검출 사례는 `FAILED / MOTION_FAILED`, `NOT_TOUCHED` |
| 개별 추적 ID를 사용하는 컵 수거 | 접근 전까지 같은 track ID 유지. 관절 피드백 문제로 `FATAL / HARDWARE_ERROR`, `NOT_TOUCHED`, `arm_recovered=false` |

마지막 실행 ID는 `743a9d90-7acf-43b2-8dc0-607149664648`이다. 모든 실행에서
`stop_confirmed=true`였고, 별도 관절 속도 관찰과 Action/조회 결과 일치도 확인했다.
마지막 실패 시 피드백 수신 지연은 약 0.9 ms였으며, 같은 timestamp의 메시지 두 개에서
position/velocity가 달랐다. 스트림 지연과 시각 중복에 따른 무효 판정을 구분할 필요가 있다.
보고서와 전환 이력은 로컬 `artifacts/manipulation-validation-20261014-*`에 보존했다.

관련 10개 패키지 빌드와 `make test-manipulation-bt` 523개 검사는 통과했다.
자동 검사 통과는 위 물리 연동 실패가 해결됐다는 뜻이 아니다. 이번 실행은 고정 베이스의
단일 컵 수거이며 Mission Manager나 모바일 베이스와의 통합 검증은 포함하지 않는다.

## 검증 기록 (2026-10-01)

이 절은 당시의 물리 연동 기록이다. 현재 코드의 자동 검사 명령은 [빌드와 검증](#빌드와-검증)을 따른다.

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
