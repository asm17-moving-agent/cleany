# cleany_manipulation_bt

승인된 `snapshot_id/object_id`의 쓰레기 하나를 MuJoCo에서 수거하는 ROS Action 서버다.
실제 BehaviorTree.CPP 4 실행기가 동작 순서를 소유하고, Python은 기존 인식, MoveIt,
그리퍼 동작과 SQLite 기록을 연결한다. Planner와 Mission Manager는 포함하지 않는다.
기존 mock 실행에는 이 패키지의 C++ 모듈을 import하지 않는다.

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
터미널의 depth/map/camera 로그는 이 대기 중에도 계속 나온다. BT의 RUNNING 전환을 보려면
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

## 계약과 증거

| ROS 경로 | 용도 |
|---|---|
| `/sim/manipulation/execute_skill` | 기존 `ExecuteManipulationSkill` 계약, `execution_profile=mujoco` |
| `/sim/manipulation/get_execution` | 기존 `GetManipulationExecution` 조회 계약 |
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
기본 지원 쓰레기는 `cup`, `crumpled tissue`다. 관측 label의 `paper cup`,
`disposable paper cup`은 같은 컵 종류 alias로 연결하며 원래 인식 label과 분류를 보존한다.
같은 종류가 여러 개 있으면 이동 전에 차단한다. 평가기의 label 식별 한계로
미지원 대상, 분실물, review 분류와 다른 목적지도 차단한다.
접근 전 재관찰은 원래 승인된 label과 기하 일치/모호성 검사만 사용하며 재선택/재집기는 하지 않는다.

파지 확인이 실패하면 들기를 시작하지 않는다. 보유 상실, controller 실패, 동작 timeout과
관절/시뮬레이션 시계 중단은 정지 평가로 연결된다. 취소는 잡기(닫기+확인), 들기(들기+보유 확인),
놓기(위치 검사+열기+이탈 확인)의 현재 checkpoint까지 제한 시간 안에서 수행한 뒤 다음 단계를 차단한다.
실패한 정상 경로 뒤에는 `StopAndAssess → FinalizeFailure`를 실행한다.
Action 결과의 BLOCKED/FAILED/CANCELED/FATAL과 BT의 FAILURE는 별도 의미다.

그리퍼 열림은 controller와 관절 피드백으로 확인한다. `ConfirmRelease`는 방출 시각 이후의
독립 안착 검증을 기다리고, 팔 복귀 뒤 `VerifyPlacedObject`에서 새 시각을 기준으로 다시 검증한다.
평가기의 false는 확인 대기다. 제한 시간 내 안착 근거를 못 얻으면
`placement_state=UNKNOWN / VERIFICATION_TIMEOUT`을 반환한다.
SUCCESS는 독립 방출/안착, 팔 복귀와 최종 정지 근거가 모두 있을 때만 저장한다.
정지는 모든 ROS Action의 terminal 결과와 정지 요청 이후 서로 다른 시각의 새 관절 샘플을 요구한다.
취소 응답만으로 정지를 인정하지 않으며, 움직임 시작 후 정지가 확인되지 않으면
`FATAL / STOP_UNCONFIRMED`와 사람 확인 필요 상태를 기록한다.

## 기록과 모니터

기본 DB는 `$XDG_STATE_HOME/cleany/manipulation_mujoco/executions.sqlite3`이다.
`XDG_STATE_HOME`이 없으면 `~/.local/state`를 사용한다. mock DB와 분리되어 있으며
`bt_database_path:=/절대/경로.sqlite3`로 바꿀 수 있다. 각 실제 동작의 시작 checkpoint를
먼저 저장하고, 완료 관측과 최종 Result를 저장한 뒤 Action을 종료한다.
`bt_transitions` 테이블에는 C++ 실행기의 실제 UID/상태 전환을 추가 저장한다.
프로세스 중단 기록은 재시작 시 INTERRUPTED/사람 확인 필요로 바꾸고 자동 재개하지 않는다.
HELD/UNKNOWN, FATAL 또는 기록 실패가 남으면 후속 Goal을 차단한다.
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
