# cleany_skill_executor

파지 후보의 MoveIt 도달성 검증, pre-grasp 실행, MuJoCo 분류·수거와
물체 하나의 모의 Manipulation Action을 담당한다.
실물 명령을 지원하는 통합 운용 단계는 아니며 외부 로봇 경로는 plan-only다.

## 문서 안내

| 하려는 일 | 읽을 문서 |
|---|---|
| 모의 Action 실행·취소·조회, 실패 재현, SQLite 이해 | [모의 Manipulation Action 사용법](docs/manipulation_mock_usage.md) |
| VS Code에서 모의 Action 진행 상태 보기 | [로컬 트리 모니터링](docs/manipulation_mock_usage.md#vs-code에서-로컬-트리-모니터링) |
| grasp 후보 선택과 가까운 물체의 pre-grasp 실행 | [Grasp 선택과 pre-grasp](docs/grasp_selection_and_pregrasp.md) |
| 스터디카페 파이프라인·분류 실행과 진단 | [스터디카페 실행과 진단](docs/study_cafe_sorting_usage.md) |
| 파지 깊이·속도·카메라·timeout 설정과 실험 기록 | [수거 설정과 실험 기록](docs/study_cafe_sorting_configuration.md) |
| 분류·접촉·충돌·수거함 운반·손목 제약 계약 | [분류·운반 실행 계약](docs/study_cafe_sorting_contract.md) |

## 모의 Manipulation Action

`manipulation_server`는 지정된 물체 하나의 `collect_trash` 또는 `collect_lost_item`을 실행한다.
현재 `execution_profile=mock`이며 동작과 확인 근거를 모의로 생성한다.
기본 namespace는 `/mock`이다.

모의 설정의 `destination_id`는 쓰레기 목적지, `lost_item_destination_id`는 분실물
목적지다. 기본값은 각각 `mock_trash_bin`, `mock_lost_item_bin`이며 잘못된 skill–목적지
조합은 준비 단계에서 차단한다. 테스트 클라이언트에서 분실물을 요청하려면
`--skill-name collect_lost_item --destination-id mock_lost_item_bin`을 지정한다.
실제 분류와 목적지 검사는 [BT 실행기](../cleany_manipulation_bt/README.md)가 담당한다.

**터미널 A — 레포 루트에서 빌드·서버 실행**

```bash
make build-manipulation
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

mock_db_dir="$(mktemp -d)"
ros2 launch cleany_skill_executor manipulation_mock.launch.py \
  database_path:="$mock_db_dir/executions.sqlite3"
```

**터미널 B — 레포 루트에서 요청**

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run cleany_skill_executor manipulation_test_client
```

실행 단계 Feedback, 최종 Result와 최신 실행 기록이 출력된다.
Feedback은 `stage/substage`로 큰 단계와 세부 동작을 함께 표시한다.
예를 들어 `GRASPING/GraspObject`는 그리퍼 닫기, `GRASPING/ConfirmGrasp`는 파지 확인이다.
Feedback 문구는 단계 시작(`Starting`), 관측 결과 수신(`Observation received`),
단계 완료(`Stage completed`), 결과 확정(`Finalizing result`), 실행 종료(`Execution finished`)를 구분한다.
정상 결과는 `SUCCESS`, `placement_state=CONFIRMED`,
`arm_recovered=true`, `stop_confirmed=true`다.
실행 기록 저장 실패는 경고로 남기고 실제 동작의 status와 error_code를 유지한다.
진행과 결과는 서버 메모리에도 보관하므로 같은 프로세스의 조회는 최신 결과를 반환한다.
저장 실패만으로 동작이나 후속 Goal을 차단하지 않으며, 실제 fault·정지 미확인·
물체 보유·상태 불명에 따른 차단은 유지한다. 메모리의 미저장 기록은 재시작 후 복구할 수 없다.
취소·조회·실패 재현과 DB 복구는 [사용법](docs/manipulation_mock_usage.md)을 따른다.

취소 모드는 `CHECKPOINT`(현재 단계와 확인을 완료하고 정지), `IMMEDIATE`(완료 대기 없이
정지), `RETURN_ARM`(현재 작업 중단, 정지 확인, 잡고 있는 물체를 그 자리에서 놓고 팔 복귀)다.
`manipulation/cancel` 서비스나 테스트 클라이언트의 `--cancel-mode`로 선택한다.
일반 Action cancel은 `CHECKPOINT`다. `RETURN_ARM`의 현재 위치 놓기는 수거함 확인을
수행하지 않으며 추가 놓기 위치 정책은 후속 검토 대상으로 둔다. 기존 정지 확인,
controller 실패와 timeout 처리는 유지한다. Mission Manager의 취소 adapter는 아직 없다.

VS Code **BehaviorTree Viewer 0.1.2**에서 진행 상태를 보려면 서버 launch에
`monitor:=true`를 추가한다. `manipulation_monitor`가 모의 실행 이벤트를 읽어
`127.0.0.1:1666`에서 XML과 노드 상태를 제공한다. 뷰어의 **Monitor** 버튼으로
연결한 뒤 터미널 B의 클라이언트를 실행한다. 모니터는 표시 전용이며 BT를 실행하지 않는다.
실패와 취소는 트리의 FAILURE로 표시되며 정확한 Action 결과는 Feedback·Result에서 확인한다.

## 다른 실행 경로

아래 Make 명령은 레포 루트에서 실행한다. 모델·API key·ROS 노드 등 준비 조건은
각 실행 안내에서 확인한다.

| 목적 | 주요 명령 | 안내 |
|---|---|---|
| MoveIt plan-only grasp 선택 | `ros2 launch cleany_skill_executor grasp_selection.launch.py` | [Grasp 선택](docs/grasp_selection_and_pregrasp.md#설정-및-검증) |
| 가까운 물체의 pre-grasp | `ros2 launch cleany_skill_executor nearest_pregrasp.launch.py` | [Pre-grasp](docs/grasp_selection_and_pregrasp.md#가까운-객체-자동-pre-grasp) |
| 스터디카페 인식·계획 파이프라인 | `make sim-mujoco-pipeline` | [실행 준비](docs/study_cafe_sorting_usage.md#센서-전용-study-cafe-검증) |
| 승인된 물체 하나의 실제 BT Action | `make sim-mujoco-manipulation` | [MuJoCo BT 서버](../cleany_manipulation_bt/README.md) |
| MuJoCo 분류·수거 | `make sim-mujoco-sorting` | [분리 수거](docs/study_cafe_sorting_usage.md#시뮬레이션-분리-수거-통합-검증-진행-중) |

시뮬레이션 분류는 기본 관찰 모드에서 `complete_unverified`를 기록한다.
모의 Action의 성공, 시뮬레이션의 개별 물체 배치와 전체 물리 수거 성공은 각각의
검증 범위를 따른다. 전체 물리 수거 성공은 검증 진행 중이다.

## MuJoCo 실행 로그

`study_cafe_nearest_grasp_demo.launch.py`와 이를 포함하는 수거·BT launch는
`log_level:=info`, `backend_log_level:=warn`을 기본으로 사용한다.
`log_level:=debug`로 후보별 IK·경로 평가, 그리퍼 수치와 손목 RPC 타이밍을
확인할 수 있다. 최종 후보 선택과 주요 작업 단계는 `INFO`로 남는다.
Action feedback과 ROS 상태 토픽·파일 기록은 로그 수준과 무관하게 유지한다.
수거 단계의 기본 로그는 단계·대상·목적지·완료 개수만 요약한다. 전체 상태 JSON은
`DEBUG`와 기존 상태 토픽·파일에서 확인한다.
외부 MuJoCo·MoveIt·TF의 상세 출력은 `backend_log_level:=info`로 켠다.
`log_level`은 끌리니 노드의 지정 로거에만 적용한다. `debug`에서도 ROS 내부
대기 루프·통신 계층은 전역 `DEBUG`로 바꾸지 않는다.

## 설정 및 검증

| 설정 파일 | 대상 |
|---|---|
| [manipulation_mock.yaml](config/manipulation_mock.yaml) | 모의 대상·시나리오·제한 시간 |
| [grasp_selection.yaml](config/grasp_selection.yaml) | MoveIt grasp 선택 |
| [nearest_pregrasp.yaml](config/nearest_pregrasp.yaml) | pre-grasp·수거 동작 설정 |

| 검사 | 레포 루트에서 실행할 명령 |
|---|---|
| 모의 core, 가짜 시계 | `make test-manipulation-core` |
| 모의 Action의 실제 ROS 통신·기록·재시작 복구 | `make test-manipulation` |
| RGB-D grasp/pre-grasp 관련 검사 | `make test-grasp-pregrasp` |
| MoveIt mock 실행 | `make test-grasp-pregrasp-runtime` |

패키지 전체 pytest는 ROS setup을 읽은 터미널에서 실행한다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ros2_ws/src/cleany_skill_executor/test
```

launch 사전 검증 테스트는 임시 모델 파일을 사용해 로컬 모델 설치 상태와 분리한다.
ROS 통신 테스트는 feedback 토픽 발견을 확인한 뒤 Goal을 전송한다.
Action 완료 후 보관 메시지 수신은 별도 ROS context에서 늦게 접속한 구독자로 검사한다.

## 책상 정리 모듈 연동 설계

Mission Manager는 행동 승인·재관찰·미션 상태 전이를 담당하고,
Skill Executor는 승인된 행동을 실행해 결과를 반환한다.
실제 BT.CPP MuJoCo Action 실행기는 [cleany_manipulation_bt](../cleany_manipulation_bt/README.md)에 있다.
기존 grasp/수거 동작을 단계별로 공유하며 자동 sorting의 기존 실행 순서는 유지한다.
Mission Manager ROS adapter와 실물 backend·verifier는 후속 구현 대상이다.
Groot2 XML은 정적 설계 미리보기다.
정상 경로는 준비·물체 잡기·물체 놓기·확인과 종료의 네 그룹으로 표시한다.
준비에는 모델과 실행 준비 확인, 대상 관측 확인, 선택 물체 3D 복원과 파지 준비를 표시한다.
서버 시작과 모델 로딩은 Goal 이전의 서비스 준비 과정이고, 책상 전체 최초 인식과
대상 선택은 상위 미션 흐름에 속한다. mock 경로의 인식과 준비 확인은 모의 처리다.
모니터 브리지는 세부 단계뿐 아니라 각 그룹의 진행·완료·실패도 표시한다.
로컬 모니터는 이 XML에 모의 서버 이벤트를 투영하며 BT.CPP tick loop를 구현하지 않는다.

| 문서 | 내용 |
|---|---|
| [01. 설계 안내](docs/01_manipulation_action_design.md) | 전체 흐름과 책임 경계 |
| [02. Action 명세](docs/02_execute_manipulation_skill_action_spec.md) | Goal·Feedback·Result 계약 |
| [03. 서버 동작 명세](docs/03_manipulation_server_behavior_spec.md) | 단계·실패·취소·기록 |
| [04. Mission 결과 매핑](docs/04_mission_result_mapping.md) | Mission Manager 후속 통합 |
| [05. 관측 데이터 명세](docs/05_initial_table_observation_spec.md) | 이미지·객체·snapshot 전달 |
| [06. BT와 Groot2 설계](docs/06_manipulation_behavior_tree_design.md) | 행동 내부 트리와 실행기 경계 |

제품 기준은 KB의 [Mission Lifecycle](../../../docs/cleany-docs/20_TECHNICAL/09%20-%20Mission%20Lifecycle.md)과
[Task Planning and Robot Capabilities](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Task%20Planning%20and%20Robot%20Capabilities.md)를 따른다.

## 기존 파지 데모 실행

기존 `grasp_execution_demo.launch.py`와 `can_grasp_execution_demo.launch.py`를
유지한다. 두 데모의 실행 파일과 전용 MuJoCo 장면은 새 Action 서버와 별도로
실행할 수 있다. 빌드 후 ROS 환경을 불러온 터미널에서 실행한다.

```bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py
ros2 launch cleany_skill_executor can_grasp_execution_demo.launch.py
```

도달 데모는 기존 목표 위치를 유지하며, 현재 CAD 팔 모델에서 접근 방향과
TCP 자세가 일치하는 후보를 사용한다. 새 파지 선택기의 접근·충돌 검사를
그대로 적용하고, 첫 번째 도달 불가 후보에서 정상 후보로 넘어가는 동작도 유지한다.
