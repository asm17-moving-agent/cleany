# cleany_skill_executor

파지 후보의 MoveIt 도달성 검증, pre-grasp 실행, MuJoCo 분류·수거와
물체 하나의 모의 Manipulation Action을 담당한다.
실물 명령을 지원하는 통합 운용 단계는 아니며 외부 로봇 경로는 plan-only다.

## 문서 안내

| 하려는 일 | 읽을 문서 |
|---|---|
| 모의 Action 실행·취소·조회, 실패 재현, SQLite 이해 | [모의 Manipulation Action 사용법](docs/manipulation_mock_usage.md) |
| grasp 후보 선택과 가까운 물체의 pre-grasp 실행 | [Grasp 선택과 pre-grasp](docs/grasp_selection_and_pregrasp.md) |
| 스터디카페 파이프라인·분류 실행과 진단 | [스터디카페 실행과 진단](docs/study_cafe_sorting_usage.md) |
| 파지 깊이·속도·카메라·timeout 설정과 실험 기록 | [수거 설정과 실험 기록](docs/study_cafe_sorting_configuration.md) |
| 분류·접촉·충돌·수거함 운반·손목 제약 계약 | [분류·운반 실행 계약](docs/study_cafe_sorting_contract.md) |

## 모의 Manipulation Action

`manipulation_server`는 지정된 물체 하나의 `collect_trash`를 실행한다.
현재 `execution_profile=mock`이며 동작과 확인 근거를 모의로 생성한다.
기본 namespace는 `/mock`이다.

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

실행 단계 Feedback, 최종 Result와 저장 기록이 출력된다.
Feedback 문구는 단계 시작(`Starting`), 관측 결과 수신(`Observation received`),
단계 완료(`Stage completed`), 결과 확정(`Finalizing result`), 실행 종료(`Execution finished`)를 구분한다.
정상 결과는 `SUCCESS`, `placement_state=CONFIRMED`,
`arm_recovered=true`, `stop_confirmed=true`다.
취소·조회·실패 재현과 DB 복구는 [사용법](docs/manipulation_mock_usage.md)을 따른다.

## 다른 실행 경로

아래 Make 명령은 레포 루트에서 실행한다. 모델·API key·ROS 노드 등 준비 조건은
각 실행 안내에서 확인한다.

| 목적 | 주요 명령 | 안내 |
|---|---|---|
| MoveIt plan-only grasp 선택 | `ros2 launch cleany_skill_executor grasp_selection.launch.py` | [Grasp 선택](docs/grasp_selection_and_pregrasp.md#설정-및-검증) |
| 가까운 물체의 pre-grasp | `ros2 launch cleany_skill_executor nearest_pregrasp.launch.py` | [Pre-grasp](docs/grasp_selection_and_pregrasp.md#가까운-객체-자동-pre-grasp) |
| 스터디카페 인식·계획 파이프라인 | `make sim-mujoco-pipeline` | [실행 준비](docs/study_cafe_sorting_usage.md#센서-전용-study-cafe-검증) |
| MuJoCo 분류·수거 | `make sim-mujoco-sorting` | [분리 수거](docs/study_cafe_sorting_usage.md#시뮬레이션-분리-수거-통합-검증-진행-중) |

시뮬레이션 분류는 기본 관찰 모드에서 `complete_unverified`를 기록한다.
모의 Action의 성공, 시뮬레이션의 개별 물체 배치와 전체 물리 수거 성공은 각각의
검증 범위를 따른다. 전체 물리 수거 성공은 검증 진행 중이다.

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

## 책상 정리 모듈 연동 설계

Mission Manager는 행동 승인·재관찰·미션 상태 전이를 담당하고,
Skill Executor는 승인된 행동을 실행해 결과를 반환한다.
Mission Manager ROS adapter, 실물 backend·verifier와 BT.CPP 실행기는 후속 구현 대상이다.
Groot2 XML은 정적 설계 미리보기다.

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
