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
전체 진행 순서를 검사하는 경로는 마지막 feedback을 수신한 뒤 Result를 조회한다.
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

query, timeout, 속도/가속도 scaling과 gripper open 위치는
`config/nearest_pregrasp.yaml`에서 설정한다. pre-grasp joint target은 운영 selector가
aim-tip IK와 FK 방향 검증까지 통과한 결과를 사용한다. 성공한 target OBB는 완료 자세에서
Planning Scene에 유지하며, 실패하거나 coordinator가 종료될 때 기존 ACM과 함께 복원한다.

Study-cafe의 네 물체를 대상으로 인식부터 접촉 집기와 후퇴까지 실행하려면 다음 launch를
사용한다.

```bash
ros2 launch cleany_skill_executor study_cafe_nearest_grasp_demo.launch.py
```

기본값은 YOLOE-seg instance mask + Gemini 3.1 Flash-Lite 상세 분류 설정이다. 기존 color adapter는
과거 머그컵/휴대폰/지우개 fixture 전용이며 현재 종이컵/레고/휴지에 대응하지 않는다.
이 study-cafe MuJoCo launch의 YOLOE 기본 체크포인트는
`~/models/yoloe/study_cafe_sim_yoloe26s_seg.pt`다. 현재 시뮬레이션 head 영상으로
fine-tune한 파일이며, 생성·학습·평가 절차는
[`cleany_perception/README.md`](../cleany_perception/README.md#스터디카페-mujoco-전용-yoloe-seg-학습)에 있다.
손목 HANDOFF는 별도 `~/models/yoloe/study_cafe_sim_all_views_yoloe26s_seg.pt`를
`wrist_yoloe_model_path`로 로드한다. 오른손목 HANDOFF는 head 체크포인트를
`wrist_right_yoloe_model_path`로 재사용한다. 실제 컵 pregrasp 오른손목 프레임에서
head 모델은 컵을 confidence 0.726으로 검출했고, 양손목 바닥 학습 모델은 검출하지
못했다. 파지 후 CHECK는
`~/models/yoloe/study_cafe_sim_held_yoloe26s_seg.pt`를
`wrist_check_yoloe_model_path`로 로드한다. Head 모델은 기존 학습본을 유지한다. 왼손목
시점 holdout의 마스크 IoU 0.5 일치율은 52/80→68/80으로 개선됐지만, 새 모델을
head에 쓸 때는 71/80→64/80으로 하락했기 때문이다. 오른손목에서 실제 라벨이
있는 컵·휴지의 일치율은 38/40→36/40이었다. 이 수치는 고정 시뮬레이션 자세와
별도 생성 영상의 평가이며 실제 집기 성공률을 뜻하지 않는다. 학습 결과의 `best.pt`를
각 손목 경로로 복사해 사용한다. 파지 후 레고 holdout은 CHECK 모델에서 18/20으로
개선됐으나, 같은 모델의 파지 후 컵·마우스·휴지 검출은 아직 0/10이다. 실제 카메라 영상에 대한 성능은
검증하지 않았으며, 실물 실행에는 별도 검증된 checkpoint를 `yoloe_model_path`로 지정한다.
스터디카페 기본 실행은 왼손목 마우스 CHECK에
`wrist_mouse_check_yoloe_model_path`의 전용 모델을 쓴다. 실제 MuJoCo 실패
프레임에서 기존 모델은 마우스 검출 0건, 전용 모델은 신뢰도 0.25였고, 마우스
단독 및 전체 실행에서 파지 후 CHECK를 통과했다. 오른손목 CHECK의 컵 전용
모델이 물체를 놓치면 기존 파지 후 통합 모델로 같은 RGB를 재검사한다. 실제
MuJoCo 휴지 CHECK 프레임에서 컵 전용 모델은 검출 0건, 통합 모델은 휴지를
신뢰도 0.364로 검출했다. 두 추가 검사 모두 예상 물체의 투영 위치와 mask
검증을 통과해야 한다.
YOLOE 추론의 최소 confidence는 0.08이고, 클래스별로 컵·마우스·휴지는 0.25,
레고는 0.08을 적용한다. 다른 클래스 목록이나 모델로 바꿀 때는
`yoloe_class_confidence_thresholds`도 같은 순서로 조정해야 한다.
시뮬레이션 수거 정책 `table_sorting_policy.yaml`의 최종 최소 confidence도 0.08로
맞췄다. 레고 외 물체는 YOLOE 단계에서 먼저 0.25 미만을 제거한다.
다른 YOLOE checkpoint는 `yoloe_model_path`로 지정할 수 있다.
전체 파이프라인은 아래 스터디카페 검증 명령으로 실행한다.

## 센서 전용 study-cafe 검증

`study_cafe_nearest_grasp_demo.launch.py`는 YOLOE-seg + Gemini,
aligned RGB-D/TF, AnyGrasp 후보와 MoveIt 검증을 연결한다.
`cleany_perception/config/yoloe_seg_gemini.yaml`을 바탕으로 스터디카페
전용 head/손목 체크포인트를 설정한다. `GEMINI_API_KEY`와 로컬 YOLOE 모델이 필요하다.
Depth obstacle scene은 객체 검출과 별도로 유지하며 target 위치를 simulator에서 받지 않는다.

```bash
make sim-mujoco-pipeline
make sim-mujoco-sorting
```

인식·수거 launch의 `scheduled_cameras` 기본은 true다. 일반 인식은 head RGB-D만
10Hz로 촬영하며, 수거는 선택한 손목 카메라와 head 촬영 빈도를 전환한다.
손목 전환을 사용하는 수거는 `scheduled_cameras:=false`를 허용하지 않는다.
GUI 기본은 `sim_viewer:=efficient`이며 센서 snapshot을 공유하는 최대 20Hz 창이다.
센서 해상도·그림자는 유지하고 GUI 그림자만 기본으로 끈다. 전체 MuJoCo UI는
`sim_viewer:=native`로 선택한다. backend 설정은 observer README를 따른다.

```bash
make sim-mujoco-pipeline PIPELINE_ARGS='headless:=true use_rviz:=false use_image_view:=false'
make sim-mujoco-pipeline PIPELINE_ARGS='headless:=false use_rviz:=false use_image_view:=false'
make sim-mujoco-pipeline PIPELINE_ARGS='headless:=false sim_viewer:=native'
```

## 시뮬레이션 분리 수거 (통합 검증 진행 중)

분리 수거 모드는 selector의 `require_pregrasp_visibility=true`를 사용한다.
관측한 OBB 8개 모서리를 현재 camera TF 기준으로 원근 투영하고, 이를 포함하는
16면 visibility cone을 구성한다. MoveIt `GetStateValidity`의
`VisibilityConstraint`가 준비 자세의 로봇 CAD와 이 영역의 겹침을 검사한다.
가리는 자세는 움직이기 전에 제외하고 다른 IK/후보를 검토한다. 반경은 물체
정답 크기나 고정 컵 치수가 아니라 RGB-D OBB에서 계산하며 기본 padding은
3 mm다. 동적 camera TF는 기본 0.5초 이내여야 하고, 검사 결과가 누락되면
통과로 취급하지 않는다. 일반 selector의 기본값은 false다.

`sorting_contact_diagnostics:=true`는 MuJoCo observer의 접촉 부위/힘
출력만 켠다. 기본은 false이며, controller 오차와 실제 접촉을 대조하는
진단용이다. coordinator나 grasp selector는 이 GT 토픽을 읽지 않는다.

```bash
make sim-mujoco-sorting

# 기본 sorting 실행은 headless이며 RViz/image_view를 시작하지 않는다.

# 수거 완료/실패 후에도 시뮬레이터와 GUI 유지 (수거 동작을 자동 재시도하지 않음)
make sim-mujoco-sorting SORTING_ARGS='headless:=false use_rviz:=true use_image_view:=true shutdown_on_sorting_exit:=false'
# 같은 실행에서 GUI 창만 생략 (vendor 카메라 렌더링에는 DISPLAY 필요)
make sim-mujoco-sorting SORTING_ARGS='headless:=true use_rviz:=false use_image_view:=false'
```

MoveIt adapter의 `First ranked grasp/pregrasp` 로그는 실제 반환 순서의 첫 IK
해의 오차를 기록한다. 예전 `Accepted grasp` 로그는 joint-motion 우선 순서와 달리
최소 방향 오차만 출력하므로 선택된 해의 자세로 해석하면 안 된다. 첫 해도 이후
충돌/계획 검사에서 탈락할 수 있어 최종 선택은 action result의 joint state가 근거다.

Study-cafe launch의 `gripper_open_position_rad`(기본 1.2rad)로 실행 전 개방 폭을
명시적으로 설정할 수 있다. Sorting은 홈에서 해당 개방까지의 sweep을 검사한 뒤,
그 상태로 pregrasp와 접근을 계획한다. 큰 물체 진입 실험용이며 close command의
보정식, 접촉 gate, joint/controller 허용치는 변경하지 않는다.

Sorting selector는 `joint_limit_margin_rad=0.005`로 arm IK endpoint에 관절 상한/하한
여유를 둔다(일반 selector 기본 0). 경계에 포화된 해는 순위 목록과 state validity
앞에서 제외한다. 43의 pregrasp가 손목 상한에 정확히 놓인 뒤 feedback가 12.44µrad
넘어 Cartesian 시작 검증이 실패한 것을 방지하기 위한 선택 조건이다. 실제 feedback나
trajectory를 clipping하지 않으며 hard joint bounds/controller 허용치는 그대로다.

Study-cafe launch는 모든 child process 시작 전에 `fastdds_profiles_file`을
`FASTRTPS_DEFAULT_PROFILES_FILE`로 전달한다. 이미 지정된 환경변수를 우선 보존하고,
없으면 `cleany_perception/config/fastdds_rgbd.xml`(UDP + participant별 16 MiB SHM)을
기본으로 쓴다. 빈 launch 값은 사용자 기본 middleware 설정으로 되돌리는 진단 옵션이다.
다른 RMW의 동작을 바꾸는 설정이 아니며 QoS·freshness 기준은 변경하지 않는다.
41의 실제 집기 구간에서 지도 receipt 2초 초과가 없었으나 모든 부하/장치에서의 보장은
아니다. 별도로 띄우는 기록기/카메라는 같은 DDS 환경을 설정해야 비교할 수 있다.

`sorting_reobserve_after_lift=true`는 head 재검출에 실패한 경우(기본 관측 경로에서는
검출 대상이 없을 경우) 한 번의 카메라 재관측 이동을 허용한다. 정착한 실제 TCP와 관측 OBB로 파지
offset을 잡고, CameraInfo/TF의 중앙 영상 ray 3개와 관측 높이 평면들의 교점을
제안한다. 관측 OBB의 bounding sphere 전체가 화면 안에 들어오는 후보만 쓰며
`sorting_reobserve_margin_px=24`, `sorting_reobserve_geometry_padding_m=0.01`,
`sorting_reobserve_max_translation_m=0.20`으로 여백·최대 이동을 제한한다.
현재 높이에서 맞지 않으면 `sorting_reobserve_max_lowering_m=0.10` 범위에서
높이를 최대 3단계로 검사한다. 낮추더라도 필수 상대 상승량에
`sorting_reobserve_height_clearance_m=0.02`를 더한 높이 아래로 내려가지 않는다.
attached geometry를 보존하는 collision-aware IK와
일반 MoveIt 경로 계획·실행을 통과해야 한다. 이동 전후 gripper contact를 확인하고
이후 같은 선택된 관측 경로의 상대 상승량 검증을 다시 수행한다. 관측 재실패·재구성 실패·
관측 높이 부족은 성공으로 처리하지 않는다. `sorting_held_association_tolerance_m=0.03`
내에서 관측 중심과 파지 추정 중심이 일치해야 운반으로 넘어간다. 이 기능은
센서 기반 재관측 동작이며 MuJoCo GT나 자동 물체 부착을 사용하지 않는다.
실제 재관측 성공 여부는 실행 기록으로 별도 검증해야 한다.
재관측/수거함 운반의 일반 joint-goal 이동에는 파지 중에만
sorting launch에서는 `sorting_payload_velocity_scaling=0.24`,
`sorting_payload_acceleration_scaling=0.20`를
적용한다. 기존 속도·가속도보다 높이는 설정은 적용하지 않으며 열린 그리퍼로
복귀할 때는 원래 값을 사용한다. 접근/역접근의 Cartesian retiming은 별도로
유지한다. 노드 단독 기본값은 둘 다 0.01이며 launch 인수로 이전 값을 재현할 수 있다.
이는 시뮬레이션 시험 설정이며, 지속 파지 성공이나 실제 하드웨어 적합성을 뜻하지 않는다.

`config/table_sorting_policy.yaml`은 Gemini의 category/reason을 받아
쓰레기를 `trash_right`, 분실물을 `lost_items_left`로 보낸다. 위험 라벨과 저신뢰,
category/reason 미제공 결과는 review로 남기고 조작하지 않는다.

현재 장면은 종이컵·마우스·휴지뭉치·확대한 레고를 사용한다. YOLOE의 명시 클래스는
`cup`, `computer mouse`, `crumpled tissue`, `lego brick`이며 Gemini는 고정 목록 없이
외형에 따른 자유 라벨을 생성한다. 모델 출력 category/reason으로 분류하고 위험·저신뢰
결과는 review로 남긴다. 장면의 형상·크기와 평가용 body mapping은
[MuJoCo README](../cleany_mujoco_sim/README.md)를 따른다.

수거 위치는 로봇 뒤쪽 +Y(좌측)의 `lost_items_left`, -Y(우측)의
`trash_right`다. `cleany_mujoco_sim/config/robot_top_bins.yaml`은
수거 위치의 알려진 보정값/형상만 공유한다. 책상과 집을 물체의 위치/형상은
카메라에서 얻으며 scene의 target 좌표를 planner에 전달하지 않는다.
수거함은 바닥과 4개 벽으로 구성되고 MoveIt에도 같은 목적지 형상을 등록한다.
내부 후면 받침판도 같은 설정으로 MoveIt 충돌 형상에 등록한다.
이 좌표는 현재 고정 베이스 시뮬레이션 전용이며 이동 베이스 운용을 보장하지 않는다.

분류 → 물체에 가까운 팔의 후보를 먼저 검사하고 실패하면 반대 팔 검사 → 재인식/집기/후퇴/들기 →
부착 물체를 포함한 MoveIt 운반 → 수거함 개구부 확인 → 열기 → 복귀 →
선택적 놓기 검증 순서다. 운반 실패 또는 비정상 물체 중심이면 그리퍼를 열지 않는다.
sorting 모드에서는 다각도 grasp 후보와 전체 팔 IK seed를 추가로 탐색하되
기존 pose/충돌 허용 기준은 유지한다. 운반 IK에는 전체 joint feedback과
`RobotState.is_diff=true`를 보내 부착한 OBB가 검사에서 빠지지 않도록 한다.
Sorting은 초기 양손 개방을 따로 명령하지 않는다. 선택된 팔의 pregrasp와
그리퍼 개방을 6관절 경로로 함께 계획하고, 목표 도착 시 모든 관절 피드백을 검사한다.
별도 닫기·놓기의 `gripper_motion_sec` 기본값은 2초이며 반환 시에도 팔 복귀와
턱 닫기를 같은 경로로 계획한다. 속도는 상단 설정 표를 따른다.
재검출 후보를 selector에 전달하기 전 이전 snapshot의 target OBB를 제거해 중복
충돌체를 방지한다. 닫기 후 잔여 각도·속도의 안정 구간이 확인돼야 attachment/lift로 진행한다.
동작 실패 뒤 depth boost 복원까지 실패하면 두 오류를 기록하고 최초 동작 오류를 유지한다.

재선택한 grasp가 현재 팔 위치에서 LIN 방향 기준을 넘으면, 반환된 새
pregrasp로 충돌 검사 이동한다. 이 이동 중 target 접촉은 허용하지 않는다.
이후 새 RGB-D로 동일 물체의 OBB 모서리 집합이 기본 5 mm 이내에서 유지되는지
확인해야 접근한다. 없거나, 오래됐거나, 여러 물체가 일치하면 중단한다.
기존 LIN 10도 기준은 그대로이며, 보정 이동 후에도 다시 검사한다.

Sorting은 `use_joint_corridor_grasp=true`, `use_seeded_cartesian_grasp=true`로
선택된 grasp **관절값**을 보존하는 접근을 사용한다. TCP는 실제 시작점과
목표 사이의 반경 1 mm 원통 안으로 제한하고, 역접근은 실제 접근 시작 관절값을
목표로 동일하게 계획한다. 두 끝 관절값의 보간을 seed로 기본 10 mm 간격
collision-aware IK를 구한 뒤 단조 cubic 곡선을 구성한다. 모든 중간 검사는
전체 관절 피드백과 현재 attached body를 보존한다.
이는 5-DOF 팔에서 임의의 6-DOF 방향을 정확히 보간하는 Pilz LIN과 동일하지 않다.
반환된 모든 waypoint를 FK로 검사해 원통 이탈/역행을 거부하며, 시작·끝 방향은
0.01 rad 이내, 중간 방향은 두 끝 자세의 SLERP에서 기본 5도 이내여야 한다.
충돌·목표 접촉 허용 정책과 controller/contact 한계는 변경하지 않는다.
관절 속도와 가속도의 곡선 전체 상한을 분석해 시간을 정하고, 실제 JTC cubic
보간 곡선을 기본 1 mm 분할 및 관절별 0.005 rad 이하 간격으로 표본화해
MoveIt 충돌/constraint/FK 검사를 거친다. 최대 2000점 초과 시 중단한다.
JTC에 position+velocity를 전달하며 acceleration 필드는 비워 동일한 cubic
보간을 유지한다. 모델 관절 한계는 `cartesian_joint_limits_file`(기본 MoveIt
`joint_limits.yaml`)에서 읽는다. 가속도 미정 관절은 명시 파라미터
`cartesian_joint_acceleration_rad_s2=1.0`에 실행 scaling을 적용한다. 이는
MoveIt 기본 가속도와 같은 시험용 상한이지 하드웨어 검증값은 아니다.

추가로 FK sample의 병진/회전 속도와 병진 가속도를 기준으로 trajectory
시간만 균일하게 늘린다. 기본 Cartesian 상한은
0.10 m/s, 0.50 rad/s, 0.20 m/s²에 실행 scaling을 적용하고, 추가 시간 여유는
`corridor_time_margin=2.0`이다. 위치/경로는 바꾸지 않고 velocity/acceleration도
같은 비율로 줄인다. 이 FK 표본 검사는 연속 시간 속도/충돌 또는 실제 하드웨어
안전 인증을 대신하지 않는다. `use_seeded_cartesian_grasp=false`이고 joint
corridor만 켜면 기존 OMPL 계획+FK 검사 경로를 사용한다. 일반 데모의 두
모드는 opt-in이며 수직 lift는
여전히 endpoint 검사를 통과한 Pilz LIN을 사용한다.

sorting launch의 현재 기본값은 `approach_velocity_scaling=1.0`,
`retreat_velocity_scaling=0.8`, `corridor_time_margin=1.05`이다. 같은 이름의
launch 인수로 조절하며 일반 데모는 기존 0.2/0.4/2.0을 유지한다.
관절 속도·가속도 한계, 충돌/경로 검사, controller tolerance는 완화하지 않는다.
sorting joint-corridor의 grasp 접근에만 `sorting_approach_acceleration_scaling=1.0`을
적용한다(기존 0.4). 계획과 FK sample 시간 배율 검사에 같은 값을 사용한다.
관절 자체 한계는 제거하지 않는다. sorting Cartesian fallback 가속도는 4 rad/s²,
별도 gripper motion은 2초, LIN 가속 배율은 0.8이다. MoveIt TOTG의 URDF 미정
관절 fallback과 Cartesian fallback은 서로 다른 경로다. 현재값 전체는 상단
속도 표를 따른다. 실기 검증값은 아니다.

sensor-scene 모드에서는 OBB attach 성공 뒤 **그 시점보다 새로운 촬영 시각**의
처리 완료 depth receipt를 기다린다. `attachment_scene_timeout_sec=5.0` 안에
오지 않으면 후퇴를 시작하지 않는다. 기존 capture/map age 2초 조건도 유지한다.
이 barrier는 새 frame 통합 전 계획을 막지만 지도 잔상이 모두 없어졌음을
보증하지는 않는다.

Sorting selector는 `align_grasp_wrist_roll=true`도 사용한다. 현재 Cleany
URDF의 wrist-roll 축과 TCP offset이 local -Y로 일치하므로, position IK가
찾은 해의 closing 방향을 이 축 둘레로 직접 맞춘 후보를 만든다. 관절 범위
안의 동치 각도만 검사하며 비대칭 jaw의 180도 반전을 허용하지 않는다.
회전한 후보는 전체 robot state 충돌 검사와 기존 FK 위치/approach/closing
허용 기준을 다시 통과해야 한다. 회전은 **계획 후보**에만 적용하며 실제
로봇/물체를 강제로 회전하지 않는다. 모델-parity 테스트가 축/TCP 가정을
검사하며 일반 selector 기본값은 false다.
부착 해제 시에는 MoveIt이 world에 다시 만든 임시 OBB까지 순차적으로
제거한다. `/sorting/status`의 실패 메시지에는 별도 `error`가 포함된다.
`sorting_artifact_directory:=/absolute/path`를 launch에 지정하면 실행마다
새 `run-<uuid>` 하위 폴더에 `inspection`(InspectScene.Result),
`grasps`(PlanGrasp.Response), `selection`(SelectReachableGrasp.Result)을
CDR로 저장한다. fresh depth/3D 추정과 실제 사용한 후보를 오프라인에서
비교하기 위한 출력 전용 옵션이며, 저장 데이터를 제어 입력으로 재사용하지 않는다.
기본값은 빈 문자열로 저장하지 않는다.
접근/후퇴/lift의 `motion_request`(MoveGroup.Goal)와 `motion_result`
(MoveGroup.Result)는 계획 실패/실행 전 거부 시에도 저장한다.
seeded 방식에서는 요청의 pipeline/planner ID가 `cleany_seeded_cartesian` /
`monotone_cubic`이며, 이는 계획 입력을 같은 형식으로 기록한 것이다.
해당 가상 pipeline ID를 MoveGroup 서버에 전송하지 않는다. 결과의 trajectory와
planning_time은 실제 IK/검증 계산으로 작성하며 실패도 기록한다.
실행 전 FK 검증을 통과한 trajectory는 `motion_plan` 이름의
`moveit_msgs/RobotTrajectory` CDR로 저장한다. 이 출력에는 corridor 시간 보정이
반영되며, 저장 자체가 물리 실행/파지 성공을 뜻하지는 않는다.
`grasp_approach_offset_m` launch 인수(시뮬레이션 수거 0.018 m, 그 외 0.016 m)는 selector의 IK 목표와
coordinator의 실행/표시 보정에 동일하게 전달한다. 회전식 비대칭 jaw의
벌림에 따른 실제 접촉 깊이를 보정하는 시험용 입력이며, 현재 기본값은
실측 하드웨어 보정이나 지속 파지 성공으로 검증된 값이 아니다.
같은 옵션은 selector의 출력 전용 service trace도 활성화한다. 별도
`services-<uuid>` 폴더에 각 MoveIt 서비스 요청/응답 CDR과 완료 시간,
취소 여부 JSON을 저장한다. 실패한 IK 요청의 seed/목표를 재현하기 위한
진단이며 응답이나 계획을 수정하지 않는다. 이 옵션의 파일 I/O가 포함된
시간을 저장 기능을 끈 운용 성능으로 해석하지 않는다.
`/sorting/status`는 단계, label, 분류, 목적지, 완료 항목을 JSON String으로
latched 발행한다. 기본은 관찰 모드이며 물체별 `placement_verified=false`와
`mission_complete_unverified`를 기록한다. 자동 검증을 켜면 물체별 안착 확인 후
`mission_complete`를 발행한다. 빈 작업영역은 연속 두 번 확인해야 한다.
`sorting_require_category_coverage=true`일 때만 `sorting_required_categories`의
모든 종류가 완료 항목에 포함됐는지 추가 검사한다.

`/sorting/verify_placement`는 독립된 결과 검증 port다. 현재는 시뮬레이션
oracle이 release 이후의 새 물체 AABB가 올바른 수거함 내부에서 안정적으로
정지했는지 확인한다. 이 oracle은 인식·분류·grasp·경로 계산에 사용하지
않는다. 실제 로봇에는 별도의 센서 기반 결과 검증 구현이 필요하다.

## 분류·운반 실행 계약

`study_cafe_sorting.launch.py`와 일반 파이프라인은 `robot_top_bins.yaml`의
내부 후면 받침판·수거함을 기본 배치로 사용한다. 책상 양끝 분류 영역과 구분선은
없으며 수거함의 고정 투하 위치만 사용한다.
개별 물체 배치 성공 기록이 있지만 전체 정리 성공은 검증 중이다.
`table_sorting_policy.yaml`의 destination으로 Gemini의 category/reason을
연결한다. label allowlist로 category를 대체하지 않으며 위험 label은 보수적으로
거부한다. RGB-D 거리순으로 시도하고 인식된 수거 구역 물체는 다시 집지 않는다.
미처리/미확인 물체가 남으면 성공으로 보고하지 않는다. 관측한 작업 물체 수와
완료 항목 수를 대조하므로 이전에 관측한 수량보다 완료 수가 적으면 중단한다.
이는 수량 검사이며 객체 identity 추적이나 관찰 모드의 독립 안착 검증을 대신하지 않는다. 빈 작업 영역은 두 번
재확인하며 최대 search/action 횟수 12에 도달하면 실패한다.

MuJoCo oracle은 배치 후 전체 AABB가 수거함 안에 들어가 정지했는지 검증한다.
기본 `sorting_exit_on_finish=true`; coordinator 종료 시 launch가 다른 노드도
종료한다. GUI를 유지하려면 launch 인수 `shutdown_on_sorting_exit:=false`를 사용한다.
현재 전체 물리 정리 성공은 아직 검증 중이다. 기록: `artifacts/table_sorting_20260908/`.
sorting은 얇고 긴 객체의 RGB-D 점군에서 중심 외의 longitudinal contact 후보도
생성한다. 자세/충돌 한계는 유지하며, 세부 범위는 cleany_grasping README를 따른다.
sorting selector/executor는 `support_patch_margin_m=0.02`로 perception OBB
바닥의 지지면을 유한 collision patch로 유지한다(일반 node 기본 0, 비활성).
patch는 OBB XY 범위에 각 방향 2cm를 더하고, OBB 바닥 아래 1cm 두께로 만든다.
2cm는 현재 target mask padding 1.5cm를 포함하는 로컬 범위다. 8cm 확장은
가까운 물체에서 로봇 본체까지 가상의 면을 확장하는 문제가 있어 사용하지 않는다.
이는 perception이 지지면 법선을 OBB Z축으로, 지지면을 바닥으로 쓰는 계약에
의존한다. 수평에서 20°를 넘는 면/잘못된 pose는 거부한다. 새 테이블 정답은
읽지 않으며, 관측 밖 연장 부분은 보수적인 로컬 평면 가정이다.
물체/support pair만 접촉 허용하고 robot/support는 허용하지 않는다. attachment
이후에도 patch가 남아 target OctoMap masking 아래의 지지면이 사라지지 않으며,
transaction 종료 시 target과 함께 제거하고 원래 ACM으로 복원한다.
`require_gripper_closure_clearance=true`는 실제 full-robot 형상으로 open→close
구간을 `gripper_sweep_step_rad=0.05` 이하 간격에서 검사한다. target 접촉만
허용하고 환경/지지면 충돌은 거부한다. 이 검사는 기하학적 샘플 검사이며 접촉
동역학/실제 파지 유지 성공을 보장하지 않는다. grasping의 support-plane 간이
검사 위임은 이 support patch 및 open/closure 검사와 함께만 사용한다.
seeded cubic 경로가 충돌로 거부되면 최대 8개 contact의 body pair, 위치/frame,
penetration depth를 오류에 기록해 주변 물체/지지면/잔상 원인 분석에 사용한다.
`sorting_head_reference_refresh_age_sec` 기본값은 `0.0`으로, pregrasp 도착 후
시간 경과만을 이유로 하는 Gemini 재인식을 비활성화한다. `(0, 30]`초로 설정하면
기존 ROS clock 기준 만료 검사와 재인식·위치 연관·동일 팔 재계획을 다시 활성화한다.
손목 서비스 자체의 관측 유효성 검사는 유지하며 관측 timestamp를 갱신하지 않는다.

파지 후 손목 CHECK의 단순 미검출(`ObserveWristTarget.NOT_DETECTED`)은 즉시
종료하지 않는다. `sorting_wrist_check_attempts`는 기본 1회이며, 미검출이어도
아래 그리퍼 확인을 통과하면 추가 카메라 요청 없이 운반으로 진행한다.
1~5회로 설정할 수 있으며, 첫 요청은 lift 완료 시각, 추가 요청은 직전 응답의
실제 촬영 시각보다 새로운 프레임을 요구한다. 실패 응답도 source/reference ID,
선택한 팔의 frame, 촬영 시각을 확인한 뒤에만 보완 경로로 처리한다.
HANDOFF는 성공해야 접근하며 CHECK의 센서·통신·TF·추론·참조 오류나 모호한
검출은 이 보완 경로에 포함되지 않는다.

각 CHECK 전후에는 요청 이후 도착한 그리퍼 위치·속도가 기존 접촉 조건을
연속 만족해야 한다. 최소 3개의 서로 다른 샘플이
`sorting_wrist_contact_confirmation_sec`(기본 0.3초)의 ROS 시간 구간을
채워야 하며, 피드백 나이와 샘플 간격은
`sorting_gripper_feedback_max_age_sec`(기본 0.5초) 이하여야 한다.
대기 상한은 `gripper_contact_feedback_timeout_sec`(기본 wall time 5초)다.
오래된 값·중복 시각·피드백 단절·빈손 완전 닫힘·접촉 상실은 성공 근거로 쓰지 않는다.
관절 메시지가 `/clock`보다 먼저 도착하면 시계가 따라올 때까지 그 값을 세지 않고
기다린다. 미래 시각이 계속되거나 시계가 멈추면 대기 상한에서 실패한다.
모든 새 영상에서 미검출이어도 이 모터 확인을 통과하면 운반으로 진행한다.
로그에는 시각적 파지 확인이 없고 그리퍼 피드백으로 진행했다는 사실을 남긴다.
물체 중심의 lift 높이는 기존처럼 FK와 파지 offset으로 계산하며, 별도 실제 높이
측정 또는 물체 식별 성공으로 표현하지 않는다. 전류·토크는 현재 판정에 사용하지 않는다.

고정 수거함 모드에서는 YOLOE-seg bbox/depth 거리로 정렬한 후보를 하나씩 처리한다.
`fixed_jaw_clearance_m`은 고정 손가락 여유거리(수거 기본 0.003 m),
`grasp_opening_margin_m`은 후보 벌림 여유폭(기본 0.008 m)이다. 여유거리는
여유폭의 절반 이하여야 한다. 5 mm 시험은 각각 `0.005`, `0.010`으로 설정한다.
벌림 여유폭은 후보 생성기와 두 TCP 보정 경로에 동일하게 전달된다.
현재 후보의 YOLOE-seg mask에만 3D 복원을 수행하고 파지 가능한 팔을 찾으면 탐색을 끝낸다.
복원·파지 계획·양팔 도달성 검사가 실패하면 다음 후보를 복원하며, 같은 주기의
복원 결과는 캐시한다. review 분류 후보는 정밀 복원 없이 미해결 대상으로 남긴다.
선택되지 않은 물체도 전체 depth 기반 OctoMap의 충돌 검사에는 계속 포함된다.
다음 후보 처리 전 snapshot TTL이 만료되면 기존 cache 오류로 처리되며 과거 영상의
시각을 갱신하지 않는다. 미해결 물체와 기존 관측 물체의 누락 검사는 유지한다.
sorting selector의 `pose_refinement_iterations=30`은 runtime `/robot_description`
URDF의 로컬 FK로 위치와 접근/closing 방향의 잔차를 함께 줄이는 bounded numerical
refinement를 사용한다. 매 후보 탐색 전 MoveIt FK와 일치하는지 대조한다.
초기 자세 예산은 pregrasp에서 `pregrasp_aim_attempts`, grasp에서 0이 아닌
`grasp_pose_seed_attempts`를 따른다(0이면 pregrasp 예산 재사용). 수치 보정도
설정 예산 전체를 사용하며, 첫 유효 해에서 종료한다. seed 수 증가는 실패
후보의 계획 시간을 늘릴 수 있으나 유효성 허용값을 변경하지 않는다.
기본 adapter 값 0은 기존 탐색이다. 각도 오차 기준, 관절 여유, 충돌검사 및
실행 전 경로 검증은 유지한다. 후보 보정 자체는 로봇에 명령을 보내지 않는다.
실제 grasp는 `grasp_position_tolerance_m=0.0015`로 별도 검사한다. pregrasp의
20mm 허용값을 grasp에도 공유하던 오류를 수정했으며, 관절 자세가 맞더라도
TCP 위치 오차가 1.5mm를 넘는 후보는 거부한다. `pose_refinement_position_weight=10`
으로 위치 잔차를 우선 줄인다. 각도/충돌/관절 한계는 완화하지 않는다.

`grasp_use_aperture_centering=true`는 검출 폭에서 opening margin 8mm를 제외한
물체 폭을 사용해 `폭/2 - fixed_jaw_inner_x(8mm)`로 TCP 옆방향 보정량을 정한다.
계획과 실행이 같은 함수를 사용한다. 기존 30mm 고정 보정은 작은 물체가 두
손가락 사이에서 벗어나는 문제가 있었다. 시뮬레이션 jaw mesh 기준으로 닫힘
근방 aperture 기울기 0.065m/rad, command opening reduction 18mm를 사용한다.
관절 정지로 파지를 확인하지 못하면 `gripper_contact_retry_steps`(기본 node 0,
sorting launch 5; 허용 0–5)만큼 `gripper_contact_retry_step_rad=0.10`씩 더 닫는다.
각 재시도는 `gripper_contact_retry_motion_sec=1.2`이며 닫힘 하한과 기존 토크 제한을
넘지 않는다. 최초 열린 위치 기준의 접촉 판정을 유지하고, 실제 마지막 명령을
운반 중 감시에도 사용한다. 한도 내에서 파지를 확인하지 못하면 lift 전에 실패한다.
이는 실제 하드웨어 캘리브레이션 값이 아니며 물리 파지 성공은 별도 검사한다.

연속 sorting에서는 `gripper_force_full_close=true`로 기존 닫힘 하한을 목표로
유지한다. 접촉 때문에 실제 관절은 그 전에 멈출 수 있으며, 기존 토크 제한과
관절 잔차/속도 기반 접촉 감시는 그대로 적용한다. 추정 폭의 중간 목표에서 멈춰
물체 자세 변화 후 추가로 조여주지 못하는 현상을 피하기 위한 설정이다.
목표까지 완전히 닫혀 잔차가 사라지면 파지 성공으로 간주하지 않는다.
초기 파지에는 정지 속도 조건을 적용한다. 이미 확인된 파지의 비동기 운반 중에는
닫히는 방향의 추가 조임을 허용하되, 목표 대비 최소 잔차/열리는 속도 상한,
관절 피드백 신선도, 손목 추적과 접촉 상실 debounce는 계속 검사한다.
이는 힘 센서 확인이 아니며 잔차가 사라지기 전의 미끄러짐을 완전히 판별하지 못한다.
sorting의 attachment 후 지도 갱신 대기 상한은 10초다. CPU 저속 시뮬레이션에서
새 지도 발행을 기다릴 시간을 확보하되, 신선도 2초와 attachment 이후 새 depth
capture 조건은 완화하지 않는다. 새 데이터가 준비되면 즉시 진행한다.
변화 없는 OctoMap 전체 메시지가 재발행되지 않아도, tree 쓰기 뒤 발행되는
processed-cloud receipt의 새로운 capture stamp로 지도 활동을 확인한다.
기존 populated-map 확인과 capture/receipt 신선도 기준은 유지하고, 원본 cloud
수신이나 중복 receipt만으로 갱신하지 않는다. 빈 지도도 통과시키지 않는다.
sorting selector는 `require_open_grasp_clearance=true`로 실제 열림 각도의 grasp
끝점도 검사한다. 센서 OBB에 대한 jaw 접촉 허용을 잠시 해제하고, 선택된 gripper
관절만 열림 각도로 바꾼 전체 로봇 MoveIt validity를 확인한다. 기존 피드백/계획은
변경하지 않고 ACM은 성공/실패/예외 모두 복구한다. 이 검사는 시작/중간 경로의
추가 안전 검증을 대체하지 않으며, 보수적인 OBB 때문에 유효한 후보도 거절할 수 있다.
고정 jaw에는 sorting에서 `grasp_fixed_jaw_clearance_m=0.003`을 적용해 경계를
맞닿게 두지 않는다. selector와 coordinator에 동일하게 적용하며, 설정 여유는
opening margin의 절반 이하여야 한다. 기존 비-sorting 기본값은 0이다.
`direct_vertical_lift:=true`는 기본 false인 동작 진단 옵션이다. 접근 경로를
역으로 물러나는 단계 대신 현재 TCP 위치에서 수직 lift를 계획한다.
`direct_vertical_lift_extra_m`(기본 0, 허용 범위 0~0.05m)은 이 진단 경로의
TCP 상승 거리에만 추가된다. 물체 중심의 기존 최소 상승 높이 검사와 경로
충돌 검사는 그대로 적용되며, 높이가 부족하면 예측/요구 중심 높이를 오류에 남긴다.
sorting에서는 끝점 position IK/FK를 확인하고 기존 seeded 위치 corridor로 계획한다.
기존 corridor 자세 한도(5°)를 넘는 끝점은 거절하고, 전체 경로의 FK/충돌/자세
변화를 검사한다. 정확한 자세 고정 LIN으로 표현하지 않는다. 파지/손목 감시는
유지하며 기본 전체 sorting 동작은 아직 역방향 retreat를 유지한다.

- [System Context](../../../docs/cleany-docs/20_TECHNICAL/01%20-%20System%20Context.md)
- [Task Planning and Robot Capabilities](../../../docs/cleany-docs/20_TECHNICAL/03%20-%20Task%20Planning%20and%20Robot%20Capabilities.md)
# 수거함 운반

## 분류된 수거함으로 직접 이동 / 손목 유지 (기본 모드)

현재 수거 시뮬레이션은 공유 `robot_top_bins.yaml`의
`simulation_ignore_mast_collision: true`로 고정 기둥 접촉을 MuJoCo에서 끄고
MoveIt의 기둥–로봇 링크 충돌을 제외한다. 외형 및 self-filter용 TF/형상은 유지한다.
이 편의 설정의 성공 결과는 실제 기둥을 피하는 실로봇 경로 검증이 아니다.

빈손 복귀와 그리퍼 닫기는 계속 동시에 실행하되,
`sorting_return_velocity_scaling: 0.12`, `sorting_return_acceleration_scaling: 0.15`로
복귀 동작만 별도 상한을 적용한다(더 낮은 전역 설정은 유지).
현 MJCF STS3215의 토크 한계 2.648Nm 및 감쇠 합 3.342Nm·s/rad에서는
무부하 정상 속도조차 약 0.78rad/s 이하라 기존 30% 어깨 yaw 상한 1.274rad/s를
지속 추종할 수 없다. 새 복귀 yaw 상한은 약 0.509rad/s이며 충돌 검사,
모터 토크 한계 및 0.12rad 궤적 추종 허용치는 변경하지 않는다.
이는 현재 시뮬레이터 모델용 설정이며 실제 모터 성능 인증값이 아니다.

운반은 고정 투하 방식이다. `config/nearest_pregrasp.yaml`의
고정 **물체 중심** 위치(base_link, m)는 다음과 같다. 수거함은 로봇 내부 후면에 부착된다.

- `sorting_fixed_release_lost_items_left_m`: `[-0.075, 0.105, 0.50]`
- `sorting_fixed_release_trash_right_m`: `[-0.075, -0.105, 0.50]`

분류 → 파지 → 안전한 들어 올리기 → 선택된 수거함 위 고정 투하점 → 놓기 → 복귀 순서다.
기본 모드에서는 파지 전 공통 경유점 IK 검사, 공동 손목 endpoint 검사 및 경유점 이동을
모두 생략한다. 그리퍼 안의 실제 추정 물체 중심 offset을 반영한 최종 투하 자세만 구한다.
도달 가능한 자세가 없으면 물체를 놓지 않고 중단한다. 직접 이동도 MoveIt의 현재
부착 물체/장애물 충돌 검사를 거친 경로로 실행하며 무검증 직선 명령이 아니다.

들어 올리기를 마치면 실제 손목 roll 각도를 캡처한다. 최종 투하 IK는 손목 roll만 변수에서
제외하고 어깨 yaw/pitch·팔꿈치·손목 pitch의 네 관절을 푼다. 손목을 위아래로 꺾는
pitch는 고정하지 않는다. 실행 경로의 roll tracking 허용
오차는 `sorting_fixed_release_wrist_tolerance_deg`(기본 0.5도)이며 자동 확대하지 않는다.
고정 투하점에서 물체 중심 도착과 손목 feedback을 확인한 뒤 그리퍼를 연다.
물체 중심 도착 허용 오차는 1cm이고, IK의 위치 허용 오차는 축별 0.5mm다.

검증된 최종 자세는 프로세스 내 캐시에 저장하며 동일 팔/손목 각도/물체 offset/투하점일 때
재사용한다. 캐시 hit도 현재 부착 물체를 포함한 충돌 및 FK를 다시 확인한다.
센서 feedback이 달라 정확히 같은 키가 아니면 재계산하므로 캐시 hit나 시간 단축을
항상 보장하지 않는다. 직접 운반 중에는 넓은 영역 탐색이나 손목 완화로 우회하지 않는다.
이 설정은 시뮬레이션용이며 실제 로봇의 고정 투하 자세가 검증됐다는 뜻은 아니다.

## 파지 후 후퇴/들어 올리기의 손목 제약

수거 중 손목 roll만 안정된 파지 접촉 시점의 실제 관절각을 기준으로 제한한다.
pitch는 위아래 동작을 위해 자유롭게 사용하되 기존 물리 관절 한계/충돌 검사는 유지한다.
`config/nearest_pregrasp.yaml`의 ROS parameter `sorting_carry_wrist_tolerances_deg`
기본값은 `[2.0, 5.0, 10.0, 20.0, 30.0]`이다. 파지 후 후퇴/수직 lift는 빈 팔의
기존 pregrasp 관절 목표를 재사용하지 않고, 현재 허용 범위에서 새 endpoint IK와
전체 Cartesian 경로를 계획한다. endpoint나 검증된 경로를 찾지 못할 때 다음 범위로
넓힌다.
기준각은 구간마다 바꾸지 않으며, 한 번 확대한 범위는 놓기까지 유지한다.
관절 한계/FK/충돌 검사는 유지한다. 최대 범위에서도 실패하면 중단한다.

MoveIt 이동에는 같은 관절 path constraint를 전달하고, 직접 생성하는 Cartesian
궤적은 local IK/refinement 경계부터 손목 제약을 반영하고 실행 전 모든 waypoint를
검사한다. 계획/검증과 실행을 분리해 계획 실패만 재시도하며, 서비스/URDF 오류나
제어기 실패/접촉 상실 후에는 제한을 완화해 재실행하지 않는다. 일반 MoveIt 운반
구간은 여전히 IK 성공 후 경로 계획 실패 시 중단한다.

손목 유지 시 어깨/팔꿈치 움직임으로 그리퍼의 세계좌표 방향은 변할 수 있다.
후퇴/lift의 새 도착 방향은 기존 빈 팔 방향을 강제하지 않고 구간 시작 방향 대비
`sorting_carry_cartesian_rotation_limit_deg`(기본 30도)로 제한한다. 이 값은 시뮬레이션
운용 설정이며 실제 로봇의 안전한 기울기 한계가 검증됐다는 뜻은 아니다. 새 방향을
잇는 전체 FK corridor의 기존 자세 오차/속도/가속도/충돌 검사는 유지한다.
그리퍼를 열고 scene attachment를 해제한 뒤에는 손목 제한을 해제해 복귀한다.
손목 변화 최소화의 전역 최적해나 물체의 수평 자세 유지는 보장하지 않는다.

운반 전과 놓기 직전의 수거함 입구 크기·bounding sphere 적합성 검사는 현재 구현되어
있지 않으며 추후 개발 예정이다 (코드의 `TODO(bin-opening-fit)`).
분류된 수거함의 설정된 고정 투하점으로 이동하고, 물체 중심·손목 feedback으로
도착을 확인한 뒤 놓는다. 따라서 큰 물체나 OBB가 과대 추정된 물체가 입구 벽이나
모서리에 걸칠 수 있는지는 현재 확인하지 않는다.

SAM 분할·기준 mask 추적·중앙 인계 상태는 제거했다. 현재 손목 검증은 YOLOE-seg의
HANDOFF/CHECK 재검출이며 이동 전후 접촉·충돌 검사는 유지한다.

예전 라벨 allowlist 정책, 테이블 배치 구역, 경유점/영역 탐색 운반은 제거했다.
