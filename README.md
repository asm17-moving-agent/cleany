# 끌리니 (Cleany)

무인 스터디카페의 지정 구역에서 쓰레기와 분실물 후보를 분류·수거하는 로봇 엣지 시스템 구현 저장소다.

AI·SW마에스트로 제17기 | 팀명: AI 에이전트는 움직이고 싶어

제품의 1차 타깃은 무인 스터디카페이며 MVP 시연 환경은 개발센터 개발공간이다.
제품 범위와 채택된 설계는 [기획 KB](docs/cleany-docs/README.md)를 따른다.
이 저장소의 시뮬레이션 설정은 실물 하드웨어 사양이나 안전 기준의 확정을 뜻하지 않는다.

## 현재 구현

| 영역 | 구현 상태와 진입점 |
| --- | --- |
| Mission lifecycle | [Mission Manager](ros2_ws/src/cleany_mission_manager/README.md)의 순수 Python FSM·port·단위 테스트. 외부 mission ROS API는 아직 없음 |
| RGB-D 인식 | [Perception](ros2_ws/src/cleany_perception/README.md)의 검출·선택 객체 분할·3D 복원·SAM2 추적. Gemini/SAM2 기본 프로필과 YOLOE 선택 프로필 |
| 파지·조작 | [Grasping](ros2_ws/src/cleany_grasping/README.md)의 기하/AnyGrasp 후보와 [Skill Executor](ros2_ws/src/cleany_skill_executor/README.md)의 IK·pregrasp·접촉 기반 집기·분류·놓기 |
| 계획·충돌 지도 | [MoveIt 설정](ros2_ws/src/cleany_moveit_config/README.md), [선택형 OctoMap updater](ros2_ws/src/cleany_scene_mapping/README.md) |
| 로봇 모델·시뮬레이션 | [URDF/MJCF](ros2_ws/src/cleany_description/README.md), [MuJoCo](ros2_ws/src/cleany_mujoco_sim/README.md), [관측·카메라 스케줄러](ros2_ws/src/cleany_mujoco_observer/README.md) |
| 주행·보정 | [Gazebo](ros2_ws/src/cleany_gazebo_sim/README.md)의 Mecanum·센서·SLAM 시험, [Navigation](ros2_ws/src/cleany_navigation/README.md) 설정, [Hand-eye calibration](ros2_ws/src/cleany_handeye_calibration/README.md) |
| Jetson·모터 | [Vision container](containers/vision/README.md)의 분리된 perception/AnyGrasp 실행 환경, [Motor controller](motor_controller/README.md)의 펌웨어·회로·하드웨어 시험 자료 |
| 미구현 경계 | `cleany_planner`, `cleany_robot_interface`, `cleany_logger`는 README scaffold. Dashboard/Backend부터 실물 수거까지의 통합 시스템은 아직 아님 |

현재 분류 시뮬레이션의 흐름은 다음과 같다. Mission Manager의 FSM과 이 데모의
coordinator는 별도 구현이며 end-to-end로 연결된 상태는 아니다.

```text
RGB-D → 검출·분류 → 선택 객체 분할·3D 복원 → 파지 후보
                                                ↓
                     MoveIt IK·충돌 검사 → 접근·파지·들기
                                                ↓
                           수거함 운반·놓기 → 복귀·재탐색
```

개별 물체의 배치 성공 기록은 있으나 연속 전체 정리 성공과 실물 운용은 검증 중이다.
[실행 기록](docs/SORTING_PIPELINE_TRIALS_20260909.md)과
[저장소 품질 점검](docs/QUALITY_REVIEW_20260914.md)에서 검증 범위와 잔여 문제를 확인한다.

## 개발과 검증

기준 환경은 Ubuntu 22.04 VM, native ROS 2 Humble, Python 3.10이다.
새 환경은 [개발환경 설치 가이드](docs/DEVELOPMENT_SETUP.md)를 먼저 따른다.
레포지토리 루트에서 실행한다.

```bash
git submodule update --init --recursive docs/cleany-docs
make deps
make build
make test
```

반복 작업에는 관련 범위의 테스트를 사용한다.

```bash
make test-grasp-pregrasp          # 인식·파지·분류·모델·충돌 지도 집중 검사
make test-grasp-pregrasp-runtime  # MoveIt / MuJoCo 실제 controller 실행
make test-mujoco                 # MuJoCo 패키지 전체 테스트
```

전체 명령과 native 실행 방법은 [ROS 2 workspace 안내](ros2_ws/README.md) 및
`make help`를 따른다. Jetson CUDA·모델·라이선스 인수검사는
[Vision container 안내](containers/vision/README.md)에서 별도로 관리한다.

## 시뮬레이션 실행

```bash
make sim                        # 기본 MuJoCo headless 실행
make sim-mujoco-study-cafe       # 스터디카페 viewer
make sim-mujoco-pipeline         # 인식·계획, 기본 plan-only
make sim-mujoco-sorting          # 시뮬레이션 집기·운반·놓기
```

인식 파이프라인은 로컬 SAM2 모델과 `GEMINI_API_KEY`가 필요하며 RGB 영상을
Google API로 전송한다. 기본 pipeline은 외부 RGB-D 입력을 기다린다.
시뮬레이터를 함께 구동하는 통합 데모는 sorting target이다.

Sorting 기본값은 작업자가 GUI로 안착을 확인하는 `sorting_verify_placement:=false`다.
완료 단계 `mission_complete_unverified`는 자동 배치 검증 성공을 뜻하지 않는다.
독립 MuJoCo 배치 검증을 포함하려면 다음처럼 실행한다.

```bash
make sim-mujoco-sorting SORTING_ARGS='sorting_verify_placement:=true'
```

수거함은 로봇 내부 후면 받침판 위 좌측 분실물함·우측 쓰레기함이다.
현재 시뮬레이션 설정은 고정 기둥의 물리 접촉과 MoveIt 로봇 링크 간 충돌을 제외한다.
이 제한과 선택형 성능 프로필은 [MuJoCo README](ros2_ws/src/cleany_mujoco_sim/README.md)를 따른다.

## 저장소와 문서

| 경로 | 내용 |
| --- | --- |
| `ros2_ws/src/` | 13개 ROS 패키지와 3개 설계 scaffold |
| `configs/` | 공통 mission·robot 설정 영역 |
| `motor_controller/` | MCU 펌웨어와 회로·하드웨어 시험 |
| `containers/vision/` | Jetson vision 실행 환경 |
| `tools/` | 개발·SLAM 평가 도구 |
| `tests/` | 테스트 진입 안내; 실행 가능한 검사는 패키지별 `test/`·`tests/`에 위치 |
| `docs/cleany-docs/` | 제품·기획·예비설계 KB submodule |
| `docs/` | 개발환경, 날짜별 구현 실행·품질 점검 기록 |
| `artifacts/`, `output/` | 로컬 실행 증거·발표 원본 및 출력물. Git 제외 |

패키지 책임·ROS 인터페이스·설정·실행법은 해당 패키지 README를 코드와 함께 갱신한다.
공통 작업 규칙은 [AGENTS.md](AGENTS.md), 기획 판단은 KB의 결정 및 질문 문서가 관리한다.
KB submodule은 명시적인 KB 수정 요청이 있을 때만 편집한다.
