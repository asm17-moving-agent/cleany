# 끌리니 (Cleany)

**무인 스터디카페 관리 로봇** — 운영자 요청에 따라 지정 구역의 쓰레기와 분실물 후보를 처리하는 이동형 로봇.

AI·SW마에스트로 제17기 | 팀명: AI 에이전트는 움직이고 싶어

---

## 개요

무인 공간 대여 시설은 빠르게 늘고 있지만, 이용자가 공간을 정리하지 않고 떠나는 경우가 많아 상주 인력 없이는 공간 품질 유지가 어렵다.

제품의 1차 타깃은 무인 스터디카페이며, 현재 MVP 시연 환경은 개발센터 개발공간으로 한정한다.
MVP에서는 운영자·대시보드의 요청을 받아 지정 구역으로 이동해 다음 작업을 수행한다.

- 사전에 정한 쓰레기와 분실물 후보의 분류
- 쓰레기 수거함으로의 쓰레기 수거
- 쓰레기 수거함과 분리된 보관함으로의 분실물 후보 보관
- 위험하거나 불확실한 물체의 건너뛰기와 사람 검토 요청
- 작업 진행 상태와 전후 결과의 대시보드 표시

분실물 분류 기준, 보관 기간, 운영자 인계 절차는 아직 정해지지 않았다.

## 문서 관리 원칙

이 README는 프로젝트의 목적과 전체 구조를 소개하는 진입점이다. 구현 범위나
기술 전제처럼 검토가 필요한 내용은 여기에서 확정하지 않고, 아래 기준 문서를
따른다.

| 확인하려는 내용 | 기준 위치 | 갱신 시점 |
|---|---|---|
| 제품 범위, 현재 상태, 미해결 질문, 주요 결정 | [기획 KB README](docs/cleany-docs/README.md) | 기획 또는 의사결정이 바뀔 때 |
| Ubuntu·ROS·Python 개발환경 설치 | [개발환경 설치 가이드](docs/DEVELOPMENT_SETUP.md) | 지원 환경 또는 설치 절차가 바뀔 때 |
| 패키지 책임, ROS 인터페이스, 설정, 실행·검증 방법 | 각 ROS 2 패키지의 `README.md` | 해당 코드 또는 인터페이스를 바꿀 때 |
| 공통 개발 규칙과 문서 수정 규칙 | [AGENTS.md](AGENTS.md) | 작업 전 확인 |

MVP 범위, 하드웨어·런타임 조합, 안전 기준, 대시보드 포함 여부처럼 아직 검토 중인
항목은 이 README의 설명만으로 확정하지 않는다. 구현 또는 설계 변경 전에는 KB의
README를 먼저 읽고, 그 안내에 따라 현재 상태와 미해결 질문을 확인한다.

## 후보 주요 기능

아래 기능은 프로젝트가 지향하는 범위다. 실제 MVP에 포함되는 기능과 우선순위는
[기획 KB README](docs/cleany-docs/README.md)의 안내를 기준으로 확인한다.

| 기능 | 설명 |
|---|---|
| 작업 요청·상태 표시 | Dashboard와 Backend가 요청, Mission Queue, 진행 상태와 결과를 관리 |
| 자율주행 | ROS 2 Nav2로 지정 구역 이동 및 대기 위치 복귀 |
| 쓰레기 수거 | 사전에 정한 쓰레기 후보를 인식해 지정 수거함으로 이동 |
| 분실물 처리 | 분실물 후보를 별도 보관함으로 옮기고, 기준이 불명확하거나 위험하면 사람 검토 요청 |

## 예비 시스템 구조

```
Dashboard / Backend → Mission Queue → Mission Manager
                                      ↓
Perception → WorldState → Agentic VLA → Rule Guard → TaskPlan
                                      ↓
Physical Skill Executor → navigate / pick / collect / store
                                      ↓
                    Sim / Real Robot
```

FSM: `IDLE → NAVIGATE_TO_TARGET → PERCEIVE → PLAN_TASKS → EXECUTE_TASKS → RETURN_HOME → REPORT` (any state → `ERROR`)

## 검토 중인 기술 구성

아래 구성은 예비설계 기준이며, 실제 기준 플랫폼과 런타임 호환성은 아직 검토 중이다.
확정 상태는 [기획 KB](docs/cleany-docs/README.md)를 따른다.

| 구분 | 내용 |
|---|---|
| 로봇 | XLeRobot 상부 모듈(듀얼 매니퓰레이터·깊이 카메라) + custom 4륜 Mecanum base |
| 컴퓨팅 | NVIDIA Jetson Orin NX 16GB |
| OS | Ubuntu 22.04 기반 JetPack 6.2 |
| 미들웨어 | ROS 2 Humble |
| 시뮬레이션 | Isaac Sim, MuJoCo, Gazebo |
| 언어 | C++ (ROS 2), Python (PyTorch, OpenCV, TensorRT) |
| 센서 | Camera/RGB-D, 2D LiDAR, IMU |
| AI | Object Detection, Agentic VLA, Rule Guard |

## 검토 중인 서브시스템

| 태그 | 담당 |
|---|---|
| **EDG** | 로봇 엣지 시스템 — Vision, Mission Manager, Skill Executor |
| **HW** | 하드웨어 플랫폼 — XLeRobot 조립, Jetson 셋업, 센서 |
| **SIM** | 시뮬레이션·학습 — Isaac Sim, MuJoCo, Gazebo, RL/IL |
| **BE** | Mission Queue, 상태·결과 관리 |
| **FE** | 운영자 요청, 진행 상태와 전후 결과 표시 |
| **INF** | 공통·인프라 |

## 레포지토리 구조

```
cleany/
├── Makefile                            # native 빌드·테스트 작업 진입점
├── tools/                              # 개발 보조 도구
├── ros2_ws/
│   └── src/
│       ├── cleany_interfaces/          # ROS 2 msg/srv/action 공통 정의
│       ├── cleany_mission_manager/     # Mission Manager FSM / mission lifecycle
│       ├── cleany_planner/             # Planner interface, RuleBasedPlanner, VLMPlanner adapter
│       ├── cleany_perception/          # Vision/perception node, detection result publisher
│       ├── cleany_skill_executor/      # navigate/pick/place/push skill 실행
│       ├── cleany_robot_interface/     # Mock / Sim / Real 공통 로봇 인터페이스
│       ├── cleany_logger/              # event log, failure code logging
│       ├── cleany_mujoco_sim/          # MuJoCo 시뮬레이션 (XLeRobot)
│       └── cleany_gazebo_sim/          # Gazebo 시뮬레이션 (mobile base)
├── configs/
│   ├── mission/                        # mission, FSM, planner 설정
│   └── robot/                          # robot, sensor, frame 설정
└── tests/
    └── integration/                    # end-to-end 통합 테스트
```

## 초기 개발 범위 (초안)

초기 MVP는 Dashboard 요청부터 전후 결과 표시까지의 end-to-end 흐름을 우선 구현한다.

1. Dashboard와 Backend에서 대상 구역 요청을 만들고 Mission Queue로 전달
2. `cleany_mission_manager`에서 `IDLE → NAVIGATE_TO_TARGET → PERCEIVE → PLAN_TASKS → EXECUTE_TASKS → RETURN_HOME → REPORT` FSM 구현
3. Perception, Agentic VLA, Rule Guard가 쓰레기·분실물 후보·사람 검토 task를 구분
4. `cleany_skill_executor`에서 `navigate`, `pick`, `collect`, `store` skill을 실행
5. `cleany_robot_interface`에서 Mock / Sim / Real 공통 인터페이스 정의
6. Backend와 Dashboard에 Mission feedback, MissionReport, 전후 결과를 전달
