# 끌리니 (Cleany)

**무인 공간 관리 로봇** — 무인 스터디카페·독서실·공유오피스에서 퇴실 후 정리 업무를 자동화하는 이동형 로봇.

AI·SW마에스트로 제17기 | 팀명: AI 에이전트는 움직이고 싶어

---

## 개요

무인 공간 대여 시설은 빠르게 늘고 있지만, 이용자가 공간을 정리하지 않고 떠나는 경우가 많아 상주 인력 없이는 공간 품질 유지가 어렵다.

끌리니는 퇴실 후 자동으로 지정 공간으로 이동해 다음 작업을 수행한다.

- 쓰레기 탐지 및 수거
- 분실물 후보 식별 및 보관
- 의자·집기 정리
- 테이블 닦기
- 소등 및 문단속

## 주요 기능

| 기능 | 설명 |
|---|---|
| 자율주행 | SLAM 기반 실내 지도 생성, ROS 2 Nav2로 목표 구역 이동 및 장애물 회피 |
| 쓰레기 수거 | RGB-D 카메라로 쓰레기 후보 탐지 → 로봇팔로 수거함 이동 |
| 공간 정리 | 밀기·정렬·이동 동작으로 집기 위치 복구 |
| 분실물 처리 | 개인 소지품 후보 식별 후 보관함 이동 |

## 시스템 구조

```
Perception Layer   →   객체 탐지 (카메라 / RGB-D / LiDAR)
       ↓
Planning Layer     →   작업 계획 (RuleBasedPlanner → VLMPlanner)
       ↓
Execution Layer    →   로봇 제어 (Nav2 / MoveIt / LeRobot)
```

FSM: `IDLE → NAVIGATE_TO_TARGET → PERCEIVE → PLAN_TASKS → EXECUTE_TASKS → RETURN_HOME → REPORT` (any state → `ERROR`)

## 현재 구현·배포 검증 기준

아래 조합은 현재 구현과 배포 검증에 사용하는 baseline이다. 제품 하드웨어
Decision 문서는 `docs/cleany-docs`에서 사람 검토 전까지 draft로 유지한다.

| 구분 | 내용 |
|---|---|
| 로봇 | XLeRobot (모바일 매니퓰레이터) |
| 컴퓨팅 | NVIDIA Jetson Orin NX |
| OS | Ubuntu 22.04 (JetPack 6.2.2 / Jetson Linux 36.5) |
| 미들웨어 | ROS 2 Humble |
| 시뮬레이션 | Isaac Sim, MuJoCo |
| 언어 | C++ (ROS 2), Python (PyTorch, OpenCV, TensorRT) |
| 센서 | Camera/RGB-D, 2D LiDAR, IMU |
| AI | 경량 Object Detection, 경량 VLM, Agentic VLA, Sim-to-Real RL |

## 서브시스템

| 태그 | 담당 |
|---|---|
| **EDG** | 로봇 엣지 시스템 — Vision, Mission Manager, Skill Executor |
| **HW** | 하드웨어 플랫폼 — XLeRobot 조립, Jetson 셋업, 센서 |
| **SIM** | 시뮬레이션·학습 — Isaac Sim, MuJoCo, RL/IL |
| **BE** | 백엔드 서버 |
| **FE** | 관리자 대시보드 |
| **INF** | 공통·인프라 |

## 팀

| 이름 | 역할 |
|---|---|
| 이동근 | PM, SLAM/Nav2 자율주행, Isaac Sim 환경, RL/IL 정책 학습, 백엔드/대시보드 |
| 박창수 | HW/시스템 통합 리드, XLeRobot 셋업, Jetson/ROS 2 환경 구축 |
| 이정현 | Vision 인식, 쓰레기/소지품 구분, VLA 파인튜닝, 온디바이스 최적화 |

**멘토**: 김윤래 (시스템 아키텍처/로보틱스), 박준용 (자율주행/HW), 신승렬 (Sim-to-Real/RL/경량 VLM)

## 개발 로드맵

- **Phase 0** (로봇 도착 전): Mission Manager FSM, RuleBasedPlanner, Mock 환경 구축
- **Phase 1** (로봇 도착 직후): 하드웨어 검수, 센서 확인, ROS 2 토픽 연결
- **Phase 2** (1차 통합): 고정 위치 pick-and-place, 단일 시나리오 시연
- **Phase 3** (고도화): Nav2 자율주행, VLMPlanner 도입, Isaac Sim 검증, 대시보드

추진 일정: 2026년 5월 ~ 11월

Jetson Orin NX 배포와 TensorRT 변환 절차는
[`docs/deployment/jetson-orin-nx.md`](docs/deployment/jetson-orin-nx.md)를 따른다.
Windows, macOS, Linux 협업 지원 범위는
[`docs/development/platform-support.md`](docs/development/platform-support.md)를
따른다.

## 레포지토리 구조

```
cleany/
├── ros2_ws/
│   └── src/
│       ├── cleany_mission_manager/     # Mission Manager FSM / mission lifecycle
│       ├── cleany_perception/          # Vision/perception node, detection result publisher
│       └── cleany_mujoco_sim/          # MuJoCo 시뮬레이션 (XLeRobot)
├── docker/jetson/                      # Orin NX 배포 이미지
├── requirements/                       # 개발/Jetson dependency lock
├── scripts/                            # 개발·ROS·Jetson 실행 wrapper
└── docs/deployment/                    # 배포 절차
```

`cleany_interfaces`, Planner, Skill Executor, Robot Interface 등은 아키텍처상
예정된 패키지이며 실제 `package.xml`이 추가되기 전에는 build 대상으로
간주하지 않는다.

## 초기 개발 범위

초기 MVP는 실제 로봇 없이도 검증 가능한 end-to-end 흐름을 우선한다.

1. `cleany_interfaces`에서 perception result, mission command, skill action 인터페이스 초안 작성
2. `cleany_mission_manager`에서 `IDLE → NAVIGATE_TO_TARGET → PERCEIVE → PLAN_TASKS → EXECUTE_TASKS → RETURN_HOME → REPORT` FSM 구현
3. `cleany_planner`에서 `Planner` 인터페이스와 `RuleBasedPlanner` 구현
4. `cleany_robot_interface`에서 Mock / Sim / Real 공통 인터페이스 정의
5. `cleany_skill_executor`에서 mock skill success/failure 실행
6. `cleany_logger`에서 구조화된 실패 원인 코드 기록
