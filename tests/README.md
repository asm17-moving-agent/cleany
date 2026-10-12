# tests

패키지 경계를 넘는 통합·E2E 검사, 시뮬레이션 평가와 실행 보조 모듈,
개발 도구의 단위 테스트를 둔다. 생성·환경 관리 도구는 [`tools/`](../tools/README.md)에 둔다.

ROS 2 패키지 내부 단위 테스트는 각 패키지의 `test/` 또는 `tests/`에 둔다.

| 폴더 | 역할 |
|---|---|
| `facility/` | 시설 시뮬레이션 검증 |
| `ros2/` | 빌드 출력 정리 도구 단위 테스트 |
| `dev_monitor/` | 개발 모니터 ROS·Gazebo smoke 검사 |
| `mission_runtime/` | Backend API mission lifecycle 검사 |
| `navigation_evaluation/` | 주행 회귀·안전성 평가, 관찰·보고와 실행 보조 모듈 |
| `scenario_validation/` | 시나리오 준비·실행·자극·판정과 단위 테스트 |
| `integration/` | 통합 테스트 안내 |

저장소 루트에서 도구 단위 테스트는 `make test-tools`, 시나리오 core 검사는
`make test-scenario-tools`를 사용한다. 두 명령은 로봇이나 Gazebo를 구동하지 않는다.
시설 목표 생성 로직의 단위 테스트는 Gazebo 패키지의 `test/test_mission_targets.py`에 있으며
`make test-gazebo-facility-tools`로 실행한다. micro-ROS 준비 테스트는 `esp32/host_tests/`에 있으며 `make test-micro-ros-setup`으로 실행한다.

런타임 스크립트는 pytest로 자동 실행되지 않는다. ROS 환경과 필요한 build overlay를
준비한 뒤 명시적으로 실행한다. 시나리오는
[`scenario_validation/README.md`](scenario_validation/README.md), 시설·주행 검증은
[`cleany_gazebo_sim/README.md`](../ros2_ws/src/cleany_gazebo_sim/README.md),
모니터 smoke는 [`cleany_dev_monitor/README.md`](../ros2_ws/src/cleany_dev_monitor/README.md)를 따른다.
