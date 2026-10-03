# Runtime 통합 검증 (2026-10-03)

`feat/runtime-integration`은 `origin/main`의 `c999500`에서 시작해 다음
브랜치를 순서대로 `--no-ff` merge했다. 원본 브랜치와 기존 워크트리의
미커밋 변경은 통합 대상에 포함하지 않았다.

| 브랜치 | 통합한 tip | merge commit |
| --- | --- | --- |
| `feat/gazebo-full-facility` | `ff02f42` | `1bfc73a` |
| `feat/navigation-runtime` | `161a77f` | `c378fcb` |
| `feat/mission-runtime` | `9819ada` | `25f2503` |
| `feat/dev-monitor` | `1451ba3` | `c7975c8` |

## 충돌 및 호환성 처리

- 최신 CAD 변환, odometry, 센서 및 navigation 구현 위에 시설/ROLY 생성 기능을 연결했다.
- 시설과 카페는 공통 launch helper를 사용한다. 기존 시설 helper import는 호환용으로 남겼다.
- 시설에도 wheel odometry, noise/profile, headless rendering 및 Fortress 서버 옵션을 전달한다.
- 시설 저장 시 DART가 사용하는 `ignition:expressed_in` 접두사를 보존한다.
- 시설 경로 YAML에 현재 `RouteLimits`가 요구하는 가속도 및 방향 오차를 명시했다.
- 미션 실행/안전 게이트와 모니터의 action result 관측을 함께 유지했다.
- Makefile, CI 및 README의 각 브랜치 기능을 보존했다. 이전 GUI 방식과 의자 좌표를
  가정하던 평가 테스트는 현재 서버/GUI 분리와 의자 10 cm 이동에 맞췄다.

## 검증

Humble Distrobox에서 실행했다. 로그는 이 워크트리의
`artifacts/integration/`에 보관하며 Git에는 추가하지 않는다.

- 전체 workspace build: 17 packages 성공 (`make test`의 build 단계).
- hand-eye 제외 workspace 검증: 16 packages, 359 tests, 0 errors/failures,
  11 opt-in skipped. 추가 Gazebo profile 도구 테스트 7 passed.
- 미션/브리지 core: 60 passed, 6 opt-in skipped (`make test-mission-core`).
- 모니터 core/camera: 8 passed, 2 opt-in skipped (`make test-dev-monitor`).
- ROS 미션 transport 및 모니터 QoS/camera: 5 passed.
- 공통 navigation synthetic backend runtime: 2 passed.
- Gazebo 별도 evaluation suite: 50 passed.
- 모니터 웹 TypeScript/Vite build 성공.
- Playwright UI: 4 passed, 2 live ROS/Gazebo UI tests skipped.
- 실제 시설 launch helper로 CAD/ROLY world 및 좌석 manifest를 생성하고 spawn override,
  마찰 namespace와 최신 backend 인자 전달을 확인했다. simulator는 실행하지 않았다.

ROS opt-in 검증은 별도 domain을 사용했다. 컨테이너의 Git 2.34는 호스트 저장소의
`relativeworktrees` 확장을 읽지 못하므로 provenance 검사에는 일시적인
`distrobox-host-exec git` wrapper를 사용했다. 저장소/Git 시스템 설정은 변경하지 않았다.
Playwright는 설치된 Chromium headless shell 경로를 환경변수로 지정했다.

전체 `make test`는 최종 실행에서 631 tests, 0 errors, 1 failure, 11 skipped였다.
남은 실패는 `cleany_handeye_calibration`의
`test_single_pose_launch_records_feedback_timed_mujoco_sample`에서 ChArUco
corner 검출 부족이다. 이전 실행에서는 MoveIt trajectory가 실패했고 해당 항목의
분리 실행은 통과했다. hand-eye 구현이나 성공 기준은 이번 통합에서 변경하지 않았다.

시설 전체 Gazebo 주행, 통합 미션의 실제 시설 Nav2 이동 및 실물 로봇 검증은
수행하지 않았다. synthetic backend, mock ROS 및 정적 world 생성 결과를
실제 시설/하드웨어의 주행 완료 근거로 해석하지 않는다.
