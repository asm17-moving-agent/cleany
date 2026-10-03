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

위 초기 통합 검증에서는 시설 전체 Gazebo 주행과 실물 로봇 검증을 수행하지 않았다.
synthetic backend, mock ROS 및 정적 world 생성 결과는 하드웨어 주행 완료 근거가 아니다.

## GUI 통합 및 자기 관측 필터 수정 (2026-10-03)

Control-plane `feat/robot-gateway` (`0919d38`)를 gateway 모드로 실행하고 운영/개발
대시보드, 전체 시설 Gazebo, 지도/voxel RViz, rqt 그래프/RGB 뷰어를 열었다.
검증용 정적 지도는 CAD collision geometry를 5cm로 rasterize했으며 SLAM 결과가 아니다.
Navigation=sim, perception/planning/execution=mock이다.

첫 44번 좌석 미션은 도착 후 복귀 중 실패했다. 당시 깊이 센서 누적 voxel 두 개가
로봇 footprint 내부에 남았고 하나는 상단 왼쪽 레일의 CAD 영역과 겹쳤다.
현재 sensor hit의 끝점만 제거하던 self-filter를 sensor→hit 선분과 고정 몸체 OBB의
교차 검사로 바꿨다. 몸체를 통과해 뒤로 튀는 hit와 clearing ray를 함께 버리고,
가리지 않은 외부 장애물 및 기존 안전 게이트를 유지한다.

- 관련 depth core/ROS transport: 17 passed. Navigation package: 87 tests, 0 failures.
- description/navigation build: 2 packages 성공.
- 별도 domain 224에서 새 누적 지도로 재검증. 이전 오염 지도/DB는 보존했다.
  부분 재시작 때 DDS discovery/시계/서비스 연결 문제가 발생해 이번 navigation 프로세스에만
  UDP transport XML을 적용했다. 시스템 DDS 설정은 바꾸지 않았다.
- 미션 `a7f360dd-2b33-40b6-b76f-500dfd398a20`: 대시보드 요청 → 좌석 도착 → 모의 청소
  → 홈 복귀 → `SUCCESS`. `navigation_result=OK`, `return_result=OK`.
  Gateway `mission.result` 및 fresh IDLE, 최종 /cmd_vel과 ground-truth 속도 0 확인.
- 반복 미션 `d731fb97-8be6-4831-b150-81c04cf6bcd4`도 14:13:55 UTC에 `SUCCESS`.
  좌석 도착/모의 청소/홈 복귀가 두 번 연속 성공했고 이후 fresh IDLE을 확인했다.
- 복귀 구간 누적 voxel의 footprint 내부 점: 0. 이전 오염 두 voxel의 depth evidence는
  각각 양수 4/3에서 새 지도에서 -3/-3으로 관측됐다.

증거는 `artifacts/gui-integration-20261003/`의 `fixed-*.json`,
`self-fixed-observations.json`, `ros-fixed-terminal.json`, `navigation-udp.log`,
`*-fixed-evidence.db` 및 화면 PNG에 보관한다. 물리 로봇/실제 청소 검증은 수행하지 않았다.
