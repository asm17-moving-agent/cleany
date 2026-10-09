# ROS 2 Workspace

Cleany의 로봇 엣지 시스템 코드가 들어가는 ROS 2 workspace다.

패키지는 `src/` 아래에 두고, 공통 인터페이스는 `cleany_interfaces`에서 먼저 정의한다.

## 개발 환경

공식 개발환경은 Ubuntu 22.04 VM과 ROS 2 Humble native 환경이다. ROS 2 Humble이
설치되어 있고 `/opt/ros/humble/setup.bash`를 사용할 수 있어야 한다.

새 VM을 준비하는 전체 절차는
[개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md)를 따른다.

MuJoCo용 custom rosdep 규칙을 처음 등록하는 방법은
[rosdep 규칙 안내](rosdep/README.md)를 따른다.

## 외부 데이터와 빌드 경로

Make target의 기본 출력은 소스 worktree 옆 `../cleany-data`다.
`CLEANY_DATA_ROOT`로 데이터 위치, `CLEANY_BUILD_ROOT`로 빌드 위치를 바꿀 수 있다.
기본 빌드 루트는 `$CLEANY_DATA_ROOT/builds/<worktree-directory>`이며, 서로 다른
worktree의 overlay를 공유하지 않는다. container에서도 같은 절대 경로가 보여야 한다.
일반 build/install/log와 navigation/scenario/Harmonic 전용 디렉터리가 이 아래에 생긴다.

저장소 루트에서 `source "$(make -s print-install-base)/setup.bash"`로 설치를 읽는다.
navigation/scenario는 각각 `print-navigation-install-base`, `print-scenario-install-base`를
사용한다. 사용자 지정 Make 변수는 빌드와 경로 출력에 동일하게 전달한다.
`make clean`은 이 외부 빌드 루트의 표준 출력만 지운다. 실행 원본·fixture·보고서는 유지한다.
경로를 바꿀 때 기존 colcon install을 이동하지 말고 새 경로에서 다시 빌드한다.

[데이터 보관 및 검증 색인](../docs/validation/README.md)에 과거 자료와 활성 입력 위치를 정리했다.

## 빠른 시작

아래 명령은 레포지토리 루트에서 실행한다.

```bash
make deps
make build
make test
```

변경한 영역부터 확인할 때는 타깃 테스트를 사용한다.

```bash
make test-mission
make test-mujoco
make test-handeye
make test-gazebo
```

Gazebo와 mission 반복 검증을 다른 호스트에서 독립 설치할 때는
`make build-scenario-runtime`을 사용한다. Gazebo, navigation, mission manager,
control bridge와 wheel odometry를 `build-scenario-runtime`/`install-scenario-runtime`/
`log-scenario-runtime`에 빌드한다. `SCENARIO_BUILD_BASE`, `SCENARIO_INSTALL_BASE`,
`SCENARIO_LOG_BASE`로 output을 바꿀 수 있다. 실제 실행과 호스트별 분리는
[시나리오 도구](../tools/scenario_validation/README.md)를 따른다.

Telemetry-only checks use `make build-telemetry` and `make test-telemetry`.
The `cleany_telemetry` package relays latest-only finite `/odom` `x`/`y`
and optional yaw over the configured WebSocket; see its README for parameters
and endpoint.

Mission FSM·BT와 Backend 연동만 검사할 때는 다음을 사용한다.

```bash
make test-mission-core
make test-mission-runtime
make build-mission-sim
source "$(make -s print-install-base)/setup.bash"
ros2 launch cleany_bringup mission_sim.launch.py post_mission:=wait_for_next
```

`test-mission-core`는 ROS 없이 실행하며 `py_trees`, PyYAML, pytest가 필요하다.
`test-mission-runtime`은 interfaces, manager, bridge를 빌드하고 colcon test를 실행한다.
실제 ROS transport 검사는 `CLEANY_RUN_ROS_TESTS=1`과 분리한 `ROS_DOMAIN_ID`를 지정해
설치 환경에서 `make test-mission-core`로 실행한다. fake Nav2 server를 사용하므로
Gazebo 주행 성공의 증거와 구분한다.

별도 ROS distro나 실험 output은 `ROS_SETUP`, `MISSION_BUILD_BASE`,
`MISSION_INSTALL_BASE`, `MISSION_LOG_BASE` Make 변수로 지정한다. 기본 output은 위 외부 빌드 루트의
`build`, `install`, `log`다. 실제 설정·Mock 경계·오류 복구는
[Mission Manager](src/cleany_mission_manager/README.md),
[Control Bridge](src/cleany_control_bridge/README.md),
[Gazebo composition](src/cleany_bringup/README.md)을 따른다.

## 실제 메카넘 base

`cleany_base_interfaces`는 MCU용 고정 크기 메시지 원본,
`cleany_base_driver`는 `/cmd_vel` 검증, 역기구학과 wheel feedback adapter를 관리한다.
`cleany_base_odometry`의 wheel odometry를 연결한다.

```bash
make build-base
make test-base
make test-motor-core
make firmware-smoke
make firmware-build
```

Base target은 Ubuntu 22.04 `ros2-humble` Distrobox에서 실행한다. 호스트에서
호출하면 Make가 Distrobox에 진입한다. Firmware와 Agent 준비는
[개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md#8-모터-컨트롤러와-micro-ros-개발환경)를
따른다.

ROS의 native 세부 명령은 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install \
  --packages-up-to cleany_base_driver cleany_base_odometry
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/cleany_base_driver/test src/cleany_base_odometry/test
```

실물 geometry와 주행 제한은 `configs/robot/`에서 명시적으로 설정한다.
장치 없는 테스트는 합성 설정과 mock MCU를 사용한다. Launch, enable 절차와
odometry/TF 소유권은 [`cleany_base_driver` README](src/cleany_base_driver/README.md),
wheel 순서와 ENABLE/STOP 계약은
[`cleany_base_interfaces` README](src/cleany_base_interfaces/README.md)를 따른다.

Hand-eye 패키지 경계만 빌드하려면 `make build-handeye`를 사용한다.
`make test-handeye`는 description, MuJoCo backend, MoveIt config와 calibration
패키지를 함께 검사하며 실제 runtime test는 자동으로 `headless:=true`를 사용한다.

MuJoCo에서 random pose 후보를 MoveIt/렌더링/PnP로 검증하고 axis parallelism과
rotation covariance를 분석해 20+5 artifact를 생성한 뒤 실제 calibration을 실행한다.

```bash
make handeye-generate-mujoco
make handeye-mujoco
make handeye-validate-mujoco
```

생성 target은 준비 단계이므로 headless이고 기본 artifact 경로를 materialize한다.
Calibration target은 operator가 motion과 target visibility를 확인할 수 있도록
`headless:=false`를 명시하며 MuJoCo viewer를 연다. Template의 `null` 값을 그대로
사용할 수 없고, pose preflight와 safety/timeouts 검토가 끝난 artifact만 허용한다.
마지막 target은 25개 row/image/hash, ChArUco/PnP 재현, provenance와 150-run
solver 결과를 `dataset_validation.json`으로 검증한다.

MuJoCo 시뮬레이터를 headless 모드로 실행한다.

```bash
make sim
```

Gazebo 시뮬레이터는 `make sim-gazebo`로 실행한다.

Gazebo만 새 환경에서 재현할 때는 아래 순서로 의존성, 기준 환경, 패키지 테스트,
headless 실행을 확인한다.

```bash
make deps-gazebo
make check-gazebo-env
make test-gazebo
make sim-gazebo
```

Gazebo Make target은 활성 `ROS_DISTRO`와 Gazebo major version을 함께 검사해
Humble/Fortress 또는 Jazzy/Harmonic profile을 선택한다. `make build-gazebo`와
`make test-gazebo`는 선택된 profile로 `cleany_gazebo_sim` 및 그 dependency까지만
빌드한다. 성공 판정 topic과 GUI 실행법은
[`cleany_gazebo_sim` README](src/cleany_gazebo_sim/README.md)를 따른다.

ROS 2 Jazzy / Gazebo Harmonic 호환 환경은 팀 표준과 분리한다. 환경 준비는
[개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md#7-선택-ros-2-jazzy--gazebo-harmonic-호환-환경)를
따른다. 두 profile 모두 `make test-gazebo`와 `make sim-gazebo`를 사용하며 Harmonic은
자동으로 전용 output을 사용한다. 자동 판정이 불가능하면
`GAZEBO_PROFILE=fortress|harmonic`으로 명시할 수 있다. 활성 `ROS_DISTRO`와 충돌하는
override는 오류로 종료한다.

지원하는 전체 작업은 `make help`로 확인한다.

## Native 표준 명령

아래 직접 colcon 예제는 workspace 내부 output을 명시적으로 사용하는 경우다.
기본 외부 출력 관리에는 위 Make target을 사용한다. Makefile은 native 명령을 짧게 제공할 뿐 `colcon`, `pytest`, `ros2`를 대체하지
않는다. 직접 실행하거나 문제를 진단할 때는 다음 흐름을 사용한다.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/cleany_mujoco_sim/test/test_scene_loader.py
colcon test
colcon test-result --verbose
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=true
```

Hand-eye에 해당하는 native 세부 명령과 artifact schema는
[`cleany_handeye_calibration` README](src/cleany_handeye_calibration/README.md)를
따른다. 실제 calibration launch의 viewer 기본값은 `headless:=false`이고 테스트만
명시적으로 headless 모드를 사용한다.

`source install/setup.bash`는 build 후 같은 terminal session에서 실행한다.

## 선택 개발도구

Helix 프로젝트 설정은 VM의 native `pyright-langserver`를 사용한다. Helix에서
Pyright를 사용하려면 [개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md)의 선택
개발도구 절을 따른다.

패키지별 topic, launch parameter, 추가 검증 명령은 각 패키지 `README.md`를 따른다.

### 전체 18층 시설 시뮬레이션

`make sim-gazebo-facility GAZEBO_GUI_RENDER_ENGINE=ogre2`로 전체 시설 벽체와
82석, D-HUB ROLY 참고 의자 모델을 실행한다. 기존 D-HUB 단독 실행은
`make sim-gazebo-study-cafe`를 사용한다. 모델 선택·도면 좌표·추정 치수와
Gazebo 이미지/scan/동선 검증은
[Gazebo package README](src/cleany_gazebo_sim/README.md#전체-18층-시설과-roly-의자)를 따른다.

## 공통 Navigation runtime (SCRUM-425)

```bash
make build-navigation
make test-navigation
make test-navigation-runtime
```

위 target은 별도 `build-navigation`/`install-navigation`/`log-navigation`을 사용해
`cleany_navigation`과 description dependency만 빌드한다. Gazebo 없이 saved-map
AMCL/Nav2와 pan → Collision Monitor → 3D voxel guard를 실행할 수 있다.
실행 계약과 필수 robot 설정은
[`cleany_navigation/README.md`](src/cleany_navigation/README.md)를 따른다.
기존 Gazebo Nav2 launch는 공통 runtime에 simulation adapter를 붙인다.
## 개발용 로봇 관제보드

저장소 루트에서 `make build-dev-monitor`로 웹 자산과 ROS 패키지를 함께 설치하고,
`ROS_DOMAIN_ID=<robot-domain> make run-dev-monitor`로 읽기 전용 관측 서버를 실행한다.
접속 주소는 `http://127.0.0.1:8768`이다. Gazebo는
`MONITOR_ROS_ARGS='-p use_sim_time:=true'`를 추가한다.

분리 빌드는 `make build-dev-monitor-web`, `make build-dev-monitor-ros`, core/API 검사는
`make test-dev-monitor`를 사용한다. `MISSION_BUILD_BASE`, `MISSION_INSTALL_BASE`,
`MISSION_LOG_BASE`로 별도 overlay 경로를 지정할 수 있다. 실행 명령의 절대 install 경로도
지원한다. 인터페이스, 토픽/QoS, 기록 재생과 브라우저 검사는
[cleany_dev_monitor README](src/cleany_dev_monitor/README.md)를 따른다.

통합 브랜치의 충돌 처리와 검증 범위는
[검증 및 원본 색인](../docs/validation/README.md)을 따른다.


저장 지도 기반 먼 거리/의자 사이 횡이동 회귀는 저장소 루트에서
`make test-gazebo-navigation-goals`로 실행한다(Humble/Fortress, GUI 없음,
설정 배속 2). `NAVIGATION_GOAL_OUTPUT`으로 새 증거 디렉터리,
`NAVIGATION_GOAL_RTF`로 배속을 지정한다. 두 케이스는 순차 실행한다.
자세한 fixture 및 판정 기준은
[`cleany_gazebo_sim/README.md`](src/cleany_gazebo_sim/README.md)를 따른다.
