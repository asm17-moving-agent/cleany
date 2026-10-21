# 실행 가이드

명령은 저장소 루트의 Ubuntu 22.04 / Humble 터미널에서 실행합니다. [환경과 출력 경로](DEVELOPMENT_SETUP.md), [연동 계약](INTEGRATION.md)을 확인한 뒤, 각 터미널에 같은 DDS domain과 같은 worktree overlay를 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
```

Navigation 전용 빌드는 `print-navigation-install-base`로 overlay를 조회합니다. 빌드 때 바꾼 Make 변수는 경로 조회 때도 동일하게 전달합니다.

## 실행 선택과 준비 순서

```mermaid
flowchart TB
    Env["Humble 환경 + worktree별 build"] --> Mode{"실행 목적"}
    Mode -->|"주행 / Mission"| Sensors["Gazebo 또는 실물 sensor / odom / TF"]
    Sensors --> Nav["지도 + Nav2 + 안전 경로 준비"]
    Nav --> Mission["Mission Runtime + Gateway bridge"]
    Mode -->|"조작 / calibration"| Mujoco["MuJoCo backend 하나 선택"]
    Mujoco --> MoveIt["MoveIt / controller 준비"]
    MoveIt --> Task["데모 또는 calibration 실행"]
    Mode -->|"base 단독 검사"| Base["mock 또는 실물 Agent + driver"]
    Mission -. "관측" .-> Monitor["Monitor / 동일 domain과 overlay"]
    Task -. "관측" .-> Monitor
    Base -. "관측" .-> Monitor
```

`mission_sim.launch.py`는 주행과 Mission 구성을 함께 띄웁니다. Sensor와 Nav2가 이미 준비돼 있다면 `mission_runtime.launch.py`만 붙입니다. 아래에서 실행 목적에 맞는 절차를 선택합니다.

## Gazebo

1. 의존성과 실행 환경을 확인합니다.
2. Headless base, study-cafe GUI, 전체 시설 GUI 중 하나를 실행합니다.

```bash
make deps-gazebo
make check-gazebo-env
make sim-gazebo                 # headless base
make sim-gazebo-study-cafe      # study-cafe GUI; 위 실행과 별도로 사용
make sim-gazebo-facility        # 전체 시설 GUI; 위 실행과 별도로 사용
```

GUI renderer는 `GAZEBO_GUI_RENDER_ENGINE=ogre|ogre2`로 선택합니다. 센서 서버는 OGRE2를 사용하므로 GPU LiDAR와 camera는 headless에서도 OpenGL context가 필요합니다. 지원 환경은 Fortress/Humble입니다.

3. 같은 overlay/domain의 별도 터미널에서 sensor와 odometry를 확인합니다.

```bash
ros2 topic hz /clock
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

Topic 등록 이후 실제 발행, 이동, 정지를 각각 확인합니다. 실행별 sensor와 odometry 소유권은 [연동 가이드](INTEGRATION.md)를 따릅니다.

## Mission Runtime과 Backend

### 주행 구성을 함께 실행

빌드와 overlay 적용 후 Gazebo, Nav2, Runtime, Gateway를 함께 실행합니다.

```bash
make build-mission-sim
source "$(make -s print-install-base)/setup.bash"
ros2 launch cleany_bringup mission_sim.launch.py \
  gateway_url:=ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws \
  post_mission:=wait_for_next
```

Backend는 별도 `cleany-control-plane` 저장소에서 실행합니다. 같은 robot에 Runtime, bridge, canonical odometry producer를 중복 실행하지 않습니다. 책상 작업은 기본 mock이며 관측 ID에는 실제 사진이 포함되지 않습니다.

### 기존 주행 구성에 Runtime 연결

AMCL/Nav2가 이미 실행 중이면 아래 명령으로 Runtime을 연결합니다.

```bash
make build-mission-runtime
source "$(make -s print-install-base)/setup.bash"
ros2 launch cleany_mission_manager mission_runtime.launch.py \
  gateway_url:=ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws \
  post_mission:=wait_for_next
```

| 실행 조건 | 설정과 동작 |
|---|---|
| Nav2 없이 구조 확인 | `navigation_backend:=mock` 추가 |
| 완료 후 동작 선택 | `post_mission:=return_home\|wait_for_next`; 제품 복귀 정책은 별도 결정 |
| 독립 실험 | 별도 `journal_path`, `bridge_journal_path` 사용 |
| 동일 미션 복구 | 기존 journal DB 유지 |

재연결, 중복, 취소, 재시작 동작은 [Mission 계약](../ros2_ws/src/cleany_interfaces/docs/mission.md)을 따릅니다.

### Gazebo 주행과 Mock 조작

베이스 주행은 Gazebo/Nav2, 인식과 계획 및 팔 조작은 Mock이다. 모든 터미널에 같은 overlay와 DDS domain을 적용한다. 기본 로컬 Mock 실행과 별도로 사용한다.

1. 통합 package를 빌드하고 환경을 적용한다.

```bash
make build-mission-sim
source "$(make -s print-install-base)/setup.bash"
export ROS_DOMAIN_ID=186
```

2. Gateway 없이 시작한다. 세 DB 경로는 분리하고 재시작 시험에서는 보존한다.

```bash
ros2 launch cleany_bringup mission_sim.launch.py \
  profile:=fortress headless:=true \
  runtime_config:=$PWD/ros2_ws/src/cleany_mission_manager/config/runtime_gazebo_manipulation_mock.yaml \
  start_manipulation_mock:=true manipulation_scenario:=success \
  post_mission:=wait_for_next enable_bridge:=false enable_telemetry:=false \
  journal_path:=$HOME/.local/state/cleany/mission-mock/missions.db \
  bridge_journal_path:=$HOME/.local/state/cleany/mission-mock/bridge.db \
  manipulation_database_path:=$HOME/.local/state/cleany/mission-mock/manipulation.sqlite3
```

3. 별도 터미널에서 `ros2 topic echo /mission/status`로 `ready: true`를 확인한 뒤 요청한다. 새 미션에는 새 ID를 사용한다. `NAV2_NOT_READY`이면 Nav2 lifecycle, odom과 TF를 먼저 확인한다.

```bash
ros2 service call /mission/offer cleany_interfaces/srv/OfferMission \
  "{mission_id: gazebo-mock-demo-001, mission_type: clean_desk, target_kind: SEAT, target_id: seat-12, requested_by: manual-test}"
ros2 topic echo /mission/result
```

수락 응답은 완료 결과가 아니다. 정상 실행은 두 물체를 서로 다른 execution ID로 처리한 뒤 추가 빈 관측을 확인한다. 물체당 정상 조작 시간은 약 40초다. [개발 Monitor](#개발-monitor)의 실행 → BT에서 ExecuteOne을 선택하거나 매니퓰레이션 화면을 연다. 같은 domain과 overlay를 사용한다.

Backend 조합과 주행 설정은 [Mock 연결 reference](../ros2_ws/src/cleany_mission_manager/docs/manipulation_mock_integration.md)를 따른다. 기본 종료 정책은 `return_home`이며 위 명령의 첫 시험은 `wait_for_next`를 명시한다.

### 주행도 Mock으로 Action 연결 확인

두 터미널에서 같은 overlay와 `ROS_DOMAIN_ID=186`을 사용한다. 첫 터미널에서 서버를 시작한다.

```bash
make build-mission-mock
source "$(make -s print-install-base)/setup.bash"
export ROS_DOMAIN_ID=186
ros2 launch cleany_skill_executor manipulation_mock.launch.py \
  mock_config:=$PWD/ros2_ws/src/cleany_mission_manager/config/manipulation_server_fixture.yaml \
  database_path:=$HOME/.local/state/cleany/mission-mock/manipulation.sqlite3
```

두 번째 터미널에서 Runtime을 시작한 뒤 위의 준비 확인과 미션 요청을 수행한다. 이 구성의 주행 성공은 Gazebo 주행 증거가 아니다.

```bash
ros2 run cleany_mission_manager mission_runtime --ros-args \
  --params-file ros2_ws/src/cleany_mission_manager/config/runtime_manipulation_mock.yaml \
  -p targets_file:=$PWD/ros2_ws/src/cleany_mission_manager/config/study_cafe_targets.yaml \
  -p post_mission:=wait_for_next \
  -p journal_path:=$HOME/.local/state/cleany/mission-mock/missions.db
```

## 저장 지도 Navigation

```bash
make build-navigation
source "$(make -s print-navigation-install-base)/setup.bash"
ros2 launch cleany_navigation navigation.launch.py --show-args
```

[Launch](../ros2_ws/src/cleany_navigation/launch/navigation.launch.py)에서 지도, backend, sensor 옵션을 확인합니다. `map → odom`의 소유자는 하나만 선택하므로 Mapping과 AMCL을 함께 실행하지 않습니다.

주행 명령은 Mission의 Navigator/Nav2가 생성합니다. Runtime은 `/cmd_vel`을 직접 만들지 않으며, 안전 경로와 sensor 만료 규칙은 [Navigation reference](../ros2_ws/src/cleany_navigation/docs/safety.md)를 따릅니다.

## MuJoCo와 조작 데모

```bash
make sim
# viewer가 필요하면 build와 overlay source 후:
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=false
ros2 launch cleany_mujoco_sim rgbd_pick_demo.launch.py
```

위 두 실행과 `handeye_backend.launch.py` 중 backend 하나만 실행합니다. RGB-D scene 출력은 grasp/skill 입력으로 사용하며, 수거 완료는 별도로 확인해야 합니다.

### 단일 물체 Manipulation Action

Mock 서버의 요청, 취소, 조회와 모니터 사용은
[Mock 실행 안내](../ros2_ws/src/cleany_skill_executor/docs/manipulation_mock_usage.md)를 따릅니다.
실제 인식과 MoveIt을 사용하는 MuJoCo BT 스택은 다음 명령으로 실행합니다.

```bash
make sim-mujoco-manipulation
```

이 스택은 승인된 물체 하나를 실행하며, 기존 sorting coordinator와 별도 구성입니다.
입력, 세부 옵션과 검증 범위는 [MuJoCo BT 서버](../ros2_ws/src/cleany_manipulation_bt/README.md)를 따릅니다.

### Plan-only 후보 선택

MoveIt을 준비한 뒤 같은 overlay의 별도 터미널에서 후보 선택을 실행합니다.

```bash
ros2 launch cleany_moveit_config mock_planning.launch.py
# 같은 overlay의 별도 터미널
ros2 launch cleany_skill_executor grasp_selection.launch.py
```

### MuJoCo 접근 데모

아래 데모는 MuJoCo controller까지 연결합니다. 두 launch는 각각 별도로 실행합니다.

```bash
ros2 launch cleany_skill_executor grasp_execution_demo.launch.py
ros2 launch cleany_skill_executor can_grasp_execution_demo.launch.py
```

| 데모 | 확인 범위 | 실행 조건 |
|---|---|---|
| 기본 grasp | 후보 선택, 두 trajectory 실행, 최종 joint 수렴 | MuJoCo controller 연결 |
| can grasp | RGB-D 검출, 후보 생성, 충돌 검사, 그리퍼 개방, 방향 보정 pre-grasp 접근 | Viewer/OpenGL 필요; `headless:=true` 미지원 |

두 데모 모두 gripper close, attach, lift와 실제 수거는 검증하지 않습니다. Plan-only action과 데모의 차이는 [Manipulation 계약](../ros2_ws/src/cleany_interfaces/docs/manipulation.md)을 따릅니다.

## 개발 Monitor

```bash
make build-dev-monitor
make run-dev-monitor
```

브라우저에서 `http://127.0.0.1:8768`을 엽니다. Monitor는 관측 전용이며, 관측 대상과 같은 DDS domain과 메시지 overlay가 필요합니다. Frontend 준비는 [개발환경](DEVELOPMENT_SETUP.md)을 따릅니다.

입력과 기록은 [monitor.yaml](../ros2_ws/src/cleany_dev_monitor/config/monitor.yaml)에서 설정합니다. 기록 DB, 재생, stale 표시의 의미는 [관측 데이터 계약](../ros2_ws/src/cleany_dev_monitor/docs/data.md)에 있습니다.

웹 자산을 다시 빌드하면 `make build-dev-monitor-ros`로 설치 자산도 갱신합니다. Node.js 22 이상과 pnpm 11을 사용합니다. 기록 위치는 `MONITOR_ROS_ARGS='-p state_directory:=/absolute/path'`로 지정할 수 있습니다.

상세 SLAM, LiDAR, 점군과 costmap은 같은 domain의 RViz에서 확인합니다. 완료된 원본 bag ZIP을 내려받아 압축을 푼 뒤 다음과 같이 재생합니다. Bag에 기록된 토픽, TF와 CameraInfo만 사용할 수 있으며 SQLite 가공 기록은 RViz 입력이 아닙니다.

```bash
ros2 bag info <bag-directory>
ros2 bag play <bag-directory> --clock
rviz2 --ros-args -p use_sim_time:=true
```

Fixed Frame은 `map_frame`에 맞추고 Cartographer Submaps display에는 해당 plugin을 설치합니다.

## Base와 plot

### 장치 없는 mock

별도 domain에서 빌드한 mock를 실행합니다.

```bash
make build-base
export ROS_DOMAIN_ID=173
source "$(make -s print-install-base)/setup.bash"
ros2 launch cleany_base_driver base_driver.launch.py mock:=true
```

Mock domain에는 실물 MCU/Agent를 연결하지 않습니다. 고장 주입 service는 mock node와 테스트가 관리합니다.

### 실물 base와 관측 GUI

Agent와 driver 양쪽에 domain 0을 적용하고 검토한 hardware config를 선택합니다.

```bash
export ROS_DOMAIN_ID=0
ros2 launch cleany_base_driver base_driver.launch.py \
  mock:=false config:="$(ros2 pkg prefix --share cleany_base_driver)/config/base_hardware.yaml"
# 같은 overlay/domain의 별도 터미널에서 관측 GUI:
ros2 launch cleany_base_driver base_monitor.launch.py preset:=true
```

초기 hardware config는 바퀴를 띄운 시험용입니다. Launch 이후 구동 허가는 별도 절차로 수행하며, mock domain과 실제 MCU domain을 혼용하지 않습니다. 실물 시험은 [ESP32 가이드](../esp32/README.md)와 [base 계약](../ros2_ws/src/cleany_base_driver/docs/command-and-odometry.md)을 따릅니다.

Plot은 목표, slew/reversal 이후 목표, encoder 속도(rad/s), PWM(%), diagnostics를 표시합니다. `preset:=false wheel:=fl|fr|rl|rr|all`로 관측 범위를 바꿀 수 있습니다. GUI를 종료해도 별도로 실행한 driver/Agent는 계속 실행됩니다.

## 시설 생성 도구와 종료

`ros2 run cleany_gazebo_sim <스크립트명> --help`로 각 도구의 입력을 확인합니다.

| CLI | 출력 |
|---|---|
| `generate_mission_targets.py` | 접근 목표 YAML과 검증 JSON |
| `render_layout_overlay.py` | 명시한 layout, reference image로 배치 SVG |
| `generate_felt_texture.py` | 명시한 PNG 경로에 펠트 질감 |
| `generate_roly_textures.py` | 명시한 폴더에 seat/back PNG |

시설 reference 이미지와 입력 fixture는 clone에 포함되지 않을 수 있으므로 [근거 색인](EVIDENCE.md)에서 위치를 확인합니다. 목표 생성은 정적 footprint 여유를 검사하며, 전체 경로와 팔 도달성은 별도 검증이 필요합니다.

Launch는 `Ctrl-C`로 종료한 뒤 센서와 명령의 발행 중단과 해당 실행의 자식 종료를 확인합니다. 시나리오 runner의 실패, cleanup 판정은 [테스트 가이드](TESTING.md)를 따릅니다.

## 실물 IMU

MCU 부팅 보정과 gyro 수신 실행은 [IMU 안내](../ros2_ws/src/cleany_base_driver/docs/imu.md)를 따른다.

## Jetson 실물 실행 경로

native 빌드 출력은 다른 실행과 같이 `CLEANY_DATA_ROOT/builds/<checkout>/`에 둔다. `CLEANY_BUILD_ROOT`와 `CLEANY_INSTALL_BASE`로 변경할 수 있으며 host 도구와 container 경로 변환도 같은 기준을 사용한다. GPU/라이선스 runtime과 version이 맞는 MoveIt overlay를 먼저 준비한 뒤 `make build-jetson`을 실행한다.

실물 수거에는 로봇별 외부 프로파일을 명시한다.

```bash
export CLEANY_MANIPULATION_COLLECTION_CONFIG=/absolute/path/to/local-collection-profile.json
export CLEANY_FEETECH_CONTROL_CONFIG=/absolute/path/to/local-controller-limits.yaml
export CLEANY_HARDWARE_MONITOR_CONFIG=/absolute/path/to/local-motion-monitor.yaml
```

실제 로봇 연결과 감독 조건을 준비한 뒤 `make collection-start`로 runtime을 시작한다. `make manipulation-start`는 idle Jetson Action 서버를 연결한다. 실행, 복구와 팔 복귀는 각각 명시적인 운영 명령이다. 프로파일의 준비 여부, controller와 정지 수단을 확인한 운영 환경에서만 사용한다.

공유하는 감독 시험 기본 파일이나 부팅 자동 구동 스크립트는 제공하지 않는다. [실물 경로와 설정 계약](../ros2_ws/src/cleany_skill_executor/docs/physical-manipulation.md), [하드웨어 통신](../ros2_ws/src/cleany_robot_interface/docs/interfaces.md)을 따른다. SDK 기반 수동 commissioning 도구는 선택적인 LeRobot 환경을 사용하며 native ROS 필수 의존성과 구분한다.
