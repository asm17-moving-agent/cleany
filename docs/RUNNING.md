# 실행 가이드

명령은 저장소 루트의 Ubuntu 22.04 / Humble 터미널에서 실행합니다. [환경과 출력 경로](DEVELOPMENT_SETUP.md), [연동 계약](INTEGRATION.md)을 확인한 뒤, 각 터미널에 같은 DDS domain과 같은 worktree overlay를 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
```

Navigation 전용 빌드는 `print-navigation-install-base`로 overlay를 조회합니다. 빌드 때 바꾼 Make 변수는 경로 조회 때도 동일하게 전달합니다.

## Jetson 관측 인터페이스 갱신

승인 전용 `/perception/revalidate_target`을 사용하려면 native와 perception
컨테이너의 `cleany_interfaces`, `cleany_perception`, native `cleany_dev_monitor`를
갱신합니다. 진행 중인 작업이 없을 때 인식기와 Monitor를 재시작하고 새 제안을
받습니다. 승인 재확인에는 Gemini를 호출하지 않으며 서비스가 없을 때 전체 분류로
대체하지 않습니다. 재확인 기록은 Monitor 제어 디렉터리의 `revalidations/`에 남습니다.

`DetectedObject2D/3D.detector_label`, 관측 서비스의 원본 클래스 필드와
`DetectedObject2D`의 분류 사유 필드 제거는
native와 GPU 컨테이너에서 같은 버전을 사용해야 합니다. 실행 중인 로봇 작업을
종료한 상태에서 native 패키지와 컨테이너 workspace를 갱신합니다.

```bash
make build-dev-monitor-web
make build COLCON_BUILD_ARGS='--packages-select cleany_interfaces cleany_perception cleany_skill_executor cleany_manipulation_bt cleany_dev_monitor'
tools/vision-container build-perception-workspace
tools/vision-container build-anygrasp-workspace
make motion-container-prepare
```

컨테이너 workspace 준비 명령은 센서나 조작 Action을 시작하지 않습니다.
새 터미널에서 native overlay를 다시 source한 뒤 필요한 실행 경로를 시작합니다.
이미 실행 중인 Dev Monitor도 진행 작업이 없을 때 재시작해 새 인터페이스와 웹 자산을 읽게 합니다.
종류명, 원본 클래스와 이전 CDR 기록의 호환성은
[관측 계약](../ros2_ws/src/cleany_interfaces/docs/observation-contracts.md#객체-메시지)을 따릅니다.

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

브라우저에서 `http://127.0.0.1:8768`을 엽니다. Monitor는 기본적으로 관측 전용이며, 관측 대상과 동일한 DDS Domain 및 메시지 Overlay가 필요합니다. 프론트엔드 빌드 준비는 [개발환경 가이드](DEVELOPMENT_SETUP.md)를 참고하세요.

실시간 입력과 logger 연결은 [monitor.yaml](../ros2_ws/src/cleany_dev_monitor/config/monitor.yaml)에서
설정합니다. `make build-dev-monitor`는 logger도 빌드하지만 Monitor 실행은 logger를
자동 시작하지 않습니다. 기록은 아래 절차로 별도 실행합니다. 웹 자산 갱신 후에는
`make build-dev-monitor-ros`로 설치 자산도 갱신합니다.
이 빌드는 조작 제어가 선언한 내부 의존 패키지까지 포함합니다.
동시 빌드 수는 `COLCON_BUILD_ARGS='--parallel-workers 2'`처럼 지정합니다.

### 독립 Logger

1. 웹 없이 logger만 빌드하려면 `make build-logger`를 실행합니다.
2. 빌드 overlay를 source한 별도 터미널에서 logger를 시작합니다.
   CLI에서 토픽을 먼저 설정하려면 녹화 대기 상태로 실행합니다.

   ```bash
   ros2 launch cleany_logger logger.launch.py record_on_start:=false
   ```

   기본값인 `record_on_start:=true`는 즉시 녹화하며, 시뮬레이션에서는
   `use_sim_time:=true`를 지정합니다. `make run-logger LOGGER_LAUNCH_ARGS='record_on_start:=true'`도
   사용할 수 있습니다.
3. 같은 DDS domain과 빌드 overlay를 사용하는 다른 터미널에서 CLI로 설정하고
   녹화합니다. 설정을 먼저 바꾸려면 logger를 `record_on_start:=false`로 실행합니다.
   이미 녹화 중이면 `stop` 후 설정합니다.

   ```bash
   ros2 run cleany_logger logger status
   ros2 run cleany_logger logger topics
   ros2 run cleany_logger logger topics set camera --topic /camera/head/color/image_raw --hz 2 --enable
   ros2 run cleany_logger logger topics set path_topic --topic /plan --hz 1 --enable
   ros2 run cleany_logger logger start
   ros2 run cleany_logger logger stop
   ```

   `topics set <ID> --disable`은 해당 항목을 끕니다. ID는 `topics` 출력에서 확인합니다.
   속도 체인은 `topics set velocity_topics --topic /first --topic /second`로 전체를
   지정하거나 `velocity_topics:0`처럼 특정 항목만 변경합니다. 출처별 `--hz all`은
   수신마다 기록하며 카메라 등 센서 원본은 숫자 Hz가 필요합니다. 카메라와 점군
   원본은 대용량이므로 필요한 토픽과 주기를 선택합니다.

   루트에서 `make logger LOGGER_ARGS='status'`로도 호출할 수 있습니다.
   다른 노드는 `--node /robot/logger`, 대기 시간은 `--timeout 3`, 자동화 출력은
   `--json`으로 지정합니다. 이 옵션은 하위 명령 앞이나 뒤에 쓸 수 있습니다.
   CLI는 한 번 요청하고 종료하며 logger 프로세스와 녹화는 계속됩니다.
4. 대시보드를 사용하려면 별도 터미널에서 `make run-dev-monitor`를 실행합니다.
   기본 저장 위치는 두 프로세스 모두 `~/.local/state/cleany/recordings`입니다.
   다른 경로는 logger YAML의 `state_directory`와 Monitor의
   `MONITOR_ROS_ARGS='-p state_directory:=/absolute/path'`에 동일하게 지정합니다.
   이전 기록은 기존 디렉터리를 명시해 조회합니다. 대시보드나 브라우저를 종료해도
   logger는 같은 세션을 녹화합니다.

`mission_sim.launch.py`는 `enable_logger:=true record_on_start:=true`로 logger를
함께 실행할 수 있습니다. 기본은 비활성이며 `logger_config:=/absolute/logger.yaml`로
설정을 지정합니다. 이 launch 구성은 시뮬레이션 시계를 logger에 전달합니다.

기록 구조는 [Logger 데이터 계약](../ros2_ws/src/cleany_logger/docs/data.md),
HTTP와 stale 처리는 [Monitor 계약](../ros2_ws/src/cleany_dev_monitor/docs/data.md)을 따릅니다.

### 웹 기반 실물 조작 제어 (작업 제안 및 승인)

실물 웹 제어가 켜져 있는 환경에서는 Mission Manager 없이도 `매니퓰레이션 → 실물 조작 제어` 메뉴에서 로봇이 추천하는 작업을 검토하고 1회씩 실행을 승인할 수 있습니다.

> [!NOTE]
> 실물 제어 기능은 원격 오작동을 방지하기 위해 [웹 제어 준비 및 인증](../ros2_ws/src/cleany_dev_monitor/README.md#선택적-웹-실물-제어와-되돌리기) 절차를 먼저 거쳐야 활성화됩니다.

1. **작업 제안 요청**: `다음 작업 제안받기` 버튼을 눌러 카메라에 포착된 대상 물체, 권장 행동, 추천 이유를 확인합니다.
2. **작업 승인 및 1회 실행**: `승인하여 1회 실행`을 누르면 대상의 현재 위치를 재확인한 뒤 해당 동작을 1회 수행합니다.
   - 시간 만료나 물체 이동으로 대상이 달라진 경우 승인이 안전하게 거절되며, 다시 제안을 받아야 합니다.
   - 특정 물체를 건너뛰고 싶다면 `이 물체 건너뛰기`를 눌러 다음 제안 목록에서 제외할 수 있습니다.
3. **결과 확인 및 후속 제안**: 동작 완료 후 결과와 다음 추천 작업을 확인합니다. 다음 작업은 사용자의 명시적 승인 없이 **절대 자동 실행되지 않습니다**.
   - 오류 발생 또는 안전 차단 시에는 표준 복구 절차를 진행합니다.

> [!TIP]
> - `물체 1개 실행`: 제안 검토 단계 없이 바로 가장 적합한 물체를 자동 선택하여 실행하는 단축 버튼입니다.
> - `pick_return_release` 모드: 물체를 집은 뒤 원래 위치에 다시 내려놓는 모드로, 최종 수거 완료(`pick_and_dump`)와 구분됩니다.

입력 및 기록 설정은 [monitor.yaml](../ros2_ws/src/cleany_dev_monitor/config/monitor.yaml)에서 관리합니다. 기록 DB, 재생, 지연(stale) 표시 기준은 [관측 데이터 계약](../ros2_ws/src/cleany_dev_monitor/docs/data.md)에 정리되어 있습니다.

웹 자산을 수정했을 때는 `make build-dev-monitor-ros`를 실행하여 패키지 설치본을 갱신합니다. (Node.js 22 이상, pnpm 11 필요)
기록 저장 경로는 `MONITOR_ROS_ARGS='-p state_directory:=/absolute/path'` 파라미터로 지정할 수 있습니다.

상세 SLAM, LiDAR, 점군과 Costmap은 동일한 DDS Domain 환경의 RViz2에서 확인합니다. 웹에서 내려받은 원본 Bag 압축을 해제한 후 아래와 같이 재생할 수 있습니다.

```bash
ros2 bag info <bag-directory>
ros2 bag play <bag-directory> --clock
rviz2 --ros-args -p use_sim_time:=true
```

Fixed Frame은 `map_frame`으로 설정하며, Cartographer 서브맵 표시는 전용 RViz 플러그인이 설치되어 있어야 합니다.

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

실물 로봇(Jetson Orin NX)에서 수거 파이프라인을 실행하는 절차입니다.
빌드 산출물은 기본적으로 `CLEANY_DATA_ROOT/builds/<checkout>/`에 저장되며, 환경 변수(`CLEANY_BUILD_ROOT`, `CLEANY_INSTALL_BASE`)로 변경할 수 있습니다. GPU 모델 가중치, 라이선스, MoveIt 오버레이를 사전에 준비한 뒤 `make build-jetson`을 실행합니다.

### 1. 환경 설정

실물 로봇별 설정 경로를 환경 파일 또는 환경 변수로 설정합니다.
설정 파일은 Git 추적 대상이 아니므로 저장소 외부(`~/.config/cleany/`)에 보관합니다.

**방법 A. 환경 파일 로드 (권장)**:
```bash
source "$HOME/.config/cleany/jetson-runtime.env"
```

**방법 B. 환경 변수 직접 지정**:
```bash
# 로봇별 수거 동작 프로파일
export CLEANY_MANIPULATION_COLLECTION_CONFIG=/absolute/path/to/local-collection-profile.json
# 모터 제어기 한계치 및 포트 설정
export CLEANY_FEETECH_CONTROL_CONFIG=/absolute/path/to/local-controller-limits.yaml
# 모션 안전 감시 기준
export CLEANY_HARDWARE_MONITOR_CONFIG=/absolute/path/to/local-motion-monitor.yaml
```

> [!NOTE]
> 환경 변수를 설정하는 것만으로는 백그라운드 서비스나 모터 구동이 시작되지 않습니다. 실제 구동 전에 비상 정지 수단과 로봇 주변 안전거리를 반드시 확보하세요.

### 2. 런타임 시작 및 실행

| 명령어 | 동작 설명 | 비고 |
|---|---|---|
| `make collection-start` | 하드웨어 드라이버, 카메라 브리지, 조작 파이프라인 전체를 백그라운드에서 시작 | 로봇 연결 및 안전 확인 후 실행 |
| `make manipulation-start` | 유휴(Idle) 상태의 Jetson Action 서버 기동 | 외부 요청 대기 모드 |
| `make manipulation-run` | 1회 수거 시퀀스(인식 → 파지 계획 → 접근 → 파지 → 복귀) 즉시 실행 | 단일 작업 실행 |
| `make manipulation-stop` | 실행 중인 조작 파이프라인 및 백그라운드 노드 안전 종료 | 안전 종료 |

### 3. 파지 후보 생성 방식 선택 (Predictor)

`make manipulation-run` 및 Dev Monitor의 실물 실행은 기본적으로 **기하학적 후보 생성(Geometric)** 및 **cuRobo IK / MoveIt 경로 계획**을 사용합니다.

- **기본 모드 (`geometric`)**:
  - 네이티브 ROS 2 노드에서 점군의 기하 형상을 기반으로 파지 후보를 계산합니다.
  - AnyGrasp 컨테이너를 구동하지 않으므로 GPU 메모리와 시스템 자원을 절약합니다.
- **AnyGrasp 모드 (`anygrasp`)**:
  - 딥러닝 기반 파지 신경망을 사용할 경우 환경 변수로 명시합니다:
    ```bash
    export CLEANY_COLLECTION_GRASP_PREDICTOR=anygrasp
    ```
  - 대상 물체의 기하 형상에 따라 4,000점 또는 8,000점의 점군 샘플링을 동적으로 적용합니다. (작거나 얇은 물체는 8,000점 유지)
  - 파지 후보 미생성 시 원본 해상도 재시도 및 메모리 반환 정책이 작동합니다. 상세 내용은 [Predictor 계약 문서](../ros2_ws/src/cleany_grasping/docs/predictors.md)를 참고하세요.

> [!WARNING]
> 이미 실행 중인 런타임의 예측기 방식과 다른 방식으로 실행을 요청하면 안전을 위해 시작 요청이 거절됩니다. 방식을 변경할 때는 런타임을 재시작해야 합니다.

상세한 파라미터 및 하드웨어 연동 계약은 [실물 경로와 설정 계약 문서](../ros2_ws/src/cleany_skill_executor/docs/physical-manipulation.md)와 [하드웨어 인터페이스 문서](../ros2_ws/src/cleany_robot_interface/docs/interfaces.md)를 참고하세요.

## 기록 역재생 운반

Jetson의 원위치 복귀 모드에서 기록을 역재생하는 운반은 목표 이동을 한 번 실행한다. 이동 완료 후 추가 높이 보정과 도착 허용오차 검사를 생략하고 놓기로 진행한다. 소스를 변경한 뒤에는 native 관련 패키지와 `make motion-container-prepare motion-container-check`를 함께 갱신해야 로봇 준비에서 소스 불일치로 차단되지 않는다. 상세 범위는 [운반 완료 기준](../ros2_ws/src/cleany_manipulation_bt/docs/execution_contract.md#jetson-원위치-운반의-완료-기준)을 따른다.

## 고정 중간 자세 복귀

Jetson의 빈 팔 복귀는 `configs/robot/arm_return_waypoints.json`의 두 중간 자세를 거친다. `manipulation_jetson.yaml`의 `bt_arm_return_collision_check_enabled=false`는 이 경로와 그리퍼 닫기의 충돌 검사를 생략한다. 관절 범위와 속도 제한은 실행기에 유지된다. 설정 변경은 실행 중인 작업이 없는 상태에서 `tools/collection-runtime action-restart`로 Action 서버에 반영한다. 상세 범위는 [고정 중간 자세 복귀 계약](../ros2_ws/src/cleany_manipulation_bt/docs/execution_contract.md#jetson-고정-중간-자세-복귀)을 따른다.

## 충돌 최적화 모델 갱신

기본 URDF는 평면화한 충돌 메시를 사용하고, cuRobo는 패키지의
`collision_spheres_joint_simplified.json`을 공통 RobotModel에서 읽는다.
머리 포함 프로필은 전체 118개 구와 고정 마스트 박스 5개이며,
hybrid 한쪽 팔의 제안 비용은 반대팔을 제외한 왼팔 60개, 오른팔 61개 구를 사용한다.
최종 MoveIt 검사와 GPU 전체 형상 검사는 반대팔, 베이스와 primitive를 유지한다.

진행 중인 조작 요청이 없는 상태에서 네이티브 설치와 모션 컨테이너를 함께 갱신한다.

```bash
make build COLCON_BUILD_ARGS='--packages-select cleany_description cleany_moveit_config cleany_skill_executor'
make motion-container-prepare motion-container-check
tools/motion-container exec -- python3 tools/validation/verify_deployed_collision_models.py
```

기존 selector, MoveIt과 robot_state_publisher가 실행 중이면 종료 후 새 overlay와
같은 URDF로 다시 시작한다. 구 배열, 메시 내용과 코드 변경은 전체 모델 캐시와
컨테이너 준비 hash에 반영되므로 기존 캐시를 수동 삭제할 필요는 없다.
인증한 메시의 hash, collision 배치나 scale이 다르면 구를 자동 재사용하지 않고
초기화를 거부한다. `generated` 모델 선택은 비교 검사 용도로 명시한다.



## 실물 어깨 전방 범위 설정

Jetson의 `~/.config/cleany/feetech_control.yaml`과 단일 팔 운영 프로파일은
1번 어깨 yaw를 `[-2.16, 0.0]`, 7번을 `[0.0, 2.0033789089786476]` rad로
제한합니다. `joint_position_bounds`를 바꾸면 실물 런타임을 재시작해야
IK, MoveIt과 모터 드라이버가 동일한 새 URDF를 읽습니다. 별도 control
profile을 지정할 때도 이 값을 유지합니다.
[전방 범위와 실행 검사](../ros2_ws/src/cleany_skill_executor/docs/physical-manipulation.md#실물-어깨-yaw의-전방-범위)를 따릅니다.
