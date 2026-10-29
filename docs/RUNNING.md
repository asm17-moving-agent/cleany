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

## Isaac Sim 통합 검증

현재 실행기는 실제 Mission Manager, AMCL/Nav2, LiDAR 충돌 감시기와 Isaac의
원본 Cleany 모델 및 ros2_control을 연결한다. 인식·계획·조작은 기존 mock이며,
물리적 책상 정리 성공을 검증하는 실행기는 아니다. 베이스는 PhysX root 속도를
지정하는 이상적 구동으로, 바퀴 slip과 메카넘 roller 동역학은 포함하지 않는다. 이 단계의 고정 바퀴 모델에는
바닥 마찰을 0으로 설정하며 장애물 충돌은 유지한다.
RGB-D, depth 안전 경로와 전체 조작 BT는 후속 단계다.
[구조 분석과 단계별 계획](ISAAC_INTEGRATION.md)을 함께 확인한다.

호스트 Python, Podman NVIDIA CDI, 설치된 `localhost/cleany-isaac-visual:6.1.0`과
`localhost/cleany-main-isaac-ros:humble` 이미지가 필요하다. 후자는 기존 Isaac
연구 작업 공간에서 만든 Humble/MoveIt 환경이다. 이 절은 해당 이미지가 있는
데스크탑의 재현 절차이며 새로운 호스트의 전체 설치 절차를 대신하지 않는다.

```bash
podman build -t localhost/cleany-mission-isaac-ros:humble \
  -f containers/isaac/Containerfile.ros containers/isaac
python tools/validation/isaac/build.py --build-root /absolute/data/builds/isaac-mission
python tools/validation/isaac/run.py \
  --build-root /absolute/data/builds/isaac-mission \
  --output /absolute/data/runs/isaac-success-001 --scenario success
```

`--output`은 존재하지 않는 새 경로여야 한다. 실행마다 scene/map/targets,
코드 snapshot, image ID와 build identity, controller·Mission 로그, 상태 전이,
물리 base 관측과 DB를 보존한다. 빌드 소스가 달라지면 실행을 거부하므로 먼저
`build.py`를 다시 실행한다. 기본 domain은 174이고 다른 ROS 실행과 공유하지 않는다.
호스트 loopback UDP만 사용하는 Fast DDS 설정을 양쪽 컨테이너에 적용한다.
다른 domain은 `--domain`으로 지정하되 기존 연구 domain 73은 사용하지 않는다.

시작 시 GPU 2048 MiB, host 10000 MiB 이상의 여유를 요구한다.
실행 중 GPU 1024 MiB 또는 host 4096 MiB 미만이면 이번 시도만 종료한다.
기본 제한 시간은 660초다. 다른 학습·시뮬레이터 컨테이너는 종료하지 않는다.
캐시는 build 아래의 전용 `isaac-cache-<domain>`에 보존한다.

기본 `--publication-profile split --physics-sync fabric`은 물리 500Hz와 기존
controller_manager 100Hz를 유지하고, `/clock` 100Hz, `/wheel/odom`과
`odom → base_link` TF 50Hz, LiDAR/자세 기록 10Hz를 정수 physics tick에 맞춘다.
Odometry 속도는 발행 사이 전체 구간의 실제 PhysX 변위에서 계산한다.
Headless 실행은 Fabric을 켜서 physics 결과의 매 step USD 쓰기를 끈다.
센서/odometry/관절/기록은 live PhysX 상태를 읽으며, 화면은 별도 기록 재생
renderer를 사용한다. Live USD pose를 읽는 새 센서나 화면을 추가하면 이 모드의
상태 갱신 계약을 다시 검증해야 한다.

비교용 `--publication-profile legacy --physics-sync usd`는 clock/odom/TF를
500Hz로 발행하고 기존 USD 동기화를 사용한다. `split + usd`로 ROS 발행 분리만의
효과를 측정할 수 있다. `--profile`은 구간/프로세스 계측을 켜며,
`--native-profile`은 첫 실제 이동에서 3초간 Carbonite CPU trace를 기록한다.
Native trace와 겹치는 창은 주행 성능 집계에서 제외한다.

독립 미션을 병렬 실행할 때는 `run.py` 프로세스마다 서로 다른 `--domain`,
`--output`, `--cache-root`를 지정한다. 예를 들어 domain 175와 176은 각각의
Isaac, ROS, 검증기 컨테이너와 DDS 통신을 분리한다. 동일 ROS build는 읽기 전용
mount와 공유 build lock으로 재사용할 수 있지만, 쓰기 가능한 Isaac cache와
Mission DB는 공유하지 않는다. 이는 전체 Facility 월드와 ROS 스택을 각각
실행하는 방식이며 Isaac Lab의 단일 프로세스 환경 복제와는 다르다.
각 프로세스의 기존 메모리 검사와 종료 처리는 자기 domain에만 적용된다.
병렬 실행 재현과 관측 기준은 [테스트](TESTING.md)를 따른다.

### Isaac Lab 단일 프로세스 환경 복제

`run_cloned.py`는 Isaac Lab의 `isaaclab.cloner.usd_replicate`로 전체 Facility와
로봇을 복제한다. 하나의 PhysX scene과 500Hz step, 공통 `/clock`을 공유하고,
각 환경은 기존 Mission/Nav2/ros2_control을 `/env_0`, `/env_1` namespace에서
실행한다. 기본 물리는 CPU이며 `--physics-device cuda:0`으로 GPU 물리를 선택한다.
환경 생성은 USD 복제 방식이며 RL training 실행기는 아니다.
원본 월드 생성은 단독 실행과 같은 `scene_assets.py`를 사용한다.

Isaac Sim 6.1에는 Isaac Lab `v3.0.0-EA`를 별도 checkout으로 사용한다.
검증한 commit은 `ae37b028ea415c91ea2bc32609efcd759ed2b974`다.
기존 Isaac Lab 설치를 변경하지 않는다. [공식 버전 호환 안내](https://github.com/isaac-sim/IsaacLab)를
확인하고, 이 실행 경로의 추가 import 의존성은 Isaac Python용 별도 디렉터리에 준비한다.

```bash
git clone --depth 1 --branch v3.0.0-EA https://github.com/isaac-sim/IsaacLab.git \
  /absolute/data/builds/isaaclab-3.0.0-ea
mkdir -p /absolute/data/builds/isaaclab-3.0.0-ea-deps
podman run --rm --entrypoint bash \
  -v /absolute/data/builds/isaaclab-3.0.0-ea-deps:/labdeps:rw \
  localhost/cleany-isaac-visual:6.1.0 -c \
  '/isaac-sim/python.sh -m pip install --target /labdeps lazy_loader==0.4 packaging==26.3'
python tools/validation/isaac/run_cloned.py \
  --build-root /absolute/data/builds/isaac-mission \
  --cache-root /absolute/data/builds/isaac-cache-177 \
  --isaaclab-root /absolute/data/builds/isaaclab-3.0.0-ea \
  --isaaclab-deps /absolute/data/builds/isaaclab-3.0.0-ea-deps \
  --facility-map /absolute/fixture/facility_integration.yaml \
  --domain 177 --targets seat-12 seat-03 \
  --output /absolute/data/runs/isaac-cloned-001 --timeout 1800
```

현재 chair 입력은 호스트의
`~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models/officechairgrey/1/meshes/OfficeChairGrey.obj`를
사용한다. 원본 Lab checkout과 추가 의존성은 읽기 전용 mount한다.
환경별 출력/DB는 `env_N/`, 공유 물리와 시스템 profile은 루트 `evaluations/`에
남긴다. 검증한 좌석 조합과 결과는 [실행 근거](EVIDENCE.md)에 기록한다.

환경 원점은 100m 간격이고 센서와 odometry는 각 환경의 로컬 좌표를 사용한다.
TF와 센서, Mission endpoint는 namespace별로 격리한다. Humble의 실제 remap
동작에 맞춰 절대 endpoint를 명시적으로 remap하며 `/clock`은 공유한다.
각 환경의 controller parameter와 Nav2 parameter는 namespace 아래에 둔다.
원본 Navigation 설정은 `navigation-unscoped.yaml`로 보존한다.
복제 실행은 기존 lifecycle manager의 autostart를 끄고 모든 manager service가
발견된 뒤 2초간 discovery를 안정화한다. `bringup_clones.py`가 각 환경의
localization/navigation/safety manager에 기존 STARTUP 서비스를 한 번씩
요청한다. 모든 실제 Mission Runtime이 ready가 된 뒤에만 검증기를 시작해
미션을 할당한다. 재시도, lifecycle reset이나 안전 경로 우회는 수행하지 않는다.
이 순서는 다중 환경 시작 시 lifecycle 응답이 유실돼 일부만 주행하는 경우를
검출하고, 모든 환경이 준비된 상태에서 비교하도록 한다. 결과는 루트
`evaluations/bringup.json`에 보존한다. 단독 `run.py`의 시작 방식은 유지한다.
검증기는 clock/odom/TF/scan/joint state의 실제 수신과 발행자 discovery를
확인하고 2초간 response endpoint discovery를 기다린 뒤 publisher 수를
검사하고 미션을 보낸다. 준비 메시지가 먼저 도착했다는 이유만으로 아직
발견되지 않은 센서 publisher를 누락으로 판정하지 않는다. 단일 publisher
조건은 그대로 적용한다.
다중 ROS 프로세스의 수신 포트 할당을 위해 `containers/isaac/fastdds.xml`은
Fast DDS `mutation_tries=256`을 사용한다. 기본 100회에서는 8환경의 일부
참가자가 unicast 수신 포트를 확보하지 못했다. discovery 방식, loopback UDP와
ROS endpoint는 유지하며, 초기 peer 탐색 범위만 늘리는 설정은 사용하지 않는다.
환경별 종료는 해당 베이스 명령을 0으로 만들며, 물리 시각과 process 종료는
공유한다. 독립적인 시각, pause/reset 또는 simulator 장애 격리는 제공하지 않는다.

`--physics-device cuda:0`은 GPU PhysX dynamics와 GPU broadphase를 켠다.
설치된 Isaac 6.1 ros2_control backend가 host tensor buffer를 사용하므로
CPU readback을 유지한다. 물리 계산 장치와 tensor 저장 장치는 다른 설정이며,
`physics-backend.json`의 `manager_device`/`tensor_device`가 `cpu`여도
GPU dynamics는 활성일 수 있다. `runtime_statistics`와 종료 시
`physics-statistics-final.json`의 실제 GPU solver/articulation heap도 함께
확인한다. GPU resident tensor 모드는 이 backend에서 device mismatch가
발생해 사용하지 않는다. 기존 제어기를 다른 구현으로 대체하지 않았다.

같은 코드와 입력으로 환경 수를 늘려 비교할 때는 다음 실행기를 사용한다.
CPU/GPU 4환경과 CPU/GPU 8환경을 순차 실행하므로 비교군끼리 자원을 경합하지
않는다. 8환경은 네 좌석을 두 번 배치한 서로 독립된 환경이다.

```bash
python tools/validation/isaac/scale_campaign.py \
  --build-root /absolute/data/builds/isaac-mission \
  --cache-root /absolute/data/builds/isaac-cache-177 \
  --isaaclab-root /absolute/data/builds/isaaclab-3.0.0-ea \
  --isaaclab-deps /absolute/data/builds/isaaclab-3.0.0-ea-deps \
  --facility-map /absolute/fixture/facility_integration.yaml \
  --counts 4 8 --targets seat-12 seat-03 seat-27 seat-46 \
  --domain 177 --timeout 2400 --output /absolute/data/runs/isaac-scale-001
```

`campaign.json`에 모든 명령과 실패를 포함한 환경별 결과를 남긴다. 각 실행은
새 출력과 Mission DB를 사용하며 전체 캠페인 동안 소스 hash가 달라지면
다음 실행을 거부한다. GPU flag만으로 가속을 주장하지 않고 실제 주행 RTF와
완료 시간, CPU/GPU 메모리를 비교한다.

### Facility 18F의 기존 월드

기존 `materialize_facility_world` 생성기, `facility_18f/facility_layout.yaml`,
ROLY 의자와 `full_facility_targets.yaml`을 그대로 사용한다. 전체 층의 정적
충돌 형상을 옮기며, 로봇은 동일 URDF/ros2_control adapter로 구동한다.
외부 fixture의 `facility_integration.yaml/.pgm`과 Fuel OfficeChairGrey OBJ가
필요하다. 지도 YAML의 image는 같은 디렉터리의 `facility_integration.pgm`이어야 한다.
입력은 읽기만 하며 실행 디렉터리에 복사하고 hash를 기록한다.

```bash
python tools/validation/isaac/run.py \
  --build-root /absolute/data/builds/isaac-mission \
  --world facility18 --target-id seat-12 \
  --facility-map /absolute/fixture/facility_integration.yaml \
  --chair-mesh /absolute/fixture/fuel-assets/officechairgrey/OfficeChairGrey.obj \
  --output /absolute/data/runs/isaac-facility18-001 \
  --timeout 2400 --record-motion
```

대기 위치는 기존 목표의 `(-1.865, -4.705, 1.5708)`이다.
사용자 지정 브랜치 `codex/nav2-tuning-recovered-20261024`의 `69b25020`에 있는
MPPIController와 Smac Lattice를 함께 사용한다. 같은 브랜치의 복구된 benchmark
FollowPath 튜닝도 적용하되 패키지의 도착/안전 기준은 유지한다.
후속 Facility 속도 프로파일은 병진 0.25m/s, 회전 0.5rad/s를 사용한다.
`--require-no-recovery`를 추가하면 왕복 Action feedback의 복구 횟수와
로그의 경로 생성 실패/복구 행동/costmap 초기화가 모두 0이어야 통과한다.
적용 범위는 [통합 계획](ISAAC_INTEGRATION.md#facility-18f와-지정-주행-브랜치)에 기록한다.
전용 `omni-r10cm.json` primitive를 기존 경로 해석 함수로 연결한다. 변환 전 지도와
벽 위치, 선택한 목표/대기 위치의 footprint를 검사한다. 이는 전체 좌석 접근성이나
물리 조작 성공 판정이 아니며, 실제 주행과 Mission 결과는 실행 후 별도로 판정한다.
시설 시작 자세는 기존 `cad_frame.yaml`의 park 설정을 사용하되 두 손목 pitch를
원본 URDF 하한으로 제한한다. 관절 한계를 넓히거나 팔을 제외하지 않는다.
원본 FK/mesh bounds로 접힌 전체 몸체가 기존 stop footprint 안에 드는지 검사하고,
주행 중 joint state가 park에서 0.12rad 이상 벗어난 상태가 0.5초 지속되면 실패한다.
이는 시뮬레이션 초기 자세이며, 팔을 접는 전환 동작과 self-collision 검증은 포함하지 않는다.
바닥/의자의 질감은 단색으로 표시하고 형상은 보존한다.
유리의 광학적 LiDAR 투과는 모델링하지 않는다.

### 성공 주행의 다각도 GIF

실행 시 `--record-motion`을 추가하면 PhysX의 실제 link world pose를 10Hz로
기록한다. 성공 후 아래 명령으로 전방 사선, 측면, 상단의 관찰 카메라를 렌더링한다.
호스트의 `ffmpeg`, `fc-match`와 DejaVu Sans 글꼴이 필요하다.

```bash
python tools/validation/isaac/run.py --build-root /absolute/data/builds/isaac-mission \
  --output /absolute/data/runs/isaac-video-001 --scenario success --record-motion
python tools/validation/isaac/render.py --build-root /absolute/data/builds/isaac-mission \
  --run /absolute/data/runs/isaac-video-001
```

`media/rendered/three-views-compact.gif`는 세 시점을 나란히 보여주며,
`three-views.gif`는 압축 전 버전이다. `front.gif`,
`side.gif`, `top.gif`는 개별 시점이다. 접근/복귀 구간을 0.5 simulation 초마다
표본화해 10fps, 명목 5배속으로 재생한다. mock 작업 대기 구간은 제외한다.
긴 Facility 주행에는 `render.py --sample-period 2`로 20배속을 선택할 수 있다.
기존 Isaac shader cache를 재사용할 때는 실행/렌더링의 `--cache-root`를 지정한다.
영상은 실제 주행 중 기록한 link pose를 Isaac RTX에서 재생한 것으로,
렌더링 중 새 물리나 ROS 제어를 실행하지 않는다. 로봇 센서 영상은 아니다.
Facility 관찰 영상에서만 천장을 숨긴다. 주행 검증의 천장 충돌 형상은 유지하며,
영상에서 숨긴 prim 경로는 `render.json`에 기록한다.
자세 원본과 scene, 원본 frame, 렌더링 명령/소스 hash를 함께 보존한다.
`render.json`은 기록된 위치와 렌더링 위치의 일치 검사도 포함한다.
다른 구도로 다시 만들 때는 `--name`에 새 이름을 지정해 기존 결과를 보존한다.

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

실시간 입력과 logger 연결은 [monitor.yaml](../ros2_ws/src/cleany_dev_monitor/config/monitor.yaml)에서
설정합니다. `make build-dev-monitor`는 logger도 빌드하지만 Monitor 실행은 logger를
자동 시작하지 않습니다. 기록은 아래 절차로 별도 실행합니다. 웹 자산 갱신 후에는
`make build-dev-monitor-ros`로 설치 자산도 갱신합니다.

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

### Isaac 센서 기반 책상 정리 개발 실행

물리 정리 경로는 기존 Nav2 mock 시험과 별도 빌드 및 결과 디렉터리를 사용한다.
실행 완료는 프로세스 종료 코드뿐 아니라 `evaluations/mission.json`의 결과와
독립 안착, 정지, 복귀 근거를 함께 확인한다. 단위 검사 통과만으로 물리 수거를
검증했다고 해석하지 않는다.

```bash
podman build -f containers/isaac/Containerfile.cleanup \
  -t localhost/cleany-cleanup-isaac-ros:humble .
podman build -f containers/isaac/Containerfile.cleanup-gpu \
  -t localhost/cleany-cleanup-isaac-gpu-ros:humble .
podman run --rm --entrypoint python3 \
  -v "$PWD/tools/validation/isaac:/integration:ro" \
  -v /absolute/path/to/cleanup-models:/models:rw \
  localhost/cleany-cleanup-isaac-gpu-ros:humble \
  /integration/download_models.py --output /models
python3 tools/validation/isaac/build.py --cleanup \
  --ros-image localhost/cleany-cleanup-isaac-gpu-ros:humble \
  --build-root /absolute/path/to/cleanup-build
python3 tools/validation/isaac/run.py --cleanup --world facility18 \
  --target-id seat-12 --cleanup-object cup --scenario success --domain 178 --timeout 4200 \
  --ros-image localhost/cleany-cleanup-isaac-gpu-ros:humble \
  --build-root /absolute/path/to/cleanup-build \
  --model-directory /absolute/path/to/cleanup-models \
  --facility-map /absolute/path/to/facility_integration.yaml \
  --output /absolute/path/to/new-cleanup-attempt
```

모델 디렉터리는 `yoloe/yoloe-26l-seg.pt`, `yoloe/mobileclip2_b.ts`,
`qwen3vl-4b/`의 Qwen/Qwen3-VL-4B-Instruct snapshot을 포함한다.
GPU 실행은 bitsandbytes NF4 이중 양자화와 BF16 연산을 사용한다.
컨테이너에서 `download_models.py --output /models`를 실행하면 공식 YOLOE asset의
SHA-256과 고정된 Hugging Face revision을 사용해 다운로드한다.
모델 hash는 `validate_models.py`로 기록한다. 원본 좌석 목표, 작업 접근 목표,
물리 fixture는 `config/room.json`에 남는다. `--cleanup-object cup|tissue`는
기존 MuJoCo 설정의 종이컵 또는 구겨진 휴지 형상과 크기, 질량을 가져온다. 로컬 VLM 분류 결과와 번호가 표시된 입력 이미지는
`evaluations/semantics/`에 보존한다. 전체 설정과 현재 제한은
[Isaac 통합 문서](ISAAC_INTEGRATION.md#센서-기반-책상-정리-통합)를 따른다.

정리 실행은 `containers/isaac/fastdds-rgbd.xml`의 4MiB UDP 송수신 버퍼를
사용한다. 이는 960×720 RGB-D의 큰 depth 메시지 누락을 줄이기 위한 설정이며,
기존 주행 전용 transport와 커널 sysctl은 변경하지 않는다. 센서 검증 시에는
RGB 수신률뿐 아니라 같은 timestamp의 depth 수신을 함께 확인한다.


Isaac cleanup의 실험 설정은 `--cleanup-profile slender-cup-95mm`로 선택한다. 이는 높이95mm 컵, CPU PGS128/8의1,000Hz 물리 계산,0.55m 접근, 기존 ROS 집기,90도 roll 재관측, 확인 성공 후 원래 arm pose 복귀 및 base_link 기준z0.60m trash 투입과0.65rad 투입용 gripper 개방 설정을 함께 사용한다. 기본 `canonical`은 기존68.4mm 컵과 기본 설정을 유지한다. 이 프로필은 수거 성공을 의미하지 않으며 실행 결과는 `docs/EVIDENCE.md`와 출력의 `evaluations/mission.json`으로 확인한다. 사용 명령은 기존 `run.py --cleanup --cleanup-object cup ...`에 해당 옵션을 추가한다. `config/room.json`에는 실제 payload와 물리/launch 설정, `config/coordinator.yaml`에는 BT/조작 설정이 남는다.

실험 컵 프로필의 가까운 작업 접근점은 기존 좌석 목표와 다르다. `e2e-70/71`의 주행은 복구가 발생해 실패했다. 가까운 경로 정렬/목표 각도 threshold 변경만으로는 충분하지 않았다. 프로필은 `path_pose_yaw_weight=0.5m/rad`로 회전 전후에 겹치는 경로의 방향도 비교하고, 진단용 `transformed_global_plan`을 발행한다. 실제 정리 E2E 성공 여부는 개별 Mission 결과로 판정한다.

물리 cleanup launch는 기존 글로벌 StaticLayer 설정을 로컬 코스트맵에도 적용한다.
낮은 LiDAR 관측에 없는 책상 영역도 동일한 시설 지도와 footprint로 검사하며,
LiDAR obstacle layer와 기존 inflation/collision monitor를 함께 유지한다.
이 실행의 rolling local costmap도 `map` 좌표계에서 같은5cm 격자를 사용해
회전된 odom 격자에 정적 지도를 다시 샘플링할 때 생기는 경계 차이를 피한다.
실험 프로필의 `GridBased.validate_swept_path=true`는 기존 연속 footprint 검사와
기존 body-heading 검색으로 native lattice의 샘플 사이 충돌을 보정한다.
실험 구동력0.2Nm에서 빈 손의 팔/그리퍼 결합 이동은 `velocity_scaling=0.04`를
사용한다. 기존 tracking tolerance나 파지 구동력은 바꾸지 않고, 그리퍼의 계획 속도가
이 실험 물리 모델의 실제 개방 속도를 넘지 않도록 제한한다. 실물 보정값은 아니다.
작업 접근의 XY 허용 오차는8mm이며, 최종 회전 진입/이탈 반경은6/8mm다.
연속 경로 검사 옵션은 격자 중심에서 정확한 요청 목표까지의 연결도 검사한다.


### Facility 18 Mission 흐름 검증과 정리 mock

`--cleanup-backend mock`은 사용자 요청에 따라 실제 Mission FSM/BT와 Nav2 왕복을 실행하고, 인식·계획·정리 Action에는 기존 mock fixture를 사용한다. Isaac의 Facility18 시설, 로봇과 주행용 LiDAR/odometry는 유지하며 팔은 주행 자세를 유지한다. mock 인식에 사용하지 않는 RGB-D 실시간 렌더링은 끄고, 주행 GIF는 기록된 자세에서 별도로 렌더링한다. 기본 backend는 `physical`이다. mock 모드에는 모델 다운로드가 필요하지 않으며 실제 컵 투입·파지·작업 후 팔 복귀의 물리 검증을 뜻하지 않는다. report와 GIF에도 이 범위를 표시한다.

현재 `slender-cup-95mm` 프로필의 physical 접근 거리는0.55m다. mock 흐름 검증은 기존 좌석 navigation target을 그대로 유지한다. mock seat-12 목표는[-0.625,1.575,1.570796]이며 물리 파지용 근접 접근과 팔 도달성은 검증하지 않는다. physical 모드는 global/local footprint_padding0.04m, XY 도착 허용 오차8mm와 GoalAngleCritic 범위0.5m를 사용한다. mock 모드는 기존 Facility18 Nav2 설정 전체를 유지하며 XY 허용 오차3cm와 기존 safety padding을 사용한다. physical navigation override와 local StaticLayer를 mock에 적용하지 않는다. 준비 도구가 새로운 접근 목표와 기존 Nav2 설정을 `config/room.json` 및 `config/navigation.yaml`에 기록한다.

접근점 생성은 저장 지도에서 중심 여유를 확인한 뒤, 원본 SDF의 의자 collision bounds와 회전된 전체 robot footprint 및 같은 padding의 겹침도 검사한다. 의자와 겹치는 후보는 제외하고 `cleanup.approach_chair_check`에 근거를 남긴다. 시설 가구는 옮기지 않는다. 이는 시험 fixture의 사전 배치 검사이며 실제 인식이나 Nav2에 정답 pose를 공급하는 기능은 아니다. mock seat-28 목표는 기존[-0.625,-1.625,1.570796]이다. 선택한 physical 접근점은 `cleanup.physical_approach_target`에 별도로 기록하며, mock에서는 그 지점으로 navigation target을 덮어쓰지 않는다. `cleanup.navigation_target_source`가 두 모드를 구분한다.

출력 검증은 실제 방출되는 shifted sequence와 측정 초기 속도를 사용한다. 모든 이동 후보가 거부됐을 때는 같은 검사로 확인한 정지 sequence만 선택적으로 허용한다(`validated_stop_fallback=true`). 이 선택적 정지 설정은 physical 프로필에 적용하며 mock은 기존 주행 설정을 사용한다. 정지는 기존12초 경로 진행 제한을 초기화하지 않는다. `evaluations/navigation.json`의 validated_standstill_cycles와 continuous_footprint_refinements는 Nav2 복구 횟수와 별도 지표다. physical local StaticLayer도 기존 시설 지도를 사용한다.

진행 감시는 controller가 현재 실행하는 경로의 위치와 yaw 진행량을 사용한다. 재계획으로 arc 기준을 바꿀 때도 누적 진행량과 마지막 실제 진행 시각을 보존하므로 경로 교체만으로 정지 시간을 연장하지 않는다. 마지막 회전의 진입/이탈 반경은 goal checker와 같은 double 정밀도로 비교한다. 허용 오차보다 큰 이탈 반경은 회전 모드를 활성화하지 않는다.

```bash
python3 tools/validation/isaac/run.py \
  --cleanup --cleanup-backend mock --cleanup-object cup \
  --cleanup-profile slender-cup-95mm --world facility18 --target-id seat-12 \
  --scenario success --domain 178 --timeout 1800 --require-no-recovery --record-motion \
  --ros-image localhost/cleany-cleanup-isaac-gpu-ros:humble \
  --build-root /home/donglee/workspace/cleany/cleany-data/builds/isaac-cleanup-20261025 \
  --facility-map /home/donglee/workspace/cleany/cleany-data/fixtures/refactor-local-materials/facility_integration.yaml \
  --cache-root /home/donglee/workspace/cleany/cleany-data/builds/isaac-mission-e2e/isaac-cache-177 \
  --output /home/donglee/workspace/cleany/cleany-data/runs/facility18-mission-flow-new
```

출력은 아직 존재하지 않는 디렉터리를 사용한다. 같은 작업 브랜치와 ROS 빌드의 source digest가 일치해야 한다. 성공은 실제 Mission SUCCESS, outbound/return OK, 최종 IDLE/ready, 두 Nav2 goal의 복구0회와 로그의 planner failure/behavior/costmap clear0회를 모두 요구한다. 판정은 `evaluations/mission.json`, `navigation.json`, 원본 로그와 SQLite journal에 남는다. GIF는 성공한 실행의 기록된 PhysX 자세를 재생하며 새로운 주행을 합성하지 않는다.

`--require-no-recovery`는 controller failure와 path progress timeout도 거부한다. 계획 실패 집계는 `failed to create plan`과 Humble의 `Planning algorithm ... failed to generate a valid path`를 모두 포함한다. 각 수치는 로그 문구의 출현 횟수이며, 정상적인 연속 경로 보정이나 검증된 정지 출력은 별도 진단으로 유지한다.

기록한 성공 실행은 다음 명령으로 세 시점 GIF를 생성한다. `--run`에는 위 실행 출력 경로를, `--name`에는 아직 없는 media 하위 디렉터리 이름을 사용한다. 렌더러와 주행은 같은 GPU에서 순차 실행한다. 렌더링 중 physics는 꺼져 있고 카메라만 기록된 로봇을 따라간다. `render.json`의 source hash와 최대 link pose 차이를 확인한다. 주행 환경의 가구 때문에 일부 시점이 가려질 수 있다.

```bash
python3 tools/validation/isaac/render.py \
  --run /home/donglee/workspace/cleany/cleany-data/runs/facility18-mission-flow-new \
  --build-root /home/donglee/workspace/cleany/cleany-data/builds/isaac-cleanup-20261025 \
  --cache-root /home/donglee/workspace/cleany/cleany-data/builds/isaac-mission-e2e/isaac-cache-177 \
  --name rendered-views
```

프로필의 `base_hold` gain5/s는 기존 ideal root velocity 구동에 정지 제동을 표현한다. 요청 속도0일 때만 마지막 정지 자세에 대한 bounded velocity 보정을 사용하며 pose reset이나 물체 부착은 없다. base.jsonl은 requested/actuated_body_velocity를 구분하고 종료 시 base-hold.json에 최대 오차를 남긴다. physical 모드에서는 shared launch의 known_geometry_clear_max_leaves 인자로 MoveIt에만 leaf 검사 예산을 전달한다. 기본200,000과 별개로 이 프로필은2,000,000을 사용한다. 전역 SetParameter는 사용하지 않는다.
