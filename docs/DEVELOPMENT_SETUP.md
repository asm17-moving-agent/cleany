# Cleany 개발환경 설치 가이드

이 문서는 새 Ubuntu VM에서 Cleany 구현 레포를 빌드하고 테스트할 수 있는 native
ROS 2 개발환경을 준비하는 절차다. 팀은 VM 이미지를 배포하지 않으므로 각 개발자가
아래 기준에 맞춰 환경을 직접 구성한다.

## 기준 환경

| 항목 | 기준 |
|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) |
| ROS | ROS 2 Humble Desktop |
| Python | Ubuntu 기본 Python 3.10.x |
| Gazebo | Fortress (Ignition Gazebo 6.x) |
| Shell | Bash |
| 기본 실행 방식 | VM의 native 환경 |

ROS 2 Humble은 Ubuntu 22.04의 amd64와 arm64를 Tier 1 플랫폼으로 지원한다. Python
가상환경으로 ROS의 system Python을 대체하지 않는다.

GitHub Actions의 자동 검사는 설치 시간을 줄이기 위해 공식
`ros:humble-ros-base-jammy` job container에서 실행하고, workspace에 필요한 추가
의존성은 `package.xml`과 rosdep 규칙으로 설치한다. 이는 개발자의 native Humble
Desktop 환경을 대체하지 않으며 GUI, 실제 센서와 actuator 검증도 포함하지 않는다.
정확한 자동 검사 구성은 [CI workflow](../.github/workflows/ci.yml)를 따른다.

설치 전 버전을 확인한다.

```bash
lsb_release -ds
python3 --version
```

## 1. Ubuntu 기본 설정

UTF-8 locale과 ROS 저장소 등록에 필요한 도구를 준비한다.

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

## 2. ROS 2 apt 저장소 등록

ROS 공식 `ros2-apt-source` 패키지로 keyring과 apt source를 등록한다.

```bash
ROS_APT_SOURCE_VERSION="$(
  curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' \
    | awk -F'"' '{print $4}'
)"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

## 3. ROS 2 Humble과 개발도구 설치

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-rmw-cyclonedds-cpp \
  ros-dev-tools \
  python3-pip git make
```

현재 terminal에서 ROS 환경을 적용한다.

```bash
source /opt/ros/humble/setup.bash
```

새 terminal마다 자동으로 적용하려면 아래 한 줄을 `~/.bashrc`에 추가한다.

```bash
source /opt/ros/humble/setup.bash
```

설치 결과를 확인한다.

```bash
test "${ROS_DISTRO}" = "humble"
python3 --version
command -v ros2 colcon rosdep
ros2 pkg prefix rmw_cyclonedds_cpp
ros2 doctor --report
```

Python은 `3.10.x`, `ROS_DISTRO`는 `humble`이어야 한다.

### 선택: Jetson vision container

일반 ROS package, mission, safety, navigation과 hardware control의 기본은 계속 native
환경이다. Jetson에서는 CUDA model만 container로 격리하며 AnyGrasp와 perception은 서로
다른 service로 실행한다.

```bash
make vision-init
make vision-config
make vision-build
make anygrasp-up
make vision-feature-id
make perception-up
```

`vision-init`은 Git의 `.env` 대신 root-owned
`/etc/cleany/jetson-identity.env`를 설치한다. AnyGrasp license는 host가 아니라 고정
MAC container에서 출력하고 pinned 값과 검증된 feature ID로 신청한다. 상세한
model/license mount, fail-closed 검사, 재부팅 검증과 ROS 2 DDS 설정은
[`containers/vision/README.md`](../containers/vision/README.md)를 따른다.

## 4. 레포지토리 준비

SSH key가 GitHub에 등록되어 있다는 전제에서 submodule과 함께 clone한다.

```bash
git clone --recurse-submodules git@github.com:asm17-moving-agent/cleany.git
cd cleany
```

이미 clone한 레포지토리라면 submodule을 초기화한다.

```bash
git submodule update --init --recursive
```

## 5. rosdep과 workspace 의존성 준비

machine에서 rosdep을 처음 사용한다면 초기화한다. 이미 초기화되어 있다는 메시지가
나오면 다시 실행하지 않아도 된다.

```bash
sudo rosdep init
rosdep update
```

그다음 [Cleany custom rosdep 규칙](../ros2_ws/rosdep/README.md)을 machine에
등록한다. 레포지토리 경로가 바뀌면 custom 규칙도 다시 등록해야 한다.

레포지토리 루트에서 workspace 의존성을 설치한다.

```bash
make deps
```

`cleany_moveit_config` 패키지를 빌드하면 package manifest에 따라 ROS 2
Humble용 MoveIt 2, KDL kinematics plugin, OMPL planner, Pilz industrial
motion planner, `ros2_control` 및
`joint_trajectory_controller`도 `make deps`가 함께 설치한다. MuJoCo hand-eye
backend를 포함한 전체 workspace 설치에서는 Humble용 `mujoco_ros2_control`도
`cleany_mujoco_sim` manifest를 통해 설치한다. 설치 후 다음으로 필수 runtime
package를 확인할 수 있다.

```bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_ros_perception
ros2 pkg prefix moveit_kinematics
ros2 pkg prefix moveit_planners_ompl
ros2 pkg prefix pilz_industrial_motion_planner
ros2 pkg prefix controller_manager
ros2 pkg prefix joint_trajectory_controller
ros2 pkg prefix mujoco_ros2_control
```

센서 기반 MoveIt 충돌 지도에는 `moveit_ros_perception`의
`PointCloudOctomapUpdater`가 필요하다. 기존 환경에 없으면
`sudo apt-get install ros-humble-moveit-ros-perception`으로 설치한다.
플러그인 없이 포인트클라우드가 RViz에 보이는 것만으로는 충돌 검사 연결을
검증한 것이 아니다.

Gazebo 패키지만 재현할 때는 MuJoCo 등 다른 workspace 의존성을 제외하고 설치할 수
있다.

```bash
make deps-gazebo
```

이 target은 `cleany_description`의 MuJoCo parity test에만 필요한 `mujoco` rosdep key를
제외합니다. 전체 workspace test를 실행할 환경에서는 custom rosdep 규칙을 등록한 뒤
`make deps`를 사용합니다.

### YOLOE-seg + Gemini runtime

`make sim-mujoco-pipeline`의 기본 인식은 YOLOE-seg + Gemini다.
API 키·네트워크와 PyTorch/Ultralytics, 로컬 YOLOE checkpoint 및 text encoder가 필요하다.
모델은 저장소에 포함하거나 실행 시 자동 다운로드하지 않는다. 기본 위치는
`~/models`이며 `CLEANY_MODEL_DIR`로 변경할 수 있다. 스터디카페 전용 head/손목 모델
파일명은 `cleany_skill_executor/README.md`의 현재 설정을 따른다.

```text
models/
  yoloe/yoloe-26s-seg.pt
  yoloe/mobileclip2_b.ts
```

VM CPU PyTorch와 Jetson JetPack/CUDA PyTorch는 환경에 맞춰 별도로 설치한다.

현재 VM에서는 관리자 권한 없이 공식 arm64 MoveIt perception `2.5.9` deb의
런타임을 `~/.local/share/cleany/moveit-perception/opt/ros/humble`에 준비했다.
`make sim-mujoco-pipeline`은 시스템 `moveit_ros_perception`이 없을 때만
이 사용자 전용 prefix를 사용하고 로그에 알린다. 다른 위치는
`CLEANY_ROS_PERCEPTION_PREFIX` Make 변수로 지정한다. 다른 머신에 자동
설치되는 것은 아니며 일반 설치는 `make deps` 또는 위의 apt 명령을 따른다.
이 overlay는 현재 시스템 MoveIt/OctoMap ABI 조합에서만 검증했다. 시스템
MoveIt 업데이트 시 호환성을 다시 검증하거나 시스템 패키지로 설치한다.

선택형 `cleany_scene_mapping` C++ updater는 위 런타임 외에 배포 deb의
header/CMake export도 필요하다. `make build`,
`make build-grasp-pregrasp`, `make test-scene-mapping`은 시스템 perception
패키지가 없고 이 prefix가 존재할 때 AMENT/CMAKE/라이브러리 검색 경로에
추가한다. 설치 파일을 자동 다운로드하거나 시스템 `/opt/ros`를 수정하지 않는다.
일반 ROS 설치는 rosdep으로 `moveit_ros_perception`,
`moveit_ros_occupancy_map_monitor`, `geometric_shapes`, `octomap`을 준비한다.

### Gemini API 설정

`make deps`는 `cleany_perception`의 Gemini adapter에 필요한 `google-genai`와 Pillow를
설치한다. API key는 파일이나 ROS parameter에 저장하지 않고 실행 terminal의 환경변수로
제공한다.

```bash
export GEMINI_API_KEY="<your-api-key>"
```

rosdep이 Gazebo 의존성을 해석하지 못할 때만 아래 APT 패키지를 직접 확인한다.
일반 설치에서는 package manifest를 기준으로 하는 `make deps-gazebo`를 우선한다.

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

## 6. 빌드와 테스트

```bash
make build
make test
```

변경한 패키지만 빠르게 확인할 수도 있다.

```bash
make test-mission
make test-mujoco
make test-gazebo
```

Hand-eye의 수학·ROS adapter·오프라인 검증 명령은
[`cleany_handeye_calibration` README](../ros2_ws/src/cleany_handeye_calibration/README.md)를
따른다. 이전 MuJoCo calibration 장면과 실행 target은 제거했다.
현재 MuJoCo 실행 경로는 스터디카페 관찰·인식·수거 시뮬레이션이다.

Gazebo 재현성만 확인할 때는 환경 검사부터 실행한다. 활성 `ROS_DISTRO`와 Gazebo major
version으로 Humble/Fortress 또는 Jazzy/Harmonic profile을 선택한 뒤, profile에 맞는
Ubuntu, Python과 ROS bridge 설치 여부를 확인한다.

```bash
make check-gazebo-env
make test-gazebo
make sim-gazebo
```

MuJoCo 시뮬레이션을 headless로 실행한다.

```bash
make sim
```

Make target과 내부 native 명령은 [ROS 2 workspace 안내](../ros2_ws/README.md)를
참고한다.

## 7. 선택: ROS 2 Jazzy / Gazebo Harmonic 호환 환경

팀의 기준 환경은 위에서 설명한 Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress다.
Jazzy/Harmonic 호환 profile이 필요하면 별도의 Ubuntu 24.04 환경을 사용한다. 이 환경은
팀 표준을 대체하지 않으며 Fortress와 build output을 공유하지 않는다.

현재 검증한 호환 환경은 다음과 같다.

| 항목 | 검증값 |
|---|---|
| OS | Ubuntu 24.04 (Noble) |
| ROS / Python | ROS 2 Jazzy / Python 3.12.x |
| Gazebo | Harmonic (`gz sim` 8.x, 검증 버전 8.11.0) |

Fedora host에서 실험 환경만 분리할 때는 Ubuntu 24.04 Distrobox를 사용할 수 있다.

```bash
distrobox create --name ros2-jazzy \
  --image docker.io/library/ubuntu:24.04 --yes
distrobox enter ros2-jazzy
```

### ROS와 Gazebo 설치

Ubuntu 24.04 환경에서 이 문서의 1절과 2절을 실행해 locale과 ROS apt source를 준비한
뒤 Jazzy와 Harmonic bridge를 설치한다.

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop ros-dev-tools python3-pip git make \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
  ros-jazzy-navigation2
source /opt/ros/jazzy/setup.bash
```

rosdep을 초기화하고 Gazebo 관련 dependency를 설치한다.

```bash
sudo rosdep init
rosdep update --rosdistro jazzy
cd ros2_ws
rosdep install --from-paths src/cleany_description src/cleany_gazebo_sim \
  --ignore-src --skip-keys mujoco --rosdistro jazzy -r -y
cd ..
```

이미 rosdep이 초기화되어 있다는 메시지가 나오면 `sudo rosdep init`은 다시 실행하지
않는다.

### 환경 확인과 실행

다음 값이 맞는지 확인한다.

```bash
source /opt/ros/jazzy/setup.bash
test "$(. /etc/os-release && echo "${VERSION_ID}")" = "24.04"
test "${ROS_DISTRO}" = "jazzy"
python3 --version
gz sim --versions
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix nav2_amcl
```

Snapdragon X2-85 host에서는 Ubuntu Noble 기본 Mesa 25.2.8이 GPU를 인식하지 못해
OGRE2 sensor server가 시작되지 않는다. 이 경우에만 격리된 Distrobox 안에서 Kisak
Mesa를 사용한다. Mesa 26.1.7과 `eglinfo`의 `Adreno (TM) X2-85` 출력을 확인한 뒤
Gazebo runtime test를 실행한다. 이 PPA를 호스트 Fedora에 추가하지 않는다.

```bash
sudo add-apt-repository -y ppa:kisak/kisak-mesa
sudo apt update
sudo apt install -y \
  libgl1-mesa-dri libegl-mesa0 libgbm1 libglx-mesa0 \
  mesa-vulkan-drivers mesa-utils
eglinfo -B
```

저장소 루트에서 공통 Gazebo 명령을 실행한다. 활성 `ROS_DISTRO=jazzy`와 Gazebo 8.x를
확인하면 Harmonic profile을 자동으로 선택한다.

```bash
make check-gazebo-env
make test-gazebo
make sim-gazebo
```

ROS 환경을 source하지 않았고 여러 배포판이 설치돼 있어 자동 판정이 불가능하면
`GAZEBO_PROFILE=harmonic make test-gazebo`처럼 profile을 명시한다. 활성
`ROS_DISTRO`와 충돌하는 profile은 허용하지 않는다.

`make sim-gazebo`는 GUI 없이 server를 실행한다. GUI까지 실행하려면 build 후
다음 명령을 사용한다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-harmonic/setup.bash
ros2 launch cleany_gazebo_sim gazebo_harmonic.launch.py headless:=false
```

Harmonic profile은 `build-harmonic/`, `install-harmonic/`, `log-harmonic/`을 사용한다.
렌더링 sensor server는 OGRE2로 실행하고 GUI는 OGRE1을 사용한다. Harmonic world의
rendering sensor는 구독 전까지 비활성화할 수 있지만, 기본 bridge는 모든 sensor
topic을 bridge한다.

## 8. 모터 컨트롤러와 micro-ROS 개발환경

모터 펌웨어는 `esp32/`의 PlatformIO ESP-IDF 프로젝트다. ROS adapter와
odometry는 Humble workspace에서 빌드한다. ESP32-S3의 **native USB Serial/JTAG**를
custom stream transport로 사용하며 USB-UART bridge용 UART transport를 선택하지 않는다.

### 도구와 외부 소스

| 항목 | 고정 기준 |
|---|---|
| PlatformIO Core | 6.1.19 |
| PlatformIO platform | espressif32 6.12.0 |
| ESP-IDF | 5.5.0 (`framework-espidf` 3.50500.0) |
| 공식 micro-ROS ESP-IDF component | Humble, `4ddd8c26e721662319ed8af981cb7cdc9ae05382` |
| micro-ROS Agent | Humble, `c93ee764e0d2ef4907aeb29233c68cb5f4b56976` |
| ROS 메시지 원본 | `ros2_ws/src/cleany_base_interfaces/msg/` |

전체 source revision과 Python dependency는
[`esp32/micro_ros.lock.json`](../esp32/micro_ros.lock.json)에서
관리한다. 외부 checkout, micro-ROS library, 생성 type support와 PlatformIO 출력은
Git에 포함하지 않는다. 메시지 변경 후 firmware type support도 다시 빌드한다.
펌웨어는 micro-ROS 단일 빌드 경로를 사용한다. GPIO/encoder, 모터 제어 task와
native USB micro-ROS 통신만 포함한다.

공식 component는 ESP-IDF 5.5와 ESP32-S3를 지원하고,
`RMW_UXRCE_TRANSPORT=custom`과 framing-enabled custom transport를 사용할 수 있다.
Firmware 빌드는 ROS overlay 변수와 분리한 shell을 사용한다. ROS 패키지와 Agent
빌드에서는 `/opt/ros/humble/setup.bash`를 적용한다.

### Distrobox 환경

Fedora 호스트에서 작업할 때는 모든 빌드와 테스트를 Ubuntu 22.04
`ros2-humble` Distrobox 안에서 실행한다.

```bash
distrobox enter ros2-humble
cd /home/changsu/Workspace/cleany
source /etc/os-release
test "$VERSION_ID" = 22.04
source /opt/ros/humble/setup.bash
test "$ROS_DISTRO" = humble
python3 --version
```

호스트 Python/PlatformIO를 Ubuntu의 system Python과 혼용하지 않는다.

### 빌드와 장치 없는 테스트

```bash
make firmware-setup
make test-motor-core
make firmware-smoke
make firmware-build
make micro-ros-agent-build
make build-base
make test-base
```

Smoke/runtime는 같은 micro-ROS library cache를 공유한다. 두 빌드는 순서대로
실행하며, project/environment 전환 시 CMake를 다시 구성해 생성 header와 library를
현재 target에 맞춘다.

위 명령은 펌웨어를 upload하거나 serial device를 열지 않는다.
Upload, Agent의 실물 serial 연결과 구동 절차는
[`esp32/README.md`](../esp32/README.md)와
[`cleany_base_driver/README.md`](../ros2_ws/src/cleany_base_driver/README.md)를 따른다.
실물 geometry는 사용자 확인 휠 직경 127 mm, 앞뒤 중심 간 350 mm, 좌우 중심 간
610 mm를 `configs/robot/base_hardware.yaml`에 반영한다. 주행 제한은 별도 안전
검토 후 명시해야 한다.
합성 mock 설정은 실제 로봇의 calibration 값이 아니다.

## 9. 선택 개발도구

### Base plot과 diagnostics

Base 관찰 GUI는 `rqt_gui`, `rqt_plot`, `rqt_robot_monitor`를 사용한다. 의존성은
`cleany_base_driver/package.xml`에 선언되어 `make deps`로 설치된다.
Base 도구만 준비하려면 Ubuntu 22.04 ROS 환경에서 다음을 실행한다.
Fedora 호스트에서는 `ros2-humble` Distrobox 안에서 설치한다.

```bash
sudo apt update
sudo apt install -y ros-humble-rqt-gui ros-humble-rqt-plot ros-humble-rqt-robot-monitor
source /opt/ros/humble/setup.bash
ros2 pkg prefix rqt_plot
ros2 pkg prefix rqt_robot_monitor
ros2 pkg prefix rqt_gui
```

GUI 실행에는 desktop display가 필요하다. Agent와 driver를 먼저 실행한 뒤
[`cleany_base_driver`의 plot 실행 절차](../ros2_ws/src/cleany_base_driver/README.md#plot과-diagnostics)를
따른다.

### Python 정적 검사

미사용 import·중복 정의·정의되지 않은 이름·문법 오류는 다음처럼 확인한다.

```bash
sudo apt install -y python3-flake8
python3 -m flake8 ros2_ws/src tools containers/vision --select F,E9
```

Make 테스트는 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`로 사용자 site-package의
관계없는 pytest 플러그인 충돌을 방지한다. `colcon test --python-testing pytest`로
실행 프레임워크를 명시하며 `setup.py`의 test extra도 pytest로 선언한다.
구버전 `tests_require`가 무시되어 `Ran 0 tests`로 끝나는 것을 통과로 해석하지 않는다.
`make test`는 CMake 재설정도 포함해 pytest가 빠진 이전 등록 상태를 갱신한다.
카메라를 포함한 전체 검사는 GUI 세션에서 `DISPLAY`를 전달하거나
CI와 같이 `xvfb-run --auto-servernum make test`로 실행한다(Xvfb/xauth 설치 필요).

### Helix와 Pyright

레포의 Helix 설정은 native `pyright-langserver`를 사용한다. Helix에서 Python
language server가 필요하면 Node.js 20 LTS와 npm을 준비한 뒤 Pyright를 추가한다.

```bash
sudo npm install --global pyright
pyright --version
```

## 문제 해결

### `ModuleNotFoundError`가 발생하는 경우

workspace를 build하고 overlay를 적용한다.

```bash
make build
source ros2_ws/install/setup.bash
```

Make의 타깃 테스트는 이 과정을 자동으로 수행한다.

### rosdep이 `mujoco` key를 찾지 못하는 경우

[Cleany custom rosdep 규칙](../ros2_ws/rosdep/README.md)을 다시 등록하고
`rosdep update`를 실행한다.

### MuJoCo viewer가 열리지 않는 경우

VM의 3D acceleration과 display 설정을 확인한다. GUI가 필요하지 않은 검증은
`make sim`의 headless 실행을 사용한다.

인식·수거의 `sim_viewer:=efficient`는 최대 20Hz의 snapshot 창이며,
`sim_viewer:=native`는 전체 MuJoCo UI다. headless 센서도 현재 GLFW backend에서
유효한 DISPLAY가 필요하다. 이 VM에서는 `DISPLAY=:0`으로 실행한다.
관찰용 bridge는 `viewer_rate_hz:=20.0 viewer_shadows:=false`를 기본으로 사용한다.
실제 제어·sensor 주기는 viewer rate와 별개다.

`make profile-mujoco-runtime`의 선택형 runtime 측정 도구에는 `psutil`이 필요하다.
없으면 `sudo apt install python3-psutil`로 설치한다. 센서/clock 메시지와 ROS 환경은
기존 workspace 설치를 사용한다. 도구는 자신이 시작한 테스트 process를 종료하며,
사용법과 측정 범위는 `cleany_mujoco_sim/README.md`를 따른다.

### `make check-gazebo-env`가 실패하는 경우

다음 명령으로 어떤 기준이 맞지 않는지 확인한다.

```bash
lsb_release -rs
python3 --version
source /opt/ros/humble/setup.bash
echo "${ROS_DISTRO}"
ign gazebo --versions
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

기대값은 Ubuntu `22.04`, Python `3.10.x`, ROS `humble`, Ignition Gazebo major
version `6`이다. 다른 ROS 배포판에서 생성된 `build/`, `install/`, `log/`를 복사하거나
재사용하지 않는다.

### Gazebo rendering sensor 또는 GUI가 시작하지 않는 경우

Camera와 GPU LiDAR는 headless server에서도 rendering context를 필요로 한다.
Fortress의 `ign gazebo -s`는 GUI만 끄며 rendering sensor를 CPU-only sensor로
바꾸지 않는다. 먼저 실행 환경 안에서 display와 ROS/Gazebo package를 확인한다.

```bash
echo "${DISPLAY}"
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

GPU driver 또는 OpenGL 문제를 구분해야 할 때만 software rendering으로 재현한다.

```bash
LIBGL_ALWAYS_SOFTWARE=1 make sim-gazebo
```

이 설정은 진단용 저속 fallback이며 표준 실행 설정이 아니다. Harmonic에서는
headless server가 OGRE2, GUI가 OGRE1을 사용한다.

## 참고 자료

- [ROS 2 Humble Ubuntu deb 설치](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Humble 지원 플랫폼](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
- [ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source)
- [Node.js 다운로드](https://nodejs.org/en/download)
- [공식 micro-ROS ESP-IDF component (Humble)](https://github.com/micro-ROS/micro_ros_espidf_component/tree/4ddd8c26e721662319ed8af981cb7cdc9ae05382)
- [micro-ROS Agent (Humble)](https://github.com/micro-ROS/micro-ROS-Agent/tree/c93ee764e0d2ef4907aeb29233c68cb5f4b56976)
- [ESP-IDF 5.5 native USB Serial/JTAG 안내](https://github.com/espressif/esp-idf/blob/v5.5/docs/en/api-guides/usb-serial-jtag-console.rst)
