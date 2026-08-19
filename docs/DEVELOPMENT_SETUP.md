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
sudo apt install -y ros-humble-desktop ros-dev-tools python3-pip git make
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
ros2 doctor --report
```

Python은 `3.10.x`, `ROS_DISTRO`는 `humble`이어야 한다.

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
Humble용 MoveIt 2, KDL kinematics plugin, OMPL planner, `ros2_control` 및
`joint_trajectory_controller`도 `make deps`가 함께 설치한다. MuJoCo hand-eye
backend를 포함한 전체 workspace 설치에서는 Humble용 `mujoco_ros2_control`도
`cleany_mujoco_sim` manifest를 통해 설치한다. 설치 후 다음으로 필수 runtime
package를 확인할 수 있다.

```bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_kinematics
ros2 pkg prefix moveit_planners_ompl
ros2 pkg prefix controller_manager
ros2 pkg prefix joint_trajectory_controller
ros2 pkg prefix mujoco_ros2_control
```

Gazebo 패키지만 재현할 때는 MuJoCo 등 다른 workspace 의존성을 제외하고 설치할 수
있다.

```bash
make deps-gazebo
```

이 target은 `cleany_description`의 MuJoCo parity test에만 필요한 `mujoco` rosdep key를
제외합니다. 전체 workspace test를 실행할 환경에서는 custom rosdep 규칙을 등록한 뒤
`make deps`를 사용합니다.

### 선택: Gemini와 SAM2 perception runtime

`make deps`는 `cleany_perception`의 Gemini adapter에 필요한 `google-genai`와 Pillow를
설치한다. API key는 파일이나 ROS parameter에 저장하지 않고 실행 terminal의 환경변수로
제공한다.

```bash
export GEMINI_API_KEY="<your-api-key>"
```

SAM2는 공식 저장소의 현재 구현과 별도 checkpoint가 필요하다. 공식 설치 과정은
PyTorch와 torchvision을 업그레이드할 수 있으므로 Jetson에서는 먼저 JetPack/CUDA와
호환되는 NVIDIA PyTorch 조합을 정한 뒤 설치한다. `make deps`는 PyTorch 또는 SAM2를
자동 설치하지 않는다.

```bash
git clone https://github.com/facebookresearch/sam2.git <external-path>/sam2
python3 -m pip install --user -e <external-path>/sam2
```

CUDA가 없는 Apple Silicon 기반 Ubuntu VM에서는 CUDA extension을 끄고 CPU device를
사용한다. 예를 들어 VM 내부에 SAM2를 설치한 경우 다음과 같이 실행한다.

```bash
cd /home/ubuntu/third_party/sam2
SAM2_BUILD_CUDA=0 python3 -m pip install --user --no-build-isolation -e .
```

checkpoint와 model config 경로는 `inspect_scene.launch.py` 인자로 전달한다. 모델
weight와 API key는 이 저장소에 commit하지 않는다.

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
make test-handeye
make test-gazebo
```

Hand-eye 개발 범위만 반복할 때는 아래 두 target을 사용한다.

```bash
make build-handeye
make test-handeye
```

`make deps`는 hand-eye package manifest를 통해 MoveIt 2/KDL/OMPL,
`ros2_control`, `mujoco_ros2_control`, OpenCV contrib, NumPy, PyYAML과 ROS service/action
dependency를 설치한다. 별도 Python virtualenv에 OpenCV나 NumPy를 다시 설치하지
않는다.

MuJoCo용 random pose set을 생성하고 실제 calibration을 viewer와 함께 실행한 뒤
완성 dataset을 검증한다.

```bash
make handeye-generate-mujoco
make handeye-mujoco
make handeye-validate-mujoco
```

첫 명령은 random seed와 제한된 workspace prior를 사용하되 MoveIt, collision,
렌더링 ChArUco/PnP를 통과한 후보만 모으고 rotation axis/covariance 분석으로 20+5를
선정한다. 두 번째 명령은 `headless:=false`를 고정해 operator 화면을 표시한다.
마지막 명령은 row/image hash, PnP 재현과 150-run solver를 검증한다. 자동 테스트는
`headless:=true`를 명시하므로 CI나 반복 회귀에서 viewer를 열지 않는다. Template의
승인 전 `null` 값은 실행 입력으로 사용하지 않는다.

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

### ROS와 Gazebo 설치

Ubuntu 24.04 환경에서 이 문서의 1절과 2절을 실행해 locale과 ROS apt source를 준비한
뒤 Jazzy와 Harmonic bridge를 설치한다.

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop ros-dev-tools python3-pip git make \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
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

## 8. 선택 개발도구

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

`make handeye-mujoco`는 실제 calibration 관찰을 위해 viewer를 강제로 사용한다.
VM 또는 Distrobox에서 실행한다면 `DISPLAY`가 전달됐는지와 X11/Wayland socket,
OpenGL acceleration을 함께 확인한다. 화면이 없는 환경에서는 실제 calibration을
진행하지 말고 `make test-handeye`로 headless runtime만 검증한다.

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
