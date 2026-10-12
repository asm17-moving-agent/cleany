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

그다음 아래 custom rosdep 규칙을 machine에
등록한다. 레포지토리 경로가 바뀌면 custom 규칙도 다시 등록해야 한다.

등록 명령은 저장소 루트에서 실행한다.

```bash
CLEANY_REPO_ROOT="$(git rev-parse --show-toplevel)"
echo "yaml file://${CLEANY_REPO_ROOT}/ros2_ws/rosdep/cleany.yaml" \
  | sudo tee /etc/ros/rosdep/sources.list.d/10-cleany.list
rosdep update
```

레포지토리 루트에서 workspace 의존성을 설치한다.

```bash
make deps
```

`cleany_navigation`의 native depth self-filter를 빌드하려면 Python 개발 헤더가
필요하다. manifest의 `python3-dev` 의존성을 `make deps`가 설치한다.

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

시설 생성 도구도 `cleany_gazebo_sim`에 포함된다. NumPy·SciPy·Pillow·PyYAML은
패키지 manifest의 `python3-numpy`, `python3-scipy`, `python3-pil`, `python3-yaml`로
선언하며 위 rosdep target으로 설치한다. CLI는 build 후
`ros2 run cleany_gazebo_sim <스크립트명>`으로 실행한다.

이 target은 `cleany_description`의 MuJoCo parity test에만 필요한 `mujoco` rosdep key를
제외합니다. 전체 workspace test를 실행할 환경에서는 custom rosdep 규칙을 등록한 뒤
`make deps`를 사용합니다.

rosdep이 Gazebo 의존성을 해석하지 못할 때만 아래 APT 패키지를 직접 확인한다.
일반 설치에서는 package manifest를 기준으로 하는 `make deps-gazebo`를 우선한다.

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

## 6. 빌드와 출력 경로

```bash
make build
source "$(make -s print-install-base)/setup.bash"
make test
```

Gazebo·Navigation·시나리오 전용 빌드는 Makefile이 선택하는 별도 출력을 사용한다.
`make -s print-navigation-install-base`, `make -s print-scenario-install-base`로 overlay를 조회한다.
같은 변수로 빌드하고 조회한다. 다른 ROS 배포판의 build/install/log를 재사용하지 않는다.
출력 정리는 Makefile의 clean target을 사용한다. 실행 중인 overlay는 지우지 않는다.

이후 작업은 [실행](RUNNING.md), [테스트](TESTING.md), [캘리브레이션](CALIBRATION.md)을 따른다.
현재 Gazebo 지원 조합은 Humble/Fortress다. 과거 Jazzy/Harmonic 환경 기록은 [EVIDENCE](EVIDENCE.md)에 보관한다.

## 7. 선택 환경

Fedora에서는 Ubuntu 22.04 Distrobox `ros2-humble` 안에 위 의존성을 준비한다.
호스트 Python과 Ubuntu ROS Python을 혼용하지 않는다. GUI 실행에는 display와 OpenGL 전달도 필요하다.

모터 펌웨어와 micro-ROS 준비는 [ESP32](../esp32/README.md)를 따른다.
고정 버전은 [micro_ros.lock.json](../esp32/micro_ros.lock.json)이 관리한다.
Firmware와 ROS/Agent의 shell 환경을 분리한다. Upload·serial 연결·모터 구동은 장치 없는 빌드와 별도다.

Monitor 웹 빌드는 Node.js 22 이상과 pnpm 11을 사용한다.

```bash
cd ros2_ws/src/cleany_dev_monitor/web
pnpm install --frozen-lockfile
pnpm build
```

`pnpm-lock.yaml`이 웹 dependency를 고정한다. ROS 런타임에는 Node.js가 필요하지 않으며
빌드된 web/dist를 package 설치에 포함한다. Playwright browser는 개발 검증용 선택 dependency다.
ROS 쪽 aiohttp·Pillow 등은 package.xml과 rosdep으로 설치한다.

## 문제 해결

- Python import 오류: 같은 worktree를 빌드하고 해당 install overlay를 source한다.
- rosdep의 mujoco key 오류: custom 규칙의 절대 경로를 갱신한 뒤 rosdep update한다.
- Gazebo 환경 검사 실패: Ubuntu 22.04, Python 3.10, ROS humble, Ignition Gazebo major 6 및 ros_gz package를 확인한다.
- Headless camera/GPU LiDAR 실패: GUI 없는 server도 rendering context가 필요하다. Display/OpenGL을 확인한다. `LIBGL_ALWAYS_SOFTWARE=1 make sim-gazebo`는 저속 진단용 fallback이다.
- Viewer 실패: VM 3D acceleration과 DISPLAY/socket을 확인한다. Hand-eye 수집 target은 viewer를 요구한다. 화면 없는 환경은 test-handeye의 headless 검사로 확인한다.

설치 방법의 외부 기준은 [ROS Humble 설치 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)다.
저장소의 재현 기준은 package.xml·rosdep 규칙·lock 파일과 [CI](../.github/workflows/ci.yml)가 관리한다.
