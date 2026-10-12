# Cleany 개발환경 설치 가이드

새 Ubuntu native ROS 개발환경을 아래 순서로 구성합니다. VM 이미지는 별도로 배포하지 않습니다.

## 기준 환경

| 항목 | 기준 |
|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) |
| ROS | ROS 2 Humble Desktop |
| Python | Ubuntu 기본 Python 3.10.x |
| Gazebo | Fortress (Ignition Gazebo 6.x) |
| Shell | Bash |
| 기본 실행 방식 | VM의 native 환경 |

ROS 2 Humble은 Ubuntu 22.04의 amd64와 arm64를 Tier 1 플랫폼으로 지원합니다. ROS의 system Python은 유지하며 Python 가상환경으로 대체하지 않습니다.

[CI workflow](../.github/workflows/ci.yml)는 Humble ros-base container에서 package 의존성과 자동 검사를 확인합니다. GUI와 실물 동작은 별도 검증이 필요합니다. 설치 전에 아래 명령으로 현재 버전을 확인합니다.

```bash
lsb_release -ds
python3 --version
```

## 1. Ubuntu 기본 설정

UTF-8 locale과 ROS 저장소 등록에 필요한 도구를 준비합니다.

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

## 2. ROS 2 apt 저장소 등록

ROS 공식 `ros2-apt-source` 패키지로 keyring과 apt source를 등록합니다.

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

현재 terminal에 ROS 환경을 적용합니다.

```bash
source /opt/ros/humble/setup.bash
```

새 terminal마다 자동 적용하려면 아래 한 줄을 `~/.bashrc`에 추가합니다.

```bash
source /opt/ros/humble/setup.bash
```

설치 결과를 확인합니다.

```bash
test "${ROS_DISTRO}" = "humble"
python3 --version
command -v ros2 colcon rosdep
ros2 doctor --report
```

Python 3.10.x와 `ROS_DISTRO=humble`을 확인합니다.

## 4. 레포지토리 준비

GitHub에 SSH key를 등록한 뒤 submodule과 함께 clone합니다.

```bash
git clone --recurse-submodules git@github.com:asm17-moving-agent/cleany.git
cd cleany
```

이미 clone한 레포지토리에서는 submodule을 초기화합니다.

```bash
git submodule update --init --recursive
```

## 5. rosdep과 workspace 의존성 준비

Machine에서 rosdep을 처음 사용한다면 초기화합니다. 이미 초기화됐다는 메시지가 나오면 이 단계를 건너뜁니다.

```bash
sudo rosdep init
rosdep update
```

의존성은 공식 rosdep 규칙을 사용합니다. MuJoCo Python 패키지는 `python3-mujoco-pip` 키로 설치합니다.

이전 설정에서 custom 규칙을 등록했다면 아래 항목을 제거한 뒤 DB를 갱신합니다.

```bash
sudo rm -f /etc/ros/rosdep/sources.list.d/10-cleany.list
rosdep update
```

레포지토리 루트에서 workspace 의존성을 설치합니다.

```bash
make deps
```

| 기능 | `make deps`로 설치하는 의존성 |
|---|---|
| Native depth self-filter | Manifest의 `python3-dev` 개발 헤더 |
| 계획, 제어 | MoveIt 2, KDL, OMPL, ros2_control, joint_trajectory_controller |
| Hand-eye backend | cleany_mujoco_sim manifest의 mujoco_ros2_control |

아래 명령으로 runtime package 설치를 확인합니다.

```bash
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_kinematics
ros2 pkg prefix moveit_planners_ompl
ros2 pkg prefix controller_manager
ros2 pkg prefix joint_trajectory_controller
ros2 pkg prefix mujoco_ros2_control
```

Gazebo만 준비할 때는 다음 target을 사용합니다.

```bash
make deps-gazebo
```

시설 CLI는 `cleany_gazebo_sim`에 포함됩니다. NumPy, SciPy, Pillow, PyYAML은 manifest와 rosdep으로 설치하며, 빌드 후 `ros2 run cleany_gazebo_sim`으로 실행합니다.

`deps-gazebo`는 MuJoCo parity-test용 `python3-mujoco-pip` 키를 제외합니다. 전체 테스트를 준비할 때는 `make deps`를 사용합니다.

일반 설치는 manifest 기반의 `make deps-gazebo`를 사용합니다. Rosdep이 Gazebo 의존성을 해석하지 못할 때는 아래 APT 패키지를 직접 확인합니다.

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

Gazebo, Navigation, 시나리오 전용 빌드는 Makefile이 선택하는 별도 출력을 사용합니다. Navigation과 시나리오 overlay는 `make -s print-navigation-install-base`, `make -s print-scenario-install-base`로 조회합니다. 빌드와 조회에는 같은 Make 변수를 전달합니다.

다른 ROS 배포판의 build/install/log는 재사용하지 않습니다. 출력은 Makefile의 clean target으로 정리하며, 실행 중인 overlay는 보존합니다.

준비 후 [실행](RUNNING.md), [테스트](TESTING.md), [캘리브레이션](CALIBRATION.md) 가이드로 이동합니다. 현재 Gazebo 지원 조합은 Humble/Fortress이며, 과거 Jazzy/Harmonic 환경 기록은 [EVIDENCE](EVIDENCE.md)에 있습니다.

## 7. 선택 환경

### Fedora

Ubuntu 22.04 Distrobox `ros2-humble` 안에 위 의존성을 준비합니다. 호스트 Python과 Ubuntu ROS Python은 혼용하지 않습니다. GUI 실행에는 display와 OpenGL 전달도 필요합니다.

### ESP32

모터 펌웨어와 micro-ROS 준비는 [ESP32](../esp32/README.md) 가이드를 따릅니다. 고정 버전은 [micro_ros.lock.json](../esp32/micro_ros.lock.json)이 관리하며, firmware와 ROS/Agent의 shell 환경은 분리합니다. 장치 없는 빌드 이후 upload, serial 연결, 모터 구동은 각각 별도로 확인합니다.

### Monitor 웹

웹 빌드는 Node.js 22 이상과 pnpm 11을 사용합니다.

```bash
cd ros2_ws/src/cleany_dev_monitor/web
pnpm install --frozen-lockfile
pnpm build
```

`pnpm-lock.yaml`이 웹 dependency를 고정합니다. 빌드된 `web/dist`를 package 설치에 포함하므로 ROS 런타임에는 Node.js가 필요하지 않습니다. Playwright browser는 개발 검증 시 설치하며, ROS 쪽 aiohttp, Pillow 등은 package.xml과 rosdep으로 설치합니다.

## 문제 해결

| 증상 | 확인과 조치 |
|---|---|
| Python import 오류 | 같은 worktree를 빌드하고 해당 install overlay를 source |
| Rosdep의 `python3-mujoco-pip` 키 오류 | `rosdep update`로 공식 규칙을 갱신 |
| Gazebo 환경 검사 실패 | Ubuntu 22.04, Python 3.10, ROS humble, Ignition Gazebo major 6, ros_gz package 확인 |
| Headless camera/GPU LiDAR 실패 | Server의 rendering context와 display/OpenGL 확인. `LIBGL_ALWAYS_SOFTWARE=1 make sim-gazebo`는 저속 진단용 fallback |
| Viewer 실패 | VM 3D acceleration과 DISPLAY/socket 확인. Hand-eye 수집에는 viewer 필요; 화면 없는 환경에서는 `test-handeye`의 headless 검사 사용 |

설치 기준은 [ROS Humble 공식 문서](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)를 따릅니다. 저장소의 재현 기준은 package.xml, 공식 rosdep 규칙, lock 파일과 [CI](../.github/workflows/ci.yml)가 관리합니다.
