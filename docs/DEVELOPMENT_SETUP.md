# Cleany 개발환경 설치 가이드

새 Ubuntu native ROS 개발환경을 아래 순서로 구성합니다. VM 이미지는 별도로 배포하지 않습니다.


## 인식과 파지 후보 패키지

`make build-perception-candidates`는 `cleany_interfaces`, `cleany_perception`,
`cleany_grasping`을 현재 checkout의 외부 빌드 경로에 설치합니다. GPU 모델과
실물 장치 없이 빌드할 수 있습니다. YOLOE와 AnyGrasp 추론용 환경은
[비전 컨테이너](../containers/vision/README.md), 모델 변환과 검증은
[모델 설정](../configs/models/README.md)을 따릅니다.

`DetectedObject2D/3D`와 `PlanGrasp`의 필드가 추가되어 ROS 직렬화 형식이 바뀝니다.
배포할 때는 이 인터페이스를 사용하는 native 노드와 비전 컨테이너를 모두
같은 소스로 재빌드하고 함께 재시작합니다. 이전 rosbag은 기록 당시의 인터페이스로 읽습니다.

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

[CI workflow](../.github/workflows/ci.yml)는 [전용 이미지](../containers/ci/Dockerfile)에 설치한 Humble package 의존성으로 자동 검사를 실행합니다. 이미지 갱신과 컴파일 캐시는 [테스트 가이드](TESTING.md#ci-실행과-재사용)를 따릅니다. GUI와 실물 동작은 별도 검증이 필요합니다. 설치 전에 아래 명령으로 현재 버전을 확인합니다.

CI 의존성은 Jetson에서 확인한 ROS upstream 버전과 MuJoCo, google-genai의 Python 의존성을 기준으로 고정합니다.
CI 이미지의 apt 저장소는 고정 snapshot을 사용합니다. 아래 native 설치 명령은 현재 apt 저장소를 사용하므로 CI와 동일한 세부 버전을 보장하지 않습니다.

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

새 인식 경로의 `google-genai`와 물리 버스용 `feetech-servo-sdk`는 로컬 rosdep DB에 공식 키가 없어 [전용 규칙](../ros2_ws/rosdep/cleany.yaml)을 사용합니다. 저장소 루트에서 등록한 뒤 DB를 갱신합니다. MuJoCo는 이 규칙에 포함하지 않습니다.

```bash
printf 'yaml file://%s/ros2_ws/rosdep/cleany.yaml\n' "$PWD" | sudo tee /etc/ros/rosdep/sources.list.d/10-cleany.list
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
| MoveIt 내부 충돌 검사 | cleany_moveit_collision manifest의 libfcl-dev, geometric_shapes, pluginlib |
| Hand-eye backend | cleany_mujoco_sim manifest의 mujoco_ros2_control |
| 단일 물체 MuJoCo Action | BehaviorTree.CPP 4.x, pybind11_vendor, python3-dev |
| Mock Action 트리 모니터 | python3-zmq; VS Code 확장의 native ZeroMQ 의존성은 별도 |

`cleany_manipulation_bt`는 BehaviorTree.CPP 4.x를 사용합니다. Humble Navigation의
`behaviortree_cpp_v3`와 별도 패키지이며 두 major 버전을 함께 설치할 수 있습니다.
개별 설치와 빌드는 다음 명령을 사용합니다.

```bash
sudo apt install ros-humble-behaviortree-cpp ros-humble-pybind11-vendor python3-dev python3-zmq
make build-manipulation-bt
```

Gemini 인식으로 MuJoCo Action을 실행할 터미널에는 `GEMINI_API_KEY`를 설정합니다.
로컬 파일을 사용한다면 권한 `600`의 `~/.config/cleany/gemini.env`에 `export` 문을
보관하고 실행 전에 source합니다. 모델 준비는 [인식 reference](../ros2_ws/src/cleany_perception/docs/runtime.md#모델-준비)를 따릅니다.

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

Gazebo와 Navigation 전용 빌드는 Makefile이 선택하는 출력을 사용합니다. Navigation overlay는 `make -s print-navigation-install-base`로 조회합니다. 빌드와 조회에는 같은 Make 변수를 전달합니다.

다른 ROS 배포판의 build/install/log는 재사용하지 않습니다. 출력은 Makefile의 clean target으로 정리하며, 실행 중인 overlay는 보존합니다.

준비 후 [실행](RUNNING.md), [테스트](TESTING.md), [캘리브레이션](CALIBRATION.md) 가이드로 이동합니다. Gazebo 지원 조합은 Humble/Fortress입니다.

## 7. 선택 환경

### Jetson Orin USB 카메라 회전

DECXIN `1M-2012V1` USB 카메라(`0bda:5844`)의 180도 회전은 native host에서
초기화합니다. 설치 대상은 JetPack 6 계열의 Ubuntu 22.04 aarch64 Jetson Orin이며,
system Python 3.10, systemd와 udev를 사용합니다. 별도 pip 패키지는 필요하지 않습니다.
ROS workspace나 vision container 안에 설치하지 않습니다.

저장소 루트에서 다음 명령을 실행합니다.

```bash
make test-camera-orientation
make camera-host-install
```

설치기는 다음 파일을 root 소유로 복사하고 systemd와 udev 설정을 다시 읽습니다.
작업 checkout은 실행 시 참조하지 않습니다.

| 설치 경로 | 역할 |
|---|---|
| `/usr/local/libexec/cleany/camera-orientation.py` | USB 장치 확인, Roll 설정과 readback, 드라이버 복구 |
| `/etc/systemd/system/cleany-camera-orientation@.service` | USB 장치별 일회성 초기화 |
| `/etc/udev/rules.d/70-cleany-camera-orientation.rules` | 해당 VID/PID의 USB 연결 시 초기화 요청 |

설치 명령은 실행 중인 카메라를 건드리지 않습니다. 설치 후 캡처 프로그램을 닫고 USB를
재연결합니다. 이후 재연결과 부팅 시 udev가 초기화를 요청하므로 별도의
`systemctl enable`은 사용하지 않습니다. 규칙은 동일 VID/PID를 가진 모든 카메라에
적용됩니다. 이 모듈의 공통 serial 문자열로 여러 카메라를 구분하지 않습니다.

호스트 설정을 바꾸지 않고 설치 결과만 확인하려면 별도 경로에 배치합니다.

```bash
python3 tools/camera/install.py install --root /tmp/cleany-camera-stage
python3 tools/camera/install.py uninstall --root /tmp/cleany-camera-stage
```

제거는 Jetson 호스트에서 `make camera-host-uninstall`로 수행합니다.
운영 중 조회, 수동 적용과 캡처 시작 순서는 [실행 가이드](RUNNING.md#jetson-usb-카메라-회전)를
따릅니다.

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

## 기존 Mission Mock VM 환경

이미 구성된 Ubuntu VM의 로컬 의존성과 통합 install은 Bash에서 `source /home/ubuntu/Documents/cleany-data/envs/mission-mock-ros/activate.bash`, zsh에서 `source /home/ubuntu/Documents/cleany-data/envs/mission-mock-ros/activate.zsh`로 적용한다. 셸에 맞는 파일을 사용한다. 이 파일은 새 환경의 설치 절차를 대신하지 않으며 Mission, 서버와 interfaces는 같은 worktree install에서 읽는다.

## Jetson 실물 제어 패키지

실물 상태 조회, Feetech 제어기와 MoveIt 연동 패키지는 다음 명령으로 빌드합니다.
ROS 의존성은 위 rosdep 절차로 준비합니다. 빌드에는 로봇 장치가 필요하지 않습니다.
`cleany_robot_interface` 빌드는 Python 노드와 Feetech C++ 플러그인을 함께 설치합니다.
Feetech 드라이버를 별도 패키지로 빌드한 환경에서 전환할 때는 새 `CLEANY_BUILD_ROOT`를
사용하고 이전 install을 source하지 않은 셸에서 빌드합니다.

rosdep의 pip 규칙에는 패키지 이름을 사용하고, CI의 Feetech SDK 1.0.0 고정은
[`pip-constraints.txt`](../containers/ci/pip-constraints.txt)에서 관리합니다. 같은 버전으로
로컬 의존성을 준비하려면 `PIP_CONSTRAINT="$PWD/containers/ci/pip-constraints.txt" make deps`를 사용합니다.

```bash
make build-jetson-control
source "$(make -s print-install-base)/setup.bash"
```

`CLEANY_DATA_ROOT`의 기본값은 저장소 옆 `cleany-data`이고,
`CLEANY_BUILD_ROOT`는 그 아래 `builds/<checkout>`입니다. `BUILD_BASE`,
`INSTALL_BASE`, `LOG_BASE`는 빌드 루트 아래의 `build`, `install`, `log`입니다.
호스트 Python 도구와 `tools/ros2/cleany-install-env`도 같은 경로 규칙을 사용합니다.
`INSTALL_BASE`를 바꿔 직접 실행할 때는 `CLEANY_INSTALL_BASE`에도 같은 값을 지정합니다.

실물 실행은 [제어 실행 절차](RUNNING.md#jetson-실물-실행-경로)를 따릅니다.

Jetson에서 시스템 MoveIt과 버전을 맞춘 사용자 설치본을 사용하는 경우에는
빌드 전에 해당 dependency overlay를 source합니다. 예를 들어 기존 MoveIt 2.5.9
설치 환경은 아래 perception/Pilz overlay를 사용합니다. 없는 환경에서는 먼저 위
rosdep 절차로 배포판 의존성을 준비합니다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/.local/share/cleany/moveit-scene-runtime/local_setup.bash"
export LD_LIBRARY_PATH="$HOME/.local/share/cleany/moveit-perception/usr/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH):${LD_LIBRARY_PATH:-}"
make build-jetson-control
```

## 실물 매핑 의존성

`cleany_bringup`의 실물 매핑은 native ROS 2 Humble의 설치된 패키지를 사용한다.
`make build-hardware-mapping`은 lockfile로 shadcn UI를 빌드한 뒤 관련 ROS 패키지를 빌드한다.
UI 빌드에는 Node.js 22.12 이상과 npm이 필요하며 실행 시 Node 서버나 외부 CDN은 필요 없다.

Cartographer, `cartographer_ros_msgs`, SLAMTEC `sllidar_ros2`, `micro_ros_agent` 실행 파일이 필요하다.
일반 ROS 의존성은 rosdep으로 설치한다. 배포판의 공식 rosdep 규칙으로 준비되지 않는 외부
패키지는 별도 고정 revision workspace에서 빌드하고 해당 설치 prefix를 환경에 제공한다.
기존 Agent 준비는 `make micro-ros-agent-build`와 [ESP32 가이드](../esp32/README.md)를 따른다.
LiDAR 드라이버 upstream은 [Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)다.
기존 실물 시험에서 사용한 revision은 [hardware.repos](../configs/dependencies/hardware.repos)에 고정했다.
별도 workspace의 src에 `vcs import src < /path/to/cleany/configs/dependencies/hardware.repos`로 가져와
빌드하고 `ros2 pkg prefix sllidar_ros2`로 설치를 확인한다.
새 운영 노드는 실행 중 Git clone이나 colcon build를 수행하지 않는다.

센서 by-id, baud, 장착 TF와 scan mode는 `cleany_bringup/config/mapping_*.yaml`에서 관리한다.
장치가 없는 단위/모의 검사는 실물 드라이버 설치 없이 실행할 수 있지만 실물 launch에는 두 외부
드라이버가 모두 설치돼 있어야 한다. 이 통합은 MCU 펌웨어를 빌드하거나 업로드하지 않는다.

Humble의 공식 rosdep 규칙에는 위 두 드라이버가 없으므로 `make deps`와 장치 없는 CI는
`sllidar_ros2`, `micro_ros_agent`만 명시적으로 제외한다. package.xml의 실행 의존성은 유지하고
실물 실행 전 두 드라이버의 package prefix를 확인한다. CI는 호스트 Node 24에서 조종 UI와
입력 검사를 먼저 빌드한 뒤 ROS 컨테이너의 workspace에 설치한다.

## Jetson 얼굴 디스플레이 환경

독립 디스플레이에는 ROS 2 Humble, colcon, `cleany_interfaces`의 rosdep 의존성, Python venv, Chromium, GNOME 그래픽 세션과 `gnome-session-inhibit`, curl, Noto CJK 글꼴이 필요하다. 부팅 후 전체 화면 표시에는 그래픽 자동 로그인이 필요하며 설치기는 로그인 설정을 바꾸지 않는다. `install-browser.sh`는 전용 폴더에 Playwright 1.64.1과 ARM64 브라우저를 설치하며 시스템 Python 패키지를 변경하지 않는다. 설치와 실행은 [디스플레이 운영](../ros2_ws/src/cleany_display/README.md)을 따른다.
