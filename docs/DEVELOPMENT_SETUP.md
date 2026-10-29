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

[CI workflow](../.github/workflows/ci.yml)는 [전용 이미지](../containers/ci/Dockerfile)에 설치한 Humble package 의존성으로 자동 검사를 실행합니다. 이미지 갱신과 컴파일 캐시는 [테스트 가이드](TESTING.md#ci-실행과-재사용)를 따릅니다. GUI와 실물 동작은 별도 검증이 필요합니다. 설치 전에 아래 명령으로 현재 버전을 확인합니다.

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

### Fedora

Ubuntu 22.04 Distrobox `ros2-humble` 안에 위 의존성을 준비합니다. 호스트 Python과 Ubuntu ROS Python은 혼용하지 않습니다. GUI 실행에는 display와 OpenGL 전달도 필요합니다.

### ESP32

모터 펌웨어와 micro-ROS 준비는 [ESP32](../esp32/README.md) 가이드를 따릅니다. 고정 버전은 [micro_ros.lock.json](../esp32/micro_ros.lock.json)이 관리하며, firmware와 ROS/Agent의 shell 환경은 분리합니다. 장치 없는 빌드 이후 upload, serial 연결, 모터 구동은 각각 별도로 확인합니다.

### 형상 검사

형상 도구와 충돌 구의 CPU 검사는 NumPy, SciPy, trimesh를 사용한다. ROS의
system Python과 Jetson CUDA 패키지를 유지하려면 system site packages를 읽는
검증 전용 가상환경을 사용한다. 아래 두 버전은
`containers/motion/requirements.txt`의 버전에 맞춘다.

```bash
python3 -m venv --system-site-packages "$HOME/.local/share/cleany/geometry-test-venv"
source "$HOME/.local/share/cleany/geometry-test-venv/bin/activate"
python -m pip install --no-deps trimesh==4.5.3 scipy==1.13.1
source /opt/ros/humble/setup.bash
source "$(make -s print-install-base)/setup.bash"
```

별도 빌드 경로를 사용한다면 해당 overlay를 source하고 Make 검사에도 같은
`INSTALL_BASE`를 전달한다.

### Monitor 웹

웹 빌드는 Node.js 22 이상과 pnpm 11을 사용합니다.

Jetson에 사용자 전용 toolchain이 이미 설치돼 있다면 SSH 세션에서도 실행 파일
경로를 지정합니다. 아래 경로의 설치본이 없는 환경에서는 먼저 위 버전의
Node.js와 pnpm을 설치합니다.

```bash
cleany_monitor_toolchain="$HOME/.local/share/cleany/dev-monitor/toolchain"
export PATH="$cleany_monitor_toolchain/node/bin:$cleany_monitor_toolchain/pnpm/node_modules/.bin:$PATH"
node --version
pnpm --version
```

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


### 선택형 Jetson 실물 조작 환경

실물 로봇 제어(Jetson 백엔드)는 `cleany_robot_interface`, `cleany_feetech_hardware`, `cleany_scene_mapping` 및 실물 전용 Grasp Selector를 사용합니다. 기존 MuJoCo 시뮬레이션 서버와 분리되어 독립적으로 동작합니다.

#### 1. MoveIt 오버레이 및 빌드 준비
MoveIt 2.5.9와 geometric_shapes 2.3.4의 잠금 패치 및 오버레이를 사용자 경로에 빌드합니다. (시스템 `/opt/ros`는 변경하지 않으며, 지정된 버전/해시가 일치할 때만 빌드됩니다.)

```bash
# 1. 씬 매핑 및 MoveIt 옥토맵 락 패치 설치
make scene-mapping-setup moveit-octomap-lock-setup

# 2. ROS 환경 및 씬 매핑 환경 변수 로드
source /opt/ros/humble/setup.bash
source tools/scene-mapping-env

# 3. Jetson 실물 패키지 빌드
make build-jetson
```

#### 2. 주요 환경 변수 설정

| 환경 변수 | 기본값 | 설명 |
|---|---|---|
| `CLEANY_INSTALL_BASE` | `../cleany-data/builds/<checkout>/install` | 빌드 산출물 설치 경로 (최우선) |
| `CLEANY_BUILD_ROOT` | `../cleany-data/builds/<checkout>/build` | 빌드 중간 산출물 경로 |
| `CLEANY_DATA_ROOT` | `../cleany-data` | 데이터 저장소 루트 경로 |
| `CLEANY_MOVEIT_GEOMETRY_CACHE_DIR` | (미설정 시 캐시 미사용) | 기하 형상 캐시 디렉터리 경로 |
| `CLEANY_SCENE_RUNTIME_PREFIX` | 사용자 기본 접두어 | 씬 매핑 런타임 설치 접두어 |
| `CLEANY_MOVEIT_PATCH_PREFIX` | 사용자 기본 접두어 | MoveIt 패치 라이브러리 접두어 |

> [!NOTE]
> - `make` 실행 시 `INSTALL_BASE`를 임의로 지정하는 경우, 래퍼 스크립트에도 동일하게 `CLEANY_INSTALL_BASE`를 맞춰주어야 경로 불일치가 발생하지 않습니다.
> - GPU 모델 가중치, 라이선스, API 키는 보안을 위해 Git 저장소 외부에서 관리합니다.
>   - TensorRT / YOLOE: [Vision 컨테이너 안내](../containers/vision/README.md)
>   - cuRobo / MoveIt: [Motion 컨테이너 안내](../containers/motion/README.md)
> - 실제 실행 시에는 로봇별 프로파일을 명시해야 합니다. 상세 실행법은 [실물 실행 가이드](RUNNING.md#jetson-실물-실행-경로)를 참고하세요.
