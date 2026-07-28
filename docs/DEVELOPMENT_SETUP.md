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
| Shell | Bash |
| 기본 실행 방식 | VM의 native 환경 |
| 선택 실행 방식 | Docker |

ROS 2 Humble은 Ubuntu 22.04의 amd64와 arm64를 Tier 1 플랫폼으로 지원한다. Python
가상환경으로 ROS의 system Python을 대체하지 않는다.

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

## 6. 빌드와 테스트

```bash
make build
make test
```

변경한 패키지만 빠르게 확인할 수도 있다.

```bash
make test-mission
make test-mujoco
```

MuJoCo 시뮬레이션을 headless로 실행한다.

```bash
make sim
```

Make target과 내부 native 명령은 [ROS 2 workspace 안내](../ros2_ws/README.md)를
참고한다.

## 7. 선택 개발도구

### Helix와 Pyright

레포의 Helix 설정은 native `pyright-langserver`를 사용한다. Helix에서 Python
language server가 필요하면 Node.js 20 LTS와 npm을 준비한 뒤 Pyright를 추가한다.

```bash
sudo npm install --global pyright
pyright --version
```

### Docker

Docker는 팀의 기본 개발환경이 아니다. 격리된 재현이나 Docker 기반 테스트가 필요할
때만 [Docker 개발환경](../docker/README.md)을 사용한다.

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

## 참고 자료

- [ROS 2 Humble Ubuntu deb 설치](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Humble 지원 플랫폼](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
- [ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source)
- [Node.js 다운로드](https://nodejs.org/en/download)
