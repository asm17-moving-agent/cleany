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

설치 전 버전을 확인한다.

```bash
lsb_release -ds
python3 --version
```

### Jetson Orin NX 런타임 기록

Jetson에서는 패키지를 설치하기 전에 레포지토리 루트에서 preflight를 실행한다. 이
도구는 추가 Python 패키지 없이 OS, L4T/JetPack, CUDA, cuDNN, TensorRT, 현재
`nvpmodel` 모드, 온도·메모리, 주요 command와 PyTorch CUDA smoke 결과를 JSON으로
기록한다. 성능 모드 번호는 장치마다 다를 수 있으므로 도구가 조회한 이름과 번호를
그대로 사용하고 특정 번호를 가정하지 않는다.

```bash
python3 tools/jetson_preflight.py --check \
  --output /tmp/cleany-jetson-preflight.json
python3 -m json.tool /tmp/cleany-jetson-preflight.json
```

`--check`는 Jetson 기본 런타임 관문 중 하나라도 실패하면 종료 코드 2를 반환한다.
ROS 2와 PyTorch는 후속 설치 전에는 없어도 되므로 JSON에 상태를 기록하되 이 기본
관문의 필수 항목에는 포함하지 않는다. 설치가 끝난 뒤 같은 명령을 다시 실행해 성공한
패키지 버전과 CUDA 동작 여부를 비교한다.

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
  curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' \
    | awk -F'"' '{print $4}'
)"
source /etc/os-release
ROS_APT_CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
ROS_APT_SOURCE_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${ROS_APT_CODENAME}_all.deb"

test -n "${ROS_APT_SOURCE_VERSION}"
echo "${ROS_APT_SOURCE_URL}"
curl -L -o /tmp/ros2-apt-source.deb \
  "${ROS_APT_SOURCE_URL}"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

## 3. ROS 2 Humble과 개발도구 설치

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y ros-humble-desktop ros-dev-tools python3-pip git make
```

Jetson에서는 upgrade 목록에서 `nvidia-jetpack`, `nvidia-l4t-*`, CUDA, cuDNN 또는
TensorRT 제거·downgrade가 보이면 진행하지 않는다. kernel package hold도 임의로
해제하지 않는다.

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

### 선택: Jetson RealSense D435 runtime

D435 실물 RGB-D 검증 환경에는 Humble용 공식 ROS wrapper를 설치한다.

```bash
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-description
```

2026-08-13 Jetson 검증 기준은 RealSense ROS `4.58.3`, librealsense `2.58.3`, D435
firmware `5.17.3.10`이다. 설치된 정확한 APT 버전은 다음 명령으로 기록한다.

```bash
dpkg-query -W \
  ros-humble-librealsense2 \
  ros-humble-realsense2-camera \
  ros-humble-realsense2-camera-msgs \
  ros-humble-realsense2-description
```

카메라 실행과 5분 aligned RGB-D 관문은
[`cleany_perception` README](../ros2_ws/src/cleany_perception/README.md#jetson-d435-입력-관문)를
따른다.

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

Jetson에서 검증한 SDK는 `google-genai==2.18.0`이며 custom rosdep 규칙도 같은 버전으로
고정한다. 설치된 wheel은 `google_genai-2.18.0-py3-none-any.whl`, SHA-256은
`4c5e60ccaed3ed35ac2ee81e87c5bebf7280cd49b81526d872a526e97ce25f46`이다. PyTorch가
포함되는 `local-tokenizer` extra는 설치하지 않는다. SDK가 요구하는 최신 `anyio`의
pytest plugin은 Ubuntu 22.04 기본 pytest `6.2.5`와 호환되지 않으므로 Jetson 검증
버전인 `pytest==8.4.2`를 함께 설치한다.

검증 wheel URL:

```text
https://files.pythonhosted.org/packages/99/63/84160760f74e6c74bab322afc26d064240f42b45a4150f184f7f2605d535/google_genai-2.18.0-py3-none-any.whl
```

```bash
python3 -m pip show google-genai
python3 -m pip show pytest
python3 -c 'from google import genai; print(genai.__name__)'
export GEMINI_API_KEY="<your-api-key>"
```

#### Jetson PyTorch CUDA 관문

NVIDIA 호환성 표는 JetPack 6.2에서 PyTorch `2.8.0a0+5228986c39`를 지원하지만 해당
release의 standalone NVIDIA wheel은 제공하지 않는다. 따라서 이 프로젝트의 2026-08-13
native 검증 후보는 NVIDIA `jetson-containers` 빌드 cache인 Jetson AI Lab에서 제공하는
JetPack 6 / CUDA 12.6용 `torch==2.8.0`, `torchvision==0.23.0`이다. 이는
`developer.download.nvidia.com`의 NVIDIA Framework wheel과 동일한 배포물이라고
간주하지 않는다.

- NVIDIA 호환성 표: <https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html#compatibility>
- Jetson containers: <https://github.com/dusty-nv/jetson-containers>
- wheel index: <https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/>

검증한 wheel과 SHA-256은 다음과 같다.

| package | wheel URL | SHA-256 |
|---|---|---|
| torch 2.8.0 | <https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl> | `62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc` |
| torchvision 0.23.0 | <https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl> | `907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1` |

system Python과 ROS 2를 유지한 채 user site에 설치한다. 먼저 wheel을 임시 directory에
받고 해시를 검증한다.

```bash
CLEANY_TORCH_WHEEL_DIR="$(mktemp -d /tmp/cleany-pytorch.XXXXXX)"
python3 -m pip download --no-deps \
  --dest "${CLEANY_TORCH_WHEEL_DIR}" \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
  torch==2.8.0 torchvision==0.23.0

cd "${CLEANY_TORCH_WHEEL_DIR}"
printf '%s  %s\n' \
  '62a1beee9f2f147076a974d2942c90060c12771c94740830327cae705b2595fc' \
  'torch-2.8.0-cp310-cp310-linux_aarch64.whl' \
  '907c4c1933789645ebb20dd9181d40f8647978e6bd30086ae7b01febb937d2d1' \
  'torchvision-0.23.0-cp310-cp310-linux_aarch64.whl' \
  | sha256sum -c -
```

두 줄 모두 `OK`인 경우에만 고정한 runtime dependency와 wheel을 설치한다. PyPI의
일반 `torch` 또는 `torchvision` package로 교체하거나 upgrade하지 않는다.

```bash
python3 -m pip install --user \
  numpy==1.26.1 Pillow==11.3.0 \
  filelock==3.32.2 typing-extensions==4.16.0 \
  sympy==1.14.0 mpmath==1.3.0 networkx==3.4.2 \
  Jinja2==3.1.6 MarkupSafe==3.0.3 fsspec==2026.7.0
python3 -m pip install --user --no-deps \
  "${CLEANY_TORCH_WHEEL_DIR}/torch-2.8.0-cp310-cp310-linux_aarch64.whl" \
  "${CLEANY_TORCH_WHEEL_DIR}/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl"
```

설치 후 새 terminal에서 CUDA 연산, torchvision import, ROS의 NumPy bridge를 함께
확인한다.

```bash
cd /path/to/cleany
python3 tools/jetson_preflight.py --check --require-torch \
  --output /tmp/cleany-jetson-pytorch.json
python3 -m json.tool /tmp/cleany-jetson-pytorch.json
python3 -c 'import torch, torchvision; print(torch.__version__, torchvision.__version__)'
python3 -c 'import cv2, numpy; from cv_bridge import CvBridge; print(cv2.__version__, numpy.__version__)'
```

검증된 임시 설치에서는 CUDA `12.6`, device `Orin`, CPU/GPU 행렬 연산 최대 절대 오차
`0.0`을 확인했다. 실제 user-site 설치 결과는 preflight JSON과 `pip show` 출력으로 다시
기록한다.

#### SAM2 설치

SAM2는 공식 저장소의 고정 commit과 별도 checkpoint가 필요하다. 공식 설치 과정은
PyTorch와 torchvision을 업그레이드할 수 있으므로 Jetson에서는 위 조합을 먼저 검증한
뒤 `--no-deps`로 설치한다. `make deps`는 PyTorch 또는 SAM2를 자동 설치하지 않는다.
아래 `SAM2_BUILD_CUDA=0`은 선택적인 SAM2 custom extension만 생략하며 PyTorch model
추론 device는 계속 CUDA를 사용한다.

```bash
git clone https://github.com/facebookresearch/sam2.git \
  /home/cleany/third_party/sam2
git -C /home/cleany/third_party/sam2 checkout --detach \
  2b90b9f5ceec907a1c18123530e92e794ad901a4
python3 -m pip install --user \
  hydra-core==1.3.2 iopath==0.1.10 tqdm==4.67.1
SAM2_BUILD_CUDA=0 python3 -m pip install --user --no-deps \
  --no-build-isolation -e /home/cleany/third_party/sam2

mkdir -p /home/cleany/models/sam2
curl -fL --retry 3 \
  -o /home/cleany/models/sam2/sam2.1_hiera_small.pt \
  https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt
printf '%s  %s\n' \
  '6d1aa6f30de5c92224f8172114de081d104bbd23dd9dc5c58996f0cad5dc4d38' \
  '/home/cleany/models/sam2/sam2.1_hiera_small.pt' \
  | sha256sum -c -
```

2026-08-13 검증 기준은 SAM2 commit
`2b90b9f5ceec907a1c18123530e92e794ad901a4`와 `sam2.1_hiera_small.pt`
checkpoint다. checkpoint 크기는 `184416285` bytes, SHA-256은 위 명령에 기록한
`6d1aa6f3...d38`이다. 모델 weight와 외부 checkout은 이 저장소에 넣지 않는다.

설치와 checkpoint 검증 뒤 실제 small model을 CUDA에 load하고 640x480 synthetic RGB에
bbox prompt를 주어 non-empty mask와 해상도 일치를 검사한다. 결과 JSON에는 model load와
inference 시간, peak CUDA allocation도 기록된다.

```bash
cd /home/cleany/cleany
python3 tools/sam2_smoke.py \
  --checkpoint /home/cleany/models/sam2/sam2.1_hiera_small.pt \
  --output /tmp/cleany-sam2-smoke.json
```

종료 코드가 0이고 `success`가 `true`, `device_name`이 `Orin`, `mask_shape`가
`[480, 640]`, `mask_pixels`가 0보다 커야 통과다. 첫 실행은 checkpoint load와 CUDA
초기화 때문에 후속 실행보다 오래 걸릴 수 있다.

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
make test-gazebo
```

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
렌더링 sensor server는 OGRE2로 실행하고 GUI는 OGRE1을 사용한다. camera, LiDAR와
RViz 사용법은
[`cleany_gazebo_sim` README](../ros2_ws/src/cleany_gazebo_sim/README.md)의 Harmonic
절을 따른다.

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

## 참고 자료

- [ROS 2 Humble Ubuntu deb 설치](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 Humble 지원 플랫폼](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
- [ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source)
- [Node.js 다운로드](https://nodejs.org/en/download)
