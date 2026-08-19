# ROS 2 Workspace

Cleany의 로봇 엣지 시스템 코드가 들어가는 ROS 2 workspace다.

패키지는 `src/` 아래에 두고, 공통 인터페이스는 `cleany_interfaces`에서 먼저 정의한다.

## 개발 환경

공식 개발환경은 Ubuntu 22.04 VM과 ROS 2 Humble native 환경이다. ROS 2 Humble이
설치되어 있고 `/opt/ros/humble/setup.bash`를 사용할 수 있어야 한다.

Jetson의 perception/AnyGrasp CUDA dependency는 선택적으로 분리된 GPU container에
격리한다. AnyGrasp만 license/model과 고정 MAC을 가지며 안전·제어·mission stack은
native에 남는다. 이는 공통 native 개발환경을 대체하지 않으며 사용법은
[`containers/vision`](../containers/vision/README.md)을 따른다.

새 VM을 준비하는 전체 절차는
[개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md)를 따른다.

MuJoCo용 custom rosdep 규칙을 처음 등록하는 방법은
[rosdep 규칙 안내](rosdep/README.md)를 따른다.

## 빠른 시작

아래 명령은 레포지토리 루트에서 실행한다.

```bash
make deps
make build
make test
```

변경한 영역부터 확인할 때는 타깃 테스트를 사용한다.

```bash
make test-mission
make test-mujoco
make test-gazebo
```

MuJoCo 시뮬레이터를 headless 모드로 실행한다.

```bash
make sim
```

Gazebo 시뮬레이터는 `make sim-gazebo`로 실행한다.

Gazebo만 새 환경에서 재현할 때는 아래 순서로 의존성, 기준 환경, 패키지 테스트,
headless 실행을 확인한다.

```bash
make deps-gazebo
make check-gazebo-env
make test-gazebo
make sim-gazebo
```

Gazebo Make target은 활성 `ROS_DISTRO`와 Gazebo major version을 함께 검사해
Humble/Fortress 또는 Jazzy/Harmonic profile을 선택한다. `make build-gazebo`와
`make test-gazebo`는 선택된 profile로 `cleany_gazebo_sim` 및 그 dependency까지만
빌드한다. 성공 판정 topic과 GUI 실행법은
[`cleany_gazebo_sim` README](src/cleany_gazebo_sim/README.md)를 따른다.

ROS 2 Jazzy / Gazebo Harmonic 호환 환경은 팀 표준과 분리한다. 환경 준비는
[개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md#7-선택-ros-2-jazzy--gazebo-harmonic-호환-환경)를
따른다. 두 profile 모두 `make test-gazebo`와 `make sim-gazebo`를 사용하며 Harmonic은
자동으로 전용 output을 사용한다. 자동 판정이 불가능하면
`GAZEBO_PROFILE=fortress|harmonic`으로 명시할 수 있다. 활성 `ROS_DISTRO`와 충돌하는
override는 오류로 종료한다.

지원하는 전체 작업은 `make help`로 확인한다.

## Native 표준 명령

Makefile은 아래 native 명령을 짧게 제공할 뿐 `colcon`, `pytest`, `ros2`를 대체하지
않는다. 직접 실행하거나 문제를 진단할 때는 다음 흐름을 사용한다.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/cleany_mujoco_sim/test/test_scene_loader.py
colcon test
colcon test-result --verbose
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=true
```

`source install/setup.bash`는 build 후 같은 terminal session에서 실행한다.

## 선택 개발도구

Helix 프로젝트 설정은 VM의 native `pyright-langserver`를 사용한다. Helix에서
Pyright를 사용하려면 [개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md)의 선택
개발도구 절을 따른다.

패키지별 topic, launch parameter, 추가 검증 명령은 각 패키지 `README.md`를 따른다.
