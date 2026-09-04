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
make test-handeye
make test-grasp-pregrasp
make test-grasp-pregrasp-runtime
make test-gazebo
```

RGB-D perception부터 grasp 후보 생성과 MoveIt pre-grasp까지 변경할 때는
`make test-grasp-pregrasp`로 관련 unit/contract 테스트만 실행한다. 전체 MuJoCo,
주행과 hand-eye calibration 테스트는 포함하지 않는다. 실제 MuJoCo controller 실행은
시간이 더 걸리는 `make test-grasp-pregrasp-runtime`으로 별도 확인한다. runtime은
가장 가까운 객체 실패 후 다음 객체 fallback과 pre-grasp 정지를 검증한다. RGB-D 캔
GUI 데모는 OpenGL viewer가 필요하므로 자동 runtime target에 포함하지 않는다.

Hand-eye 패키지 경계만 빌드하려면 `make build-handeye`를 사용한다.
`make test-handeye`는 description, MuJoCo backend, MoveIt config와 calibration
패키지를 함께 검사하며 실제 runtime test는 자동으로 `headless:=true`를 사용한다.

MuJoCo에서 random pose 후보를 MoveIt/렌더링/PnP로 검증하고 axis parallelism과
rotation covariance를 분석해 20+5 artifact를 생성한 뒤 실제 calibration을 실행한다.

```bash
make handeye-generate-mujoco
make handeye-mujoco
make handeye-validate-mujoco
```

생성 target은 준비 단계이므로 headless이고 기본 artifact 경로를 materialize한다.
Calibration target은 operator가 motion과 target visibility를 확인할 수 있도록
`headless:=false`를 명시하며 MuJoCo viewer를 연다. Template의 `null` 값을 그대로
사용할 수 없고, pose preflight와 safety/timeouts 검토가 끝난 artifact만 허용한다.
마지막 target은 25개 row/image/hash, ChArUco/PnP 재현, provenance와 150-run
solver 결과를 `dataset_validation.json`으로 검증한다.

MuJoCo 시뮬레이터를 headless 모드로 실행한다.

```bash
make sim
```

Gazebo Fortress 시뮬레이터는 `make sim-gazebo`로 실행한다.

Gazebo만 새 환경에서 재현할 때는 아래 순서로 의존성, 기준 환경, 패키지 테스트,
headless 실행을 확인한다.

```bash
make deps-gazebo
make check-gazebo-env
make test-gazebo
make sim-gazebo
```

Gazebo Make target은 지원 환경인 Ubuntu 22.04, ROS 2 Humble과 Gazebo Fortress를
검사한다.
`make build-gazebo`와 `make test-gazebo`는 `cleany_gazebo_sim` 및 그 dependency까지만
빌드한다. 성공 판정 topic과 GUI 실행법은
[`cleany_gazebo_sim` README](src/cleany_gazebo_sim/README.md)를 따른다.

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

Hand-eye에 해당하는 native 세부 명령과 artifact schema는
[`cleany_handeye_calibration` README](src/cleany_handeye_calibration/README.md)를
따른다. 실제 calibration launch의 viewer 기본값은 `headless:=false`이고 테스트만
명시적으로 headless 모드를 사용한다.

`source install/setup.bash`는 build 후 같은 terminal session에서 실행한다.

## 선택 개발도구

Helix 프로젝트 설정은 VM의 native `pyright-langserver`를 사용한다. Helix에서
Pyright를 사용하려면 [개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md)의 선택
개발도구 절을 따른다.

패키지별 topic, launch parameter, 추가 검증 명령은 각 패키지 `README.md`를 따른다.
