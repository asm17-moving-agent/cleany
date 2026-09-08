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
make test-scene-mapping
make test-mujoco-observer
make test-grasp-pregrasp-runtime
make test-gazebo
```

RGB-D perception부터 grasp 후보 생성과 MoveIt pre-grasp까지 변경할 때는
`make test-grasp-pregrasp`로 관련 unit/contract 테스트만 실행한다. 전체 MuJoCo,
주행과 hand-eye calibration 테스트는 포함하지 않는다. 실제 MuJoCo controller 실행은
시간이 더 걸리는 `make test-grasp-pregrasp-runtime`으로 별도 확인한다. runtime은
OMPL과 Pilz LIN 동시 로딩 및 LIN FK 직선성, 가장 가까운 객체 실패 후 다음 객체
fallback과 pre-grasp 정지를 검증한다. RGB-D 캔
GUI 데모는 OpenGL viewer가 필요하므로 자동 runtime target에 포함하지 않는다.

`make test-scene-mapping`은 선택형 known-geometry OctoMap updater만 빌드하고
C++ geometry/TF guard 및 plugin 로딩을 검사한다. 이 검사도
`make test-grasp-pregrasp`에 포함된다. 실제 물체 집기/분리 수거 완료 검사는 아니다.
저장한 정지 상태 RGB-D/URDF fixture가 있으면
`make profile-scene-mask MASK_FIXTURE=/absolute/path/to/fixture`로 ROS node 없이
serial/분할 self-mask의 점 단위 일치와 처리 시간을 비교한다. fixture 계약과
robot-only 측정 범위는 `src/cleany_scene_mapping/README.md`를 따른다.

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

Gazebo study-cafe와 같은 벽·책상·파티션·모니터·의자 배치를 MuJoCo viewer에서
실행하려면 전용 target을 사용한다.

```bash
make sim-mujoco-study-cafe
```

Gemini Flash-Lite + 로컬 SAM2-tiny와 카메라 기반 충돌 지도를 포함한 기본 파이프라인은
레포 루트에서 `make sim-mujoco-pipeline`으로 실행한다. 모델 옵션 없이 공통
프로필을 로딩하며, 기본은 실제 팔 명령을 내리지 않는 plan-only다. 실행 환경에
`GEMINI_API_KEY`가 필요하고 RGB 영상은 검출 시 Google API로 전송된다.
모델/플러그인 준비는 `docs/DEVELOPMENT_SETUP.md`, 동작과 한계는
`src/cleany_skill_executor/README.md`의 센서 전용 항목을 따른다.

로봇 후면 선반 위 좌측 분실물함/우측 쓰레기함과 분류·집기·놓기 흐름은
`make sim-mujoco-sorting`으로 실행한다. 시뮬레이션에서만 실제 관절 명령을
보내는 통합 검증 경로이며, 두 종류의 물리적 수거 성공은 검증 진행 중이다.
GUI 없이 실행하려면 `DISPLAY=:0 make sim-mujoco-sorting
SORTING_ARGS='headless:=true use_rviz:=false use_image_view:=false'`를 사용한다.
현재 vendor 카메라 렌더링은 headless에서도 사용 가능한 X display가 필요하다.
tracking 중단은 `sam2_tracking_enabled:=false`, 외부 GPU perception 사용은
`start_perception:=false`를 추가한다. 외부 노드도 같은 tracking/시계 설정으로
실행한다. 현재 배치에는 중앙 인계 구역이 없고 후면 운반 성공은 아직 미검증이다.

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
