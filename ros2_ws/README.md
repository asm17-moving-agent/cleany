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

`make test`는 13개 패키지를 빌드하고 C++/Python 검사와 패키지에 등록된
MoveIt mock runtime 검사, 개발 도구·vision container 계약 검사를 실행한다.
Python은 `pytest`를 명시적으로 선택하고 외부 플러그인 자동 로드를 차단한다.
CMake도 다시 configure해 이전 환경에서 빠졌던 pytest 등록을 복원한다.
카메라 runtime은 headless여도 유효한 X display가 필요하다. GUI 세션의
`DISPLAY`를 전달하거나 CI처럼 Xvfb 환경에서 실행한다.
`make test-mujoco`는 현재 장면과 공통 bridge의 `test/` 전체를 검사한다. 짧은 개별 검사는 아래 native pytest 명령을 사용한다.

변경한 영역부터 확인할 때는 타깃 테스트를 사용한다.

```bash
make test-mission
make test-manipulation-core
make test-manipulation
make test-mujoco
make test-grasp-pregrasp
make test-scene-mapping
make test-mujoco-observer
make test-grasp-pregrasp-runtime
make test-gazebo
```

`make test-manipulation-core`는 물체 하나의 `collect_trash` core를 가짜 시계로 검사한다.
`make test-manipulation`은 모의 Action 서버·조회·이벤트와 실제 DDS 통신,
프로세스 강제 종료 후 복구까지 검사한다. 실행·설정은
[`cleany_skill_executor` README](src/cleany_skill_executor/README.md#모의-manipulation-action)를 따른다.

RGB-D perception부터 스터디카페 집기·분류까지 변경할 때는
`make test-grasp-pregrasp`로 관련 unit/contract 테스트를 실행한다.
`make test-grasp-pregrasp-runtime`은 MoveIt mock backend에서 OMPL/Pilz 계획과
controller 실행을 검사한다. 이전 캔·박스·hand-eye 전용 MuJoCo 데모와
runtime 테스트는 제거했다. 전체 물리 수거 성공은 이 테스트의 검증 범위가 아니다.

`make test-scene-mapping`은 선택형 known-geometry OctoMap updater만 빌드하고
C++ geometry/TF guard 및 plugin 로딩을 검사한다. 이 검사도
`make test-grasp-pregrasp`에 포함된다. 실제 물체 집기/분리 수거 완료 검사는 아니다.
저장한 정지 상태 RGB-D/URDF fixture가 있으면
`make profile-scene-mask MASK_FIXTURE=/absolute/path/to/fixture`로 ROS node 없이
serial/분할 self-mask의 점 단위 일치와 처리 시간을 비교한다. fixture 계약과
robot-only 측정 범위는 `src/cleany_scene_mapping/README.md`를 따른다.

Hand-eye의 수학·ROS adapter·오프라인 dataset 검증은
[`cleany_handeye_calibration` README](src/cleany_handeye_calibration/README.md)를 따른다.
전용 MuJoCo calibration 실행 경로는 제거했다.

스터디카페 관찰 장면을 headless 모드로 실행한다.

```bash
make sim
```

스터디카페의 벽·책상·파티션·모니터 배치를 MuJoCo viewer에서
실행하려면 전용 target을 사용한다.

```bash
make sim-mujoco-study-cafe
```

YOLOE-seg + Gemini 상세 분류와 카메라 기반 충돌 지도를 포함한 기본 파이프라인은
레포 루트에서 `make sim-mujoco-pipeline`으로 실행한다. 모델 옵션 없이 공통
프로필을 로딩하며, 기본은 실제 팔 명령을 내리지 않는 plan-only다. 실행 환경에
`GEMINI_API_KEY`가 필요하고 RGB 영상은 검출 시 Google API로 전송된다.
모델/플러그인 준비는 `docs/DEVELOPMENT_SETUP.md`, 동작과 한계는
`src/cleany_skill_executor/README.md`의 센서 전용 항목을 따른다.

로봇 내부 후면 받침판 위 좌측 분실물함/우측 쓰레기함과 분류·집기·놓기 흐름은
`make sim-mujoco-sorting`으로 실행한다. 시뮬레이션에서만 실제 관절 명령을
보내는 통합 검증 경로이며, 두 종류의 물리적 수거 성공은 검증 진행 중이다.
기본 인식은 YOLOE-seg의 객체·mask와 Gemini의 상세 라벨·분류를 사용한다.
파지 뒤 확인은 선택한 실행 모드의 head RGB-D 재검출 또는 YOLOE-seg 손목 CHECK로 수행한다.
GUI 없이 실행하려면 `DISPLAY=:0 make sim-mujoco-sorting
SORTING_ARGS='headless:=true use_rviz:=false use_image_view:=false'`를 사용한다.
현재 vendor 카메라 렌더링은 headless에서도 사용 가능한 X display가 필요하다.
외부 GPU perception 사용은
`start_perception:=false`를 추가한다. 외부 노드도 같은 tracking/시계 설정으로
실행한다. 현재 배치에는 중앙 인계 구역이 없다. 기본은 작업자 관찰 모드로
`complete_unverified` 및 `mission_complete_unverified`를 발행한다.
`SORTING_ARGS='sorting_verify_placement:=true'`로 독립 배치 검증을 켠다.
현재 `robot_top_bins.yaml`은 시뮬레이션 기둥 충돌 제외 설정을 사용한다.
개별 배치 성공 기록과 전체 수거 성공은 구분하며 자세한 동작·제한은
[`cleany_skill_executor` README](src/cleany_skill_executor/README.md)를 따른다.

고정 베이스 성능 비교는 `make profile-mujoco-tabletop`을 사용한다.
`sim_performance_profile:=tabletop_fast`는 선택형이며 기본은 `baseline`이다.
프로필은 먼 정적 배경 충돌·그림자 설정만 바꾸고 주행에는 사용하지 않는다.

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
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/cleany_mujoco_sim/test/test_scene_loader.py
colcon test --python-testing pytest
colcon test-result --verbose
ros2 launch cleany_mujoco_sim mujoco_study_cafe.launch.py headless:=true
```

`source install/setup.bash`는 build 후 같은 terminal session에서 실행한다.

## 선택 개발도구

Helix 프로젝트 설정은 VM의 native `pyright-langserver`를 사용한다. Helix에서
Pyright를 사용하려면 [개발환경 설치 가이드](../docs/DEVELOPMENT_SETUP.md)의 선택
개발도구 절을 따른다.

패키지별 topic, launch parameter, 추가 검증 명령은 각 패키지 `README.md`를 따른다.
