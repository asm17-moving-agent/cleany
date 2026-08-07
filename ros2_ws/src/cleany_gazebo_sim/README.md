# cleany_gazebo_sim

Gazebo Fortress 기반의 Cleany mobile-base simulation backend입니다. 이 패키지는
MuJoCo의 motor-voltage dynamics를 복제하지 않고, ROS 차체 속도 계약의
`cmd_vel -> odom / TF` 경계를 headless에서 검증하는 데 초점을 둡니다.

## Scope

- Gazebo `MecanumDrive` system을 이용한 `linear.x`, `linear.y`, `angular.z` 이동
- `cmd_vel` 유효성 검사, 속도 제한, command timeout 정지
- `/clock`, `/odom`, `/joint_states`, `odom -> base_link` TF bridge
- MuJoCo의 head RGBD와 좌·우 wrist RGB camera image bridge
- RPLIDAR A1 후보 사양의 GPU LiDAR와 ROS `/scan` bridge
- `base_link +X`를 camera-forward 전면으로 사용하는 canonical 4-wheel/arm 배치
- controller UI와 `joint_states`에는 4개의 drive wheel joint만 노출

좌표와 회전은 ROS REP-103을 따릅니다. `base_link`는 `+X` 전방, `+Y` 좌측,
`+Z` 상방인 오른손 좌표계이며 양의 yaw는 위에서 볼 때 반시계 방향입니다.
wheel joint axis는 `base_link +Y`로 명시되어 양의 wheel 회전이 `+X` 전진을
뜻합니다. camera image의 frame id는 REP-103 `_optical_frame`
(`+X` right, `+Y` down, `+Z` forward) 이름을 사용합니다.

각 wheel은 12개의 고정 capsule roller visual을 사용합니다. 실제 접촉은 Gazebo
Fortress의 mecanum 예제와 같은 diagonal anisotropic friction sphere로 단순화해,
48개의 passive roller joint를 GUI나 ROS interface에 노출하지 않습니다.

Gazebo world는 `cleany_description/meshes/`를 resource path로 참조해 팀의
Cleany/RASKOG base, dual-arm, gripper visual mesh를 재사용합니다. arm/gripper의
joint pose, axis, limit과 extended-link mass/center-of-mass/full inertia tensor,
collision mesh도 Cleany description에서 가져왔습니다. 다만 arm/gripper controller가 아직
없어 arm link의 gravity는 비활성화한 상태입니다. Fortress와 Harmonic 기본 profile은
네 개의 camera image와 GPU LiDAR `/scan` bridge를 포함합니다. Nav2, MoveIt,
Mission Manager integration은 아직 포함하지 않습니다.

## Dependencies

Ubuntu 22.04 / ROS 2 Humble에서 Gazebo Fortress와 ROS bridge가 필요합니다.
새 machine의 ROS 설치와 rosdep 초기화는
[`docs/DEVELOPMENT_SETUP.md`](../../../docs/DEVELOPMENT_SETUP.md)를 먼저 따릅니다.
그다음 저장소 루트에서 Gazebo 관련 의존성만 설치하고 환경을 확인합니다.

```bash
make deps-gazebo
make check-gazebo-env
```

환경 검사는 Ubuntu 22.04, ROS 2 Humble, Python 3.10, Ignition Gazebo 6.x,
`ros_gz_sim`, `ros_gz_bridge`를 확인합니다. rosdep 문제를 진단할 때 사용할 APT
fallback은 개발환경 설치 가이드에 정리되어 있습니다.

## Build and validation

다음 명령은 `cleany_description`과 `cleany_gazebo_sim`까지만 빌드하고 패키지의
parameter, Fortress world structure, Harmonic profile isolation test를 실행합니다.

```bash
make test-gazebo
```

모든 pytest가 통과해야 하며, 생성된 mecanum wheel world가 canonical description의
link, joint, mesh 구조를 유지하는지도 함께 검사합니다.

네 개 camera와 GPU LiDAR를 실제로 실행해 RTF와 sensor 수신 주기를 측정하는 테스트는
일반 test suite와 분리된 opt-in test입니다. 먼저 해당 profile을 build한 뒤, 준비된
환경 안에서 실행합니다.

Fortress/Humble:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
source install/setup.bash
python3 -m pytest -s \
  src/cleany_gazebo_sim/test/test_runtime_sensor_performance.py \
  --run-sim-runtime --sim-profile=fortress
```

Harmonic/Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install-harmonic/setup.bash
python3 -m pytest -s \
  src/cleany_gazebo_sim/test/test_runtime_sensor_performance.py \
  --run-sim-runtime --sim-profile=harmonic
```

기본값은 10초 warm-up과 30초 측정이며 `--warmup-sec`, `--measure-sec`,
`--startup-timeout-sec`로 조절합니다. test는 `/clock`, camera 네 topic, `/scan`을 모두
수신하고 simulation time이 전진하는지를 검사하며 RTF, wall Hz, simulation Hz를
출력합니다. 원본 world를 변경하지 않고 명암 줄무늬가 있는 네 벽을 추가한 임시
validation world를 생성하며, warm-up 중 저속 이동 후 camera 해상도·encoding·frame ID·timestamp,
빈/단색 frame 여부와 frame 변화, LiDAR의 360개 range·유한 장애물 거리·선언 범위를
검사합니다. 성능 기준도 실패 조건으로 사용할 때만 `--min-rtf`,
`--min-camera-sim-hz`, `--min-lidar-sim-hz`를 지정합니다. 기본 `make test-gazebo`에는
실제 simulator를 띄우는 이 test가 포함되지 않습니다.

## Run

저장소 루트에서 실행합니다.

```bash
make sim-gazebo
```

이 명령은 활성 ROS와 Gazebo version을 검사하고 `make build-gazebo`를 먼저 실행한 뒤,
선택된 Fortress 또는 Harmonic 서버를 GUI 없이 시작합니다. 종료할 때는 `Ctrl-C`를
누릅니다.

다른 terminal에서 명령을 보냅니다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.05}, angular: {z: 0.1}}'
```

다른 terminal에서 다음 항목을 확인합니다.

```bash
ros2 topic echo --once /clock
ros2 topic echo --once /odom
ros2 topic echo --once /joint_states
ros2 topic echo --once /scan
ros2 topic list | grep -E '^/(clock|gazebo_cmd_vel|gazebo_odom|joint_states|odom|scan|tf)$'
```

재현 성공 기준은 다음과 같습니다.

- simulator와 bridge process가 조기 종료하지 않는다.
- `/clock`, `/odom`, `/joint_states`, `/scan`에서 message를 한 개 이상 수신한다.
- `/cmd_vel`을 보내면 guard output `/gazebo_cmd_vel`이 발행되고 `/odom`이 변한다.
- `Ctrl-C`로 launch process가 종료된다.

GUI가 필요하면 build 후 직접 launch합니다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_gazebo_sim gazebo_sim.launch.py headless:=false
```

GUI와 camera sensor는 host의 OpenGL/OGRE 호환성에 영향을 받습니다. Fortress의
GPU LiDAR와 camera sensor server는 OGRE2로 실행하고 GUI는 OGRE1을 사용합니다.
Fortress의 server-only `-s`는 GUI만 끄며 rendering sensor가 있으면 server 내부에서
여전히 rendering context를 생성합니다.

headless 실행도 renderer 오류로 종료된다면 software rendering으로 같은 절차를
진단할 수 있습니다.

```bash
LIBGL_ALWAYS_SOFTWARE=1 make sim-gazebo
```

이 설정은 GPU driver 문제를 분리하기 위한 저속 fallback이며 팀 표준 실행 설정은
아닙니다.

## Runtime and build state

APT/rosdep package와 colcon output이 존재한다는 사실만으로 현재 runtime에
ROS/Gazebo package가 설치됐다고 판단하지 말고, 실행할 환경 안에서 다음 명령을
확인합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
```

Humble/Python 3.10의 `build/`, `install/`, `log/`는 다른 ROS 배포판이나 Python
version에서 재사용하지 않습니다. 세부 native 명령은 `ros2_ws/README.md`를
참고합니다.

## Optional Harmonic compatibility profile

팀의 재현성 기준은 위의 Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress 조합입니다.
별도 환경에서 Jazzy/Harmonic 구성이 필요할 때는 호환 프로필로 격리하며 팀 표준
환경을 대체하지 않습니다.

Harmonic용 파일은 다음처럼 명시적인 이름을 사용합니다.

- `launch/gazebo_harmonic.launch.py`
- `worlds/cleany_mecanum_harmonic.sdf`
- `config/bridge_harmonic.yaml`
- `config/lidar_bridge_harmonic.yaml`

ROS 2 Jazzy와 Gazebo Harmonic 환경 준비는
[`개발환경 설치 가이드`](../../../docs/DEVELOPMENT_SETUP.md#7-선택-ros-2-jazzy--gazebo-harmonic-호환-환경)를
따릅니다. 준비된 환경에서 저장소 루트의 공통 명령을 실행합니다.

```bash
make sim-gazebo
```

Jazzy와 Gazebo 8.x가 확인되면 Harmonic을 자동 선택하며, Fortress와 빌드 결과를
공유하지 않도록 `build-harmonic/`, `install-harmonic/`, `log-harmonic/`을 사용합니다.
자동 판정이 불가능하면 `GAZEBO_PROFILE=harmonic make sim-gazebo`처럼 명시할 수
있습니다. headless 서버 센서는 OGRE2, GUI 실행 시 서버는 OGRE2, GUI는 OGRE1을
사용합니다. Harmonic world의 렌더링 센서는 구독이 생기기 전까지 비활성화할 수
있도록 `always_on=false`로 정의되어 있습니다. 기본 `bridge_harmonic.yaml`은 모든
센서 topic을 bridge하므로 launch와 함께 모든 렌더링 센서가 활성화됩니다. LiDAR만
분리해서 확인할 때는
Fortress의 `lidar_bridge.yaml` 또는 Harmonic의 `lidar_bridge_harmonic.yaml`을 사용할
수 있습니다.
