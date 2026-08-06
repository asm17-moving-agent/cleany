# cleany_gazebo_sim

Gazebo Fortress 기반의 Cleany mobile-base simulation backend입니다. 이 패키지는
MuJoCo의 motor-voltage dynamics를 복제하지 않고, ROS 차체 속도 계약의
`cmd_vel -> odom / TF` 경계를 headless에서 검증하는 데 초점을 둡니다.

## Scope

- Gazebo `MecanumDrive` system을 이용한 `linear.x`, `linear.y`, `angular.z` 이동
- `cmd_vel` 유효성 검사, 속도 제한, command timeout 정지
- `/clock`, `/odom`, `/joint_states`, `odom -> base_link` TF bridge
- `base_link -> lidar_link / imu_link` static sensor TF
- MuJoCo의 head RGBD와 좌·우 wrist RGB camera image bridge
- RPLIDAR A1 후보 사양의 GPU LiDAR와 ROS `/scan` bridge
- base-aligned simulation IMU와 ROS `/imu/data` bridge
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
네 개의 camera image, GPU LiDAR `/scan`, IMU `/imu/data` bridge를 포함합니다. Nav2, MoveIt,
Mission Manager integration은 아직 포함하지 않습니다.

## Simulation IMU contract

Gazebo는 `/model/cleany_mecanum/imu`를 발행하고 기본 bridge가 이를
`sensor_msgs/msg/Imu` ROS topic `/imu/data`로 전달합니다. `header.frame_id`는
`imu_link`이고 update rate는 50 Hz입니다. `imu_link`는 현재 `base_link`와 같은
위치·방향으로 고정되어 있으며, SDF에 별도 stochastic noise 또는 bias 모델은
설정하지 않았습니다. 이 값은 LiDAR·IMU·TF 시뮬레이션 검증을 위한 후보값이며
실제 하드웨어 실장 위치와 noise 모델은 추후 하드웨어 검토에서 확정합니다.

## TF ownership

Gazebo odometry adapter가 동적 `odom -> base_link`를 발행하고, Gazebo sensor TF
publisher가 고정 `base_link -> lidar_link`와 `base_link -> imu_link`를
`/tf_static`에 발행합니다. Gazebo SDF의 fixed joint와 ROS static TF parameter는
구조 test에서 같은 값인지 검사합니다. 현재 sensor mount는 simulation 후보값이며
hardware description의 확정 mount로 취급하지 않습니다.

## Environment

팀 표준인 Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 환경과 선택적인
Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic 환경의 설치·의존성·renderer 진단은
[`DEVELOPMENT_SETUP.md`](../../../docs/DEVELOPMENT_SETUP.md)를 따른다. 이 README는
준비된 환경에서의 Gazebo backend 계약과 실행·검증만 다룬다.

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

LiDAR, IMU, odometry, TF만 검증하는 navigation runtime test는 저장소 루트에서
다음 명령으로 실행합니다. 활성 ROS/Gazebo 환경을 감지해 Fortress 또는 Harmonic
profile을 선택하고, 카메라 bridge는 시작하지 않습니다.

```bash
make test-gazebo-nav-runtime
```

이 test도 기본값으로 10초 warm-up 후 30초를 측정합니다. 임시 world에 네 개의
벽을 추가하고 카메라 sensor를 끈 뒤 `/scan`, `/imu/data`, `/odom`, `/clock`을
동시에 관찰합니다. 측정 초반에는 `cmd_vel`을 보내 LiDAR와 IMU가 장착된 로봇의
odometry가 실제로 변하는지 확인합니다. 다음 조건을 모두 만족해야 통과합니다.

- LiDAR의 frame, timestamp, 360개 range, 선언 범위와 장애물 거리 분포가 유효함
- 거의 모든 LiDAR 광선이 `range_min`에 붙는 self-hit 상태가 아님
- IMU의 frame, timestamp, quaternion, 중력 크기와 회전 명령 응답이 유효함
- `base_link -> lidar_link`, `base_link -> imu_link` static TF와
  `odom`까지 이어지는 TF chain이 유효함
- 측정 중 simulation time과 모든 필수 topic이 진행됨

실패 기준으로 성능 하한도 적용하려면 `--min-rtf`, `--min-lidar-sim-hz`,
`--min-imu-sim-hz`를 pytest 직접 실행 시 지정할 수 있습니다. GPU LiDAR는
headless 실행에서도 rendering sensor이므로, 선택한 Gazebo profile에서 동작하는
OpenGL display 또는 headless rendering 환경이 필요합니다.

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
ros2 topic echo --once /imu/data
ros2 run tf2_ros tf2_echo odom lidar_link
ros2 run tf2_ros tf2_echo odom imu_link
ros2 topic list | grep -E '^/(clock|gazebo_cmd_vel|gazebo_odom|imu/data|joint_states|odom|scan|tf|tf_static)$'
```

재현 성공 기준은 다음과 같습니다.

- simulator와 bridge process가 조기 종료하지 않는다.
- `/clock`, `/odom`, `/joint_states`, `/scan`, `/imu/data`에서 message를 수신한다.
- `odom -> base_link -> lidar_link / imu_link` TF lookup이 성공한다.
- `/cmd_vel`을 보내면 guard output `/gazebo_cmd_vel`이 발행되고 `/odom`이 변한다.
- `Ctrl-C`로 launch process가 종료된다.

GUI가 필요하면 build 후 직접 launch합니다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_gazebo_sim gazebo_sim.launch.py headless:=false
```

## Profiles

`make sim-gazebo`와 `make test-gazebo`는 활성 ROS 배포판과 Gazebo major version으로
Fortress 또는 Harmonic profile을 선택한다. 팀 표준은 Fortress이며 Harmonic은 호환
profile이다. 환경 준비와 자동 판정·output 분리 규칙은
[`DEVELOPMENT_SETUP.md`](../../../docs/DEVELOPMENT_SETUP.md)를 따른다.

Harmonic 서버는 OGRE2를 사용한다. GUI를 함께 실행할 때도 server는 OGRE2, GUI는
OGRE1을 사용한다. Harmonic world의 rendering sensor는 구독 전까지 비활성화할 수
있지만, 기본 bridge는 모든 sensor topic을 bridge한다.
