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
collision mesh도 Cleany description에서 가져왔습니다. 조작용 arm controller는 아직
없지만, 양팔은 world materialization 시 어깨를 안쪽으로 돌리고 팔꿈치를 접은 대기 자세로
고정됩니다. 시작 후 관절 제어기로 자세를 이동하지 않으므로 자유 상태의 mobile base에
팔 구동 반작용이 전달되지 않습니다. 이 고정은 SLAM 주행 중 팔 자세를 유지하기 위한
kinematic 잠금입니다. 대기 자세는 좌·우 shoulder
yaw `-1.5708`/`1.5708`, shoulder pitch `3.0`, elbow pitch `2.4`, wrist pitch
`1.2`, wrist roll `0.0`, gripper `0.8` rad입니다. 물리 servo effort는
모사하지 않습니다. 현재 arm link의 gravity는
비활성화한 상태입니다. Fortress와 Harmonic launch는
공통 IMU `/imu/data` bridge와 필요한 rendering sensor bridge만 실행하는 sensor
profile을 제공합니다. 기본값은 GPU LiDAR `/scan`만 추가로 활성화하는
`lidar_nav`입니다. 2D mapping용 `slam_toolbox` profile은 제공하지만 Nav2 navigation,
MoveIt, Mission Manager integration은 아직 포함하지 않습니다.

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

`MecanumDrive` odometry는 `/gazebo_odom`을 거쳐 ROS `/odom`과
`odom -> base_link`를 소유합니다. 별도 `OdometryPublisher`의 simulator ground truth는
`/ground_truth/odom`으로만 bridge하며 TF를 발행하지 않습니다. 두 source를 같은
odometry topic에 섞지 않아 RViz와 SLAM의 기준 frame이 교대로 점프하지 않게 합니다.

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

선택한 sensor profile을 실제로 실행해 RTF와 sensor 수신 주기를 측정하는 테스트는
일반 test suite와 분리된 opt-in test입니다. 먼저 해당 Gazebo profile을 build한 뒤,
준비된 환경 안에서 실행합니다.

Fortress/Humble:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
source install/setup.bash
python3 -m pytest -s \
  src/cleany_gazebo_sim/test/test_runtime_sensor_performance.py \
  --run-sim-runtime --sim-profile=fortress \
  --sensor-profile=all_cameras
```

Harmonic/Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install-harmonic/setup.bash
python3 -m pytest -s \
  src/cleany_gazebo_sim/test/test_runtime_sensor_performance.py \
  --run-sim-runtime --sim-profile=harmonic \
  --sensor-profile=all_cameras
```

기본값은 10초 warm-up과 30초 측정이며 `--warmup-sec`, `--measure-sec`,
`--startup-timeout-sec`로 조절합니다. `--sensor-profile` 기본값은
`all_cameras`이며 다른 launch profile도 동일하게 선택할 수 있습니다. test는
`/clock`과 선택한 sensor topic을 수신하고 비활성 sensor topic에서는 message가 오지
않는지 검사하며 RTF, wall Hz, simulation Hz를 출력합니다. 원본 world를 변경하지
않고 명암 줄무늬가 있는 네 벽을 추가한 임시 validation world를 생성하며, warm-up 중
저속 이동 후 선택한 camera의 해상도·encoding·frame ID·timestamp, 빈/단색 frame
여부와 frame 변화를 검사합니다. `lidar_nav`에서는 LiDAR의 360개 range·유한 장애물
거리·선언 범위를 검사합니다. 성능 기준도 실패 조건으로 사용할 때만 `--min-rtf`,
`--min-camera-sim-hz`, `--min-lidar-sim-hz`를 지정합니다. 기본
`make test-gazebo`에는 실제 simulator를 띄우는 이 test가 포함되지 않습니다.

## Sensor profiles

두 Gazebo launch는 `sensor_profile` argument로 rendering sensor 부하를 선택합니다.
차체 명령, odometry, joint state, clock, IMU bridge는 모든 profile에서 실행됩니다.
`bridge_config`를 지정하면 sensor profile 대신 해당 단일 bridge 설정을 사용합니다.

| Profile | LiDAR scans | Head RGB | Head depth | Left wrist | Right wrist |
| --- | --- | --- | --- | --- | --- |
| `lidar_nav` (기본값) | O | X | X | X | X |
| `head_rgbd` | X | O | O | X | X |
| `left_wrist` | X | X | X | O | X |
| `right_wrist` | X | X | X | X | O |
| `all_cameras` | X | O | O | O | O |

`all_cameras`는 head color/depth와 좌·우 wrist color를 합친 네 image stream의
부하 profile입니다. 선택되지 않은 rendering sensor는 bridge 구독자를 만들지 않으며,
world의 `always_on=false` 설정과 함께 lazy 상태를 유지합니다.

Fortress/Humble에서 profile을 직접 선택하는 예시는 다음과 같습니다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_gazebo_sim gazebo_sim.launch.py \
  headless:=true sensor_profile:=head_rgbd
```

Harmonic/Jazzy에서는 launch 파일과 install 경로를 호환 profile에 맞춥니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-harmonic/setup.bash
ros2 launch cleany_gazebo_sim gazebo_harmonic.launch.py \
  headless:=true sensor_profile:=all_cameras
```

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

## Demo study-room evaluation world

`gazebo_study_cafe.launch.py`는 실제 시연실 좌석도를 단순화한 48석 평가 공간을
제공합니다. 벽 안쪽 크기는 12.26×10.94 m이며 로봇은 남쪽의 왼쪽 세로 통로
`(x=-1.865, y=-4.705, yaw=1.5708)`에 배치됩니다. 여덟 책상 열은 3-2-3 블록으로
나뉘고 여섯 행은 두 행씩 마주 붙은 세 묶음으로 배치됩니다. 각 행의 첫째와 마지막
책상 옆면은 서쪽·동쪽 벽면에 닿지만 첫째·마지막 행의 앞뒤는 벽에서 떨어져 있습니다.

개별 책상은 1.2×0.77 m이고 흰색 상판 최고점은 바닥에서 0.72 m입니다. 파티션에 닿는
두 모서리는 직각이고 의자 쪽 두 모서리는 반경 0.06 m로 둥글게 구성합니다. 흰색
A형 다리는 상판 좌우 및 앞뒤 가장자리에서 0.08 m 안쪽에 발을 두고 상부 중앙으로
모이며, 상단 crossbar를 포함한 visual과 collision이 같은 형상을 사용합니다. 마주
붙은 두 행의 중앙 파티션은 바닥 0.30 m에서 상판 위 0.30 m인 1.02 m까지 이어지며
네 모서리는 반경 0.05 m로 둥글게 처리합니다. 벽면은 흰색, roughness 0.92,
metalness 0.0의 무광 석고 재질입니다. 의자 좌판 앞쪽은 상판 끝과 0.23 m 겹치도록
책상 아래로 들어갑니다. 같은 행에서 떨어진 3-2-3 블록의 상판 사이 간격은 1.33 m,
서로 붙지 않은 다른 행의 상판 사이 간격은 1.63 m입니다. 배치된 의자 등판 사이의
실제 통과 폭은 0.83 m입니다. 가장 가까운 행의 상판 가장자리와 남북 벽면 사이는
1.53 m이고, 의자 등판과 벽 사이 실제 통로는 1.13 m이므로 로봇 주행 경로를 만들기
전 의자를 포함한 통과 가능성을 별도로 확인해야 합니다.

각 책상에는 검정색 27인치 16:9 모니터가 하나씩 배치됩니다. 화면 크기는
0.598×0.336 m이며 패널 중심은 의자 반대편 상판 끝에서 0.10 m 안쪽에 있습니다.
화면 면은 배정된 의자를 향하고, 패널·스탠드·받침대는 각각 primitive collision을
사용합니다.

의자 visual은 CC BY 4.0의 OpenRobotics Gazebo Fuel `OfficeChairGrey`를 0.9배로
사용합니다. 의자 yaw는 local `+X` 정면이 배정된 책상을 향하도록 계산하며 collision은
캐스터 영역, 중앙축, 좌판, 등판 primitive로 분리합니다. 최초 실행 시 Fuel asset
다운로드를 위해 network가 필요하고 이후에는 Gazebo cache를 사용합니다.

로컬 Jazzy/Harmonic Distrobox에서 GUI 배율 1.0으로 실행합니다.

```bash
make sim-gazebo-study-cafe
```

## 2D SLAM candidate profile

`slam_toolbox` online async profile은 SCRUM-315 비교 실험을 시작하기 위한 첫 후보이며
선정 결과가 아닙니다. 현재 simulation의 `/scan`과 `odom -> base_link`를 입력으로
사용하고 `map -> odom`과 occupancy grid를 발행합니다. Cartographer와 RTAB-Map 등
다른 후보에 동일한 sensor recording을 재생해 정확도·지도 품질·실시간성을 측정한 뒤
사용자가 결과를 검토해 최종 알고리즘을 결정합니다.

후보 parameter는 `config/slam_toolbox.yaml`에 있습니다. Ceres scan matcher와
Huber loss, loop closure를 사용하며 LiDAR 범위는 simulation 계약과 같은
0.15–12 m입니다. `/imu/data`는 `slam_toolbox`에 직접 연결하지 않습니다. 추후 IMU
융합이 필요하면 `robot_localization` 등에서 `odom -> base_link` 추정을 개선한 뒤 같은
SLAM 입력 계약을 유지합니다.

Gazebo를 실행한 상태에서 다른 terminal에 SLAM node를 시작합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-harmonic/setup.bash
ros2 launch cleany_gazebo_sim slam_mapping.launch.py
```

반복 구조에서 잘못된 loop closure가 발생하는지 분리해서 확인할 때는 baseline
설정 파일을 바꾸지 않고 launch override를 사용합니다.

```bash
ros2 launch cleany_gazebo_sim slam_mapping.launch.py do_loop_closing:=false
```

Loop closure를 유지하면서 반복 구조에 더 보수적인 진단 조건을 적용할 수도 있습니다.

```bash
ros2 launch cleany_gazebo_sim slam_mapping.launch.py \
  loop_search_maximum_distance:=2.0 \
  loop_search_space_dimension:=4.0 \
  loop_match_minimum_response_coarse:=0.50 \
  loop_match_minimum_response_fine:=0.60
```

launch는 `slam_toolbox` lifecycle node를 자동으로 configure·activate합니다.
지도와 scan을 함께 보려면 다음 RViz launch를 사용합니다. ARM Adreno Mesa에서
RViz의 indexed-palette Map shader가 실패하는 문제를 피하기 위해 `/map`의 점유 셀을
표준 `Marker`로 변환해 표시합니다. 원본 `/map` topic과 저장 결과는 바꾸지 않습니다.

```bash
ros2 launch cleany_gazebo_sim slam_visualization.launch.py
```

Harmonic에서 카메라와 다른 높이 LiDAR를 모두 비활성화하고 12 cm LiDAR 하나만
표준 `/scan`으로 노출하려면 study-cafe launch에
`bridge_config:=.../slam_<height>cm_bridge_harmonic.yaml`을 전달합니다. 12, 26,
45, 70 cm 높이별 전용 bridge는 선택한 LiDAR 하나만 표준 `/scan`으로 노출하고
이 전용 bridge는
비교 실험 중 사용하지 않는 GPU sensor에 subscriber를 만들지 않습니다.
headless 가속 실험은 `physics_max_step_size:=0.003`과
`physics_real_time_factor:=2.0`처럼 launch 시 world physics에 적용합니다. 실행 중
`set_physics`로 부분 갱신하면 `enable_physics` 기본값 때문에 동역학이 꺼질 수 있으므로
평가 실행에는 사용하지 않습니다.

`ros2 topic echo --once /map`과 `ros2 run tf2_ros tf2_echo map base_link`로 map 및
TF chain을 확인할 수 있습니다.

## LiDAR mount SLAM evaluation

SCRUM-316 평가 후보는 `config/lidar_mount_profiles.yaml`의 `floor_12cm`,
`floor_26cm`, `floor_45cm`, `floor_70cm` 네 가지입니다. 모두 `base_link +X=0.16 m`를
유지하고 scan 중심을 바닥에서 0.12, 0.26, 0.45, 0.70 m 높이에 둡니다. 상대 Z는
각각 `-0.26 m`, `-0.12 m`, `+0.07 m`, `+0.32 m`이며 실제 하드웨어 실장 치수로
확정된 값이 아닙니다. 평가 준비 도구는
선택 위치를 Gazebo SDF와
`base_link -> lidar_link` static TF 설정에 동시에 반영해 서로 다른 pose가 섞이는 것을
방지합니다.

네 후보를 한 화면에서 비교할 때는 기본 world에 네 LiDAR를 동시에 장착합니다. ROS
topic은 `/scan_12cm`, `/scan`, `/scan_45cm`, `/scan_70cm`이며 26 cm의 기존 `/scan`
계약은 SLAM 호환성을 위해 유지합니다. 생성 world에서는 현재 로봇 visual에
`0x02`, LiDAR에 `0x01` visibility mask를 사용하므로 센서는 교체 예정인 기존 차체를
투과해 환경만 봅니다. GUI 표시와 물리 collision에는 영향을 주지 않습니다.

### Study cafe evaluation route

`config/study_cafe_route.yaml`은 현재 48석 시연실의 네 가로 통로와 두 세로 통로를
순서대로 훑고 spawn으로 돌아오는 약 94.30 m의 17-waypoint 폐루프입니다. 경로 중심은
가로 `y=-4.705, -1.585, 1.585, 4.705 m`, 세로 `x=-1.865, 1.865 m`이고,
좌우 sweep 끝점은 `x=-5.65, 5.65 m`입니다. 경로는 고정 가구 배치와 현재 로봇
footprint를 기준으로 하므로 가구 위치나 footprint가 달라지면 다시 검증합니다.

`ground_truth_route_follower`는 `/ground_truth/odom`을 경로 제어에만 사용하고
`/cmd_vel`을 발행합니다. SLAM에는 ground truth를 전달하지 않습니다.

Gazebo study cafe를 LiDAR profile로 먼저 실행합니다. 이 profile은 LiDAR, IMU,
odometry, ground truth와 TF에 필요한 bridge만 실행하고 camera bridge는 만들지
않습니다. 별도 terminal에서 경로를 시작합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=false sensor_profile:=lidar_nav
```

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_gazebo_sim study_cafe_route.launch.py
```

직선 속도는 0.25 m/s, 회전 속도는 0.5 rad/s이며 경로가 끝나거나 ground-truth
odometry가 0.5초 이상 끊기면 정지 명령을 발행합니다.

먼저 패키지를 빌드한 뒤 각 후보의 독립된 run directory를 만듭니다. 아래 예시는
Harmonic 예시입니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install-harmonic/setup.bash
ros2 run cleany_gazebo_sim lidar_slam_evaluation prepare \
  --package-root ros2_ws/src/cleany_gazebo_sim \
  --profiles ros2_ws/src/cleany_gazebo_sim/config/lidar_mount_profiles.yaml \
  --profile floor_26cm \
  --simulator harmonic \
  --output /tmp/cleany-slam-front-low-01

ros2 launch cleany_gazebo_sim gazebo_harmonic.launch.py \
  world:=/tmp/cleany-slam-front-low-01/world.sdf \
  sensor_config:=/tmp/cleany-slam-front-low-01/sensor_tf.yaml
```

다른 terminal에서 위의 `slam_mapping.launch.py`를 실행합니다. 세 후보 모두 동일한
world, 주행 경로(`route_id`), 주행 시간과 trial 수를 사용해야 비교할 수 있습니다.
최소 3회 반복을 권장하며 다음 지표와 artifact를 `result.json` 양식에 기록합니다.

- 정량: ATE RMSE, translation/rotation RPE RMSE, map coverage, valid scan ratio,
  평균 scan rate, real-time factor
- 정성: map image, trajectory artifact, 사각·가림·벽 왜곡·loop closure 관찰 내용

측정값을 별도 JSON에 작성한 뒤 run manifest와 schema가 일치하는지 검증하여
기록합니다.

```bash
ros2 run cleany_gazebo_sim lidar_slam_evaluation record \
  --run-dir /tmp/cleany-slam-front-low-01 \
  --input /path/to/measured-result.json
```

현재 저장소에는 재현 가능한 후보 materialization과 결과 schema만 포함하며 측정하지
않은 성능 수치를 임의로 채우지 않습니다. 최종 위치 선정은 동일 조건의 실제 runtime
결과를 수집한 뒤 결정합니다.

## Run

저장소 루트에서 실행합니다.

```bash
make sim-gazebo
```

이 명령은 활성 ROS와 Gazebo version을 검사하고 `make build-gazebo`를 먼저 실행한 뒤,
선택된 Fortress 또는 Harmonic 서버를 `lidar_nav` sensor profile과 GUI 없는 상태로
시작합니다. 종료할 때는 `Ctrl-C`를 누릅니다.

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

GUI와 camera sensor는 host의 OpenGL/OGRE 호환성에 영향을 받습니다. Fortress의
GPU LiDAR와 camera sensor server는 OGRE2로 실행하고 GUI는 OGRE1을 사용합니다.
Fortress의 server-only `-s`는 GUI만 끄며 rendering sensor가 있으면 server 내부에서
여전히 rendering context를 생성합니다.

`make sim-gazebo`와 `make test-gazebo`는 활성 ROS 배포판과 Gazebo major version으로
Fortress 또는 Harmonic profile을 선택한다. 팀 표준은 Fortress이며 Harmonic은 호환
profile이다. 환경 준비와 자동 판정·output 분리 규칙은
[`DEVELOPMENT_SETUP.md`](../../../docs/DEVELOPMENT_SETUP.md)를 따른다.

Harmonic 서버는 OGRE2를 사용한다. GUI를 함께 실행할 때도 server는 OGRE2, GUI는
OGRE1을 사용한다. Harmonic world의 rendering sensor는 구독 전까지 비활성화할 수
있으며, launch는 선택한 sensor profile에 해당하는 bridge만 실행한다.

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
- `config/*_bridge_harmonic.yaml`

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
사용합니다. Harmonic world의 rendering sensor는 구독이 생기기 전까지 비활성화할
수 있도록 `always_on=false`로 정의되어 있습니다. launch는 선택한 sensor profile에
해당하는 `*_bridge_harmonic.yaml`만 실행합니다.
