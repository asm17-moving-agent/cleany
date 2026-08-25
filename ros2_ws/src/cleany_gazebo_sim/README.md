# cleany_gazebo_sim

ROS 2 Humble / Gazebo Fortress 기반의 Cleany mobile-base simulation
backend입니다. `cmd_vel -> odom / TF` 계약과 LiDAR·IMU·camera sensor를
headless 및 GUI 환경에서 검증합니다.

## 지원 환경

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress 6

설치와 renderer 진단은
[`DEVELOPMENT_SETUP.md`](../../../docs/DEVELOPMENT_SETUP.md)를 따릅니다.

```bash
source /opt/ros/humble/setup.bash
make check-gazebo-env
```

## 빠른 실행

저장소 루트에서 headless simulation을 실행합니다.

```bash
make sim-gazebo
```

48석 study-cafe GUI를 실행합니다. Sensor server는 항상 OGRE2를
사용하고 GUI renderer는 machine별로 선택합니다.

```bash
# 기본 GUI renderer: OGRE1
make sim-gazebo-study-cafe

# OGRE2가 필요한 machine
GAZEBO_GUI_RENDER_ENGINE=ogre2 make sim-gazebo-study-cafe
```

직접 launch할 때는 `gui_render_engine:=ogre|ogre2`를 지정합니다.

```bash
source ros2_ws/install/setup.bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=false gui_render_engine:=ogre2
```

## LiDAR noise profile

`gazebo_fortress.launch.py`는 기본 world를 생성할 때 RPLIDAR A1M8 Gaussian
noise profile을 적용합니다. 두 profile 모두 고정 bias 없이 `mean=0.0`을 사용합니다.

| Profile | `stddev` | 용도 |
| --- | ---: | --- |
| `measured` | 0.0025 m | 1–3 m 실물 벽 측정의 초기 근사값 |
| `stress` | 0.01 m | localization robustness 시험 |

```bash
ros2 launch cleany_gazebo_sim gazebo_fortress.launch.py \
  headless:=true lidar_noise_profile:=measured

ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=true lidar_noise_profile:=stress
```

`world:=...`를 직접 지정하면 해당 world를 그대로 사용하므로 noise profile을
별도로 적용하지 않습니다.

## ROS interface

| Direction | ROS topic | Type / role |
| --- | --- | --- |
| Input | `/cmd_vel` | 사용자 차체 속도 명령 |
| Internal | `/gazebo_cmd_vel` | guard를 통과한 Gazebo 명령 |
| Output | `/clock` | simulation clock |
| Output | `/odom` | `odom -> base_link` 기준 pose |
| Evaluation | `/ground_truth/odom` | 평가 전용 simulator pose |
| Output | `/joint_states` | 4개 drive wheel joint |
| Output | `/scan` | 360-sample GPU LiDAR |
| Output | `/imu/data` | `imu_link`, 50 Hz simulation IMU |

Camera profile은 head RGB·depth와 좌·우 wrist RGB를 다음 topic으로
발행합니다.

- `/camera/head/color/image_raw`
- `/camera/head/depth/image_raw`
- `/camera/left_wrist/color/image_raw`
- `/camera/right_wrist/color/image_raw`

### TF ownership

- `gazebo_odom_tf_publisher`: `odom -> base_link`
- `gazebo_sensor_tf_publisher`: `base_link -> lidar_link / imu_link`
- Camera optical frame: REP-103 `_optical_frame`

Stock Fortress `MecanumDrive` 플러그인은 odometry message를 발행하지
않습니다. 따라서 Fortress profile은 `OdometryPublisher`의 ground-truth
출력을 `/gazebo_odom`과 `/ground_truth/odom`에 동시에 bridge합니다.
현재 `/odom`은 wheel drift나 slip이 반영된 odometry가 아닙니다.

## Sensor profiles

`sensor_profile` launch argument로 rendering sensor 부하를 선택합니다.
차체, clock, odometry, joint state, IMU bridge는 모든 profile에서 실행됩니다.

| Profile | LiDAR | Head RGB | Head depth | Left wrist | Right wrist |
| --- | --- | --- | --- | --- | --- |
| `lidar_nav` (기본값) | O | X | X | X | X |
| `head_rgbd` | X | O | O | X | X |
| `left_wrist` | X | X | X | O | X |
| `right_wrist` | X | X | X | X | O |
| `all_cameras` | X | O | O | O | O |

```bash
ros2 launch cleany_gazebo_sim gazebo_fortress.launch.py \
  headless:=true sensor_profile:=head_rgbd
```

`bridge_config` launch argument를 지정하면 sensor profile 대신 해당 bridge
설정 하나를 사용합니다. 선택되지 않은 rendering sensor는
`always_on=false`와 bridge subscriber 부재로 lazy 상태를 유지합니다.

## Study-cafe world

`gazebo_study_cafe.launch.py`는 12.26×10.94 m, 48석 study-cafe 평가
공간을 생성합니다. 로봇 spawn, 방 크기, 책상·의자 배치는
`config/study_cafe/study_cafe_layout.yaml`이 관리하며 생성된 world는
`/tmp/cleany_study_cafe.sdf`에 기록됩니다.

LiDAR 높이는 `lidar_profile` argument로 선택합니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=false lidar_profile:=floor_26cm
```

World의 반복 가구는 primitive collision을 사용하고, 로봇 visual과
LiDAR에 별도 visibility mask를 적용해 self-hit를 방지합니다.
의자 visual은 OpenRobotics Gazebo Fuel `OfficeChairGrey` (CC BY 4.0)를
사용하며 최초 실행 시 network가 필요할 수 있습니다.

## SLAM evaluation

LiDAR 높이별 bag 기록, slam_toolbox·Cartographer·RTAB-Map 비교,
의자 이동 localization 실험은
[`tools/slam_evaluation/README.md`](../../../tools/slam_evaluation/README.md)를
참고합니다. 실험 생성물은 `ros2_ws/slam_results/`에 저장하며
커밋하지 않습니다.

## Validation

정적 world·bridge·TF·command guard 계약을 검증합니다.

```bash
make test-gazebo
```

SLAM 실험과 study-cafe 형상 계약을 검증합니다.

```bash
make test-gazebo-evaluation
```

LiDAR·IMU·odometry·TF와 차체 구동을 실제 Fortress runtime에서
검증합니다.

```bash
make test-gazebo-nav-runtime
```

모든 camera stream의 해상도·encoding·frame·timestamp와 실제 image 변화를
검증하려면 opt-in runtime test를 실행합니다.

```bash
cd ros2_ws
source install/setup.bash
python3 -m pytest -s \
  src/cleany_gazebo_sim/test/test_runtime_rendering_sensors.py \
  --run-sim-runtime --sim-profile=fortress \
  --sensor-profile=all_cameras
```

## Configuration map

- `worlds/cleany_mecanum_fortress.sdf`: robot, physics, sensor system
- `config/base.yaml`: command guard와 TF publisher parameter
- `config/bridge/`: Gazebo transport / ROS bridge
- `config/lidar_mount_profiles.yaml`: LiDAR 높이 후보
- `config/lidar_noise_profiles.yaml`: 실측 근사·stress LiDAR noise profile
- `config/study_cafe/`: study-cafe layout과 평가 route
- `launch/gazebo_fortress.launch.py`: core Fortress backend
- `launch/gazebo_study_cafe.launch.py`: study-cafe scenario
- `launch/evaluation_*.launch.py`: SLAM replay·visualization·route 평가
- `test/evaluation/`: 실험 전용 계약 테스트

## Known limitations

- Mobile base 주행에 초점을 두며 arm controller와 manipulation은 포함하지
  않습니다. 양팔은 접은 대기 자세로 고정됩니다.
- Mecanum roller는 visual로만 표현하고 contact는 anisotropic friction으로
  단순화합니다.
- Simulation IMU에 stochastic noise와 bias model이 없습니다.
- Fortress odometry fallback은 wheel drift와 slip을 모사하지 않습니다.
- GPU LiDAR와 camera는 headless 실행에서도 OpenGL rendering context가
  필요합니다.
