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
| Output | `/odom` | 선택한 odometry source를 재발행한 canonical pose |
| Internal | `/wheel/odom_raw` | encoder와 Mecanum 기구학만 적용한 pose |
| Output | `/wheel/odom` | 선택한 simulation odometry error를 적용한 pose |
| Evaluation | `/ground_truth/odom` | 평가 전용 simulator pose |
| Output | `/joint_states` | 4개 drive wheel joint |
| Optional | `/wheel_encoder/joint_states` | 가상 quadrature encoder 측정값 |
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

기본 `odometry_source:=wheel`은 `/wheel/odom`을 canonical `/odom`과
`odom -> base_link` TF로 재발행하므로 SLAM과 navigation이 wheel odometry를
사용합니다. 비교용 `odometry_source:=gazebo`는 기존 `/gazebo_odom`을 사용합니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_harmonic.launch.py \
  odometry_source:=gazebo
```

Stock Fortress `MecanumDrive` 플러그인은 odometry message를 발행하지
않습니다. 따라서 Fortress profile은 `OdometryPublisher`의 ground-truth
출력을 `/gazebo_odom`과 `/ground_truth/odom`에 동시에 bridge합니다.
Fortress에서 `odometry_source:=gazebo`를 선택하면 `/odom`은 wheel drift나
slip이 반영된 odometry가 아닙니다.

`cleany_base_odometry`는 `/joint_states`의 네 drive wheel 누적 회전각을
Mecanum kinematics로 적분합니다. Gazebo launch에서는 가상 encoder의 양자화된
`/wheel_encoder/joint_states`를 입력으로 사용하고 결과를 `/wheel/odom_raw`로
발행합니다. Simulation 전용 error node가 이를 `/wheel/odom`으로 변환하며 직접
TF를 발행하지는 않습니다.

### Simulated wheel encoder

`simulated_encoder_node`는 Gazebo `/joint_states`의 휠 각도를 모터 사양
`13 PPR`, 감속비 `1:61`, 4배 quadrature 기준인 `3172 tick/rev`로
양자화해 `/wheel_encoder/joint_states`를 발행합니다. 휠별 scale과 이동 중
tick 증분의 Gaussian noise를 선택적으로 적용할 수 있습니다.

기본 설정 `config/simulated_encoder.yaml`은 tick 양자화만 수행합니다.
`config/simulated_encoder_synthetic_noise.yaml`의 편차는 실제 측정값이 아닌
파이프라인 검증용 합성값입니다. Gazebo launch는 기본 encoder 설정을 자동으로
실행하고 raw wheel odometry 입력을 `/wheel_encoder/joint_states`에 연결합니다.
Gazebo JointStatePublisher는 world 이름과 무관한
`/model/cleany_mecanum/joint_state` transport topic을 사용하므로 일반 world와
Study-cafe world가 같은 `/joint_states` bridge 계약을 공유합니다.

### Simulated odometry error

`simulated_odometry_error_node`는 `/wheel/odom_raw`의 프레임 간 전진·횡이동·회전
변화량에 오차를 적용하고 다시 적분해 `/wheel/odom`을 발행합니다. 기본
`odometry_error_ideal.yaml`은 값을 그대로 통과시킵니다.

`odometry_error_stress.yaml`은 전진·횡이동·회전 scale, 이동량 비례 Gaussian
noise, yaw bias random walk, 거리당 yaw drift와 일정 시간 유지되는 slip event를
적용합니다. 고정 seed를 사용해 반복 실행할 수 있지만 수치는 실제 측정값이 아닌
SLAM 강건성 시험용 합성값입니다.

`odometry_error_level1.yaml`과 `odometry_error_level2.yaml`은 ideal에서
stress까지 각 수치 parameter를 1/3, 2/3로 선형 보간한 비교 실험 전용
profile입니다. 네 단계 bag·SLAM 비교 절차는 SLAM evaluation 문서를 따릅니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_harmonic.launch.py \
  odometry_error_config:=$(ros2 pkg prefix cleany_gazebo_sim)/share/cleany_gazebo_sim/config/odometry_error_stress.yaml
```

기존 encoder 및 기구학 합성 profile은 각 계층의 단독 시험용입니다. Stress odometry
profile과 동시에 적용하면 같은 성격의 오차가 중복되므로 기본 encoder 및
`wheel_odometry.yaml`과 함께 사용합니다. 같은 인자는 `gazebo_study_cafe.launch.py`와
`gazebo_study_cafe_fortress.launch.py`에도 전달됩니다.

## Sensor profiles

`sensor_profile` launch argument로 rendering sensor 부하를 선택합니다.
차체, clock, odometry, joint state, IMU bridge는 모든 profile에서 실행됩니다.

| Profile | LiDAR | Head RGB | Head depth | Left wrist | Right wrist |
| --- | --- | --- | --- | --- | --- |
| `lidar_nav` (기본값) | O | X | X | X | X |
| `lidar_depth_nav` | O | X | O (depth + CameraInfo) | X | X |
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
`config/study_cafe/study_cafe_layout.yaml`이 관리합니다. launch마다
`/tmp/cleany-study-cafe-*/` 전용 디렉터리를 만들고 그 안에 world와 sensor TF
설정을 기록하므로 동시 실행이나 이전 실행의 파일 권한과 충돌하지 않습니다.
Study-cafe 평가의 기본 physics timestep은 2 ms입니다.
책상 사이 파티션은 바닥 0.26 m부터 0.98 m까지 배치합니다.

LiDAR 높이는 `lidar_profile` argument로 선택합니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=false lidar_profile:=floor_26cm
```

책상·파티션·모니터는 상세 visual과 보수적 primitive collision을
분리해 사용하고, 로봇 visual과 LiDAR에 별도 visibility mask를 적용해
self-hit를 방지합니다.
의자 visual은 OpenRobotics Gazebo Fuel `OfficeChairGrey` (CC BY 4.0)를
사용하며 최초 실행 시 network가 필요할 수 있습니다.
의자는 static 모델이며 visual과 collision 모두 동일한 `OfficeChairGrey.obj`를
같은 0.9 배율·90도 yaw로 사용합니다. 좌판 아래와 다리 사이 빈 공간을 막던
`chair_envelope_collision` 박스는 제거했습니다. 거친 `OfficeChairGrey_Col.obj`나
전체 convex hull은 사용하지 않습니다. 다른 가구의 단순화된 충돌 형상은 별도 검토 대상입니다.

평가 route는 waypoint마다 방향 오차가 0.08 rad 이하가 될 때까지 제자리
회전한 후 주행합니다. 주행 중 오차가 0.15 rad 이상이면 다시 정지 회전하며,
전진 속도는 0.20 m/s²로 증가시켜 좁은 통로의 급격한 선회 진입을 방지합니다.

## SLAM evaluation

LiDAR 높이별 bag 기록, slam_toolbox·Cartographer·RTAB-Map 비교,
의자 이동 localization 실험은
[`tools/slam_evaluation/README.md`](../../../tools/slam_evaluation/README.md)를
참고합니다. 실험 생성물은 `ros2_ws/slam_results/`에 저장하며
커밋하지 않습니다.

## AMCL / Nav2 study-cafe evaluation

### SCRUM-306: local costmap과 동적 장애물 평가

Jazzy/Harmonic용 `amcl_nav2_safety.launch.py`는 기존 Nav2 설정에 깊이
VoxelLayer와 Collision Monitor를 추가합니다. controller와 BT navigator는
`/wheel/odom`을 사용합니다. 시뮬레이션은 `sensor_profile:=lidar_depth_nav`,
`odometry_source:=wheel`, `lidar_profile:=floor_30cm`으로 실행합니다.

```bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=true sensor_profile:=lidar_depth_nav \
  lidar_profile:=floor_30cm lidar_noise_profile:=measured odometry_source:=wheel

# 별도 terminal, 같은 ROS domain에서 실행합니다.
ros2 launch cleany_gazebo_sim amcl_nav2_safety.launch.py \
  map:=/absolute/path/to/map_final.yaml
```

| 인터페이스 | 역할 |
| --- | --- |
| `/scan` | 30 cm 수평 LiDAR, 기존 obstacle layer와 Collision Monitor 입력 |
| `/camera/head/depth/image_raw`, `/camera/head/depth/camera_info` | Gazebo의 640×480 깊이 영상과 같은 센서의 내부 파라미터 |
| `/camera/head/depth/points` | `depth_clearance`가 투영한 X-forward 포인트와 빈 공간 ray endpoint |
| `/nav2/cmd_vel` | controller/behavior가 Collision Monitor에 보내는 Twist |
| `/cmd_vel` | Collision Monitor가 내보내는 Twist; 기존 command guard가 받아 Gazebo로 전달 |
| `/collision_monitor_state` | `StopZone`, `SlowZone`, `invalid source` 등 제한 사유 |

`config/nav2_safety.yaml`의 수치는 **시뮬레이션 평가값**입니다. KB의 안전
기준은 아직 draft이며 이 프로필은 물리 로봇의 승인된 안전 설정이 아닙니다.
기본 StopZone은 base 기준 앞뒤 0.48 m·좌우 0.40 m, SlowZone은 앞뒤
0.80 m·좌우 0.55 m이고 감속 시 입력 속도에 0.35를 곱합니다. 관측이
0.6초 이상 오래되면 정지합니다. `invalid source` 정지는 장애물 검출에
의한 `StopZone` 정지와 구분해 평가합니다.

전역·지역 costmap의 경로 계산 footprint에는 StopZone과 같은 직사각형에
2 cm padding을 적용하고, inflation 반경은 그 외접 반경보다 큰 0.8 m로
설정합니다. 실제 base의 padding 포함 크기 0.64×0.54 m와 이 계획용
영역 1.00×0.84 m를 구분합니다. 좁은 통로에서 base만 지날 수 있는 경로가
정지 영역에 반복해서 걸리는 것을 줄이기 위한 평가 설정입니다.

MPPI는 80×0.05초 예측 구간을 사용합니다. 우회 중 경로에서 벗어날 수
있도록 PathAlign 가중치 3, PathFollow 가중치 15와 전방 offset 20을
적용합니다. 최고 속도 0.25 m/s에서의 예측 거리 1 m에 맞춰 PathFollow와
PathAlign을 끝내고 Goal critic으로 전환합니다. Goal 가중치도 15로 두어
목표 접근 시 진행 유도가 급격히 약해지지 않도록 합니다.
[Nav2 MPPI 튜닝 근거](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/).

LiDAR는 `odom` 원점보다 8 cm 아래에 있으므로 두 costmap의 obstacle layer와
scan source에 `min_obstacle_height: -0.33`을 적용합니다. 기본값 0을 쓰면
실제 `/scan` 메시지가 있어도 그 관측이 costmap에서 제외됩니다.

깊이 영상의 optical Z-depth를 CameraInfo로 투영하고, 출력은 Gazebo
카메라의 +X 전방·+Y 좌측·+Z 상방 좌표계 `head_camera_depth_frame`을
사용합니다. 수평으로 고정된 시뮬레이션 head의 nominal TF를 적용하며,
head pan/tilt 움직임이나 물리 카메라 캘리브레이션은 이 프로필 범위 밖입니다.
기본 `pixel_stride: 4`는 각 축에서 4개 픽셀마다 투영합니다.

Gazebo Ogre2의 `+inf`는 far clip 밖의 관측이므로 이 프로필에서만
`positive_infinity_is_free: true`로 해석합니다. 5 m ray endpoint는
장애물 marking 최대 거리 4 m보다 멀고 센서 far clip 10 m보다 가깝습니다.
`NaN`, `-inf`, 0 이하 깊이는 빈 공간으로 바꾸지 않습니다. 물리 RGB-D의
invalid depth에 이 가정을 적용하면 안 됩니다. 바닥·천장 hit도 clearing
ray에는 보존하고, 실제 marking 높이는 VoxelLayer에서 따로 제한합니다.

구현 근거: [Gazebo Ogre2 깊이 출력](https://github.com/gazebosim/gz-rendering/blob/gz-rendering8/ogre2/src/Ogre2DepthCamera.cc),
[Nav2 VoxelLayer](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/plugins/voxel_layer.cpp),
[Collision Monitor 연결](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_collision_monitor/using_collision_monitor/).

자동 평가는 저장 지도를 입력받아 **새 Gazebo와 ROS domain**에서 실행하며,
기존 실험 디렉터리를 덮어쓰지 않습니다. ROS/Jazzy 환경에서 저장소 루트 기준:

```bash
make test-gazebo
make test-gazebo-safety SAFETY_MODE=sensors \
  SAFETY_MAP=/absolute/path/to/map_final.yaml \
  SAFETY_OUTPUT=/absolute/path/to/new-sensors-run
# SAFETY_MODE=monitor: 접근 장애물에 대한 감속/정지/제거 후 재개
# SAFETY_MODE=avoid: 실제 NavigateToPose 중 진입하는 장애물 우회
```

`config/nav2_safety_evaluation.yaml`은 fixture 크기·이동, 목표, 관찰 시간과
진단 임계값을 관리합니다. `sensors`는 높이별 marking과 제거 후 clearing을
확인합니다. `monitor`는 Nav2 액션 없이 monitor 입력에 일정 속도를 주입하는
분리 시험이며, `avoid`만 실제 Nav2 액션으로 로봇을 주행시킵니다.
fixture는 실제 Gazebo collision/visual geometry를 `set_pose`로 옮기는
스크립트 장애물이며 사람의 물리 운동 모델을 의미하지 않습니다.

각 실행은 `result.json`, 실제 활성 설정 `runtime_verified.json`, 설정/실행
snapshot, `samples.csv`, 센서/속도/costmap bag, 센서별 NPZ를 남깁니다.
실패도 결과와 로그를 보존하고 종료 코드는 1입니다. 원본 depth 영상 대신
평가용 point subsample을 bag에 저장하며 제어 입력은 subsample topic을
사용하지 않습니다. `clearance_m`는 ground truth로 계산한 **base의 padding
포함 평면 footprint와 시험 fixture 사이**의 거리입니다. 모든 가구와의
최소 거리나 3D contact sensor 측정은 아닙니다.

세 모드의 결과를 모아 그림과 JSON으로 렌더링할 수 있습니다.

```bash
python3 tools/navigation_evaluation/render_safety_report.py \
  --sensors /absolute/path/to/sensors-run \
  --monitor /absolute/path/to/monitor-run \
  --avoid /absolute/path/to/avoid-run \
  --output /absolute/path/to/report
```

2026-09-09 Jazzy/Harmonic 실행 결과입니다. 30 cm measured LiDAR, L0
`wheel/odom`, 저장된 `cartographer_30cm_level0_replay_20260908T222611/map_final.yaml`을
사용했습니다. 세 실행 모두 lifecycle 9개 active, 실제 L0 parameter,
controller/BT의 odom 입력, `/cmd_vel`의 단일 Collision Monitor publisher를
확인했습니다. 지도 이미지 SHA256은
`bbfa7571d29ab0404a17c00761bdb2de59781a67abe467a6913c2c10a686db33`입니다.

| 시험 | 실제 관측 | 결과 |
| --- | --- | --- |
| LiDAR 단독, 1.4 m 높이 fixture | scan 22점; ROI lethal cell 0 → 14 → 0 | marking/제거 후 clearing 통과 |
| LiDAR 위 돌출물, 바닥에서 0.60–1.10 m | scan 0점, 평가용 depth 443점; lethal cell 0 → 33 → 0 | 깊이 관측/clearing 통과 |
| 15 cm 낮은 물체, 전방 1 m | scan 0점, depth 0점, 추가 lethal cell 0 | 사각 확인 |
| 접근 fixture, monitor 분리 시험 | 속도 비율 0.35; 정지 구간 실제 속도 0 m/s; 제거 후 재개 | 통과, 최소 base 간격 16.0 cm |
| Nav2 주행 중 fixture 진입 | `(4, 0)` 목표 SUCCEEDED; 최대 횡변위 37.2 cm; 최소 base 간격 24.1 cm | 통과, 최종 GT 위치 오차 6.3 cm |

최종 실행 ID는 `scrum306_sensors_06`, `scrum306_monitor_03`,
`scrum306_avoid_09`입니다. 로컬 `ros2_ws/slam_results/`에 각 실행의
bag·CSV·설정·로그와 `scrum306_report_20260909/`의 그림/JSON을 보존했습니다.
GT는 지정 spawn을 역변환한 고정 평가 좌표이며 실행마다 궤적 정합으로
오차를 줄이지 않았습니다.

우회 성공 사례도 **196.846초, 이동 거리 4.157 m, 목표 부근 복구 3회**가
필요했습니다. 이는 감속·우회·정지 기능의 제한된 검증 결과이며, 주행
효율이나 복구 없는 운행의 완료 판정이 아닙니다. 개발 중 초기 설정과 더
좁은 배치를 시험한 `scrum306_avoid_01`–`08`의 진단 기록도 보존했습니다. 최종 배치는
교차 통로 `(3.15, 1.0)`에서 `(3.15, 0.45)`로 진입하는 0.4×0.4 m fixture
한 사례이며, 최종 설정의 반복 성공률·중앙 차단·늦은 횡단은 별도 검증이
필요합니다. NavFn의 2D 경로와 직사각형 footprint의 협소 공간 통과 가능성을
같은 것으로 간주하지 않습니다.

`make test-gazebo`는 관련 4개 패키지 build 성공 후 **96 passed, 2 skipped**
였습니다. 두 skip은 명시 옵션이 필요한 기존 rendering/simulation runtime
테스트이며, 이번 SCRUM-306 실제 실행은 위 세 모드로 별도 수행했습니다.

확인해야 할 사각과 한계:

- 30 cm LiDAR는 그 수평면과 만나지 않는 낮은 물체나 돌출물을 볼 수 없습니다.
- 높이 약 1.18 m의 수평 head depth는 전방 일부만 봅니다. 로봇 앞 1 m에
  놓인 높이 15 cm 물체는 이 배치에서 두 센서 모두 검출하지 못했습니다.
- 전방 depth의 근거리 하단·좌우·후방 사각은 costmap이나 Collision Monitor로
  복원되지 않습니다. 회전·횡이동·후진 안전을 전부 보장하는 프로필이 아닙니다.
- Gazebo depth는 반사·투명체·조명에 따른 실센서 실패를 충실히 모델링하지 않습니다.
  L0 wheel odometry와 제한된 fixture 경로의 결과를 실로봇 안전성이나 성공률로
  일반화하지 않습니다.

### RViz에서 저장 지도와 실시간 코스트맵 겹쳐보기

현재 `make view-gazebo-costmap`은 `ROBOT_MODEL=cad_frame`을 기본으로 사용합니다.
이전 RASKOG 평가 모델은 `ROBOT_MODEL=legacy`로 재현할 수 있습니다.

```bash
# Jazzy/Harmonic 환경에서 저장소 루트 기준, 새 출력 경로를 사용합니다.
make view-gazebo-costmap \
  SAFETY_MAP=/absolute/path/to/map_final.yaml \
  SAFETY_OUTPUT=/absolute/path/to/new-live-session
```

`config/rviz/navigation_costmap.rviz`는 `map` 좌표계에서 저장 지도 `/map`,
반투명 `/local_costmap/costmap`, LiDAR와 계획용 footprint를 겹쳐 표시합니다.
지역 costmap은 현재 설정에서 10 Hz로 계산하고 2 Hz로 화면용 grid를 발행합니다.
Global costmap은 왼쪽 Displays에서 선택적으로 켤 수 있습니다.
초록 선은 Nav2가 발행하는 계획 경로 `/plan`입니다.

같은 명령에 `SAFETY_DRIVE=1`을 추가하면 정지 시연 대신
`nav2_safety_evaluation.yaml`의 회피 주행을 한 번 실행합니다(현재 목표 `[4, 0, 0]`).
주행 결과는 `drive_result.json`에 저장하고 도착 후에도 RViz를 유지합니다.
짧은 검은 글씨는 `DRIVING`/`ARRIVED` 등 시연 상태 라벨입니다.
스크립트를 직접 실행할 때 `--drive-delay 35`로 GUI 준비 시간을 둘 수 있습니다.
`--idle --initial-pose X Y YAW`는 저장 지도 좌표의 지정 위치에 로봇을 놓고
AMCL 초기 자세도 맞춘 뒤, 주행·시험 장애물 없이 화면을 유지합니다.

별도 ROS domain 207과 Gazebo partition에서 30 cm/L0/wheel odom 시뮬레이션을
시작하고, 정지한 로봇 앞에 tall/raised 시험 장애물을 6초씩 넣었다 제거합니다.
RViz를 닫으면 이 실행이 시작한 Gazebo/Nav2도 종료합니다. `session.json`에
supervisor PID와 종료 방법을 기록합니다. 기존 bag 재생이나 SLAM 지도 재작성은
하지 않습니다. 화면의 저장 지도는 고정되고 센서 기반 costmap만 갱신됩니다.

이미 실행 중인 동일 ROS domain의 시뮬레이션에 보기만 연결하려면:

```bash
rviz2 -d ros2_ws/src/cleany_gazebo_sim/config/rviz/navigation_costmap.rviz \
  --ros-args -p use_sim_time:=true
```

### 전역 장애물 관측 기록

맵 전체 순회는 `tools/navigation_evaluation/live_costmap.py --full-map-route --map <지도.yaml>
--output <새 디렉터리>`로 실행합니다. `study_cafe_route.yaml`의 17개 경유점(원본 94.3 m)을
기존 `ground_truth_route_follower`로 추종합니다. 시험용 장애물은 투입하지 않습니다.
`--route-end-inset 0.30`은 양 끝 X 경유점을 30 cm 안쪽으로 옮겨 회전 여유를 확보하며,
네 통로를 방문하는 순서를 유지합니다. 적용값과 표시용 map 좌표는 `scenario.json`에 기록합니다.

추종기는 Gazebo world 좌표의 `/ground_truth/odom`을 사용하고 `/nav2/cmd_vel`로 명령을 보내
Collision Monitor를 통과합니다. AMCL과 코스트맵의 odometry는 계속 `/wheel/odom`을 씁니다.
이는 시뮬레이션 관측 순회이며 Nav2 자율주행 성능 검증 결과가 아닙니다. 이동·회전이
30 simulation seconds 동안 없으면 추종기를 정지합니다. 진행은 `survey_follower.log`와
화면 `SURVEY n/17`에 표시하며 완료·정지 결과는 `drive_result.json`에 저장합니다.
시뮬레이션 truth와 추정 map pose의 차이가 1초 이상 0.35 m 또는 25도를 넘으면
주행을 중단하고 누적 관측을 동결합니다. 한계는 `--max-survey-position-error`,
`--max-survey-yaw-error-deg`로 지정합니다. 이는 시뮬레이션 전용 실패 감시이며
실제 로봇의 localization 신뢰도 추정 기능은 아닙니다. 누적 동결/재개 서비스는
`/obstacle_memory/set_enabled` (`std_srvs/SetBool`)입니다.

`tools/navigation_evaluation/live_costmap.py --drive --no-test-obstacle`은 시험용 장애물을
투입하지 않고 같은 목표까지 주행합니다. 기존 가구는 유지하며, 이 모드에서는 횡방향
회피량과 시험용 장애물까지의 이격 거리를 성공 조건으로 검사하지 않습니다.
`--restore-memory`를 생략하면 누적 기록도 새로 시작합니다.

`make view-gazebo-costmap`은 `/obstacle_memory/grid`에 저장 지도와 같은 크기의
누적 장애물 지도를 발행합니다. 원본 지도는 고정하고 `map` 좌표에 변환한
LiDAR `/scan`, depth `/camera/head/depth/points` 관측만 누적합니다.
`/obstacle_memory/lidar`, `/obstacle_memory/depth`는 센서별 결과입니다.
RViz의 `Persistent obstacle memory`에서 로컬 창 밖에 남아 있는 기록을 볼 수 있습니다.

- XY는 저장 지도 해상도, Z는 10 cm voxel, 높이는 바닥 위 5 cm~2 m입니다.
  현재 평지 좌표계에서 바닥 `map.z=-0.38`을 전제로 하며 경사면 분류기는 아닙니다.
- 관측 광선이 통과한 같은 센서의 3D voxel만 지웁니다. 바닥으로 향한 광선이
  높은 장애물을 2D 투영만으로 지우거나, 라이다가 depth 기록을 지우지 않습니다.
- 점유 증거는 프레임별 hit +2 / free -1, 범위 [-3, 5]입니다. 양수면 점유로 투영합니다.
  미관측 칸은 -1입니다. 시간 경과만으로 점유 기록을 삭제하지 않으므로 재관측하지 않은
  물체는 오래 남을 수 있습니다. 마지막 관측 시각으로 오래된 기록을 구분할 수 있습니다.
- 기본 갱신 2 Hz, 저장 5초, marking 4 m / clearing 5 m, depth 최대 1200 ray입니다.
  센서 노이즈와 localization 오차도 누적될 수 있으며 원시 센서 전체를 저장하는 기능은 아닙니다.
- 전역 Nav2 costmap은 `static_layer -> memory_layer -> obstacle_layer -> inflation_layer`로
  구성하고 maximum 결합을 사용합니다. 누적 관측이 원본 지도 벽을 삭제하지 않습니다.
  안전 여유 영역은 저장하지 않고 현재 footprint로 다시 계산합니다.

출력 디렉터리의 `obstacle_memory/obstacles.npz`에는 센서별 voxel 증거, 칸별 마지막
관측 Unix 시각, 누적 hit 프레임 수, 지도 식별자를 저장합니다. `changes.jsonl`은
합쳐진 2D 지도 상태 변경 이력이고 `status.json`은 관측 면적과 센서 처리 횟수입니다.
NPZ는 임시 파일을 쓴 후 교체합니다. 기존 기록 복원은 스크립트 인자
`--restore-memory /absolute/path/to/obstacles.npz`를 사용하며, 지도 내용이나 좌표·높이
격자가 다르면 복원을 거부합니다. 명시 저장은 `/obstacle_memory/save` Trigger 서비스입니다.

### CAD 프레임 모델과 Gazebo 연결

`feat/migrate-cleany-description`의 `e718ac57e861f3d34b6e0e10b0d18d87a348bf51`
모델을 `cleany_description`으로 가져왔습니다. `world/cad_frame.py`는 공유 Xacro를
확장하고, `config/cad_frame.yaml`의 주차 자세를 URDF에 반영한 뒤 `gz sdf -p`로
변환합니다. 원본 URDF/MJCF의 관절은 유지하며 Gazebo 이동 평가에서만 팔과 머리를
고정합니다. 현재 머리 틸트는 바닥과 평행한 정면에서 아래로 30도(`head_tilt_joint=0.5235987755982988 rad`)이며,
카메라 위치와 회전 TF도 같은 자세에서 계산합니다. 생성된 SDF는 차체와 네 바퀴의 5개 물리 링크, 4개 구동 관절을 가집니다.

- 차체·팔·머리·바퀴 외형과 질량은 공유 description에서 가져옵니다. 질량·관성은
  상위 모델의 추정값이며 실측 동역학 검증을 의미하지 않습니다.
- 바퀴 충돌은 반지름 6.35 cm, 폭 5.1 cm의 원통 및 방향성 마찰 근사로 교체합니다.
  기존 구형 접촉의 과도한 좌우 폭을 제거했습니다. 48개 롤러의 개별
  접촉 물리는 시뮬레이션하지 않습니다. 구동과 `/wheel/odom` 모두 앞뒤 간격
  0.35 m, 좌우 간격 0.6038 m, 반지름 0.0635 m를 사용합니다.
- LiDAR/IMU와 카메라 센서는 기존 Gazebo 센서 정의를 연결합니다. 머리 카메라
  좌표계는 새 URDF를 따릅니다. 손목 카메라 위치는 기존 평가용 장착 오프셋을
  새 gripper frame에 연결한 명목값이며 보정된 실측 extrinsic이 아닙니다.
- 저장 지도는 동일한 study-cafe 환경의 30 cm LiDAR 지도를 재사용합니다.
  `world.model.json`에 소스 커밋, 고정 자세, 외형과 충돌 형상을 모두 포함하는 AABB,
  머리 카메라 TF, 바퀴 치수를 남깁니다. STL 정점은 각 링크 변환 후 경계를 계산합니다.
  팔을 움직이면 footprint 재계산이 필요합니다.
- CAD 평가와 live viewer는 이 경계에서 전역·지역 footprint 및 정지·감속 영역을
  자동 생성합니다. 현재 접힌 자세의 외곽은 약 53.949 × 65.480 cm이며 중심은
  `base_link`에서 X +3.122 cm입니다. 차체·바퀴만의 47.7 cm 길이와 구분합니다.
  `cad_frame.yaml`의 `navigation_margins`는 정지 여유 8 cm, 추가 계획 padding 2 cm,
  감속 여유 20 cm로 설정합니다. 계획용 사각형은 약 73.949 × 85.480 cm입니다.
  이 여유는 시뮬레이션 평가값이며 실물 제동거리에서 도출한 값은 아닙니다.
  `scenario.json`에 생성된 다각형과 중심을 남기고 장애물 간격 계산에도 중심 오프셋을
  반영합니다. legacy 모델의 정적 YAML 치수는 CAD 평가에 적용하지 않습니다.
- 2026-09-10 검증: `cad_geometry_monitor_20260910_03`은 감속 비율 0.35,
  정지 실속도 0, 재출발, 2 cm 패딩 외곽 기준 최소 이격 3.28 cm로 통과했습니다.
  `cad_geometry_drive_20260910_final`은 목표 `[4,0,0]` 도착, 복구 0회,
  위치 오차 6.60 cm, 시험 상자와의 패딩 외곽 최소 이격 16.19 cm로 통과했습니다.
  `geometry_validation.json`과 `drive_samples_complete.csv`에 결과를 보존합니다.
  정지 여유 3 cm와 7 cm의 선행 실행은 이격 기준 미달로 보존했으며 성공으로
  집계하지 않습니다. 단일 경로·설정의 시뮬레이션 결과이며 실물 안전 인증은 아닙니다.
- 로봇 자체 외형은 기존 visibility mask로 센서 관측에서 제외됩니다. 자가 가림이나
  실제 센서 사각지대까지 재현한 모델로 해석하지 않습니다.

Jazzy/Harmonic 환경에서 빌드 후, Gazebo만 실행하려면:

```bash
ros2 launch cleany_gazebo_sim gazebo_cad_frame.launch.py headless:=false
```

지도·코스트맵·Nav2 주행은 위 `make view-gazebo-costmap` 명령에
`ROBOT_MODEL=cad_frame SAFETY_DRIVE=1`을 추가합니다. Gazebo GUI를 별도로 붙일 때는
출력 디렉터리 `session.json`의 `gz_partition`을 `GZ_PARTITION`으로 설정한 뒤
`gz sim -g --render-engine-gui ogre2`를 실행합니다.

HiDPI 화면에서 메뉴에 비해 3D 장면이 흐리면 Gazebo GUI 프로세스에만
아래 설정을 적용할 수 있습니다. 이 환경에서는 Qt의 논리 화면이
1440×900 / DPR 2에서 2880×1800 / DPR 1로 바뀌었습니다.
Gazebo GUI 8의 MinimalScene은 논리 item 크기를 렌더 타깃 크기로 사용하므로
자동 배율을 끄면 3D 화면 확대를 피할 수 있습니다. 메뉴·버튼은 작아질 수 있으며,
호스트 디스플레이 배율이나 센서 이미지 해상도를 바꾸는 설정은 아닙니다.

```bash
QT_QPA_PLATFORM=xcb QT_AUTO_SCREEN_SCALE_FACTOR=0 \
QT_ENABLE_HIGHDPI_SCALING=0 QT_SCALE_FACTOR=1 QT_FONT_DPI=144 \
gz sim -g --render-engine-gui ogre2
```

검증은 `make test-gazebo`의 CAD 변환·관절 자세·구동·센서·안전 영역 검사와
`cleany_description/test/test_model_parity.py`의 공유 모델 일치 검사를 사용합니다.

### 기본 AMCL / Nav2 프로필

`amcl_nav2.launch.py`는 저장된 26 cm LiDAR 지도와 simulation topic 계약을 이용해
AMCL, map server, planner, controller, recovery behavior, BT navigator를 실행합니다.
AMCL은 Mecanum odometry를 반영하는 `OmniMotionModel`, controller는 lateral velocity를
생성할 수 있는 MPPI `Omni` model을 사용합니다. Nav2가 발행한 `/cmd_vel`은 기존
`gazebo_command_guard`를 거쳐 simulator에 전달됩니다.

기본 지도 `maps/study_cafe_26cm.yaml`은
`compare_26cm_strict_loop_2p5x_trial1`에서 저장한 26 cm LiDAR 지도입니다. 현재
canonical `lidar_link`와 입력 높이를 맞추기 위한 integration 기준이며, 실제 LiDAR
설치 위치나 최종 localization 구성을 확정하는 선택은 아닙니다. 기본 AMCL initial
pose `(x=0, y=0, yaw=0)`도 기존 study-cafe spawn과 같은 새 simulation world에서만
유효합니다.

Jazzy/Harmonic 환경에서 패키지를 빌드한 뒤 첫 terminal에 study-cafe와 26 cm LiDAR
profile을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon --log-base log-harmonic build --symlink-install \
  --build-base build-harmonic --install-base install-harmonic \
  --packages-up-to cleany_gazebo_sim
source install-harmonic/setup.bash
ros2 launch cleany_gazebo_sim gazebo_study_cafe.launch.py \
  headless:=true lidar_profile:=floor_26cm sensor_profile:=lidar_nav
```

두 번째 terminal에서 AMCL/Nav2를 시작합니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
source install-harmonic/setup.bash
ros2 launch cleany_gazebo_sim amcl_nav2.launch.py
```

AMCL과 Nav2가 active 상태가 되고 `map -> odom -> base_link`가 연결됐는지 확인합니다.

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_link
```

fresh spawn에서 첫 navigation goal을 보냅니다. 좌표는 저장 지도 `map` frame
기준입니다.

```bash
ros2 action send_goal --feedback \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}'
```

simulation을 재사용해 로봇이 이미 움직였거나 spawn을 바꾼 경우에는 기본 initial pose를
사용하지 말고 RViz의 `2D Pose Estimate` 또는 `/initialpose`로 실제 map pose를 먼저
제공합니다. 같은 `/cmd_vel`에 route follower나 teleop을 동시에 연결하지 않습니다.

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
- `config/nav2_amcl.yaml`: AMCL/Nav2 simulation 평가 parameter
- `config/study_cafe/`: study-cafe layout과 평가 route
- `maps/study_cafe_26cm.*`: AMCL 평가용 저장 지도
- `launch/amcl_nav2.launch.py`: AMCL/Nav2 evaluation stack
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

### 부품별 3D StopZone 비교 관찰

`body_guard_observer`는 현재 CAD 모델의 **고정 park 자세**에서 각 visual/collision
요소의 로컬 경계 상자(OBB)를 `base_link`로 변환한다. 전체 로봇을 하나의
직육면체로 채우지 않으므로 부품 사이의 빈 공간과 높이를 구분한다. 각 요소
내부의 오목한 부분은 여전히 채워지며, 실제 mesh 충돌 검사나 실시간 관절
자세 추적은 아니다. 실행 중인 시뮬레이션과 동일한 `cad_frame.yaml`을 사용한다.

```bash
ros2 run cleany_gazebo_sim body_guard_observer --ros-args \
  -p use_sim_time:=true \
  -p profile:=/absolute/run/config/cad_frame.yaml \
  -p legacy_config:=/absolute/run/config/nav2_safety.yaml \
  -p output_directory:=/absolute/run/body_guard_comparison
rviz2 -d ros2_ws/src/cleany_gazebo_sim/config/rviz/body_guard_comparison.rviz \
  --ros-args -p use_sim_time:=true
```

기존 세션에 연결할 때는 동일한 `ROS_DOMAIN_ID`를 설정한다. 기존 주행 창을
종료하지 않고 별도 RViz로 관찰할 수 있다. `/body_guard/markers`는 원본 visual mesh와 primitive를 원래 재질 색으로 표시한다.
녹색 얇은 선은 margin을 포함하는 부품별 경계 상자, 자홍색은 기존 사각 StopZone에
들어온 점, 빨간색은 새 3D 정지 범위에 들어온 점이다. 녹색 윤곽선은 Euclidean
margin의 시각화용 외접 상자라 모서리에서 실제 판정보다 넓다.

`/scan`, `/camera/head/depth/points`를 센서 시각의 TF와 odom으로 현재
`base_link`에 옮긴다. 기존 depth 높이 필터를 적용하며 4m 이상 depth 점은
clearing 반환값과 혼동하지 않도록 제외한다. 기존 StopZone은 사각형을 전제로
비교한다. 관측 점에서 가장 가까운 부품까지 거리, 기존/신규 영역의 점 개수,
판정을 `/body_guard/state`, 출력 디렉터리 `latest.json`과 `changes.jsonl`에
남긴다. `geometry.json`과 센서별 `*_latest.npz`는 오프라인 재검증용이다.

기본 margin 0.02m, 반응시간 0.15s, 선감속 0.4m/s², 각감속 0.6rad/s²는
**평가용 가정이며 실측 제동 성능이 아니다**. `/nav2/cmd_vel`의 요청 속도로
반응시간 + 제동시간 동안의 일정한 곡률 경로를 샘플링하며, 샘플 사이 이동량
상한을 margin에 추가한다. 감속 중 곡률 유지 가정에서 경로 길이를 늘려
검사하는 방식이고, 축별 감속 차이·미끄러짐·움직이는 장애물까지 보장하지
않는다. 명령이 오래되면 정지 상태 비교로 돌아가므로 실측 속도 기반의 실제
정지 제어기로 사용할 수 없다. 후방/측방 미관측 공간도 안전하다고 판정하지
않는다. 센서/TF 누락은 `STOP_UNAVAILABLE`, 정상 관측 중 위험 점이 3개 이상이면
`STOP`, 그 외는 `NO_OBSERVED_COLLISION`이다.

이 노드는 **비교 기록 전용(shadow)**이며 속도 토픽을 발행하지 않는다.
실제 `/cmd_vel` 정지는 기존 Collision Monitor가 계속 담당한다. 실기 적용에는
실시간 자세, 실측 속도와 제동 특성, 센서 사각지대 대응 및 별도 검증이 필요하다.

이전 8cm 비교 실행의 정지 장면 `chair_surface_full_route_20260910_02/body_guard_comparison`의
실행 관측에서는 기존 사각 영역 depth 점 8개, 3D 부품 + 8cm 영역 점 0개,
최소 부품 경계 거리 0.09143m였다. 센서 점 9,510개를 보존한 오프라인 비교에서
당시 요청 각속도 +0.3rad/s의 정지 예상 범위에는 0개, 반대 방향 -0.3rad/s에는
316개가 들어왔다. 실제 로봇을 움직인 결과가 아니며 제동 가정을 포함한다.
이 프로토타입의 해당 회전 검사에는 약 1.2초가 걸렸으므로 실시간 제어에
연결하기 전에 충돌 계산 최적화와 실제 처리 지연 반영이 필요하다.

2cm는 시뮬레이션 비교용 시험값이다. 실제 mesh 표시는 판정 방식 변경을 뜻하지
않으며, 판정은 계속 visual/collision 요소의 OBB 거리로 수행한다. 기존 Collision
Monitor의 8cm 설정은 이 관찰 노드의 margin과 별개이며 변경하지 않는다.

사진을 참고한 기본 양팔 park 자세는 좌우 shoulder pitch 3.06rad, elbow pitch
3.14rad, wrist pitch -2.0rad, wrist roll 0rad, gripper 0rad이다. 어깨 yaw는
좌 -π/2, 우 +π/2로 유지해 양팔이 대칭인 전후 수직 평면에서 접히도록 했다.
3번 팔꿈치는 URDF 상한 3.14rad까지 접는다. 어깨는 3.06rad로 유지하고 손목 pitch를 -2.0rad로
조정해 그리퍼가 수직에서 팔꿈치 쪽으로 약 20도 기울도록 했다. 단일 사진을 참고한
시뮬레이션 근사 자세이며 실물 관절각 캘리브레이션 또는 하드웨어 명령이 아니다.
사용자 요청에 따라 `simulation_park_limits`로 양 손목 하한만 -2.0rad까지
확장했다. 원본 URDF의 실물용 한계는 수정하지 않으며, 이 override는 고정
Gazebo 자세와 동일한 자세를 표시/비교하는 body guard에만 적용된다.
고정 자세가 SDF에 반영되므로 변경 후 Gazebo 세션을 다시 생성해야 하며,
body guard에는 해당 세션에 저장한 동일한 profile을 지정한다.

팔꿈치 상한은 모델에 명시된 관절 한계이며 실물 링크 접촉 거리나 자기충돌
안전성을 검증한 수치는 아니다.

바퀴는 각 `*_wheel_link`의 roller/hub visual 및 collision 경계들을
해당 링크 좌표계에서 합쳐 **바퀴당 OBB 하나(총 4개)**로 비교하고 표시한다.
2cm margin은 통합한 물리 경계에 한 번만 추가한다. 바퀴 내부 요소 사이의
틈은 채워지며, 원본 visual mesh와 Gazebo 물리 collision은 그대로 유지한다.

양팔은 shoulder yaw부터 그리퍼까지의 자식 링크 전체와 가운데 카메라 지주,
head pan/tilt 및 카메라를 **상부 직육면체 하나(`upper_body/merged`)**로 통합한다.
프레임 상단 위로 돌출하는 base_link의 장착 부품도 포함한다. 현재 고정 park
자세에서 모든 대상 OBB를 포함하도록 base_link 축에 맞추며, 양팔 사이 빈
공간도 채운다. 2cm margin은 통합 후 한 번 적용하고 상세 mesh는 유지한다.

알루미늄 프로파일은 base_link의 rail/upright mesh와 20mm 단면의 대응
collision box를 합쳐 `profile_frame/merged` 직육면체 하나로 처리한다.
그 안에 완전히 포함되는 고정 부품의 중복 OBB도 제거한다. 프레임 내부 빈
공간은 장애물 통과 공간으로 취급하지 않으며, 2cm margin은 통합 후 적용한다.
상부 통합 박스와 바퀴별 박스, 상세 visual mesh는 별도로 유지한다.

### 3D 누적 voxel 지도와 직렬 속도 가드 (시뮬레이션)

`tools/navigation_evaluation/live_costmap.py --guard-3d --idle --robot-model cad_frame
--map /absolute/map.yaml --output /absolute/new-run`으로 활성화한다. 지도 좌표계는
저장 지도와 같은 `map`, odometry는 `/wheel/odom`이다. 기본 실행에는 영향을
주지 않으며 3D 가드는 CAD 모델 + 장애물 기록이 있는 실행에만 연결된다.

기존 ObstacleMemory의 센서별 3차원 evidence를 재사용한다. 이 모드에서는
voxel이 지도 XY 해상도 × 지도 XY 해상도 × 0.05m이며, 현재 5cm 지도에서는
5cm 정육면체이다. `/obstacle_memory/voxels`(PointCloud2)는 점유 voxel 중심,
`/obstacle_memory/voxel_status`(String JSON)는 실제 셀 크기, 소스 관측 시각,
기록 상태, 경계와 map ID를 제공한다. `obstacles.npz`에는 원래의 3D evidence가
저장된다. 기존 2D `/obstacle_memory/grid`와 Nav2 경로 계획도 계속 사용한다.

Depth의 유효 marking 점은 전부 반영하고 clearing ray만 최대 1,200개로
줄인다. 장애물은 해당 센서의 새 관측 ray가 같은 높이의 voxel을 통과해야
제거되며, 화면 밖으로 나갔다거나 오래됐다는 이유만으로 삭제하지 않는다.
LiDAR가 낮은 곳에서 관측한 빈 공간으로 높은 depth 장애물을 지우지 않는다.
3D 해상도를 바꿨으므로 이전 10cm 높이 해상도 NPZ는 이 모드에 복원할 수 없다.

속도 연결은 다음과 같다.

```text
/nav2/cmd_vel → Collision Monitor → /safety_2d/cmd_vel
             → voxel_guard → /cmd_vel → Gazebo command watchdog → drive
```

`voxel_guard`는 voxel을 크기 없는 점으로 간주하지 않고 OBB와 실제 셀 부피의
겹침을 15축 분리축 검사로 판정한다. 2cm 여유는 로봇 OBB에만 추가하며 셀
크기는 별도로 검사한다. 현재 요청 속도와 실제 wheel odom 속도 각각으로
정지 예상 경로를 검사한다. 설정 반응시간에 입력 관측 지연을 더하며, 샘플
사이의 이동량도 경계에 포함한다. 가까운 충돌은 `STOP_OBSTACLE`, 더 먼
충돌은 `SLOW_OBSTACLE`, 감지된 충돌이 없으면 `PASS_OBSERVED_SPACE`이다.

`config/voxel_guard.yaml`의 기본 제한은 평면 속도 0.15m/s, 각속도 0.3rad/s다.
반응 0.25s, 선감속 0.4m/s², 각감속 0.6rad/s²는 실험 가정이다. 비례 감속으로
곡률이 유지되는 경로의 길이를 보수적으로 늘려 검사하며, 축별 제동 차이나
미끄러짐, 이동 장애물의 미래 운동을 보장하지 않는다. 셀 해상도 및 2cm
margin도 실제 센서/로봇의 보장 오차가 아니다.

누락/오래된 센서·voxel·wheel odom·TF, 기록 중지, 오래된 명령, 잘못된 입력은
`STOP_UNAVAILABLE`, 처리 시간 한도 초과는 `STOP_PROCESSING_OVERRUN`으로
0 속도를 보낸다. steady timer를 사용하며 Gazebo의 독립 명령 watchdog도
유지한다. 큰 map→odom 점프나 clock reset은 정지를 latch한다. 이때는 기록이
잘못된 좌표로 섞였을 수 있으므로 새 지도로 세션을 다시 시작해야 한다.

`/body_guard_3d/state`와 실행 폴더 `voxel_guard/latest.json`, `transitions.jsonl`에
판정·입출력 속도·처리 시간을 기록한다. 충돌 예상 voxel은
`/body_guard_3d/risk_voxels`로 발행한다. `config/rviz/voxel_navigation.rviz`에서
높이별 누적 3D voxel과 빨간 위험 셀, `/body_guard/markers`의 로봇 형상을
겹쳐 볼 수 있다. 로봇 형상 표시는 같은 세션 profile의 `body_guard_observer`를
별도로 실행하면 된다.

이 기능은 **관측된 voxel에 대한 시뮬레이션 추가 가드**다. 미관측 공간을
자유 공간으로 입증하지 않으며, 후방/측방 3D 가시성이나 팔 자세 변경을
해결하지 않는다. 기존 2D Collision Monitor도 유지하므로 기존 사각 영역에
의한 보수적 정지가 여전히 가능하다. 고정 park 자세의 확인과 시뮬레이션
검증을 위한 단계이며 실물 안전 제어기로 검증된 것은 아니다.

### 단일 목적지 실험: 우회 전 15초 대기

`config/behavior_trees/wait_before_detour.xml`을 `NavigateToPose.Goal.behavior_tree`에
설치된 절대 경로로 지정하면 최초 경로는 즉시 계획하고, 이후 경로가
`IsPathValid` 검사에서 막혔을 때 제어를 취소하고 최대 15초(sim time) 기다린다.
대기 중 경로가 유효해지면 조기에 재개하며, 계속 막혀 있으면 15초 후 재계획한다.
컨트롤러 실패도 15초 대기 후 재계획하며 최대 3회 재시도한다.
지도 강제 초기화나 즉시 회전·후진 복구는 하지 않는다.
이 정책은 단일 목적지 시뮬레이션용이며 사람 추적·통과시간 예측은 포함하지 않는다.
3D 가드만 막히고 2D 경로는 유효하면 컨트롤러 실패를 통해 대기에 진입하므로,
가드 정지 시점부터 정확히 15초라는 의미는 아니다. 기존 기본 BT는 유지한다.

### 전후·좌우·회전 분리 주행

`amcl_nav2_safety.launch.py`는 기본 `axis_motion:=true`로
`config/nav2_axis_motion.yaml`을 병합해 DWB와 `cleany_axis_controller::AxisGenerator`를
사용한다. `axis_motion:=false`면 기존 MPPI 설정을 사용한다.
`axis_params_file`로 별도 단일 축 파라미터 파일을 지정할 수 있다.
평가 실행은 사용한 YAML을 결과 폴더에 복사한다.
단일 축 후보를 계획 단계에서 평가하고, 축 전환은 wheel odom 정지를
0.3초 확인한 뒤 허용한다. 센서 pan 제어는 이 변경에 포함되지 않는다.
15초 대기 BT 및 직렬 2D/3D 가드는 그대로 함께 사용할 수 있다.

### 대기 후 즉시 재대기하던 현상 수정

단일 축 프로필의 전역 플래너는 5cm Omni primitive를 사용하는
`nav2_smac_planner::SmacPlannerLattice`다. Navfn의 중심 경로와 polygon
footprint 재검사 간 차이를 줄이기 위해 footprint를 고려해 검색하며
경로 smoothing은 끈다. primitive 경로는 설치된 `nav2_smac_planner` 패키지에서
해석한다. 센서 갱신으로 생성 직후에도 경로가 달라질 수 있으므로 안전 검사는 유지한다.

15초 대기 BT는 `TruncatePathLocal`로 전방 경로 1.5m만 검사한다.
먼 곳의 움직이는 장애물이 바뀔 때마다 즉시 멈추지 않으며, 다가갈 구간이
막혔을 때만 양보 대기 후 전역 경로를 다시 계산한다.
`YieldWait`는 의도적인 양보이므로 recovery count를 증가시키지 않는다.
컨트롤러 실패 시의 `Wait`는 기존처럼 복구로 집계한다.
ReactiveSequence가 FollowPath를 중단하므로 CancelControl을 중복 호출하지 않는다.

FollowPath 재시작에는 원래 경로 전체 대신 `TruncatePathLocal`로 현재 위치부터
남은 경로를 전달한다. 지나온 시작점 때문에 DWB가 빈 로컬 경로를 만드는
문제를 방지한다. 2026-09-10 Gazebo 재시험에서 조기 재개(0.77초),
15.05초 대기 후 재계획·출발, 양보 대기 후 recovery count 0을 확인했다.
이는 대기/재개 검증이며 전체 목적지 도달 검증은 별도다.

### 이동 방향을 먼저 보는 pan 인터록 (CAD / Harmonic 시뮬레이션)

`cad_frame.yaml`의 `dynamic_pan: true`는 `head_pan_joint`를 가동 관절로 유지한다.
모델 제한은 ±3.2 rad이며 시뮬레이션 회전 속도 설정은 1 rad/s이다.
실제 하드웨어의 배선·간섭 및 허용 범위를 검증한 값은 아니다.

안전 launch는 `/nav2/cmd_vel → pan_motion_gate → /pan_ready/cmd_vel →
collision_monitor → 기존 3D guard` 순서로 연결한다. 횡이동은 좌 +90°, 우 −90°,
전진은 0°로 향한다. 방향 변경 시 wheel odom으로 차체 정지를 확인하고, 측정 pan
각도 오차 0.04 rad 이내 및 회전 속도 0.05 rad/s 이하에서 0.2초 안정화한 뒤
그 이후에 촬영된 새 depth cloud를 받아야 속도 명령을 통과시킨다. 오래된 입력과
복합 축 명령은 차단한다. `/head_pan/state`에서 목표/측정 각도와 차단 사유를 본다.

`pan_motion_gate.yaml`의 `allow_reverse`는 기본 false다. true이면 후진 전에
pan을 π rad로 돌리고 같은 인터록을 적용한다. 전방 카메라만으로 회전 중 전체
차체 주변이나 미관측 공간이 안전하다고 보장하지 않으며 기존 가드를 유지한다.
카메라 TF는 `/joint_states` 측정값 및 측정 시각으로 발행한다. 상부 OBB 하나는
카메라 pan의 전체 회전 범위를 포함하며 기존 2cm 마진은 한 번만 적용한다.
회전 중 촬영된 cloud도 해당 시각의 동적 TF로 지도에 투영한다.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             