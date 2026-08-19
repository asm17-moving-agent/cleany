# cleany_mujoco_sim

XLeRobot MuJoCo 시뮬레이션을 ROS 2 `ament_python` 패키지로 연결한다.

## 상태와 책임

이 패키지는 두 개의 배타적인 MuJoCo 실행 경로를 제공한다. 기존
`mujoco_sim_node`는 mobile-base와 센서 개발용 custom bridge이고,
`handeye_backend.launch.py`는 좌·우 arm trajectory를 위한
`mujoco_ros2_control` backend다. 한 프로세스에서 두 경로를 함께 실행하지 않는다.
운영 환경의 내비게이션 및 실제 하드웨어 adapter는 이 패키지의 범위에 포함하지
않는다.

## 실행과 테스트

아래 명령은 레포지토리 루트에서 실행한다.

```bash
make sim
make test-mujoco
```

MuJoCo viewer가 필요하면 build 후 실행한다.

```bash
make build
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=false
```

RGB-D pick demo는 custom backend만 실행하며 hand-eye ros2_control backend와 함께
사용하지 않는다.

```bash
ros2 launch cleany_mujoco_sim rgbd_pick_demo.launch.py
```

이 장면의 table은 `1.20 x 0.77 x 0.03 m`, 중심은
`(0.635, -0.002, 0.710) m`이며 고정 box/can과 정렬된 RGB-D 및 평가용 OBB를
발행한다. 물체 동역학이나 실제 파지 실행을 검증하는 장면은 아니다.

Hand-eye arm controller backend는 별도로 실행한다. 이 launch는 기존
`mujoco_sim_node`를 include하거나 시작하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_mujoco_sim handeye_backend.launch.py \
  headless:=true sim_speed_factor:=1.0
```

이 backend의 기본값은 `scenes/handeye.xml.in`이다. 전용 scene은 canonical MJCF를
그대로 include하고 `chassis`를 world에 weld하며, 고정 table/stand와
7×5 ChArUco target을 추가한다. 기존 `mujoco_sim.launch.py`와 custom simulator의
기본 `scenes/default.xml.in` 경로는 그대로 유지된다.

Scene 계약과 printable board provenance는
`config/handeye_scene.yaml` (`cleany.handeye_scene/v1`)에 기록한다. Simulation은
30 mm square/15 mm marker nominal 값을 사용한다. Physical profile의 실측값은 현재
`not_measured`/`null`이며 임의 수치로 대체하지 않는다. 실제 인쇄물을 100% scale,
page fitting 비활성화로 출력하고 실측 provenance를 채우기 전에는 physical
preflight가 실패한다.

같은 manifest는 wrist camera render/public contract도 고정한다. 640×480,
vertical FOV 93°에서
`f=(height/2)/tan(fovy/2)=227.751496 px`, `cx=319.5`, `cy=239.5`를 사용한다.
Public `CameraInfo`는 `plumb_bob`, 5개 zero `D`, identity `R`, manifest의 exact
`K/P`를 사용한다. Width/height/FOV/formula/K/D/R/P 중 하나라도 바뀌면 simulation
preflight가 실패하고 adapter가 collection message를 발행하지 않는다.

```bash
ros2 run cleany_mujoco_sim handeye_scene_preflight --profile simulation
# 실측값 기록 전에는 의도적으로 exit 2
ros2 run cleany_mujoco_sim handeye_scene_preflight --profile physical
```

Printable SVG/PDF는 OpenCV 4.5.4 `CharucoBoard`의 7×5,
`DICT_5X5_100`, marker IDs 0–16 패턴을 같은 lossless vector run으로 표현한다.
파일별 SHA-256과 210×150 mm media 크기는 manifest에 고정되어 있다. Target GT는
OpenCV 4.5 object frame(인쇄면 좌하단 원점, +X 오른쪽, +Y 위, +Z 인쇄면 밖)의
`base_T_target`이며 평가 전용이다. PnP 후보 선택이나 solver 입력에는 사용하지 않는다.

전용 raw action/runtime 계약은 다음으로 검증한다.

```bash
cd ros2_ws
pytest -q -s \
  src/cleany_mujoco_sim/test/test_handeye_backend_runtime.py \
  src/cleany_mujoco_sim/test/test_handeye_camera_runtime.py
```

## ROS contract

### Custom simulation backend

`mujoco_sim_node`는 다음 topic을 발행한다.

- `joint_states` (`sensor_msgs/JointState`): drive-wheel joint만 노출하며 passive
  mecanum roller DOF는 내부에 유지한다.
- `odom` (`nav_msgs/Odometry`)
- `scan` (`sensor_msgs/LaserScan`)
- `publish_odom_tf=true`일 때 `tf` (`odom` -> `base_link`)
- laser scan이 활성화됐을 때 `tf_static` (`base_link` -> `laser`)

다음 topic을 구독한다.

- `~/joint_cmd` (`sensor_msgs/JointState`): controller가 아닌 시뮬레이션 시험용
  목표 관절 위치 명령
- `cmd_vel` (`geometry_msgs/msg/Twist`): 기본 namespace에서는 `/cmd_vel`로
  노출되는 mobile-base 차체 속도 명령

`cmd_vel`은 [`cleany_interfaces` mobile-base contract](../cleany_interfaces/docs/mobile_base.md)를
따른다. 지원 축은 `linear.x`, `linear.y`, `angular.z`이며, 잘못된 값은 거부하고
축별 속도 제한과 command timeout 정지를 적용한다. 검증된 차체 속도는 메카넘
역기구학으로 네 휠의 목표 각속도로 변환된다.

`rgbd_pick_demo.launch.py`의 `mujoco_rgbd_sim_node`는 추가로 color-aligned
`camera/color/image_raw`, `camera/depth/image_raw`, 두 CameraInfo와 평가 전용
`ground_truth/objects`를 같은 timestamp로 발행한다.

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.05}, angular: {z: 0.1}}'
```

발행 중에는 전진·좌측 횡이동·반시계 회전이 함께 적용되고, 발행이 중단되면
command timeout 후 정지 목표를 적용한다.

### Hand-eye arm-control backend

`handeye_backend.launch.py`는 `mujoco_ros2_control/ros2_control_node`,
`robot_state_publisher`, `joint_state_broadcaster`와 side별
`joint_trajectory_controller`, camera contract adapter를 시작한다.

- `/left_arm_controller/follow_joint_trajectory`: left arm 5축만 claim
- `/right_arm_controller/follow_joint_trajectory`: right arm 5축만 claim
- `/joint_states`: 양팔 10축 position/velocity와 left/right gripper read-only
  position/velocity
- `/left_wrist_camera/image_raw`: 640×480 RGB, source simulation stamp,
  `left_wrist_rgb_optical_frame`
- `/left_wrist_camera/camera_info`: image와 같은 source stamp/frame 및 manifest의
  exact pinhole model

그리퍼는 MoveIt current-state completeness를 위해 상태만 내보내며 command
interface는 없다. 이 backend는 private `~/joint_cmd` topic을 만들거나 사용하지
않는다. Controller의 joint path/goal tolerance baseline은 각각 `0.05 rad`와
`0.01 rad`이며 `config/handeye_ros2_controllers.yaml`에서 관리한다.

ROS 2 Humble binary의 `mujoco_ros2_control` 0.0.3은 MuJoCo 3.4를 vendor하므로
canonical model의 MuJoCo 3.7 `dcmotor`를 읽을 수 없다. Default `.xml.in` scene을
hand-eye backend로 실행할 때 scene loader가 임시 model copy를 만들고 네 wheel
`dcmotor` actuator와 그 default만 제거한다. 이 임시 copy에는 0.0.3 plugin이
control contract 밖의 head actuator까지 finite command로 초기화하도록 startup
keyframe도 추가한다. Canonical MJCF와 기존 `mujoco_sim_node` materialization은
변경하지 않는다. 직접 `scene_path`에 완성된 `.xml`을 넘기면 호출자가 MuJoCo 3.4
호환성과 `handeye_ros2_control_home` keyframe을 보장해야 한다.

동일한 0.0.3 release에는 별도 `CameraPlugin`이 없다. 이 구현은 release에 실제로
포함된 `mujoco_ros2_control::MujocoCameras`가 `hardware_info.sensors`의
`frame_name`, `info_topic`, `image_topic`, `depth_topic`을 읽는 경로를 사용한다.
Vendor `/left_wrist_rgb/*` 이름은 launch remap으로
`/cleany/internal/mujoco/left_wrist_camera/*` 아래에 격리되고, 작은 adapter만 위의
두 public topic을 발행한다. Adapter는 vendor image와 CameraInfo를 exact source
stamp로 pair한 뒤 public frame/model을 정규화하며 wall clock stamp를 만들지 않는다.
Hand-eye template marker가 있는 경우에만 scene loader가 임시 canonical include의
`left_wrist_rgb` resolution을 640×480으로 materialize한다. Source canonical MJCF,
default scene, custom simulator, Gazebo 경로의 bytes/동작은 바꾸지 않는다.

Simulation camera GT는 manifest의 `evaluation_ground_truth.camera_transform`과
`ground_truth_evaluation.py` pure accessor/metric에만 존재한다. 이는 compiled
`Fixed_Jaw` body에서 `left_wrist_rgb_optical_frame` site까지의 transform이며 solver
입력이나 canonical TF/topic으로 publish되지 않는다.

## 베이스 구동 모델

각 휠은 독립적인 `PG42-4266-1270NE` output-shaft DC motor 모델을 사용한다.
actuator 제어 입력은 `rear_left_drive`, `rear_right_drive`, `front_left_drive`,
`front_right_drive`의 모터 단자 전압이다. `base_link +X`가 전방이며, 양의 yaw는
`+Z` 기준 반시계 방향이다.

- 정격 공급 전압: `12 V`
- 기어박스: `61:1`; 제조사 출력 토크에는 `72%` 효율이 반영되어 있다.
- 제조사 정격 출력: `103 rpm`에서 `2.94 N.m`
- 계산한 무부하 출력: `7000 / 61 rpm` (`12.017 rad/s`)
- 10% 마진을 적용한 시뮬레이션 운용 한계: `10.8 V`, `2.646 N.m`
- 디레이팅한 정격점: `92.7 rpm` (`9.708 rad/s`)
- 디레이팅한 무부하 평형점: `103.28 rpm` (`10.815 rad/s`)

MJCF actuator는 기어박스 출력축에서 직접 모델링하므로(`gear=1`) 감속비와 효율을
다시 적용하지 않는다. 매 physics step마다 실제 joint 속도를 읽고, feed-forward와
anti-windup을 포함한 휠별 PID controller로 `-10.8~10.8 V` 전압을 계산한다.
PID gain은 MuJoCo DC motor step response를 기준으로 한 시뮬레이션 값이며 실제
ESP32 controller에 그대로 사용하지 않는다.

## 팔 서보 모델

양팔은 Feetech 12 V 시리얼 서보를 사용한다.

- left/right shoulder pitch와 elbow pitch: `STS3250`
- shoulder rotation, wrist pitch/roll, jaw, head pan/tilt: `STS3215`

모델의 출력 한계에는 제조사 사양 대비 10% 운용 마진을 적용한다. 위치 actuator와
joint force 한계에는 최대 정지 토크의 90%를 적용한다. 전류, 발열 및 2초 과부하
차단 동작은 아직 모델링하지 않았다.

## 실행 인자

`mujoco_sim.launch.py`는 다음 launch argument를 제공한다.

- `scene_path`: MuJoCo scene XML 또는 `.xml.in` template. 기본값은
  `scenes/default.xml.in`이다.
- `publish_rate_hz`: simulation publish/timer rate. 기본값 `60.0`
- `headless`: viewer를 숨길지 여부. 기본값 `true`
- `scan_rate_hz`: laser scan 발행 주기. 기본값 `5.5`
- `scan_samples`: scan당 ray 수. `0`이면 `scan_sample_rate_hz`에서 계산
- `max_linear_x`, `max_linear_y`, `max_angular_z`: `cmd_vel` 축별 제한값
- `cmd_vel_timeout_sec`, `timeout_check_rate_hz`: 명령 timeout 설정
- `wheel_radius`, `wheelbase_length`, `track_width`, `max_wheel_speed`: 메카넘
  역기구학 설정
- `base_drive_enabled`, `wheel_kp`, `wheel_ki`, `wheel_kd`,
  `motor_voltage_limit`, `motor_no_load_speed`: 휠 drive controller 설정

`MujocoSimNode`는 `base_body_name`, `lidar_site_name`, `odom_frame_id`,
`base_frame_id`, `laser_frame_id`, `publish_odom_tf`, `scan_enabled`,
`scan_sample_rate_hz`, `scan_range_min`, `scan_range_max`도 지원한다.

`handeye_backend.launch.py`는 다음 launch argument를 제공한다.

- `scene_path`: control용 MuJoCo scene XML 또는 `.xml.in` template. 기본값은
  `scenes/handeye.xml.in`
- `headless`: native viewer 비활성화 여부. 기본값 `true`
- `sim_speed_factor`: wall time 대비 simulation speed. 기본값 `1.0`

Camera publish rate와 public model/topic은 launch override가 아니라
`config/handeye_scene.yaml`의 preflight 계약으로 관리한다.

ChArUco의 211개 vector ink box는 target geometry의 authoritative source로
유지한다. 640×480 wrist render에서는 sub-pixel box edge가 ArUco bit를 손상시킬 수
있으므로 `scene_loader`가 같은 box 좌표를 1400×1000 lossless grayscale PNG로
deterministic rasterize하고, materialized temporary scene의 non-collision render
surface에만 적용한다. 보드 바깥 10 mm 흰 quiet zone도 render-only/non-collision이며
210×150 mm object point와 planning-scene collision 형상은 바꾸지 않는다. Canonical
MJCF, `default.xml.in`, SVG/PDF source asset은 수정하지 않으며 temporary texture도
source tree에 기록하지 않는다.

## 범위

구현된 기능:

- `cleany_description`의 authoritative MJCF를 simulator-owned scene에 load
- 독립적인 PG42 motor를 사용하는 네 개의 5-inch mecanum drive wheel 시뮬레이션
- ROS timer로 simulator step, joint state·odometry·laser scan·TF 발행
- 시뮬레이션 시험용 직접 joint position 명령
- `/cmd_vel` 검증·제한·timeout과 메카넘 역기구학
- 휠별 PID, feed-forward, anti-windup, 전압 제한을 통한 actuator 제어
- 좌·우 5축별 표준 `FollowJointTrajectory` action과 12개 arm/gripper joint state를
  제공하는 배타적인 MuJoCo `ros2_control` backend
- Humble release `MujocoCameras` 기반 left wrist RGB/CameraInfo와 public contract
  normalizer

아직 구현되지 않은 기능:

- `cleany_robot_interface`, `cleany_perception` 또는 Mission FSM 연동
- 실제 encoder noise, 전류, 열, 과부하 차단 동작
- 베이스 또는 매니퓰레이터용 하드웨어 현실성 기반 controller

## 관련 KB와 문서 갱신

- [Robot Platform XLeRobot](../../../docs/cleany-docs/20_TECHNICAL/04%20-%20Robot%20Platform%20XLeRobot.md)
- [Navigation and Mapping](../../../docs/cleany-docs/20_TECHNICAL/05%20-%20Navigation%20and%20Mapping.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)

발행 topic, launch parameter, 시뮬레이션 모델 가정 또는 테스트 명령이 바뀌면 이
README도 갱신한다. 시뮬레이션 하드웨어 파라미터를 관련 KB 결정 없이 확정된 실제
하드웨어 사양으로 표현하지 않는다.
