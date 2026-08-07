# cleany_mujoco_sim

XLeRobot MuJoCo 시뮬레이션을 ROS 2 `ament_python` 패키지로 연결한다.

## 상태와 책임

이 패키지는 구현된 시뮬레이션 브리지이며, 전체 로봇 인터페이스는 아니다. MuJoCo
장면을 불러오고 상태를 발행하며, 테스트용 관절 위치 직접 명령과 mobile-base
`cmd_vel` 명령을 처리한다. 운영 환경의 내비게이션, 매니퓰레이션 및 하드웨어
adapter는 이 패키지의 범위에 포함하지 않는다.

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

## ROS contract

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

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.05}, angular: {z: 0.1}}'
```

발행 중에는 전진·좌측 횡이동·반시계 회전이 함께 적용되고, 발행이 중단되면
command timeout 후 정지 목표를 적용한다.

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

## 범위

구현된 기능:

- `cleany_description`의 authoritative MJCF를 simulator-owned scene에 load
- 독립적인 PG42 motor를 사용하는 네 개의 5-inch mecanum drive wheel 시뮬레이션
- ROS timer로 simulator step, joint state·odometry·laser scan·TF 발행
- 시뮬레이션 시험용 직접 joint position 명령
- `/cmd_vel` 검증·제한·timeout과 메카넘 역기구학
- 휠별 PID, feed-forward, anti-windup, 전압 제한을 통한 actuator 제어

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
