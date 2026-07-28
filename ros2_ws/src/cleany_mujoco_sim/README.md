# cleany_mujoco_sim

XLeRobot MuJoCo 시뮬레이션을 ROS 2로 연결하는 `ament_python` 패키지입니다.

## 실행

화면 없이 시뮬레이션을 실행합니다.

```bash
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py
```

MuJoCo viewer와 함께 실행합니다.

```bash
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=false
```

## 토픽

`mujoco_sim_node`가 발행하는 토픽은 다음과 같습니다.

- `joint_states` (`sensor_msgs/JointState`)
- `odom` (`nav_msgs/Odometry`)
- `scan` (`sensor_msgs/LaserScan`)
- `publish_odom_tf`가 `true`일 때 `tf` (`odom` -> `base_link`)
- laser scan 발행이 활성화됐을 때 `tf_static` (`base_link` -> `laser`)

`mujoco_sim_node`가 구독하는 토픽은 다음과 같습니다.

- `~/joint_cmd` (`sensor_msgs/JointState`): joint position을 직접 설정합니다.
  controller가 아닌 단순한 시뮬레이션 시험용 인터페이스입니다.
- `cmd_vel` (`geometry_msgs/msg/Twist`): mobile base의 차체 속도 명령을
  수신합니다. 기본 namespace에서는 `/cmd_vel`로 노출됩니다.

`cmd_vel` 입력은 공통
[`cleany_interfaces` mobile base 계약](../cleany_interfaces/docs/mobile_base.md)을
따라 유효성 검사, 축별 속도 제한, command timeout 정지를 적용합니다. 검증된
차체 속도는 메카넘 역기구학을 통해 네 바퀴의 목표 각속도로 변환합니다.

MuJoCo 모델은 메카넘 휠 DC 모터 네 개의 독립적인 전압 입력을 노출합니다.
각 physics step에서 실제 joint 속도를 읽고, feed-forward와 anti-windup을 포함한
휠별 PID controller로 `-10.8~10.8 V`의 모터 전압을 계산합니다.

휠 목표 속도와 전압 controller는 backend 내부 세부사항이며 `/cmd_vel` 계약에
추가되는 공개 ROS 토픽이 아닙니다. 향후 실제 로봇 backend는 동일한 휠 목표
속도를 ESP32에 전달하고, ESP32에서 encoder feedback, PID, PWM 제어를 수행할 수
있습니다.

## Mobile base 구동 모델

각 휠은 독립적인 `PG42-4266-1270NE` 출력축 DC 모터 모델을 사용합니다.
actuator 제어 입력은 다음 이름을 가진 모터 단자 전압입니다: `rear_left_drive`,
`rear_right_drive`, `front_left_drive`, `front_right_drive`.

- 정격 전압: `12 V`
- 감속비: `61:1`
- 제조사 표기 감속기 효율: `72%`이며 제조사 출력 토크에 이미 반영됨
- 제조사 정격 출력: `103 rpm`에서 `2.94 N.m`
- 계산된 무부하 출력: `7000 / 61 rpm` (`12.017 rad/s`)
- 10% margin을 적용한 시뮬레이션 운용 한계: `10.8 V`, `2.646 N.m`
- margin을 적용한 정격점: `92.7 rpm` (`9.708 rad/s`)
- margin을 적용한 무부하 평형 속도: `103.28 rpm` (`10.815 rad/s`)

MJCF actuator는 감속기 출력축에 직접 모델링되어 있습니다(`gear=1`). 따라서
감속비와 효율을 다시 적용하지 않습니다. 모터 입력은 목표 휠 속도가 아니라
전압이므로 향후 base adapter가 휠 mixing과 폐루프 속도 제어를 수행해야 합니다.

## 로봇팔 서보 모델

두 로봇팔은 다음과 같이 Feetech 12 V serial servo를 사용합니다.

- `Pitch_L`, `Elbow_L`, `Pitch_R`, `Elbow_R`: `STS3250`
- shoulder rotation, wrist pitch/roll, jaw, head pan/tilt: `STS3215`

모델의 출력 한계에는 제조사 사양 대비 10%의 운용 margin을 적용했습니다.

- `STS3215` (`ST-3215-C018`): `2.648 N.m` peak 한계,
  margin 적용 정격 토크 `0.883 N.m`, 무부하 한계 `4.245 rad/s`
- `STS3250` (`ST-3250-C001`): `4.413 N.m` peak 한계,
  margin 적용 정격 토크 `1.412 N.m`, 무부하 한계 `7.086 rad/s`

position actuator와 joint force 한계에는 peak stall torque의 90%를 사용합니다.
joint damping은 포화 상태에서 각 무부하 속도의 90%를 재현하도록 보정했습니다.
Feetech는 PID를 설정할 수 있다고 명시하지만 고정된 factory gain을 공개하지
않으므로 기존 시뮬레이션 position-loop gain을 유지합니다. 전류, 열, 2초 과부하
차단 동작은 아직 모델링하지 않았습니다.

## Launch 인자

`mujoco_sim.launch.py`가 제공하는 인자는 다음과 같습니다.

- `scene_path`: MuJoCo scene XML 경로. 기본값은 `hardware/scene.xml`
- `publish_rate_hz`: 시뮬레이션 발행 및 timer 주기. 기본값은 `60.0`
- `headless`: MuJoCo viewer를 숨길지 여부. 기본값은 `true`
- `scan_rate_hz`: laser scan 발행 주기. 기본값은 `5.5`
- `scan_samples`: scan당 ray 개수. `0`이면 `scan_sample_rate_hz`에서 계산
- `max_linear_x`: 전후 방향 최대 속도의 절댓값. 기본값은 `0.3 m/s`
- `max_linear_y`: 좌우 방향 최대 속도의 절댓값. 기본값은 `0.3 m/s`
- `max_angular_z`: yaw 최대 회전 속도의 절댓값. 기본값은 `0.8 rad/s`
- `cmd_vel_timeout_sec`: 새 명령이 없을 때 정지하기까지의 시간. 기본값은
  `0.5 s`
- `timeout_check_rate_hz`: command timeout 확인 주기. 기본값은 `20.0`
- `wheel_radius`: 메카넘 휠의 유효 반지름. 기본값은 `0.0635 m`
- `wheelbase_length`: 앞뒤 휠 중심 사이 거리. 기본값은 `0.30 m`
- `track_width`: 좌우 휠 중심 사이 거리. 기본값은 `0.51 m`
- `max_wheel_speed`: 목표 휠 속도의 최대 절댓값. 기본값은
  `10.815 rad/s`
- `base_drive_enabled`: 폐루프 휠 구동 controller 활성화 여부. 기본값은
  `true`
- `wheel_kp`: 휠 속도 PID의 proportional gain. 기본값은 `1.0`
- `wheel_ki`: 휠 속도 PID의 integral gain. 기본값은 `5.0`
- `wheel_kd`: 휠 속도 PID의 derivative gain. 기본값은 `0.0`
- `motor_voltage_limit`: 모터 전압의 최대 절댓값. 기본값은 `10.8 V`
- `motor_no_load_speed`: 최대 전압에서의 무부하 휠 속도. 기본값은
  `10.815 rad/s`

`MujocoSimNode`가 추가로 지원하는 parameter는 다음과 같습니다.

- `base_body_name`: robot base로 사용하는 MuJoCo body. 기본값은 `chassis`
- `lidar_site_name`: laser 원점으로 사용하는 MuJoCo site. 기본값은
  `lidar_site`
- `odom_frame_id`: odometry frame ID. 기본값은 `odom`
- `base_frame_id`: base frame ID. 기본값은 `base_link`
- `laser_frame_id`: laser frame ID. 기본값은 `laser`
- `publish_odom_tf`: 동적 odom-to-base transform 발행 여부. 기본값은 `true`
- `scan_enabled`: `scan` 및 정적 laser transform 발행 여부. 기본값은 `true`
- `scan_sample_rate_hz`: `scan_samples`가 `0`일 때 사용하는 가상 ray sampling
  주기. 기본값은 `8000.0`
- `scan_range_min`: 유효한 최소 scan 거리. 기본값은 `0.15`
- `scan_range_max`: 유효한 최대 scan 거리. 기본값은 `12.0`

## 구현 범위

이 패키지는 아직 전체 robot interface가 아닌 simulation bridge입니다.

구현된 항목:

- XLeRobot MuJoCo scene 불러오기
- 독립적인 PG42 drive motor를 사용하는 5-inch 관절형 메카넘 휠 네 개 시뮬레이션
- ROS timer를 통한 시뮬레이터 step 진행
- joint state, odometry, laser scan, TF data 발행
- 시뮬레이션 시험을 위한 직접 joint position 명령 적용
- `/cmd_vel` 유효성 검사, 속도 제한, command timeout 정지 목표 적용
- 차체 속도를 네 바퀴 목표 각속도로 변환하는 메카넘 역기구학
- 네 바퀴의 속도 비율을 유지하는 목표 휠 속도 제한
- physics timestep마다 실행되는 휠별 속도 PID와 전압 feed-forward
- `-10.8~10.8 V` 전압 제한 및 integral windup 방지
- 목표 휠 속도에 따른 MuJoCo DC motor actuator 제어

아직 구현되지 않은 항목:

- `cleany_robot_interface`, `cleany_perception`, mission FSM 연결
- 실제 encoder noise, 전류, 열, 과부하 차단 동작
- manipulator의 실제 하드웨어 특성을 반영한 controller

현재 `/cmd_vel` subscriber는 다음 명령으로 확인할 수 있습니다.

```bash
ros2 node info /mujoco_sim
```

시험 명령은 별도 terminal에서 반복 발행합니다.

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.05}, angular: {z: 0.1}}'
```

위 명령을 발행하는 동안 시뮬레이션 로봇은 전진, 좌측 횡이동, 반시계 회전을
동시에 수행합니다. 명령 발행이 중단되면 command timeout 후 정지합니다.

기본 PID gain은 현재 MuJoCo DC motor 모델의 step response를 기준으로 설정한
시뮬레이션 값입니다. 실제 ESP32 motor controller에는 그대로 사용하지 않고
실물 encoder와 구동계를 기준으로 별도 튜닝해야 합니다.
