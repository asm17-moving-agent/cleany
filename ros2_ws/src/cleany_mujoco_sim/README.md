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

아직 `/cmd_vel` 기반의 mobile base 명령 인터페이스는 구현되지 않았습니다.
MuJoCo 모델은 메카넘 휠 DC 모터 네 개의 독립적인 전압 입력을 노출하므로,
차체 속도 명령을 모터 입력으로 변환하는 ROS 메카넘 명령 adapter가 필요합니다.
향후 adapter는 공통
[`cleany_interfaces` mobile base 계약](../cleany_interfaces/docs/mobile_base.md)을
따라야 합니다.

1. `/cmd_vel`에서 `geometry_msgs/msg/Twist`를 받아 검증합니다.
2. 지원하는 차체 속도를 네 바퀴의 목표 속도로 변환합니다.
3. 폐루프 휠 속도 제어로 MuJoCo 모터 전압을 계산합니다.

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

아직 구현되지 않은 항목:

- `/cmd_vel` 또는 Nav2 호환 base 명령 처리
- ROS 메카넘 명령 adapter 및 폐루프 휠 속도 controller
- `cleany_robot_interface`, `cleany_perception`, mission FSM 연결
- mobile base 및 manipulator의 실제 하드웨어 특성을 반영한 controller
