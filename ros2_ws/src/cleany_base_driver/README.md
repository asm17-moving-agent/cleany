# cleany_base_driver

`/cmd_vel`을 메카넘 wheel 목표로 변환하고, micro-ROS WheelState를 wheel
JointState와 diagnostics로 변환하는 Humble driver다. `cleany_base_odometry`를
연결해 기본 wheel odometry를 발행한다.

```text
cmd_vel → 검증 / monotonic timeout / 역기구학 / 비율 scaling
        → base/wheel_command → Agent → MCU의 독립 motor-control task
MCU → base/wheel_state → joint_states → wheel/odom → odom 및 TF relay
```

## ROS 인터페이스

모든 node topic과 service는 상대 이름을 사용한다. Root namespace의 기본 계약:

| 이름 | 타입 | 역할 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `base_link` 기준 x, y, yaw 명령 |
| `/base/wheel_command` | `cleany_base_interfaces/msg/WheelCommand` | 50 Hz MCU 목표 |
| `/base/wheel_state` | `cleany_base_interfaces/msg/WheelState` | 50 Hz MCU 피드백 |
| `/base/enable` | `std_srvs/srv/SetBool` | 명시적 구동 허가 / 소프트웨어 STOP |
| `/joint_states` | `sensor_msgs/msg/JointState` | 누적 wheel rad, 측정 rad/s |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 피드백 stale, fault, 구동 허가 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | wheel odometry node 출력 |
| `/odom` | `nav_msgs/msg/Odometry` | canonical odometry relay 출력 |
| `/tf` | `tf2_msgs/msg/TFMessage` | relay의 `odom → base_link` |

Wheel topic은 best effort, volatile, keep last 1이며 `cmd_vel`은 reliable이다.
Wheel 배열 순서는 FL, FR, RL, RR이며 MCU가 PCB 순서로 변환한다. Joint 이름은
`front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`,
`rear_right_wheel_joint`다. ROS에서 wheel 또는 encoder 부호를 다시 반전하지 않는다.

### 명령과 구동 허가

- 여섯 Twist 축 중 NaN/Inf가 있으면 전체 명령을 폐기하고 STOP한다.
- 지원하지 않는 `linear.z`, `angular.x`, `angular.y`는 경고하고 무시한다.
- x, y, yaw를 차체 제한으로 각각 clamp한 뒤, 휠 최대값을 넘으면 모든 휠을 같은
  비율로 줄인다. 역기구학의 wheel 비율을 개별 clipping으로 바꾸지 않는다.
- 명령 timeout과 통신 피드백 감시는 monotonic time을 사용한다. 안전 timer도
  steady clock을 사용하므로 ROS clock 변경에 구동 timeout을 의존하지 않는다.
- 시작, 재부팅, 피드백 중단과 재연결 시 driver는 구동 비허가 상태다. Agent 연결이나
  `/cmd_vel` 발행만으로 구동 허가를 얻지 않는다.
- `/base/enable` true 요청으로 zero ENABLE을 보내며 이전 목표를 버린다.
  MCU의 enabled 확인 뒤 요청 이후 수신한 새 목표를
  전송하며, 아직 목표가 없으면 zero VELOCITY로 대기한다.
- ENABLE 전달 중 손실은 **동일 sequence/deadline**의 패킷으로 재시도한다.
  원래 유효기한을 연장하지 않는다. 이미 허가된 상태에서 enable 요청은 멱등적이다.
- STOP, MCU 명령 만료, 연결 손실과 재부팅 뒤 cached 이동 목표를 복원하지 않는다.
  다시 구동하려면 명시적 enable과 새 명령이 필요하다.
- 피드백 sequence 검사는 수신 경계에서 한 번 수행한다. 중복과 역순은 무시하고
  누락 개수가 아닌 실제 수신 지연으로 연결 상태를 판단한다.

Protocol v2의 ENABLE/STOP, sequence와 MCU deadline 계약은
[`cleany_base_interfaces`](../cleany_base_interfaces/README.md)를 따른다.
한 시점에는 하나의 `/cmd_vel` 명령원만 사용한다. Nav2, teleop, 시험 명령의 선택은
상위 command mux의 책임이다. 한 MCU의 wheel-command publisher는 하나의 driver가
소유하며 MCU는 micro-ROS 단일 제어 경로를 사용한다.

### 피드백과 odometry

MCU timestamp는 monotonic microseconds이며 ROS epoch가 아니다. JointState는
ROS 수신 시각으로 stamp를 붙인다. Signed int32 encoder rollover는 modular delta로
누적한다. 부팅 ID 변경 및 관측 단절 뒤에는 baseline을 재설정하고 누적 wheel
각도를 유지해 가짜 odometry 이동을 만들지 않는다. 단절 중 이동량은 관측되지 않는다.

기본 launch의 publisher 소유권은 다음과 같다.

- `cleany_base_odometry`: `/wheel/odom`, TF 없음.
- `base_odom_relay`: `/odom`과 `odom → base_link` TF의 단일 소유자.
- 추후 융합 estimator가 canonical odometry를 소유하면 `relay_odom:=false`로
  relay를 끈다. Sim backend와 Real backend를 같은 canonical topic에 함께 실행하지 않는다.

## 설정

원본은 레포의 `configs/robot/`에 있고 package share의 `config/`로 설치한다.
하나의 profile geometry를 driver와 wheel odometry에 함께 전달한다.

- `base_hardware.yaml`: `mode: hardware`. 사용자가 확인한 휠 직경 127 mm,
  앞뒤 중심 간 거리 350 mm, 좌우 중심 간 거리 610 mm를 사용한다.
  아래 제한값은 바퀴를 띄운 초기 시험용이다. 바닥 주행에는 하중과 정지 성능을
  검증한 별도 제한값을 사용한다.
- `base_synthetic.yaml`: `mode: synthetic`, 장치 없는 mock 전용 합성값이다.
  `mock:=false`와 함께 사용할 수 없다. 실물 calibration 또는 주행 허용값이 아니다.

Geometry는 wheel radius, wheelbase, wheel separation의 양의 유한 meter 값이다.
실물 profile은 각각 `0.0635`, `0.350`, `0.610 m`다. 프레임 외곽 치수가 아니라
휠 중심 간 거리이며, 휠의 127 mm는 반지름이 아닌 직경이다.
Limits는 x/y m/s, yaw rad/s, wheel rad/s와 command timeout seconds다.
Wheel 제한은 MCU 계약인 10 rad/s 이하여야 한다.

| 실물 초기 시험 설정 | 값 |
|---|---|
| x/y 입력 속도 상한 | 각각 0.5 m/s |
| yaw 입력 속도 상한 | 1.0 rad/s |
| 각 휠 목표 속도 상한 | 10.0 rad/s |
| `cmd_vel` timeout | 0.3초 |

x/y, yaw와 휠 상한은 각각 지정된 값이다. 휠 반지름 `r = 0.0635 m`에서
순수 전후 또는 좌우 0.5 m/s 명령은 driver 기준 약 7.874 rad/s 휠 목표가 된다.
메카넘 회전 계수 `(wheelbase + wheel_separation) / 2 = 0.480 m`에서
제자리 회전 1 rad/s 명령은 약 7.559 rad/s 휠 목표가 된다.
대각선 이동이나 회전과 병진이 섞여 휠 10 rad/s를 넘으면 모든 휠을 같은 비율로
줄인다. MCU는 휠 목표를 0.1 rad/s 단위로 반올림한다.
실제 이동 속도는 하중, 슬립과 제어 응답을 측정해 확인한다.

`cmd_vel` 수신이 0.3초를 넘게 끊기면 host가 STOP한다. 그동안 driver는 마지막
목표를 50 Hz로 전송하며, 각 MCU 명령 deadline은 추정 MCU 시각에서 최대 200 ms다.
MCU watchdog 상한 250 ms와 host 피드백 stale 기준 250 ms는 별도 동작한다.
Launch는 누락된 제한값과 양수가 아닌 값, NaN/Inf 및 휠 10 rad/s 초과를 거부한다.

## 장치 없는 실행과 검증

Ubuntu 22.04 `ros2-humble` Distrobox에서 레포 루트 기준:

```bash
make build-base
make test-base
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=173
ros2 launch cleany_base_driver base_driver.launch.py mock:=true
```

별도 terminal에서도 같은 ROS overlay와 domain을 적용한다. Mock 시험 시 Agent나
실물 MCU를 이 domain에 연결하지 않는다. 연결 뒤 명시적으로 enable하고 **그 뒤**
새 합성 주행 명령을 발행할 수 있다.

```bash
ros2 topic echo /diagnostics
ros2 service call /base/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic pub -r 50 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.05}, angular: {z: 0.0}}'
# 별도 terminal에서 소프트웨어 정지
ros2 service call /base/enable std_srvs/srv/SetBool '{data: false}'
```

Mock는 `mock/reboot` (`std_srvs/Trigger`)와 `mock/transport` (`std_srvs/SetBool`)
고장 주입 service를 제공한다. `mock/drop_next_enable`과 `mock/force_disarm`
(`std_srvs/Trigger`)으로 ENABLE 손실과 구동 허가 해제를 주입할 수 있다.
Mock는 MCU와 같은 ENABLE/STOP, 순번과 단일 deadline 검증을 사용하고,
이상적인 wheel 속도에서 encoder를 적분한다.
Core test와 ROS graph test는 명령 검증, 구동 허가,
timeout, 재부팅, 피드백 중단, encoder rollover와 odometry/TF 소유권을 검사한다.
Mock 결과는 실제 PI 응답, USB 장치 동작 또는 물리 정지 거리의 검증이 아니다.

## 실물 실행 절차

Upload와 하드웨어 시험은 별도 명시 요청 후 수행한다. 로봇 연결과 emergency stop
준비, 시험 조건에 맞는 hardware profile 및 wheel sign 확인이 선행 조건이다.
레포의 초기 profile을 사용할 때는 로봇을 지지대에 고정하고 모든 바퀴가 바닥과
접촉하지 않는 상태에서 시험한다.

1. [`esp32`](../../../esp32/README.md)의 명시적 upload 절차로
   micro-ROS firmware를 선택한다. Native USB persistent by-id path를 확인한다.
2. 고정 Agent overlay를 적용한 terminal에서 serial Agent를 실행한다.
3. ROS overlay를 적용한 다른 terminal에서 시험 조건에 맞는 hardware profile로
   launch한다. 아래 예시는 레포 루트에서 바퀴를 띄운 초기 시험용 profile을 선택한다.

```bash
source /opt/ros/humble/setup.bash
source esp32/micro_ros/agent/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev "$CLEANY_ESP_PORT" -b 115200

# 별도 terminal
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch cleany_base_driver base_driver.launch.py \
  mock:=false config:="$(pwd)/configs/robot/base_hardware.yaml"
```

Native USB의 baud 값은 Agent serial 설정값이며 USB-UART bridge를 뜻하지 않는다.
현재 MCU의 DDS domain은 0이다. Mock의 domain 173 설정을 실물 terminal에 남기지 않는다.
상태/diagnostics, `/wheel/odom`, `/odom`과 TF publisher 소유권을 확인하고
[`esp32`의 실물 인수 체크리스트](../../../esp32/README.md#robot-acceptance)를
수행한다.
