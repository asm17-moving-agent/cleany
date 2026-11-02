# cleany_base_driver 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## cleany_base_driver

실행 파일: `base_driver_node`. 원본: [driver_node.py](../cleany_base_driver/driver_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 발행 | `base/wheel_command` | `cleany_base_interfaces/msg/WheelCommand` | B1; 50 Hz |
| 토픽 발행 | `joint_states` | `sensor_msgs/msg/JointState` | R10; fresh wheel feedback 수신 시 |
| 토픽 발행 | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | R10; 10 Hz |
| 토픽 구독 | `base/wheel_state` | `cleany_base_interfaces/msg/WheelState` | B1 |
| 토픽 구독 | `cmd_vel` | `geometry_msgs/msg/Twist` | R10 |
| 서비스 제공 | `base/enable` | `std_srvs/srv/SetBool` | Service 기본값 |

이름은 ROS remap으로 변경한다. 구동 허가와 정지 규칙은 [Base 계약](command-and-odometry.md)을 따른다.

## mock_base_mcu

실행 파일: `mock_mcu_node`. 원본: [mock_node.py](../cleany_base_driver/mock_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 발행 | `base/wheel_state` | `cleany_base_interfaces/msg/WheelState` | B1; 50 Hz, transport 허용 시 |
| 토픽 구독 | `base/wheel_command` | `cleany_base_interfaces/msg/WheelCommand` | B1 |
| 서비스 제공 | `mock/reboot` | `std_srvs/srv/Trigger` | Service 기본값 |
| 서비스 제공 | `mock/transport` | `std_srvs/srv/SetBool` | Service 기본값 |
| 서비스 제공 | `mock/drop_next_enable` | `std_srvs/srv/Trigger` | Service 기본값 |
| 서비스 제공 | `mock/force_disarm` | `std_srvs/srv/Trigger` | Service 기본값 |

실물 MCU 대신 실행하는 장치 없는 시험 노드다.

## base_odom_relay

실행 파일: `odom_relay_node`. 원본: [odom_relay.py](../cleany_base_driver/odom_relay.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `wheel/odom` | `nav_msgs/msg/Odometry` | R10 |
| 토픽 발행 | `odom` | `nav_msgs/msg/Odometry` | R10; 입력 수신 시 |
| 토픽 발행 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

`odom → base_link` TF를 발행한다. 외부 estimator 또는 Gazebo canonical publisher와 중복 실행하지 않는다.

## cleany_imu

실행 파일: `imu_driver_node`. 원본: [imu_node.py](../cleany_base_driver/imu_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `base/imu_state` | `cleany_base_interfaces/msg/ImuState` | B1 |
| 토픽 발행 | `imu/data_raw` | `sensor_msgs/msg/Imu` | B1; 정상 IMU 수신 시 |
| 토픽 발행 | `imu/data` | `sensor_msgs/msg/Imu` | B1; RUNNING 또는 FALLBACK |
| 토픽 발행 | `imu/status` | `diagnostic_msgs/msg/DiagnosticArray` | R/T1; 상태 변경 시와 1 Hz |
| 토픽 발행 | `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | R10; 상태 변경 시와 1 Hz |

이름은 ROS remap으로 변경한다. 보정과 오류 상태는 [IMU 계약](imu.md)을 따른다.
