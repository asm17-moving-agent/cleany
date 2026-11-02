# cleany_safety_adapter 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## safety_adapter

실행 파일: `safety_adapter`. 원본: [node.py](../cleany_safety_adapter/node.py), [evidence.py](../cleany_safety_adapter/evidence.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 발행 | `safety/status` | `cleany_interfaces/msg/SafetyStatus` | R10; `rate_hz` 20 Hz |
| 토픽 구독 | `<estop_released_topic>` | `std_msgs/msg/Bool` | R10; 기본 빈 값이면 구독 없음 |
| 토픽 구독 | `<protection_ready_topic>` | `std_msgs/msg/Bool` | R10; 기본 빈 값이면 구독 없음 |
| 토픽 구독 | `odom` | `nav_msgs/msg/Odometry` | Sensor; `odom_topic` |
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | Sensor; `joint_states_topic` |

독립 안전 계층의 증거를 System Supervisor에 전달할 뿐이며 구동을 차단하거나 E-stop을 해제하지 않는다. 확인할 수 없는 항목은 항상 안전하지 않은 값으로 보고한다.

| 출력 | 증거 | 증거가 없거나 오래됐을 때 |
|---|---|---|
| `protection_ready` | 보호 계층 준비 Bool | false |
| `estop_engaged`, `physical_release_confirmed` | 물리 해제와 회로 reset 완료 Bool | engaged=true, release=false |
| `stopped` | 최신 odom twist와 모든 관절 속도가 임계값 이하 | false |
| `detail` | 판정 이유 | 미설정 source와 stale 항목 명시 |

Bool에는 header가 없어 수신 monotonic 시각으로 freshness(`evidence_timeout_s` 0.3초)를 판단한다. 설정한 하드웨어 source가 각각 한 번 보고하기 전에는 발행하지 않으므로, 부팅 순서 때문에 Supervisor가 E-stop을 latch하지 않는다. Source를 설정하지 않았으면 즉시 안전하지 않은 값을 발행한다. `stopped`는 encoder 기반 측정이며 독립 정지 센서나 STO 상태를 대체하지 않는다. 정지 임계값(`stopped_linear_mps` 0.01, `stopped_angular_radps` 0.02, `stopped_joint_radps` 0.02)은 미검증 초기값이다.

현재 Cleany에는 이 Bool을 발행하는 하드웨어가 없다. E-stop 회로 설계와 adapter 또는 MCU 펌웨어는 KB에서 미정이다.
