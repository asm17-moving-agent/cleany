# cleany_base_odometry 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## wheel_odometry

실행 파일: `wheel_odometry_node`. 원본: [wheel_odom_node.py](../cleany_base_odometry/wheel_odom_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | Sensor; `input_topic` |
| 토픽 발행 | `wheel/odom` | `nav_msgs/msg/Odometry` | R10; 유효 관측 구간 준비 후 입력 수신 시, `output_topic` |

TF는 발행하지 않는다. 의미는 [Base odometry 계약](../../cleany_base_driver/docs/command-and-odometry.md)을 따른다.
