# cleany_telemetry 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## cleany_telemetry

실행 파일: `telemetry_node`. 원본: [node.py](../cleany_telemetry/node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/odom` | `nav_msgs/msg/Odometry` | B1; `odom_topic` |

ROS 발행, 서비스, action은 없다. `url`의 Backend pose WebSocket으로 전송하며 `rate` 기본 5 Hz와 `input_timeout` 기본 1.5초를 적용한다.
