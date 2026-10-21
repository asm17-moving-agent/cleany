# cleany_grasping 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## grasp_server

실행 파일: `grasp_server`. 시작 설정: [runtime.py](../cleany_grasping/runtime.py). 노드 구현: [grasp_node.py](../cleany_grasping/grasp_node.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 서비스 제공 | `grasp/plan` | `cleany_interfaces/srv/PlanGrasp` | Service 기본값; `service_name` |
| 토픽 발행 | `grasp/debug_image` | `sensor_msgs/msg/Image` | R/T1; 계획 요청 처리 시, `debug_image_topic` |
| 토픽 구독 | `/tf` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |
| 토픽 구독 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF 기본 QoS |

`PlanGrasp`는 서비스다. 호출 의미는 [Scene와 Grasp 계약](../../cleany_interfaces/docs/scene-and-grasp.md)을 따른다.
