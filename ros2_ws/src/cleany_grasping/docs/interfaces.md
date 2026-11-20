# cleany_grasping 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리합니다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/implementation/ros-interfaces.md)을 따릅니다. 이름 설정이 없는 항목은 ROS remap으로 변경합니다.

## grasp_server

- **실행 파일**: `grasp_server`
- **시작 설정**: [runtime.py](../cleany_grasping/runtime.py)
- **노드 구현**: [grasp_node.py](../cleany_grasping/grasp_node.py)

| 종류 / 방향 | 기본 토픽 및 서비스 이름 | ROS 타입 | QoS 및 파라미터 매핑 | 설명 |
|---|---|---|---|---|
| **서비스 제공** | `grasp/plan` | `cleany_interfaces/srv/PlanGrasp` | 기본 Service QoS (`service_name`) | 파지 후보 생성 및 계획 요청 처리 |
| **서비스 제공** | `/grasp/prepare_window` | `std_srvs/srv/SetBool` | 기본 Service QoS (`anygrasp_prepare_service`) | AnyGrasp 모델 사전 준비 및 180초 유지 Lease 설정 |
| **토픽 구독** | `/manipulation/execute_skill/_action/status` | `action_msgs/msg/GoalStatusArray` | Reliable, Transient Local 1 (`anygrasp_task_status_topic`) | 조작 액션 수행 중 워커 유지 Lease 갱신 |
| **토픽 발행** | `grasp/debug_image` | `sensor_msgs/msg/Image` | Reliable, Transient Local 1 (`debug_image_topic`) | 파지 후보 및 바운딩 박스가 표시된 디버그 이미지 |
| **토픽 구독** | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | TF2 기본 QoS | 카메라-로봇 간 좌표 변환 |

`PlanGrasp`는 서비스입니다. 호출 의미는 [Scene와 Grasp 계약](../../cleany_interfaces/docs/scene-and-grasp.md)을 따릅니다.
