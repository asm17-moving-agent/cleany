# cleany_grasping 노드 인터페이스

`cleany_grasping` 패키지가 제공하는 파지 계획(Grasp Planning) 관련 노드와 ROS 2 인터페이스 명세입니다.
전체 시스템의 인터페이스 색인 및 QoS 표준은 [ROS 인터페이스 종합 색인](../../../../docs/ROS_INTERFACES.md)을 따릅니다.

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

`PlanGrasp` 서비스의 데이터 필드 및 파지 규격은 [Scene과 Grasp 인터페이스 계약](../../cleany_interfaces/docs/scene-and-grasp.md)을 따릅니다.
