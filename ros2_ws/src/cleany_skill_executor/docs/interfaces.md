# cleany_skill_executor 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## grasp_selection_server

실행 파일: `grasp_selection_server`. 원본: [grasp_selection_node.py](../cleany_skill_executor/grasp_selection_node.py), [moveit_adapter.py](../cleany_skill_executor/moveit_adapter.py), [planning_scene.py](../cleany_skill_executor/planning_scene.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `joint_states` | `sensor_msgs/msg/JointState` | R20; `joint_state_topic` |
| 액션 제공 | `grasp/select_reachable` | `cleany_interfaces/action/SelectReachableGrasp` | Action 기본값; `action_name` |
| 서비스 호출 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Service 기본값 |
| 서비스 호출 | `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | Service 기본값 |
| 서비스 호출 | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Service 기본값 |
| 서비스 호출 | `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Service 기본값 |
| 액션 호출 | `/move_action` | `moveit_msgs/action/MoveGroup` | Action 기본값; plan-only |

도달성 검사 action이며 controller 실행을 제공하지 않는다.

## grasp_execution_demo

실행 파일: `grasp_execution_demo`. 원본: [grasp_execution_demo.py](../cleany_skill_executor/grasp_execution_demo.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/joint_states` | `sensor_msgs/msg/JointState` | R20 |
| 토픽 발행 | `/grasp_demo/markers` | `visualization_msgs/msg/MarkerArray` | R/T1; 데모 단계 변화 시 |
| 액션 호출 | `/grasp/select_reachable` | `cleany_interfaces/action/SelectReachableGrasp` | Action 기본값; `selection_action` |
| 액션 호출 | `/move_action` | `moveit_msgs/action/MoveGroup` | Action 기본값; `move_group_action`, 실행을 요청하는 데모 |

선택한 pregrasp와 grasp endpoint를 데모에서 실행한다. 자체적인 `ExecuteManipulationSkill` action server는 없다.

## can_grasp_execution_demo

실행 파일: `can_grasp_execution_demo`. 원본: [can_grasp_execution_demo.py](../cleany_skill_executor/can_grasp_execution_demo.py), [grasp_execution_demo.py](../cleany_skill_executor/grasp_execution_demo.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/joint_states` | `sensor_msgs/msg/JointState` | R20 |
| 토픽 발행 | `/grasp_demo/markers` | `visualization_msgs/msg/MarkerArray` | R/T1; 데모 단계 변화 시 |
| 액션 호출 | `/grasp/select_reachable` | `cleany_interfaces/action/SelectReachableGrasp` | Action 기본값; `selection_action` |
| 액션 호출 | `/move_action` | `moveit_msgs/action/MoveGroup` | Action 기본값; `move_group_action`, 실행을 요청하는 데모 |
| 서비스 호출 | `/grasp/plan` | `cleany_interfaces/srv/PlanGrasp` | Service 기본값; `grasp_service` |
| 서비스 호출 | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Service 기본값 |
| 서비스 호출 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Service 기본값 |
| 서비스 호출 | `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | Service 기본값 |
| 액션 호출 | `/left_gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Action 기본값 |
| 액션 호출 | `/right_gripper_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Action 기본값 |
| 토픽 발행 | `/grasp/can_grasp_image` | `sensor_msgs/msg/Image` | R/T1; 데모 결과 이미지 |
| 토픽 구독 | `/cleany/internal/mujoco/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | Sensor; `color_topic` |
| 토픽 구독 | `/cleany/internal/mujoco/left_wrist_camera/depth` | `sensor_msgs/msg/Image` | Sensor; `depth_topic` |
| 토픽 구독 | `/cleany/internal/mujoco/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor; `camera_info_topic` |

`GraspExecutionDemo`를 상속하므로 위 표에는 부모의 endpoint도 포함한다. Gripper 개방과 pregrasp 접근까지만 확인한다.
