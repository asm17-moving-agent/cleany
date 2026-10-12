# cleany_handeye_calibration 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## single_pose_handeye_orchestrator

실행 파일: `single_pose_calibration`. 원본: [single_pose_runtime.py](../cleany_handeye_calibration/single_pose_runtime.py), [moveit_fk.py](../cleany_handeye_calibration/moveit_fk.py), [ik_port.py](../cleany_handeye_calibration/ik_port.py), [motion_port.py](../cleany_handeye_calibration/motion_port.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/joint_states` | `sensor_msgs/msg/JointState` | Sensor |
| 토픽 구독 | `/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | Sensor |
| 토픽 구독 | `/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor |
| 서비스 호출 | `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Service 기본값 |
| 서비스 호출 | `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | Service 기본값 |
| 서비스 호출 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Service 기본값 |
| 서비스 호출 | `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | Service 기본값 |
| 액션 호출 | `/move_action` | `moveit_msgs/action/MoveGroup` | Action 기본값 |
| 액션 호출 | `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | Action 기본값 |

`request_file`에서 실행 설정을 읽는다. Topic과 MoveIt endpoint는 코드 상수이므로 이름 변경에는 ROS remap을 사용한다.

## multi_pose_handeye_orchestrator

실행 파일: `multi_pose_calibration`. 원본: [multi_pose_runtime.py](../cleany_handeye_calibration/multi_pose_runtime.py), [single_pose_runtime.py](../cleany_handeye_calibration/single_pose_runtime.py), [moveit_fk.py](../cleany_handeye_calibration/moveit_fk.py), [ik_port.py](../cleany_handeye_calibration/ik_port.py), [motion_port.py](../cleany_handeye_calibration/motion_port.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/joint_states` | `sensor_msgs/msg/JointState` | Sensor |
| 토픽 구독 | `/left_wrist_camera/image_raw` | `sensor_msgs/msg/Image` | Sensor |
| 토픽 구독 | `/left_wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Sensor |
| 서비스 호출 | `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Service 기본값 |
| 서비스 호출 | `/compute_fk` | `moveit_msgs/srv/GetPositionFK` | Service 기본값 |
| 서비스 호출 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Service 기본값 |
| 서비스 호출 | `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | Service 기본값 |
| 액션 호출 | `/move_action` | `moveit_msgs/action/MoveGroup` | Action 기본값 |
| 액션 호출 | `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | Action 기본값 |
| 서비스 제공 | `/handeye/cancel_run` | `std_srvs/srv/Trigger` | Service 기본값 |

`pose_manifest`와 `runtime_config`로 실행한다. SinglePoseRuntimeEffects를 재사용하며 cancel service 응답은 요청 접수이고 실제 정지 완료는 별도 확인한다.

## handeye_pose_generator

실행 파일: `generate_pose_manifest`. 원본: [pose_generation_runtime.py](../cleany_handeye_calibration/pose_generation_runtime.py), [ik_port.py](../cleany_handeye_calibration/ik_port.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 토픽 구독 | `/joint_states` | `sensor_msgs/msg/JointState` | R20 |
| 서비스 호출 | `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Service 기본값 |
| 서비스 호출 | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | Service 기본값 |
| 서비스 호출 | `/check_state_validity` | `moveit_msgs/srv/GetStateValidity` | Service 기본값 |

후보 생성과 IK/충돌 검사에 사용한다. 수집 노드의 image 구독과 trajectory 실행은 생성하지 않는다.
