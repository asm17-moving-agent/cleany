# cleany_moveit_config 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## handeye_collision_scene_applier

실행 파일: `apply_handeye_collision_scene`. 원본: [apply_collision_scene.py](../cleany_moveit_config/apply_collision_scene.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 서비스 호출 | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Service 기본값; `scene_config` YAML의 `apply_service` |

Scene 수락 후 종료하는 노드다. [Planning 계약](planning.md)을 따른다.

## FreeCAD 검사

`freecad_inspection.launch.py`는 `/freecad_inspection`에 MoveIt과
`robot_state_publisher`를 배치한다. [Planning 계약](planning.md#freecad-형상-검사)을 따른다.

| 종류 / 방향 | 이름 | ROS 타입 |
|---|---|---|
| 구독 | `/freecad_inspection/joint_states` | `sensor_msgs/msg/JointState` |
| 구독 | `/freecad_inspection/scene_cloud` | `sensor_msgs/msg/PointCloud2` |
| 발행 | `/freecad_inspection/scene_cloud_filtered` | `sensor_msgs/msg/PointCloud2` |
| 서비스 제공 | `/freecad_inspection/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` |
| 서비스 제공 | `/freecad_inspection/check_state_validity` | `moveit_msgs/srv/GetStateValidity` |
| 서비스 제공 | `/freecad_inspection/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` |

TF와 robot description도 같은 namespace를 사용한다.
`runtime_test/check_freecad_lower_body.py`는 같은 namespace에서 관절 상태와
점군을 발행하고 필터 결과를 구독한다. 시험 장애물은 위 scene 적용 서비스로
추가하고 상태 검사 서비스에서 접촉을 조회한 뒤 제거한다.
점군 발행과 구독은 sensor-data QoS, 관절 상태 발행은 reliable depth 10이다.
