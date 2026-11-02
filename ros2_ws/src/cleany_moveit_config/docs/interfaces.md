# cleany_moveit_config 노드 인터페이스

응용 인터페이스와 adapter에서 생성하는 endpoint를 노드별로 정리한다. 이름과 QoS 표기, ROS 자동 인터페이스는 [전체 색인](../../../../docs/ROS_INTERFACES.md)을 따른다. 이름 설정이 없는 항목은 ROS remap으로 변경한다.

## handeye_collision_scene_applier

실행 파일: `apply_handeye_collision_scene`. 원본: [apply_collision_scene.py](../cleany_moveit_config/apply_collision_scene.py).

| 종류 / 방향 | 기본 이름 | ROS 타입 | QoS, 주기와 이름 설정 |
|---|---|---|---|
| 서비스 호출 | `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Service 기본값; `scene_config` YAML의 `apply_service` |

Scene 수락 후 종료하는 노드다. [Planning 계약](planning.md)을 따른다.
