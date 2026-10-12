# MoveIt planning 계약

| Group | Chain | Controller |
|---|---|---|
| left_arm | base_link → left_gripper_frame | left_arm_controller |
| right_arm | base_link → right_gripper_frame | right_arm_controller |
| left_grasp_arm | base_link → left_grasp_tcp | plan-only |
| right_grasp_arm | base_link → right_grasp_tcp | plan-only |

## 관절·충돌

- Group: 팔 5관절만 포함. Gripper·base 제외
- Grasp group: 같은 팔 5관절 + fixed TCP joint
- IK: KDL, position_only_ik=true. 목표 위치·전체 arm seed 사용; orientation 미제약
- Home: left_home/right_home 각 팔 5관절=0 rad
- SRDF: 직접 parent/child만 충돌 제외. 비인접·좌우 cross-arm 충돌 검사 유지

## Controller

- 타입: control_msgs/action/FollowJointTrajectory
- 이름: `/left_arm_controller/follow_joint_trajectory`, `/right_arm_controller/follow_joint_trajectory`
- Backend: 팔·gripper 전체 `/joint_states` 발행 필수

## 고정 collision scene

- Hand-eye 원본: config/handeye_collision_objects.yaml, schema cleany.moveit_collision_objects/v1
- Box: 전체 길이 + base_link 기준 primitive_pose. Top-level CollisionObject.pose 의존 없음
- 적용: move_group 실행 후 한 번

```bash
ros2 launch cleany_moveit_config handeye_collision_scene.launch.py
```

- Apply: /apply_planning_scene에서 handeye_table / handeye_target_stand / charuco_target 수락 후 종료
- 일반 move_group launch: hand-eye scene 자동 주입 없음
- Pick-demo table: config/pick_demo_collision_objects.yaml, rgbd_pick_demo.xml.in과 크기·pose 일치
- Box/can target: reachable-grasp action에서 동적 등록
- 안내: [실행](../../../../docs/RUNNING.md), [검증](../../../../docs/TESTING.md)
