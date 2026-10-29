# MoveIt planning 계약

MoveIt은 팔별 5관절을 계획하며 gripper, base는 planning group에서 제외한다. Grasp group은 같은 팔 관절을 사용하고 fixed TCP joint까지 chain을 확장한다.

## Group과 controller

```mermaid
flowchart TB
    Backend["활성 backend 하나"] -->|"팔 / gripper joint_states"| State["RobotState"]
    Model["URDF / SRDF / KDL 설정"] --> MoveIt["MoveIt / move_group"]
    State --> MoveIt
    Scene["Collision scene<br/>고정 형상 / 동적 대상"] --> MoveIt
    MoveIt --> Arm["left_arm / right_arm<br/>계획 trajectory"]
    Arm -->|"FollowJointTrajectory"| Controller["좌우 arm controller"]
    Controller --> Backend
    MoveIt --> Grasp["left_grasp_arm / right_grasp_arm<br/>fixed TCP / plan-only"]
    Grasp --> Result["도달성 검사 결과<br/>controller 실행과 별개"]
```

| Group | Chain | Controller |
|---|---|---|
| left_arm | base_link → left_gripper_frame | left_arm_controller |
| right_arm | base_link → right_gripper_frame | right_arm_controller |
| left_grasp_arm | base_link → left_grasp_tcp | plan-only |
| right_grasp_arm | base_link → right_grasp_tcp | plan-only |

| 항목 | 계약 |
|---|---|
| IK | KDL, position_only_ik=true; 목표 위치와 전체 arm seed 사용 |
| Home | left_home/right_home의 팔 5관절 모두 0 rad |
| Controller 타입 | control_msgs/action/FollowJointTrajectory |
| Action 이름 | /left_arm_controller/follow_joint_trajectory, /right_arm_controller/follow_joint_trajectory |
| Backend 상태 | 팔, gripper 전체 /joint_states 발행 필수 |

Position-only IK는 목표 orientation을 제약하지 않는다. SRDF는 직접 parent/child만 충돌 검사에서 제외하며, 비인접, 좌우 cross-arm 검사는 유지한다.
## 시간 배정

양팔의 팔 이동, 파지, 사전 조준, 그리퍼 개방과 복귀 그룹은
`cleany/RobustTimeParameterization`을 공유한다.
좁은 곡선 구간의 수치 오차 기준과 마지막 시간 표본 중복을 수정한 TOTG
계산기를 사용하며 대체 시간 배정기는 두지 않는다. 요청 배율과 관절별
속도, 가속도 제한을 적용한다. 반환된 위치, 속도와 가속도의 5차 보간
전체를 제한 안에 두도록 같은 비율로 시간을 늘릴 수 있다. 이 과정은
보간 경로의 모양을 바꾸지 않는다. 실물 실행기의 별도 시간 배정과 최종
충돌 검사는 유지한다.

짧은 구간은 보존하고 정확한 방향 반전은 속도 0 경계 조건으로 계산한다.
반환 표본 사이의 5차 보간 전체가 정리된 경로의 `path_tolerance`와 관절
위치 한계 안에 들어가도록 필요한 구간을 세분한다. 나노초 시간 반올림도
형상과 미분 상한 계산에 포함한다. 관절 공간의 허용오차는 실제 공간의
충돌 여유를 보장하지 않으므로 최종 실행 경로의 충돌 검사는 유지한다.

유효한 시간 궤적을 만들지 못하면 계획 응답은 `INVALID_MOTION_PLAN`과
빈 궤적이다. 계획용 기본 제한과 지원 범위는
[플러그인 계약](../../cleany_moveit_capabilities/README.md)을 따른다.

## 고정 collision scene

Hand-eye scene은 config/handeye_collision_objects.yaml의 cleany.moveit_collision_objects/v1 schema로 정의한다. Box는 전체 길이와 base_link 기준 primitive_pose를 사용하며 top-level CollisionObject.pose에 의존하지 않는다.

move_group 실행 후 scene을 한 번 적용한다.

```bash
ros2 launch cleany_moveit_config handeye_collision_scene.launch.py
```

Applier는 /apply_planning_scene이 handeye_table, handeye_target_stand, charuco_target을 수락한 뒤 종료한다. 일반 move_group launch는 이 scene을 자동 주입하지 않는다.

Pick-demo table은 config/pick_demo_collision_objects.yaml에 있으며 rgbd_pick_demo.xml.in과 크기, pose가 일치한다. Box/can target은 reachable-grasp action에서 동적으로 등록한다.

관련 안내: [실행](../../../../docs/RUNNING.md), [검증](../../../../docs/TESTING.md).

노드별 통신 목록: [인터페이스 표](interfaces.md).

## Terminal logging

`move_group.launch.py` accepts `backend_log_level` (standalone default `info`)
and `log_level` (default `info`). The sensor-driven MuJoCo workflow passes
`backend_log_level:=warn` to reduce backend initialization and planning detail.
Warnings and errors remain visible. The child logger `move_group.scene_mapping`
uses `log_level` independently: `log_level:=debug` enables throttled depth
callback and map timing diagnostics without enabling every MoveIt logger.
In quiet mode, capability banners printed directly to stdout go to the launch
log; this process sends ROS logs to stderr so warnings and errors remain on
screen even if the parent environment selects stdout. `backend_log_level:=info`
also restores those banners to the terminal.

## Humble controller plugin teardown

`keep_controller_plugin_loaded:=true` keeps the default controller plugin library
mapped through process teardown using a process-local `LD_PRELOAD`. Existing
preload entries are preserved. It applies only to
`moveit_simple_controller_manager/MoveItSimpleControllerManager`.

In this VM, Ctrl+C caused a SIGSEGV in `rclcpp::CallbackGroup` while the controller
manager node was being destroyed. MoveIt 2.5.9 declares the plugin loader after
that node, so the loader is destroyed first; the core dump is consistent with
a remaining weak-pointer control block referencing unloaded plugin code.
See the [upstream member declarations](https://github.com/moveit/moveit2/blob/2.5.9/moveit_ros/planning/trajectory_execution_manager/include/moveit/trajectory_execution_manager/trajectory_execution_manager.h).

Preloading prevented the observed crash. The workaround does not change
trajectory execution or controller selection. Set
`keep_controller_plugin_loaded:=false` when validating a patched MoveIt install.
The workaround may leave the upstream `class_loader` unload warning visible;
it prevents the observed crash without suppressing that warning.

The runtime regression exercises `/check_state_validity` in the default MuJoCo
logging mode (`backend_log_level:=warn`) and requires clean SIGINT termination
without SIGTERM escalation. It also checks that ROS warnings remain on screen.
