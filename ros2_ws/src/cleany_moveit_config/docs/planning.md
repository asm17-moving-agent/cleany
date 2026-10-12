# MoveIt planning 계약


| Group | Chain | Controller |
|---|---|---|
| `left_arm` | `base_link` -> `left_gripper_frame` | `left_arm_controller` |
| `right_arm` | `base_link` -> `right_gripper_frame` | `right_arm_controller` |
| `left_grasp_arm` | `base_link` -> `left_grasp_tcp` | plan-only |
| `right_grasp_arm` | `base_link` -> `right_grasp_tcp` | plan-only |

Each group contains only its five arm joints. Gripper joints and the mobile
base are not planning-group joints. Both groups use
`kdl_kinematics_plugin/KDLKinematicsPlugin` with `position_only_ik: true`, so
the target position and the complete arm seed determine the resolved joint
configuration; target orientation is not constrained by IK.

Grasp groups share the same five active joints as their corresponding arm
group and extend the chain only through the fixed `*_grasp_tcp_joint`.

The named states `left_home` and `right_home` set all five corresponding arm
joints to `0.0 rad`. The SRDF collision matrix disables only direct
parent/child pairs. Non-adjacent arm/base pairs and all left/right cross-arm
pairs remain collision checked.

The production controller contract is:

```text
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
```

Both actions use `control_msgs/action/FollowJointTrajectory`. An active robot
backend must also publish all arm and gripper state on `/joint_states`.

## Fixed hand-eye collision scene

The fixed calibration table, target stand, and ChArUco backing are defined in
`config/handeye_collision_objects.yaml` using the
`cleany.moveit_collision_objects/v1` schema. Every box has a full-extent
dimension and an explicit `primitive_pose` in `base_link`; no unused
top-level `CollisionObject.pose` is assumed.

After `move_group` is running, apply the scene once with:

```bash
ros2 launch cleany_moveit_config handeye_collision_scene.launch.py
```

The applier calls `/apply_planning_scene` and exits only after MoveIt accepts
all three object IDs: `handeye_table`, `handeye_target_stand`, and
`charuco_target`. The generic `move_group.launch.py` does not inject these
hand-eye-only objects automatically.

The RGB-D pick demo table is separately defined in
`config/pick_demo_collision_objects.yaml`, with the same full size and pose as
`rgbd_pick_demo.xml.in`. Box/can targets are registered dynamically by the
reachable-grasp action instead of this fixed scene.


실행·검증은 [RUNNING](../../../../docs/RUNNING.md), [TESTING](../../../../docs/TESTING.md)를 따른다.
