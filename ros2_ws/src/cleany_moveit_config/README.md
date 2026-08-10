# cleany_moveit_config

MoveIt 2 configuration shared by Cleany's left and right arms. The package
uses the authoritative URDF from `cleany_description`; it does not copy robot
geometry, link names, joint names, or hard limits.

## Planning contract

| Group | Chain | Controller |
|---|---|---|
| `left_arm` | `base_link` -> `left_gripper_frame` | `left_arm_controller` |
| `right_arm` | `base_link` -> `right_gripper_frame` | `right_arm_controller` |

Each group contains only its five arm joints. Gripper joints and the mobile
base are not planning-group joints. Both groups use
`kdl_kinematics_plugin/KDLKinematicsPlugin` with `position_only_ik: true`, so
the target position and the complete arm seed determine the resolved joint
configuration; target orientation is not constrained by IK.

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

## Launch

Start only `move_group` against an already active robot backend and
`robot_state_publisher`:

```bash
ros2 launch cleany_moveit_config move_group.launch.py use_sim_time:=false
```

Start the self-contained `ros2_control` mock backend, both trajectory
controllers, `robot_state_publisher`, `move_group`, and optionally RViz:

```bash
ros2 launch cleany_moveit_config mock_planning.launch.py use_rviz:=true
```

The mock backend is a planning/configuration test fixture only. It is not the
MuJoCo calibration backend and must not be used as a physical robot driver.

## Verification

Build and run this package's tests in the native ROS 2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to cleany_moveit_config
source install/setup.bash
colcon test --packages-select cleany_moveit_config
colcon test-result --verbose
```

The static contract test checks the SRDF chains and homes, conservative
self-collision policy, position-only KDL settings, URDF limit parity, and
controller joint ownership. The runtime smoke test launches the headless mock
stack, verifies the all-zero state is collision-free, resolves position-only
IK for each side, confirms orientation does not change the same seeded IK
request, and plans/executes each resolved joint goal through its side-specific
controller.
