# cleany_moveit_config

MoveIt 2 configuration shared by Cleany's left and right arms. The package
uses the authoritative URDF from `cleany_description`; it does not copy robot
geometry, link names, joint names, or hard limits.

## Planning contract

| Group | Chain | Controller |
|---|---|---|
| `left_arm` | `base_link` -> `left_gripper_frame` | `left_arm_controller` |
| `right_arm` | `base_link` -> `right_gripper_frame` | `right_arm_controller` |
| `left_grasp_arm` | `base_link` -> `left_grasp_tcp` | plan-only |
| `right_grasp_arm` | `base_link` -> `right_grasp_tcp` | plan-only |
| `left_pregrasp_open` | `left_grasp_arm` + `left_gripper_joint` | left arm + gripper controllers, opt-in |
| `right_pregrasp_open` | `right_grasp_arm` + `right_gripper_joint` | right arm + gripper controllers, opt-in |
| `left_return_close` | `left_grasp_arm` + `left_gripper_joint` | left arm + gripper controllers, opt-in |
| `right_return_close` | `right_grasp_arm` + `right_gripper_joint` | right arm + gripper controllers, opt-in |

The existing arm/IK groups contain only their five arm joints. The mobile
base is not a planning-group joint. Arm IK groups use
`kdl_kinematics_plugin/KDLKinematicsPlugin` with `position_only_ik: true`, so
the target position and the complete arm seed determine the resolved joint
configuration; target orientation is not constrained by IK.

The `*_pregrasp_open` and `*_return_close` groups accept joint goals only and add the selected
jaw to the collision-checked OMPL path. They do not add a six-DOF IK solver or
relax the existing arm/collision constraints. `move_group.launch.py
enable_gripper_execution:=true` selects `sorting_moveit_controllers.yaml`,
adding the existing disjoint gripper FollowJointTrajectory controllers to
MoveIt's execution manager. Sorting enables this option; generic/mock defaults
remain arm-only. A single coordinated goal must complete both controllers
before the skill proceeds. No separate gripper-open command is sent in parallel.
Return-close is dispatched only after intentional release and attachment cleanup;
the saved arm home pose and closed jaw are checked as one path, not independent motions.

Grasp groups share the same five active joints as their corresponding arm
group and extend the chain only through the fixed `*_grasp_tcp_joint`.

The named states `left_home` and `right_home` set all five corresponding arm
joints to `0.0 rad`. The SRDF collision matrix disables only direct
parent/child pairs. Non-adjacent arm/base pairs and all left/right cross-arm
pairs remain collision checked.

OMPL remains the default pipeline and each arm group uses `RRTConnect` for
joint-space motion. The additional `pilz_industrial_motion_planner` pipeline
provides `LIN` for the final approach, reverse retreat, and vertical lift.
Its Cartesian limits are 0.10 m/s translation, 0.20 m/s² acceleration,
-0.20 m/s² deceleration, and 0.50 rad/s rotation. A LIN request must start
from a stopped robot state.

Position-only KDL does **not** enforce the orientation of a Pilz pose request.
The grasp executor checks the returned endpoint FK before dispatch. Sorting
approach/retreat instead use selected joint goals with a 1 mm TCP corridor;
the five-joint endpoint is preserved without assuming arbitrary 6-DOF IK.
Grasp groups use `longest_valid_segment_fraction: 0.001`, and TOTG uses
`path_tolerance: 0.0001` (joint-space units). The previous coarse sampling and
0.1 rounding could produce a postprocessed trajectory outside a narrow
Cartesian corridor. These settings do not prove continuous clearance;
MoveIt validation and the executor's sampled FK checks are both required.

The production controller contract is:

```text
/left_arm_controller/follow_joint_trajectory
/right_arm_controller/follow_joint_trajectory
```

Both actions use `control_msgs/action/FollowJointTrajectory`. An active robot
backend must also publish all arm and gripper state on `/joint_states`.

## Sensor-only depth collision map

`move_group.launch.py enable_depth_octomap:=true` loads the
`occupancy_map_monitor/PointCloudOctomapUpdater` from `moveit_ros_perception`.
`config/depth_octomap.yaml` specifies a 1 cm OctoMap in `base_link`, a 2 m
range and up to 2 Hz updates. `/perception/scene_cloud` is the input;
`/perception/scene_cloud_filtered` shows the updater's self-filtered output.
The URDF and capture-time TF are used for robot self-filtering. No object
positions, sizes, class labels or simulator mesh data are supplied to this map.
Self-filter padding is 15 mm: the 1 cm voxel half-diagonal plus a 5 mm
sensor/model allowance, rounded up. This masks near-model observations; it
does not disable robot/environment collision checks. It is not a complete
solution for occluded historical voxels left by moving robot links.
The common RViz configuration also shows the observed depth surface as points.

Optional simulation diagnostic: set
`depth_octomap_plugin:=cleany_scene_mapping/KnownGeometryOctomapUpdater` to
exclude old occupied cells fully contained within one known padded collision
shape. The default remains the standard updater. This does not remove planning
scene collision shapes or disable raycasting. Boundary/outside cells and unknown
space remain unchanged by the extra exclusion; incomplete capture-time transforms
skip it. See `cleany_scene_mapping/README.md` for limits and tests. Physical grasp
success and real-hardware safety are not established by this option.

The generic launch remains sensor-disabled by default for existing workflows.
The study-cafe sensor launch does not run the fixed collision scene loader.
Use a **fresh MoveIt process** when switching from a static scene: disabling a
loader does not remove objects that are already present in an existing scene.

Read geometry updates from `/monitored_planning_scene`: the tree must have a
populated `OcTree` payload, not just a visible cloud topic. Do not continuously
poll `/get_planning_scene` with OCTOMAP (32) while this Humble updater writes:
a runtime crash was observed in service-side OctoMap serialization. The scene
publisher uses the OctoMap read lock, unlike the service callback in the
[Humble source](https://github.com/moveit/moveit2/blob/humble/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp).
The coordinator therefore consumes published map updates and checks both map
receipt age and filtered-cloud capture age. This avoids that polling path;
it is not a patch to the installed MoveIt service. MoveIt process exit now
requests shutdown of the parent launch instead of leaving a controller
workflow alive. Its `on_exit` handler uses the `launch.actions.Shutdown`
action, not the raw event; the latter is rejected as a launch entity on exit.
This handler does not fix a crash in the installed MoveIt binary itself.
The WORLD_OBJECT_GEOMETRY component (16) should contain no preloaded study-cafe
objects. A temporary target OBB reconstructed from perception may still be used
by the grasp selector; it is sensor-derived, not prior environment knowledge.

This is an observed-space planning prototype. Unknown/occluded space is not
certified free; the standard MoveIt collision checker does not enforce an
unknown-space safety policy. The sensor study-cafe workflow defaults to
plan-only for that reason. Depth holes, voxel aliasing (especially thin phones),
calibration error, moving obstacles and accumulated stale occupancy require
further validation. Do not clear the map on every frame or exclude all target
voxels simply to force a grasp to pass. Multi-view coverage and target-contact
handling remain necessary before autonomous physical execution.

Reference: [MoveIt perception pipeline](https://moveit.picknik.ai/humble/doc/examples/perception_pipeline/perception_pipeline_tutorial.html).

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

The legacy configured study-cafe fixture is
`config/study_cafe_grasp_collision_objects.yaml`. Its fixed list contains the
robot-side desk, partition, monitor, paper cup, wallet, and crumpled tissue.
The LEGO target is omitted because the reachable-grasp target transaction owns
its OBB and temporarily permits contacts only for the selected jaw links.
These boxes are fixture approximations, not observations. The current
`sensor_scene:=true` workflow does not load this file; its environment is built
from camera depth instead. The object replacement does not inject simulator
geometry or poses into that sensor-only planning path.

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
controller joint ownership. It also checks collision geometry/message parity.
The runtime smoke tests launch the headless mock
stack, verifies the all-zero state is collision-free, resolves position-only
IK for each side, confirms orientation does not change the same seeded IK
request, plans/executes each resolved joint goal through its side-specific
controller, and query MoveIt after applying the fixed hand-eye world objects.
