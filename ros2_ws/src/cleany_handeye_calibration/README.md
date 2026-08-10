# cleany_handeye_calibration

ROS-independent mathematical core and, in later commits, ROS adapters for
Cleany's left wrist eye-in-hand calibration workflow.

The current package contains immutable data models, frame-aware rigid
transforms, the version-1 draft sample record, and planar ChArUco detection and
PnP. It also provides a failure-isolated OpenCV hand-eye solver, evaluation
metrics, and a motion-only MoveIt/MuJoCo launch. It does not yet collect camera
data, orchestrate calibration poses, or publish calibration TF.

## Transform convention

`RigidTransform(parent_frame, child_frame, ...)` represents
`parent_T_child`: it maps a point expressed in the child frame into the parent
frame. Composition checks frame names before multiplying transforms.

For left eye-in-hand calibration, every synchronized sample follows:

```text
base_T_gripper @ gripper_T_camera @ camera_T_target = base_T_target
```

| Value | Direction | Source or owner |
|---|---|---|
| `base_T_gripper` | gripper coordinates to base coordinates | feedback-joint FK |
| `camera_T_target` | target coordinates to camera coordinates | target PnP |
| `gripper_T_camera` | camera coordinates to gripper coordinates | unknown calibration result |
| `base_T_target` | target coordinates to base coordinates | fixed target relation/evaluation |

Translations use metres, joint and Rodrigues values use radians, timestamps
use integer nanoseconds, and serialized quaternions use ROS `xyzw` order.
Rotation matrices are accepted only when all values are finite,
`||R^T R - I||_F <= 1e-6`, and `|det(R) - 1| <= 1e-6`.

## Core models

- `PositionTarget`: position and reference frame used by position-only IK
- `JointPose`: ordered complete joint vector, used for seeds and solutions
- `IkResult`: mutually exclusive success or failure result
- `TimedJointSample`: timestamped feedback positions and optional velocities
- `CalibrationSample`: one `base_T_gripper` / `camera_T_target` pair and split
- `CalibrationSampleRecord`: versioned draft of the synchronized dataset row

Models copy mutable input into immutable tuples, reject duplicate joint names,
length mismatches, invalid timestamps, and non-finite numeric values. The
sample schema serializes to built-in dictionaries and lists so a later dataset
writer can safely encode it as JSONL or YAML without carrying NumPy objects.

## Planar ChArUco and PnP contract

The fixed target has 7 x 5 squares, a 30 mm square length, a 15 mm marker
length, and uses `DICT_5X5_100` with `legacy_pattern: false`. The resulting
inner chessboard contains 24 corners. A detection is valid only when at least
16 unique, in-range ChArUco corner IDs cover all four object-space board
quadrants.

Ubuntu 22.04 provides OpenCV 4.5.4, so the detector intentionally uses its
legacy Python API: `CharucoBoard_create`, `DetectorParameters_create`,
`detectMarkers`, `interpolateCornersCharuco`, and `board.chessboardCorners`.
It does not use the newer `ArucoDetector` or `CharucoDetector` classes.

PnP uses `solvePnPGeneric(..., flags=SOLVEPNP_IPPE)`, never the four-point
`SOLVEPNP_IPPE_SQUARE` variant. Both candidates must contain finite values and
place every target point at positive camera Z. Each valid raw candidate is
refined independently with `solvePnPRefineVVS`; cheirality is checked again,
and full-corner Euclidean reprojection RMSE is recomputed. Raw and refined
candidate transforms, minimum depth, RMSE, and failure reason remain in the
result for later artifact recording.

The lower-RMSE candidate is rejected as `ambiguous_pnp` when both candidate
RMSE values are at most `1e-12 px`, or when the best RMSE is above that value
and `second / best < 1.05`. Ground truth is not an input to detection, PnP,
refinement, or candidate selection.

## Hand-eye solver and metrics

The symbolic registry contains every method exposed by OpenCV's
`calibrateHandEye`: Tsai, Park, Horaud, Andreff, and Daniilidis. Preflight
fails if an expected constant is missing or a new `CALIB_HAND_EYE_*` constant
appears without an explicit registry update. All five methods receive copied
values from the same validated calibration samples; an exception or invalid
transform from one method is recorded without stopping the other methods.
The caller must also provide a positive finite `max_translation_norm_m` through
`HandEyeTransformValidityPolicy`. This is an approved robot-specific physical
camera-offset envelope rather than a hidden solver constant; a finite estimate
outside it is recorded as an invalid result for that method.

The OpenCV argument directions are intentionally explicit:

```text
R_gripper2base, t_gripper2base <- base_T_gripper
R_target2cam, t_target2cam     <- camera_T_target
R_cam2gripper, t_cam2gripper   -> gripper_T_camera
```

Only samples in the `calibration` split are accepted by the solver. The
expected base, gripper, camera, and target frame names are checked before an
OpenCV method is called, so an inverted transform cannot silently become a
different hand-eye problem. Ground truth is not part of the solver API.

The separate evaluator reports translation error in metres and rotation error
in radians. Held-out consistency reconstructs `base_T_target` for every
held-out sample and reports the median and 95th percentile of all pairwise
translation and rotation disagreements. The deterministic synthetic fixture
contains 20 calibration poses and 5 held-out poses with a known transform.

## Motion-only MoveIt/MuJoCo integration

`handeye_mujoco.launch.py` composes the `cleany_moveit_config` move group with
the `cleany_mujoco_sim` ros2_control backend. The backend owns the sole
`robot_state_publisher`, both side-specific trajectory controllers, and the
complete arm/gripper `/joint_states` feedback. This launch does not start the
legacy custom `mujoco_sim_node`, Gazebo, a calibration orchestrator, or a
calibration scene.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_handeye_calibration handeye_mujoco.launch.py \
  headless:=true sim_speed_factor:=1.0
```

On ROS 2 Humble, planning and execution use separate action boundaries:
`/move_action` with `planning_options.plan_only=true`, followed by
`/execute_trajectory` with the returned trajectory. A left-arm calibration
motion therefore reaches only
`/left_arm_controller/follow_joint_trajectory`; the right controller continues
publishing feedback but receives no goal.

The installed Humble MoveIt `ExecuteTrajectory` capability does not service an
action cancel request while its execution callback is blocking. The timeout
compatibility path consequently sends the standard `CancelGoal` request
directly to the active side's `FollowJointTrajectory` action. The expected
terminal contract is controller `CANCELED`, then `/execute_trajectory`
`ABORTED` with `MoveItErrorCodes.PREEMPTED`. This is a documented direct
controller fallback, not a claim that Humble propagates an
`/execute_trajectory` cancel request downstream.

Run the focused runtime integration test after building the participating
packages:

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to \
  cleany_handeye_calibration
source install/setup.bash
python3 -m pytest -q -s \
  src/cleany_handeye_calibration/test/test_handeye_mujoco_runtime.py
```

The test verifies separate per-arm plan and execute success, left-only
controller routing, complete feedback-backed MoveIt current state, direct
controller cancel response and terminal statuses, cancel hold, launch liveness,
and process-group cleanup.

## Dependencies

The transform conversion functions use the Ubuntu/ROS system installations of
NumPy and OpenCV. On the target Ubuntu 22.04 / ROS 2 Humble environment these
are provided through the `python3-numpy` and `python3-opencv` rosdep keys.
The mathematical core does not import `rclpy` and its focused unit tests need no
running ROS graph. The motion-only launch depends on the MoveIt and MuJoCo ROS
packages, while its runtime integration test uses `rclpy` and the standard ROS
action/message packages declared as test dependencies.

## Verification

Run the focused tests and package build in the ROS 2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws/src/cleany_handeye_calibration
python3 -m pytest test
cd ../..
colcon build --symlink-install --packages-select cleany_handeye_calibration
source install/setup.bash
colcon test --packages-select cleany_handeye_calibration
colcon test-result \
  --test-result-base build/cleany_handeye_calibration --verbose
```
