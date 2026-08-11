# cleany_handeye_calibration

ROS-independent mathematical core and ROS adapters for Cleany's left wrist
eye-in-hand calibration workflow.

The current package contains immutable data models, frame-aware rigid
transforms, the version-1 synchronized sample record, and planar ChArUco
detection and PnP. It also provides a failure-isolated OpenCV hand-eye solver,
evaluation metrics, bounded joint-feedback synchronization, a MoveIt
feedback-FK adapter, position-only IK/state-validity/motion adapters, a pure
feedback settle gate, exact wrist-camera acquisition, a recoverable dataset
writer, an exact nine-stage single-pose orchestrator, and MoveIt/MuJoCo
launches. It does not publish calibration TF; generated transforms remain
review-only artifacts.

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
- `CalibrationSampleRecord`: complete versioned synchronized dataset row

Models copy mutable input into immutable tuples, reject duplicate joint names,
length mismatches, invalid timestamps, and non-finite numeric values. A stored
sample requires the canonical 12 dual-arm/gripper feedback joints, the five
left-arm IK seed and result joints, exact image/CameraInfo stamps,
interpolation provenance, FK and PnP transforms, and the ordered ChArUco
correspondences used by PnP. The exact `K`, `D`, `R`, and `P` values and their
canonical SHA-256 are kept in every row so later experiments can perturb image
points and repeat PnP rather than perturbing a final transform.

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
For the 640 x 480 wrist stream it performs detection on a deterministic 2x
cubic-upscaled grayscale image with subpixel marker refinement, then divides
the detected ChArUco coordinates back into the original image pixel frame
before PnP or recording. This does not relax the 16-corner/four-quadrant gate.

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

## Joint feedback synchronization and FK

`JointStateRingBuffer` stores a bounded, increasing sequence of feedback
samples. A valid sample contains the five arm joints and read-only gripper
joint for both sides (12 revolute joints total). Additional joints may pass
through the buffer but are not sent to MoveIt. Duplicate and small regressing
stamps are rejected without changing the buffer. A regression at least the
configured clock-reset threshold starts a new ROS-time epoch and discards the
old one. Capacity and timing limits are explicit constructor inputs.

For an image header stamp, the buffer requires distinct samples bracketing that
stamp and requires each source to be within `max_sample_distance_ns`. Position
is interpolated linearly by joint name, so publisher ordering may change.
Velocity is recorded and interpolated when both sources provide it, but it is
not an FK requirement. The result retains the original before/after ROS stamps
and exact interpolation ratio. Missing, stale, incompatible, incomplete,
duplicate, out-of-order, and clock-reset cases have explicit result statuses.

`timed_joint_sample_from_message` obtains data time only from the
`JointState.header.stamp`. `MoveItForwardKinematicsAdapter` sends exactly the
12 known joints in canonical order to `/compute_fk`, with
`RobotState.is_diff=false`, and requests
`base_link -> left_gripper_frame`. It validates the MoveIt error code, returned
link, output frame, pose count, and rigid transform. Service readiness and
response timeout use a monotonic wall clock, so paused or reset ROS simulation
time cannot disable the client deadline. Its client and executor poll hook are
injectable for tests that do not start a ROS graph.

## Position IK, state validity, motion, and settle

Commit 12 exposes four fixed Humble boundaries. Position IK calls
`/compute_ik`, state validation calls `/check_state_validity`, plan-only motion
uses `/move_action`, and execution uses `/execute_trajectory`. The calibration
allowlist is exactly `left_arm` with `left_gripper_frame`; a right group or tip
is rejected before a service or action client is contacted. IK targets must be
expressed in `base_link`.

`validate_dual_arm_current_state` is the startup and per-motion safety gate. It
requires fresh ROS-stamped position and velocity feedback for the canonical 12
arm/gripper joints, then verifies the five right-arm joints are stationary at
the approved all-zero park state within the configured tolerance. Freshness,
right-park position tolerance, and every IK, state-validity, plan, execute,
cancel, and settle timeout are required `MujocoMotionConfig` inputs; there is no
shared implicit timeout.

`MoveItPositionIKAdapter` builds a full, non-diff `RobotState` in canonical
12-joint order. It preserves feedback for the right arm and both grippers,
replaces the five left positions with the complete manifest seed, explicitly
sets `ik_link_name=left_gripper_frame`, and enables collision avoidance. The
identity quaternion is only a valid placeholder: the configured KDL solver is
position-only and does not receive an orientation constraint. Successful
responses must contain a finite solution for all five left joints.

`MoveItStateValidityAdapter` overlays the resolved left solution on the same
full feedback state and checks joint/state validity and collision. Only a
successful check yields a `ValidatedJointGoal`. `MoveItMotionAdapter` refuses a
raw `JointPose`, stale/partial state, or right-arm park drift. It sends that
validated joint goal to `MoveGroup` with `plan_only=true`, 0.10 velocity and
acceleration scaling, and then sends only the returned trajectory to
`ExecuteTrajectory`. Action status and the exact MoveIt error code are retained
for success, rejection, abort, cancel, transport failure, and timeout. Timeout
cancellation has its own bounded monotonic-clock budget. A Humble cancel is
confirmed only when the response is `ERROR_NONE` and its `goals_canceling`
array contains the exact requested goal UUID; a success code for another goal
does not count as confirmation.

The controller baseline is a 0.05 rad path tolerance and 0.01 rad goal
tolerance. The original 0.005 rad MuJoCo settle-position trial was below the
measured gravity-loaded steady-state error; repeated runs reached 0.010403
rad, so the explicit MuJoCo E2E profile uses 0.015 rad after review. The 0.01
rad/s settle velocity and continuous 1.0 s interval are unchanged. These are
simulation values, not approved real-robot tolerances. The pure
`JointSettleDetector` uses only feedback ROS stamps. Every left joint must meet
both thresholds simultaneously; a threshold violation or ROS clock regression
resets the interval. Successful PLAN/EXECUTE actions only arm this gate and
never authorize sample acquisition on their own. `MonotonicSettleMonitor`
applies the separate settle-stage wall-clock timeout, so paused simulation time
cannot leave the workflow waiting forever and timeout never authorizes a
sample.

## Camera acquisition and dataset artifacts

`RosExactCameraPairAdapter` copies ROS `Image` and `CameraInfo` messages into a
ROS-independent bounded buffer. A pair is accepted only when both messages
have the same nonzero ROS stamp and use
`left_wrist_rgb_optical_frame`. The fixed runtime contract is RGB8 640 x 480,
`step=1920`, `plumb_bob`, zero five-coefficient distortion, identity `R`, and:

```text
K = [227.751496, 0, 319.5,
     0, 227.751496, 239.5,
     0, 0, 1]
```

`P` carries the same focal lengths and principal point with zero translation.
The focal length follows
`fy = height / (2 * tan(vertical_fov / 2))` for the MuJoCo camera's vertical
FOV of 93 degrees. Image dimensions, encoding, endianness, row step, payload
length, `K`, `D`, `R`, and `P` are all checked. Acquisition returns the
earliest compatible frame whose ROS stamp is strictly later than settle
completion. Queue capacity is explicit, while the wait deadline uses a
monotonic wall clock and therefore still expires if simulation time pauses.

`DatasetWriter` writes to a caller-selected artifact root; the repository
default layout is ignored by Git:

```text
artifacts/handeye/<run_id>/
├── manifest.yaml
├── samples.jsonl
└── images/<sample_id>.png
```

The manifest is JSON-compatible YAML with its own hash. It records Git commit
and dirty state; generated URDF, MJCF, and pose-manifest hashes; ROS, MoveIt,
OpenCV, MuJoCo, `mujoco_ros2_control`, and vendor versions; the full camera and
target contracts; board SVG/PDF hashes and size provenance; simulation,
controller, image, and joint-state rates; every calibration parameter; and the
random seed. Callers must supply these values explicitly.

For each sample, the lossless PNG is fsynced and renamed first, then a journal
record is fsynced, and finally a complete replacement `samples.jsonl` is
fsynced and renamed. Reopening the writer replays a valid journal, removes an
unreferenced image left before journal creation, and verifies row, image,
camera, and manifest hashes. Thus a partial write cannot create a committed row
with a missing image, and samples committed before an interruption remain
readable. Run and sample identifiers reject traversal, dataset directories and
images may not be symlinks, and the writer never edits URDF/Xacro or a source
calibration profile.

## Single-pose orchestration

`SinglePoseOrchestrator` executes exactly this no-retry sequence:

```text
RESOLVE_POSITION_IK -> VALIDATE_RESOLVED_POSE -> PLAN -> EXECUTE
-> WAIT_SETTLED -> ACQUIRE_IMAGE -> DETECT_TARGET
-> COMPUTE_FEEDBACK_FK -> RECORD_SAMPLE
```

Every stage has its own required positive monotonic timeout. A started and
succeeded row is appended to `orchestration.jsonl`; an exception appends the
exact failed stage and reason and prevents every later effect. The validation
stage requires canonical left-arm soft limits, a required collision margin,
precomputed clearance evidence tied to the expected resolved joint vector,
and MoveIt state-validity success. There are deliberately no production
defaults for the still-unapproved safety values.

The installed `config/single_pose_request.template.json` keeps unresolved
values as `null`. It is documentation and a materialization starting point,
not a runnable profile: strict preflight rejects it until the artifact root,
pose, hashes, versions, soft limits, clearance evidence, right-park tolerance,
and all stage timeouts have been supplied. The artifact root must be absolute
and should be outside the source tree.

`single_pose_mujoco.launch.py` starts the fixed-base calibration scene, both
controllers, MoveIt, the three required collision objects, and the
orchestrator. The node waits for complete fresh 12-joint feedback and verifies
that `handeye_table`, `handeye_target_stand`, and `charuco_target` are present
before IK. It records the first exact Image/CameraInfo pair strictly after
settle, interpolates feedback at that image stamp, obtains feedback FK, and
atomically stores the row and PNG.

Operator-observed calibration shows the MuJoCo viewer by default:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cleany_handeye_calibration single_pose_mujoco.launch.py \
  request_file:=/absolute/path/to/materialized_request.json \
  headless:=false
```

Automated tests explicitly override `headless:=true`. Their temporary
MuJoCo-only profile has at least 0.128 m target/arm clearance under simulated
feedback sag, uses a 0.100 m required margin, yields at least 16 corners in all
four quadrants, and produces a non-ambiguous IPPE result. Ground truth is not
published to TF and is not used by detection, PnP, or stored solver inputs.

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
  src/cleany_handeye_calibration/test/test_handeye_mujoco_runtime.py \
  src/cleany_handeye_calibration/test/test_single_pose_mujoco_runtime.py
```

The test verifies separate per-arm plan and execute success, left-only
controller routing, complete feedback-backed MoveIt current state, direct
controller cancel response and terminal statuses, cancel hold, launch liveness,
and process-group cleanup.

## Dependencies

The transform conversion functions use the Ubuntu/ROS system installations of
NumPy and OpenCV. On the target Ubuntu 22.04 / ROS 2 Humble environment these
are provided through the `python3-numpy` and `python3-opencv` rosdep keys.
The mathematical, synchronization, configuration, and settle core does not
import `rclpy`. The FK, IK, validity, and motion adapters use `rclpy`,
`action_msgs`, `moveit_msgs`, and `sensor_msgs`, but their focused tests use fake
clients, goal handles, and futures without a running ROS graph. The motion-only
and single-pose launches depend on the MoveIt and MuJoCo ROS packages, while
their runtime integration tests use the standard ROS action/message packages
declared in the manifest.

## Verification

Run the focused tests and package build in the ROS 2 Humble environment:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws/src/cleany_handeye_calibration
python3 -m pytest \
  test/test_joint_state_sync.py test/test_moveit_fk.py \
  test/test_motion_config.py test/test_ik_adapter.py \
  test/test_motion_adapter.py test/test_settle_detector.py \
  test/test_camera_acquisition.py test/test_ros_camera_adapter.py \
  test/test_schema.py test/test_dataset_writer.py
python3 -m pytest \
  test/test_single_pose_orchestrator.py \
  test/test_single_pose_runtime_config.py
python3 -m pytest test
cd ../..
colcon build --symlink-install --packages-select cleany_handeye_calibration
source install/setup.bash
colcon test --packages-select cleany_handeye_calibration
colcon test-result \
  --test-result-base build/cleany_handeye_calibration --verbose
```
