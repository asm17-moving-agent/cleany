# cleany_handeye_calibration

ROS-independent mathematical core and, in later commits, ROS adapters for
Cleany's left wrist eye-in-hand calibration workflow.

The current package contains immutable data models, frame-aware rigid
transforms, and the version-1 draft sample record. It does not yet detect the
target, run PnP or hand-eye solvers, call MoveIt, collect camera data, or
publish calibration TF.

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

## Dependencies

The transform conversion functions use the Ubuntu/ROS system installations of
NumPy and OpenCV. On the target Ubuntu 22.04 / ROS 2 Humble environment these
are provided through the `python3-numpy` and `python3-opencv` rosdep keys.
The package has no `rclpy` dependency and its core tests need no running ROS
graph.

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
