# cleany_description

Authoritative robot description assets shared by MuJoCo, TF, and MoveIt.

## Contents

- `urdf/cleany_geometry.xacro`: backend-neutral canonical physical model
- `urdf/cleany.urdf.xacro`: plugin-free dual-arm URDF used by
  `robot_state_publisher` and MoveIt
- `urdf/cleany_control.urdf.xacro`: control-backend extension entrypoint; it
  adds the MuJoCo `ros2_control` hardware interface for the ten arm joints and
  read-only state interfaces for both grippers
- `urdf/cleany_mujoco_ros2_control.xacro`: reusable MuJoCo hardware macro
- `mjcf/cleany.xml`: MuJoCo robot model included by simulator scenes
- `meshes/`: visual and collision CAD assets referenced by both descriptions
- `test/test_model_parity.py`: canonical joint and randomized FK parity checks

## Coordinate and rotation convention

The descriptions follow ROS REP-103:

- right-handed `base_link`: `+X` forward, `+Y` left, `+Z` up
- positive rotation follows the right-hand rule
- roll, pitch, and yaw rotate about `+X`, `+Y`, and `+Z`
- positive yaw is counter-clockwise when viewed from above
- wheel axes are `+Y`, so positive wheel rotation drives toward `+X`
- `_optical_frame` axes are `+X` right, `+Y` down, `+Z` forward

The public mobile-base joint contract contains only
`rear_left_wheel_joint`, `rear_right_wheel_joint`,
`front_left_wheel_joint`, and `front_right_wheel_joint`. MuJoCo retains named
passive roller degrees of freedom internally for contact physics. The names
allow `MujocoSystemInterface` to validate the MJCF, but these joints are not
listed in `ros2_control`, are not commandable, and are not published in the
control backend's `joint_states`.

The default head camera points toward `base_link +X`. Physical `+Y` is the
canonical left arm and physical `-Y` is the canonical right arm. Camera
optical frames are intentionally absent from the URDF; the active calibration
profile owns those transforms. MuJoCo nevertheless carries matching optical
sites, and Gazebo image messages use the corresponding frame names.

`cleany_control.urdf.xacro` registers the `left_wrist_rgb` MJCF camera as a
`ros2_control` sensor for the hand-eye MuJoCo backend. Its vendor topic names
and 10 Hz render rate are consumed by Humble `mujoco_ros2_control` 0.0.3; the
simulation package launch owns remapping and the public camera contract.

Publish the description:

```bash
ros2 launch cleany_description description.launch.py use_sim_time:=true
```

The default `cleany.urdf.xacro` remains plugin-free. The MuJoCo control
backend expands `cleany_control.urdf.xacro` with the materialized scene path
and runtime options:

```bash
xacro urdf/cleany_control.urdf.xacro \
  mujoco_model:=/absolute/path/to/control_scene.xml \
  headless:=true \
  sim_speed_factor:=1.0
```

The control entrypoint exposes position commands plus position and velocity
state for the five joints of each arm. Both gripper joints expose read-only
position and velocity state so MoveIt receives a complete dual-arm model
state; they have no command interface. The base, head, and passive roller
joints remain outside this control contract.
