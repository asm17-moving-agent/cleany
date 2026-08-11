# cleany_description

Authoritative robot description assets shared by MuJoCo, TF, and MoveIt.

## Contents

- `urdf/cleany_geometry.xacro`: backend-neutral canonical physical model
- `urdf/cleany.urdf.xacro`: plugin-free dual-arm URDF used by
  `robot_state_publisher` and MoveIt
- `urdf/cleany_control.urdf.xacro`: control-backend extension entrypoint; it
  intentionally contains no hardware plugin until a backend adds one
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
`front_left_wheel_joint`, and `front_right_wheel_joint`. MuJoCo retains
unnamed passive roller degrees of freedom internally for contact physics;
they are not commandable robot joints and are not published in
`joint_states`.

The default head camera points toward `base_link +X`. Physical `+Y` is the
canonical left arm and physical `-Y` is the canonical right arm. Camera
optical frames are intentionally absent from the URDF; the active calibration
profile owns those transforms. MuJoCo nevertheless carries matching optical
sites, and Gazebo image messages use the corresponding frame names.

Publish the description:

```bash
ros2 launch cleany_description description.launch.py use_sim_time:=true
```

Backend packages must assemble `cleany_geometry.xacro` from their own
top-level description and keep `<ros2_control>` hardware declarations out of
the default `cleany.urdf.xacro`. The control entrypoint is only an extension
seam at this stage; it does not start or select a hardware backend.
