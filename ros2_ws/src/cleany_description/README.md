# cleany_description

Authoritative robot description assets shared by MuJoCo, TF, and MoveIt.

## Contents

- `urdf/cleany.urdf.xacro`: dual-arm URDF used by
  `robot_state_publisher` and MoveIt
- `urdf/head_camera.xacro`: nominal head pan/tilt and RGB-D frame tree
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
canonical left arm and physical `-Y` is the canonical right arm.

The nominal head camera frame tree is shared by URDF and MJCF:

```text
base_link
└── top_base_link
    └── head_pan_link
        └── head_tilt_link
            └── head_camera_link
                ├── head_camera_rgb_frame
                │   └── head_camera_rgb_optical_frame
                └── head_camera_depth_frame
                    └── head_camera_depth_optical_frame
```

RGB and aligned depth use colocated nominal optical origins. These fixed
transforms describe the current simulation assembly; they are not a measured
RealSense calibration. A real deployment must validate or replace them with
its calibration profile while preserving the public frame contract.

Each arm exposes `${side}_grasp_tcp` as a fixed frame and MuJoCo site at
`(0, -0.100, 0) m` in `${side}_gripper_frame`. It is a nominal point near the
center of the jaw tips for position-only IK. Its orientation inherits the
gripper frame and is not a calibrated grasp orientation.

Publish the description:

```bash
ros2 launch cleany_description description.launch.py use_sim_time:=true
```
