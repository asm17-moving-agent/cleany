# cleany_description

Shared CAD-frame robot model for MuJoCo, TF, and MoveIt.

## Files

- `mjcf/cleany.xml`: MuJoCo geometry, dynamics, cameras, and actuators.
- `urdf/cleany_geometry.xacro`: base, arm mounts, and wheels.
- `urdf/physical_properties.xacro`: URDF geometry, colors, and inertials.
- `urdf/dual_arm.xacro`, `urdf/head_camera.xacro`: joint and frame trees.
- `urdf/cleany.urdf.xacro`: plugin-free description; 29 links, 18 movable joints.
- `urdf/cleany_control.urdf.xacro`: arm-only ros2_control description;
  21 links, 12 movable joints.
- `meshes/`: shared assets; runtime needs no external CAD export.

## Model contract

- REP-103: `base_link` +X forward, +Y left, +Z up; wheel axes +Y.
  Optical frames use +X right, +Y down, +Z forward.
- Wheelbase **0.35 m**, track **0.6038 m**, wheel radius **0.0635 m**.
  URDF wheels are rigid 500 g assemblies; 48 passive roller joints are MJCF-only.
- `${side}_grasp_tcp` is fixed at `(0, -0.100, 0)` m in
  `${side}_gripper_frame`; the original hinged gripper is retained.
- Head RGB-D transforms are nominal, not measured calibration.
  Wrist camera ground-truth edges remain out of runtime TF for hand-eye calibration.
- The control entrypoint commands ten arm joints and reads both grippers.
  Gripper commands are opt-in via `enable_gripper_command:=true`.
  Base, head, and rollers are outside this control contract.
- Arm-only MoveIt uses `include_head_camera:=false include_wheel_joints:=false`.
  These default to `true` in the full description; disabling wheel joints
  retains their geometry as fixed links.
- Fourteen legacy servo position actuators and four PG42 voltage actuators
  remain active. New servo DC-motor defaults are commented out pending the
  separate MuJoCo/ros2_control upgrade. The 3.4 backend removes only wheel
  actuators from its temporary scene; see [simulation setup](../cleany_mujoco_sim/README.md).

Mass properties are estimates: total **15.933871 kg**,
including a provisional **5.20 kg, 195×165×175 mm battery**, **0.8 kg per PG42
motor**, and estimated PLA parts.

## Use

After building and sourcing `ros2_ws`, publish the full description:

```bash
ros2 launch cleany_description description.launch.py use_sim_time:=true
```

Expand the control description from this package directory:

```bash
xacro urdf/cleany_control.urdf.xacro \
  mujoco_model:=/absolute/path/to/control_scene.xml headless:=true
```

Run geometry, color, mass, joint-contract, and randomized FK checks from `ros2_ws`:

```bash
python3 -m pytest src/cleany_description/test/test_model_parity.py
```
