# Robot model 계약


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
  actuators from its temporary scene; see [simulation setup](../../cleany_mujoco_sim/docs/backends.md).

Mass properties are estimates: total **15.933871 kg**,
including a provisional **5.20 kg, 195×165×175 mm battery**, **0.8 kg per PG42
motor**, and estimated PLA parts.


Gazebo navigation adapter는 park posture로 팔과 head를 고정하고 wheel joint·센서를 부착한다.
실물 형상·관성·센서 보정의 검증값으로 취급하지 않는다.
