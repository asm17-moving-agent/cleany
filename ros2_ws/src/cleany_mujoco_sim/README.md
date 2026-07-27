# cleany_mujoco_sim

ROS 2 (ament_python) package wrapping the XLeRobot MuJoCo simulation.

## Status and responsibility

This is an implemented simulation bridge, not the full robot interface. It
loads the MuJoCo scene, publishes simulation state, and accepts a direct joint
position test hook. Production navigation, manipulation, and hardware adapters
remain outside this package.

## Run

Headless simulation:

```bash
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py
```

Simulation with the MuJoCo viewer:

```bash
ros2 launch cleany_mujoco_sim mujoco_sim.launch.py headless:=false
```

## Test

From the repository root:

```bash
./scripts/pytest-ros src/cleany_mujoco_sim/test/test_scene_loader.py
./scripts/ros2-build
```

## Topics

`mujoco_sim_node` publishes:

- `joint_states` (`sensor_msgs/JointState`)
- `odom` (`nav_msgs/Odometry`)
- `scan` (`sensor_msgs/LaserScan`)
- `tf` (`odom` -> `base_link`) when `publish_odom_tf` is true
- `tf_static` (`base_link` -> `laser`) when laser scan publishing is enabled

`mujoco_sim_node` subscribes:

- `~/joint_cmd` (`sensor_msgs/JointState`) - sets target joint positions
  directly. This is a simple simulation hook, not a controller.

There is not yet a `/cmd_vel` base command interface. The MuJoCo model exposes
four independent mecanum-wheel DC motor voltage inputs; a ROS mecanum command
adapter is still required to convert chassis velocity commands into them.

## Base Drive Model

Each wheel uses an independent `PG42-4266-1270NE` output-shaft DC motor model.
The actuator controls are terminal voltages named `rear_left_drive`,
`rear_right_drive`, `front_left_drive`, and `front_right_drive`.

- Nominal supply: `12 V`
- Gearbox: `61:1`, with the published `72%` efficiency already included in the
  manufacturer output torque
- Manufacturer rated output: `2.94 N.m` at `103 rpm`
- Calculated no-load output: `7000 / 61 rpm` (`12.017 rad/s`)
- Simulation operating limit with 10% margin: `10.8 V`, `2.646 N.m`
- Derated rated point: `92.7 rpm` (`9.708 rad/s`)
- Derated no-load equilibrium: `103.28 rpm` (`10.815 rad/s`)

The MJCF actuator is modeled directly at the gearbox output (`gear=1`), so the
gear reduction and efficiency are not applied a second time. The motor input is
voltage, not target wheel velocity; the future base adapter must perform wheel
mixing and closed-loop velocity control.

## Arm Servo Model

Both arms use Feetech 12 V serial servos with the following assignment:

- `Pitch_L`, `Elbow_L`, `Pitch_R`, and `Elbow_R`: `STS3250`
- Shoulder rotation, wrist pitch/roll, jaws, and head pan/tilt: `STS3215`

The modeled output limits apply a 10% operating margin to the manufacturer
specifications:

- `STS3215` (`ST-3215-C018`): `2.648 N.m` peak limit,
  `0.883 N.m` derated rated torque, and `4.245 rad/s` no-load limit
- `STS3250` (`ST-3250-C001`): `4.413 N.m` peak limit,
  `1.412 N.m` derated rated torque, and `7.086 rad/s` no-load limit

Position actuator and joint force limits use 90% of peak stall torque. Joint
damping is calibrated to reproduce 90% of each no-load speed while saturated.
The existing simulation position-loop gains are retained because Feetech
documents the PID as configurable but does not publish one fixed factory gain.
Current, thermal, and two-second overload shutdown behavior are not yet modeled.

## Launch Arguments

`mujoco_sim.launch.py`:

- `scene_path` - MuJoCo scene XML. Defaults to `hardware/scene.xml`.
- `publish_rate_hz` - simulation publish/timer rate. Defaults to `60.0`.
- `headless` - whether to hide the MuJoCo viewer. Defaults to `true`.
- `scan_rate_hz` - laser scan publish rate. Defaults to `5.5`.
- `scan_samples` - number of rays per scan. `0` derives the sample count from
  `scan_sample_rate_hz`.

Additional node parameters supported by `MujocoSimNode`:

- `base_body_name` - MuJoCo body used as the robot base. Defaults to `chassis`.
- `lidar_site_name` - MuJoCo site used as the laser origin. Defaults to
  `lidar_site`.
- `odom_frame_id` - odometry frame id. Defaults to `odom`.
- `base_frame_id` - base frame id. Defaults to `base_link`.
- `laser_frame_id` - laser frame id. Defaults to `laser`.
- `publish_odom_tf` - publish dynamic odom-to-base transform. Defaults to
  `true`.
- `scan_enabled` - publish `scan` and static laser transform. Defaults to
  `true`.
- `scan_sample_rate_hz` - virtual ray sample rate used when `scan_samples` is
  `0`. Defaults to `8000.0`.
- `scan_range_min` - minimum valid scan range. Defaults to `0.15`.
- `scan_range_max` - maximum valid scan range. Defaults to `12.0`.

## Scope

This package is still a simulation bridge, not the full robot interface.

Implemented:

- Load an XLeRobot MuJoCo scene.
- Simulate four articulated 5-inch mecanum wheels with independent PG42 drive
  motors.
- Step the simulator on a ROS timer.
- Publish joint state, odometry, laser scan, and TF data.
- Apply direct joint position commands for simulation tests.

Not implemented yet:

- `/cmd_vel` or Nav2-compatible base command handling.
- A ROS mecanum command adapter and closed-loop wheel-speed controller.
- Binding to `cleany_robot_interface`, `cleany_perception`, or the mission FSM.
- Hardware-realistic controllers for the base or manipulators.

## Related KB and documentation updates

- [Robot Platform XLeRobot](../../../docs/cleany-docs/20_TECHNICAL/04%20-%20Robot%20Platform%20XLeRobot.md)
- [Navigation and Mapping](../../../docs/cleany-docs/20_TECHNICAL/05%20-%20Navigation%20and%20Mapping.md)
- [Safety and Risk](../../../docs/cleany-docs/20_TECHNICAL/08%20-%20Safety%20and%20Risk.md)

Update this README with any changes to its published topics, launch parameters,
simulation model assumptions, or test commands. Do not present simulated
hardware parameters as confirmed physical-hardware specifications without the
corresponding KB decision.
