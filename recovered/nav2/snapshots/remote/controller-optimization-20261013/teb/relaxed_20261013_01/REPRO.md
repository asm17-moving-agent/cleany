# Gazebo-only relaxed stage

This stage is run after the conservative final campaign and REPORT.md are saved.
It reuses its exact effective map/localization/costmap setup, isolated compatible
TEB overlay, geometry, unique Gazebo partitions, ROS domain 227 and localhost
discovery. No physical robot run is included.

The final native settings, runner, generated world/launch/BT YAML, frozen
overrides and raw bags are all retained. TEB footprint and feasibility remain
enabled; overrides disabling them are rejected in this formal runner.

```bash
# A selected representative goal, never an initial five-repeat campaign:
TEB_PILOT_SIM_LIMIT=180 distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/run.sh pilot_new 3 '{}'
# Final script is written after selection; output must be new/unused:
distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/run_final.sh final_new
# Decode full raw physics contacts, including brief contacts missed by live QoS:
distrobox enter cleany-ros-humble-cdi -- bash -lc 'source /opt/ros/humble/setup.bash; python3 /home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/read_contacts.py'
# Analyze descriptive metrics:
distrobox enter cleany-ros-humble-cdi -- python3 /home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01/analyze.py
```

The conservative source/patches/build logs are in
`/home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/`.
The actual TEB shared-library path/hash is recorded per run. Control frequency,
compute wall budget, actual sim command intervals and RTF are recorded separately.
The 240sim-second mark is informational; final hard watchdog is
1200sim/1800wall seconds per goal. A pilot segment limit only applies when the
explicit pilot environment variable is set and is recorded in its result.

The command path is Nav2 raw -> coupled mecanum wheel envelope -> cmd_vel ->
Gazebo command guard. Wheel geometry is r=.0635, wheelbase=.35,
separation=.6038, cap=5rad/s. The whole requested twist is scaled uniformly,
never individually clipped to the old box. The selected implementation uses
4.99rad/s internally to leave numerical margin under the physical 5rad/s cap.
A 20Hz convex twist interpolation limits every wheel's commanded slew to
8rad/s². The Gazebo MecanumDrive body-axis ±2 acceleration limiter is removed
in this stage: independent axis interpolation can exceed the coupled wheel
envelope between valid endpoints. The first pilot measured joint max5.302
with capped commands; the corrected pilot tests the replacement actuator layer.
This common actuator change must be applied to every controller for a fair
comparison within this relaxed condition. Geometry and physics dt stay fixed. The Gazebo guard's .4/.4/1.0 bounds
are outside this wheel envelope and its wall command watchdog is 2 seconds.
Counterfactual pan/freshness, Collision Monitor and voxel guard outputs are
routed to ignored/cmd_vel; their decisions are not actual waiting durations.

Contact sensors cover every collision on every robot link and use the
Fortress Contact system; contact_sensors.patch and the original unsensored
world are retained. Robot geometry is unchanged. The native system publishes
only while touching at physics-update frequency; the SDF update_rate does not
throttle it. All contact messages go into the bag. Live JSON retains non-floor
contacts plus 1Hz floor positive-control snapshots to bound memory, while
read_contacts.py derives formal metrics from the full bag. Floor/self contact
is excluded from obstacle-contact episode counts, with .25sim-second gap as
the episode boundary. Unknown attribution stays null with a reason.

Goal checker still uses .15m/10degrees. Nav2 status and GT XY/yaw error are
independent. GT smoothness uses the identical 20Hz/centered5-point/.25s-trim
method as the conservative stage. Both low jerk and stalled time must be read
together. Different CPU hosts are not ranked by absolute compute/CPU values.

The wheel envelope is max|wheel_i|=(|vx|+|vy|+K|wz|)/r with
K=(.35+.6038)/2. Uniform scaling preserves the requested twist direction.
Slew interpolates the previously emitted twist toward the capped target by
alpha=min(1,8*dt/max|wheel(target-previous)|), where dt<=.05sim s.
Convex interpolation stays within the wheel envelope. The cap's .01rad/s
margin absorbs numerical feedback error; it does not redefine the physical cap.
The watchdog chooses a zero target after2wall seconds without raw controller
commands and decelerates within the same slew bound. Gazebo guard timeout2s
remains the downstream fallback. Steady-clock watchdog and sim-clock smoothing
are recorded separately in wheel_limiter.json.

The exact final limiter and runner sources are copied to each goal directory;
artifact_checks.json compares their hashes with frozen_overrides.json, checks
the one goal issue per run, full contact attribution, command and joint limits,
internal feasibility, unchanged collision XML and terminal stop. These audit
checks do not assign an arrival acceptance score.
