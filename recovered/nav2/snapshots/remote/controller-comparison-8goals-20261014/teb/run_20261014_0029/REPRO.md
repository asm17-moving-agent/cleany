# TEB 18F facility, eight goals once each

Common canonical preflight and goals are owned by DWB's local session.
No campaign may begin before common/READY and goals.locked.json exist.
The runner consumes canonical goal-specific worlds/maps without editing goals,
removing chairs independently, shrinking physical/padded footprints, or
returning to the older study-cafe world.

```bash
distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-comparison-8goals-20261014/teb/run_20261014_0029/run.sh final8
```

Use a fresh output group for any separate later campaign. This run's campaign
is eight independent fresh-world goals, each issued once. Startup failures
before issuing a goal may have a labeled fresh startup retry; goal-issued
failures are never retried. Domain227, UUID partitions and localhost discovery
isolate this controller; other owners' processes and evidence are untouched.

World-aligned map initial pose is[-1.865,-4.705,1.5708]. The actual Nav2 goal
checker parameters are queried and asserted as.30m/10degrees, stateful.
Ground-truth position/yaw are world aligned; Nav2 status and GT errors are
independent diagnostic fields. No overall score or acceptance threshold is
assigned to the controller results.

The previous relaxed TEB profile remains singleband, no multithreading,
controller10Hz, inner/outer5/4, dt_ref.1, dt_hysteresis.02, min_obstacle_dist.05.
Raw axis maxima are.3175m/s and.665758rad/s yaw. The actual command route is
Nav2->uniform mecanum wheel scaling->convex wheel slew limiter->cmd_vel
->Gazebo command guard. Radius.0635m, wheelbase.35m, separation.6038m,
physical wheel cap5rad/s, numerical output cap4.99rad/s, wheel slew8rad/s²,
20Hz output. Gazebo's body-axis acceleration limiter is removed as in the
previous final profile; collision geometry is preserved.

External pan/fresh-depth, Collision Monitor and voxel guard outputs remain
counterfactual diagnostics at ignored/cmd_vel. Costmap observations and TEB
internal footprint/collision feasibility remain active. No additional safety
bypass is permitted. Hard watchdog1200sim/1800wall seconds per goal; the
240sim-second mark is informational. Headless server and EGL rendering are
requested; GUI launch is disabled. RTFtarget1; physics step follows canonical
world and is recorded rather than silently replaced with the older.002s step.

Each output materializes fixture://URIs into its own fixture directory and
checks files/hashes and parent-host path leakage before launch. Actual Gazebo
scene service is saved and compared against canonical model identities and
visual mesh filenames, destination-chair absence, all other models' presence,
and server mesh-loading diagnostics. Sensor streams must initialize normally
before any goal is issued. Canonical per-collision contact sensors are bridged
and all contacts are preserved in rosbag2; floor contacts are a positive control,
not obstacle episodes. Unknown contact attribution remains null with reason.

GT jerk uses20Hz interpolation, centered5point smoothing before derivatives,
.25sim-second boundary trimming. The common stationary definition is
hypot(vx,vy)<.01m/s AND abs(wz)<.02rad/s. Failed/stalled low jerk is never
interpreted as smoother useful navigation. Controller compute median/p95/p99,
exceptions, wall budget100ms, sim/wall control periods, CPU/RSS and RTF are
reported. Local ARM and remote x86 absolute compute/CPU rankings are invalid.

The matching current-kernel overlay module and isolated NVIDIA610.57.04
library/CDI repair are recorded in environment-repair/repair.json. Existing
container/image, native driver libraries and running NVIDIA kernel module
were preserved. Do not delete the private driver root while its CDI spec or
containers depend on it. This host environment repair is a comparison caveat.

Locked common planner uses a .025m global raster, 80s planning budget and 32
heading bins. The local costmap resolution and padded CAD collision footprint
are retained. Three source-host texture filenames are relocated to identical
named assets in the pinned source; hashes and paths are recorded in each run.
Canonical SDF LiDAR mount gives nominal .30m height while common sensor TF gives
.26m. Both inputs are retained. Read PREPARATION_STATUS.md for current campaign
state; offline preparation artifacts are not navigation measurements.

Campaign completion: all8 accepted navigation goals were run once; see campaign_status.json. The seventh index other_room aborted. No further measured goal was issued.

The distant bag is partial after recorder SQLite read-lock termination. Do not inspect a live SQLite bag. Its original DB bytes were verified unchanged by offline metadata reindex; full contact count remains null. All independent observer and compute records are retained.

The original TEB batch parameter query returned an empty batch due undeclared goal-tolerance keys. Original result.json is preserved. parameter_replay.py configures the same binary/YAML with no Gazebo, activation or navigation to verify provided settings; its evidence is explicitly separate from original runtime queries. The captured original_analysis_snapshot.py matches the original enrichment dependency byte for byte.
