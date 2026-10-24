# Reproduction

Run on x86_64 host `142spp-endeavour` using existing Distrobox
`cleany-ros-humble-cdi`. The original pinned Cleany overlay and isolated TEB
overlay must be present at the paths in `environment.json`; this is not a
portable binary archive. Use a new unused output group for every run.

```bash
distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-optimization-20261013/teb/sol61_20261013/build.sh
```

The compatible TEB source bundle is
`/home/donglee/workspace/cleany/controller-comparison-20261013/teb-compatible-source.tar.gz`,
already includes `compatibility.patch`, and must not receive that patch twice.
`diagnostics.patch` adds pose/index/exception reason logging without changing
feasibility decisions. `teb_local_planner/CMakeLists.txt` forces Release,
including when its CMake cache build-type field is empty.

```bash
# Single representative straight run, original compatible binary:
TEB_EXISTING_BUILD=1 distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/run.sh baseline_new 0 '{}'
# Final frozen parameters; script is written after pilot selection:
distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/run_final.sh final_new
# Analyze our new outputs only:
distrobox enter cleany-ros-humble-cdi -- python3 /home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/analyze.py
# Inspect exact owned partitions; does not stop anything:
distrobox enter cleany-ros-humble-cdi -- python3 /home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01/check_owned.py
```

All runs keep ROS domain 227, loopback discovery/IPs, unique UUID Gazebo
partitions, RTF target 1, 0.002s physics step, 20Hz controller wall timer,
240sim/360wall navigation watchdog, 15cm/10degree tolerances, common map,
StraightFirstPlanner, padded footprint and full safety/feasibility checks.
Each final goal starts a fresh simulation at the common initial pose.

Raw artifacts per run: `bag/`, `launch.log`, `bag.log`, `samples.csv`,
`velocity.csv`, `commands.csv`, `joints.json`, `diagnostics.json`,
`compute_timing.csv`, `resources.json`, `events.json`, `runtime_verified.json`,
`config/`, `scenario.json`, `result.json`, `analysis.json`,
`command_transitions.json`, and exact generated world/launch files.

`result.json.completed` means the measurement completed, not that Nav2
arrived. Nav2 status, GT terminal error and GT tolerance observation are
separate. Failed runs remain in summaries; their error is remaining distance,
not arrival accuracy. Command transitions use receiver sim/steady-wall time,
not a source stamp (Twist has no stamp). Nearby guard decisions are context,
not proof of causal ownership of a preceding stop.

GT smoothness uses 20Hz sim-time interpolation, centered 5-point velocity
smoothing, twice differentiated at 0.05s, and 0.25s trimming on both sides.
Stationary time and direction changes accompany jerk to prevent long failed
stops from appearing as superior smoothness.
