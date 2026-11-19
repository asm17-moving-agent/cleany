# Controller compute timing

This standalone Humble package wraps any `nav2_core::Controller` without changing
the algorithm namespace, lifecycle methods, plan, speed limit, commands, or
exceptions. Source it after the ROS and original controller overlays.

```yaml
controller_server:
  ros__parameters:
    controller_plugins: [FollowPath]
    FollowPath:
      plugin: cleany_benchmark::TimedController
      wrapped_plugin: teb_local_planner::TebLocalPlannerROS
      benchmark_timing_file: /absolute/run/directory/controller_compute.csv
      # Original FollowPath parameters go here unchanged.
```

RPP's original plugin is
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
The destination directory must already exist. CSV files are appended to preserve
existing evidence; use a distinct file per run. A new wrapper instance restarts
sequence numbering at 1, while a cleanup/reconfigure of the same instance keeps it.

CSV columns:

| Column | Meaning |
|---|---|
| `sequence` | Invocation number, starting at 1 |
| `sim_time_ns` | Node clock immediately before the call, nanoseconds; simulated time when `use_sim_time=true` |
| `wall_start_ns` | Monotonic steady clock start, nanoseconds since its implementation-defined epoch |
| `compute_us` | Steady wall duration of the wrapped compute call, microseconds |
| `success` | 1 when the method returns, 0 when it throws; exception is rethrown unchanged |

File writes, mutex acquisition, and flushes happen after the end timestamp. A
64 KiB stream buffer is requested; explicit flush occurs every 128 calls and on
cleanup/destruction. Abrupt process termination may lose the last buffered rows.
The measurement includes the original method's own visualization/logging and
thread scheduling, and excludes the surrounding Nav2 cycle. Successful rows mean
the method returned, not that navigation arrived at the goal.

For a 20 Hz controller, report a 50 ms wall deadline and the separately identified
RTF-adjusted wall budget (25 ms at actual RTF 2). Do not infer actual RTF from its
configured target. Report p95/p99 for successful calls and exception calls
separately, retaining both raw populations. `wall_start_ns` is monotonic, not UTC.

Build inside the installed ROS container:

```sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
