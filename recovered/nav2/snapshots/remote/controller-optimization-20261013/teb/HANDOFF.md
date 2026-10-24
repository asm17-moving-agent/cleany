# TEB optimization handoff (stopped by superseding request)

Status: **stopped before any Gazebo/Nav2 run**.  No final five-goal validation,
no candidate measurement, and no new bag was produced in this chat.

## Isolated assets

- Output root: `/home/donglee/workspace/cleany/controller-optimization-20261013/teb`
- Pinned controller checkout:
  `/home/donglee/workspace/cleany/controller-optimization-20261013/teb/source`
  at detached `b3ca816ba420452ee7960837bc16ec74148e07b0`.
- Experiment runner (uncommitted, output-local change only):
  `run_teb_optimization.py`.
- Timing-wrapper source:
  `measurement_ws/src/cleany_benchmark`.  Its build/install/log directories
  were cleared after discovering they had been copied with stale absolute
  paths; rebuild this workspace in the intended Humble container before use.

No user checkout was edited, and the isolated checkout remains clean.  The
runner itself is not inside that checkout.

## Evidence inspected

Baseline artifact: `/home/donglee/workspace/cleany/controller-comparison-20261013`.
Its formal TEB results were 1/5 arrivals, with p95 controller compute time
58.9--65.0 ms.  In the straight baseline launch log, TEB emits repeated
`trajectory is not feasible. Resetting planner...`, then Nav2 aborts.  The
baseline runner constructed the TEB polygon by adding `planning_padding` to
the already padded `navigation_geometry.stop` footprint.  The costmap uses
the stop footprint with its configured padding, so this is a candidate
controller-only footprint mismatch worth testing; it is not evidence that
safety should be weakened.

## Pending runner changes

`run_teb_optimization.py` changes only TEB controller parameters:

1. `footprint_alignment` (default) uses `navigation_geometry.stop` directly,
   avoiding the extra 2 cm expansion in the TEB polygon.
2. `bounded_search` additionally disables homotopy/multithreading and sets
   `no_inner_iterations=3`, `no_outer_iterations=2`, `dt_ref=.4`, lookahead
   2 m.  This profile has not been run or validated.

The compatibility assumptions from the original artifact remain required:
TEB humble-devel `630a22e88dc9fd45be726a762edbb5b776bef231` with the existing
latest-TF transform and null-converter cleanup guard patches.  Do not mistake
the runner change for a replacement of those patches.

## Resume outline

In the intended `cleany-ros-humble-cdi` environment, source Humble, the
original isolated Cleany install, patched TEB install, and then build/source
this output's `measurement_ws/install/setup.bash`.  Set
`CLEANY_COMPARISON_REPO` to this output's `source`,
`CLEANY_BENCHMARK_TIMING=1`, `CLEANY_TEB_PROFILE=footprint_alignment`, and
use domain 241 plus a unique Gazebo partition.  First execute only goal 0;
audit infeasible-reset count and compute p95 before attempting `bounded_search`.
Preserve RTF 1, the shared map/safety/planner, velocity box, and wheel checks.

This host's direct shell does not expose `/opt/ros/humble` or `colcon`, and
`distrobox` could not access Podman because `/run/user/1000/libpod` is
read-only.  Therefore no build or simulation process remains from this chat.

## Latest continuation (Sol 6.1)

The replacement chat subsequently created `sol61_20261013/` and stopped at the parent's explicit app-server restart request. See `sol61_20261013/HANDOFF.md` for the latest preserved code, incomplete build/run, ownership cleanup, and resume commands. Use domain **227**, superseding the invalid 241 example above.
