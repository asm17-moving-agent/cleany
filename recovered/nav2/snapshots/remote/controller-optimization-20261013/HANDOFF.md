# RPP optimization handoff — stopped at user request

## Preserved ownership and paths

- Detached controller-only checkout: `rpp-source/` at `b3ca816ba420452ee7960837bc16ec74148e07b0` (clean, no branch).
- RPP-only runner: `run_rpp_optimization.py`.
- Intended output root: `rpp/`.
- No new build directory was created; trial used the existing comparison build under
  `/home/donglee/workspace/cleany/controller-comparison-20261013/cleany-data/builds/source/install`
  and its `measurement_ws/install` timing adapter.

## Status

- The explicitly requested ROS domain 242 was rejected by Fast DDS before ROS graph
  initialization (`Calculated port number is too high`; Fast DDS permits domain IDs
  through 232). The incomplete attempt is preserved at `rpp/01_short_ttc/`.
- A second trial used domain 232 only because 242 is invalid. It started, reached
  `runtime_verified`, and issued the chair-gap goal, but was interrupted immediately
  when the user cancelled the optimization task. Its incomplete raw output is at
  `rpp/01_short_ttc_232/`; no `result.json` exists and it is not a usable result.
- No source code under `rpp-source/` was modified and no final comparison, tuning
  selection, report, JSON/CSV summary, or commit was produced.

## Diagnosis obtained before cancellation

From the preserved baseline
`/home/donglee/workspace/cleany/controller-comparison-20261013/rpp_rtf1/2_chair_gap`:

- `/nav2/cmd_vel` first became non-zero at sim 8.476 s and changed to a continuing
  zero command at 39.142 s.
- The 3-D guard still passed zero commands around 39.2–40 s; the much later
  `Command timeout` is therefore a downstream consequence after collision-monitor
  output stopped, not the initiating stop. Do not label the 206.858 s observation
  as obstacle waiting or sensor delay.
- The planned first isolated test (`short_ttc`) keeps the common collision monitor,
  voxel guard, footprint, map, and planner intact. It changes only RPP native
  `max_allowed_time_to_collision_up_to_carrot` from 1.0 to 0.35 s, so the candidate
  can test whether RPP's own forward collision projection caused the earlier zero.

## Resume

Enter `cleany-ros-humble-cdi`, source Humble plus the two existing install setups,
and run `run_rpp_optimization.py` for a *new*, unused output directory. Use an
allowed ROS domain (for example 232 if no other owner uses it) and retain the
runner's UUID Gazebo partition and localhost settings. The runner supports:
`baseline`, `short_ttc`, `short_lookahead_ttc`, and `terminal_rotation`.

Do not reuse either incomplete output as measured evidence. First re-run one
representative chair-gap case, then select changes one cause at a time before any
one-pass five-goal final validation.

## Process cleanup

At handoff, `ps` found no process whose command contained
`controller-optimization-20261013` or the generated comparison partition. The
owned launch PID recorded by the interrupted run was 78166; it is no longer live.
