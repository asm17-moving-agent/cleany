# TEB Sol 6.1 handoff — paused for app-server restart

Stopped on the explicit request relayed by the parent chat. No complete new measurement or final five-goal validation exists.

## Preserved work

- Dedicated root: `/home/donglee/workspace/cleany/controller-optimization-20261013/teb/sol61_20261013`.
- Controller checkout remains clean, detached at `b3ca816ba420452ee7960837bc16ec74148e07b0`: `../source`.
- `runner.py` copied from the original comparison runner, retains the original expanded TEB polygon (stop footprint plus 2 cm planning padding). The earlier pending `footprint_alignment` runner was not used because the instruction forbids shrinking footprint.
- New JSON controller override interface: `CLEANY_TEB_OVERRIDES`. Safety/footprint/feasibility overrides are explicitly rejected. Recording added `/safety_2d/cmd_vel` between Nav2 and the 3-D guard.
- Original compatible TEB source archive was extracted here, including both latest-TF and converter-null cleanup patches. Added diagnostics in `teb_local_planner/src/optimal_planner.cpp`: `TEB_POSE_REJECT` with exception text and pose; `TEB_FEASIBILITY` with failed vertex/interpolated sample and index. Behavior remains unchanged; diagnostic code is **unvalidated** because build was interrupted.
- `cleany_benchmark` timing adapter source copied locally.
- `build.sh` builds the isolated TEB and benchmark overlay; `run.sh` sources the shared pinned Cleany install plus this dedicated overlay. `TEB_EXISTING_BUILD=1` instead sources the original compatible TEB and original timing overlay.
- Domain is **227 only**, localhost discovery and IPs are set. Every run receives a UUID partition. RTF target 1 and timeout 240sim/360wall remain unchanged.

## Interrupted operations

- Build began with `-DCMAKE_BUILD_TYPE=Release`. Dependencies and timing adapter completed; TEB compile was interrupted using its owned process group 83286. Do not regard the overlay as completely installed or validated.
- Important new finding: the original `/tmp/cleany-teb-humble/build/teb_local_planner/CMakeCache.txt` has empty `CMAKE_BUILD_TYPE`. This dedicated build uses Release. Build optimization must be measured separately from parameter tuning (and diagnostic overhead); do not attribute all speed improvement to iteration/candidate changes.
- `00_baseline/0_straight` used the original installed TEB with no parameter overrides. Interrupted during startup, before issuing a goal. `result.json` correctly has `completed=false`, `error="KeyboardInterrupt: "`, elapsed wall 11.63 s. No bag or usable trajectory metrics were produced.
- Original incomplete evidence is preserved. Use new unique output groups, such as `01_baseline_restart`, when resuming.

## Process cleanup verified

Runner PID 85250 received SIGINT, and its launch PID 85330 was stopped by the runner's cleanup. Build process group 83286 received SIGTERM. `check_owned.py` inspected exactly partition `cleany_comparison_e556ba5392a34938a74a4ac686c7c507` and build group 83286; it returned `[]`, verifying no owned leftovers. Other owners' RPP processes were observed but left untouched.

## Execution environment

The host is already `142spp-endeavour` (the recorded desktop comparison host), with `cleany-ros-humble-cdi` available through `distrobox enter`. `/tmp/cleany-teb-humble` exists in the container; it has packages directly at workspace root, not a `src/` directory. Host `/opt/ros` is absent; use the container.

Session permission profile was workspace-write, not Full access/approval never. Distrobox prefix was already authorized; for reliable execution specify escalation when the sandbox's Podman path fails. One inline heredoc distrobox invocation failed with read-only libpod; rerunning the equivalent saved script under approved escalation succeeded.

## Resume commands

```bash
distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-optimization-20261013/teb/sol61_20261013/build.sh
distrobox enter cleany-ros-humble-cdi -- bash -lc 'TEB_EXISTING_BUILD=1 bash /home/donglee/workspace/cleany/controller-optimization-20261013/teb/sol61_20261013/run.sh 01_baseline_restart 0 "{}" > /home/donglee/workspace/cleany/controller-optimization-20261013/teb/sol61_20261013/01_baseline_restart.log 2>&1'
```

Then test the Release diagnostic build on straight with no overrides, before changing a single parameter cause at a time (homotopy class count, iterations, discretization). Inspect feasibility rejection causes first. The TEB feasibility critic catches exceptions; recording `what()` and failed pose/index should distinguish off-map/unknown/lethal footprint from optimizer/velocity failure. No causal finding yet exists.

Final task remains: optimize until evidence shows no further benefit or structural limit; freeze settings; run five common goals once each; write REPORT.md, raw bag/logs, JSON/CSV, timing median/p95/p99 and exceptions/deadlines, CPU/RSS/RTF, command-stage stoppage trace, wheel and velocity-box audit, GT smoothness, final YAML and reproducible patches. Preserve shared layers and separate any auxiliary fixes.
