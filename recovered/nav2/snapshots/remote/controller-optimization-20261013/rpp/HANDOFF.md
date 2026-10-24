# RPP v2 handoff — paused for app-server permission restart

User/parent requested immediate stop for applying Full access. No RPP-owned processes remain (verified by exact IGN/GZ partition environment scan).

## Source and build

- Reused isolated clean baseline checkout `../rpp-source`, HEAD b3ca816ba420452ee7960837bc16ec74148e07b0. Original user checkout/evidence preserved.
- New dedicated build: `../rpp-build-v2/` built with `make build-gazebo CLEANY_BUILD_ROOT=<absolute rpp-build-v2 path>` in cleany-ros-humble-cdi. Initial build succeeded; subsequent native-RPP adapter build succeeded in 5.39s (`build-v2.log`).
- New scripts: `../run_rpp_common_v2.py`, `../run_rpp_optimization_v2.py`, `run-v2.sh`. Root env is now set BEFORE importing common module, unlike old runner. Domain 228 enforced. UUID partitions and localhost variables retained.
- Added command subscriptions and bag topics `/pan_ready/cmd_vel`, `/safety_2d/cmd_vel`, `/pan_motion_gate/state`. Existing nav2/final commands, GT, joints, diagnostics and compute timing retained.

## Completed new representative measurement

`v2_01_short_ttc/2_chair_gap` completed and cleaned up naturally immediately before the stop request was handled; it is NOT an interrupted measurement. Action status6 (aborted), navigation39.04sim s, remaining XY1.373808m/yaw92.564deg, actual RTF0.841363. Short TTC0.35 alone did not solve the original near-identical stuck pose. Final actual GT velocity zero. Bag, commands, joints, resources, diagnostics and timing preserved; derived analysis not yet generated. Do not describe as improvement over original240s timeout solely because it aborts sooner. Old `01_short_ttc` and `01_short_ttc_232` remain incomplete and unusable.

Partition `cleany_comparison_11b63f618d344db4a96ca46f22066b5c`, supervisor82526 and launch82551 have exited. Exact partition scan returned no processes. Unrelated friction Gazebo77630 was untouched.

## Confirmed baseline cause

Original chair-gap first RPP exception sequence637 at sim39.142s: collision ahead; 1701 compute exceptions. Nav2 continued zero commands thereafter. Collision monitor stops emitting sustained zeros after stop_pub_timeout2s; final guard then reports Command timeout for206.858sim s. This is a consequence of RPP rejection, not proof of obstacle waiting or stale sensor delay. Native collision checker tests current footprint AND projected arc. Need inspect costmap at failure to distinguish those causes; TTC result suggests current footprint/corner tracking is important but is not proof yet.

Upstream primary reference inspected: https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp . Native tracking computes angular=v*curvature; common guard independently clips angular and leaves vx, changing the path radius. Baseline raw nav2 wheel command can exceed final rate, while final guard caps yaw.

## Unvalidated code candidate (built, NOT Gazebo tested)

Added `cleany_navigation::CurvatureLimitedRPP`, subclassing installed native RPP and scaling returned vx/wz proportionally when |wz| exceeds configurable tracking_yaw_limit(default.14). Native footprint checking and exceptions remain intact. The scaled command follows a prefix of the original checked circular arc. No smoothing, planner, map, footprint or safety layer edits.

Changed files: cleany_navigation/CMakeLists.txt, package.xml; new rpp_plugins.xml and src/curvature_limited_rpp.cpp. Code compiles but is not selected by any runner tune yet. Need add a tune that swaps `wrapped_plugin` to this class (keep TimedController outer plugin), compare one variable at a time, add meaningful regression tests/README documentation, review dynamic parameter behavior, then test Gazebo. No source commit/push.

## Next steps after resume

1. Re-read this handoff and check permissions; use domain228 ONLY.
2. Analyze new short_ttc bag and stage timing; compare with original collision onset. Fix sustained-zero timeout diagnosis separately as common-layer auxiliary if needed; do not claim it cures navigation.
3. Test curvature adapter with original TTC1.0 and original lookahead first; then tune lookahead/rotation based on evidence. Current min lookahead.3 is active at speed.15 (velocity*1.5=.225 below floor).
4. Final frozen configuration: five goals once each,240sim/360wall limits; report accuracy, sim time, compute median/p95/p99/exceptions/deadline, CPU/RSS same-host, command box/wheel maxima and measured wheel rates, actual RTF, GT20Hz five-point acceleration/jerk with.25s trim and stop/reversal counts. Safety and tolerances15cm/10deg unchanged.
5. Write REPORT.md, JSON/CSV, final YAML/source patch, reproducible commands, provenance and all raw evidence. The final campaign and required report remain pending.

Representative reproduction:
`distrobox enter cleany-ros-humble-cdi -- bash /home/donglee/workspace/cleany/controller-optimization-20261013/rpp/run-v2.sh --tune short_ttc --output /home/donglee/workspace/cleany/controller-optimization-20261013/rpp/<NEW_UNIQUE_DIRECTORY> --goal-index 2 --domain 228`
