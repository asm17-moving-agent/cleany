# TEB/RPP controller comparison — desktop host

Source: `b3ca816ba420452ee7960837bc16ec74148e07b0`, cloned from the local chat's source bundle into `source/`. Original repositories and dirty changes are preserved. Both shared map/layout SHA256 hashes verified.

Environment: 12 logical CPUs, 46 GiB RAM; Distrobox `cleany-ros-humble-cdi`, ROS Humble, Gazebo Fortress 6.18.0. RPP installed version `1.1.20-1jammy.20260915.232155`. Cleany `make build-gazebo` succeeds.

TEB upstream `humble-devel` SHA `630a22e88dc9fd45be726a762edbb5b776bef231`, costmap_converter `humble` SHA `9565858432664e9cfe0b3556e1809336c4ab06d4`; both source-built. PR421 merged; PR437 remains open, not used. Experiment source is `/tmp/cleany-teb-humble`; build evidence and patch are preserved in this artifact directory.

Pilot RPP straight succeeded: GT position error 0.1070 m, yaw 0.5487 degrees, sim 16.712 s, wall 17.201 s, actual RTF .9716. Pilot TEB aborted after reaching approximately (1.406,-.075,.385): old plan timestamp expires from TF buffer; optimization polygon from base YAML mismatches runtime safety costmap polygon. These pilot results are retained, excluded from the formal five cases.

TEB compatibility changes before formal measurement: guard null costmap converter in cleanup; transform persistent map plan with latest map-to-odom TF, matching Nav2 semantics. Correct optimization footprint to the runtime padded safety polygon; min_obstacle_dist=0 because physical stop/planning margins are already included. All changes are isolated and recorded as a patch.

Formal requested RTF=1. Each candidate gets the same five goals, independent spawn/reset, physics dt=.002 s, noise seed42, common planner, BT, maps, costmaps and safety. Domains 221/222 with independent Gazebo partitions/output paths. Exact compute timing uses standalone `measurement_ws` TimedController forwarding wrapper; no algorithm commands are changed by it.

Known limit: Humble RPP has no native translational acceleration or tracking angular-velocity ceiling. Common final voxel guard caps yaw/speed, and actual wheel command rates are checked. Native acceleration=.4 applies to DWB/MPPI/TEB; no common velocity smoother exists in the provided pipeline. RPP comparison must disclose this rather than claiming identical native acceleration enforcement.

Controller CSV compute durations gathered during concurrent experiments describe parallel scheduling. Same-host DWB auxiliary baseline and isolated compute samples will be retained separately. Contact has not been instrumented yet; absence of log errors does not establish zero contact.

Progress: all ten formal cases completed with frozen settings, no startup retry needed. RPP straight, turn_arrival and far_heading arrived; chair_gap reached the fixed 240 sim-second timeout; distant aborted. TEB turn_arrival arrived; all other four goals aborted. `remote_results.csv` / `.json` contain exactly the ten formal cases, with arrival and remaining errors separated. No pass/fail score is assigned. Supplemental single-instance straight measurements use `aux_dwb`, `isolated_rpp`, `isolated_teb` and separate `auxiliary_results.csv` / `.json`; they are not part of the formal twenty-case comparison.

Recording audit: `/collision_monitor_state` has no recorded messages in completed cases; default zero probe action counts must not be interpreted as zero safety intervention. Some bags lack `/plan`, limiting retrospective path reconstruction. The independent body-guard diagnostic stream is present: STOP_OBSTACLE duration is separated from STOP_UNAVAILABLE and STOP_PROCESSING_OVERRUN. Actual physics contacts remain unknown. Static projected physical-footprint clearance is a conservative, sampled geometric estimate, not contact instrumentation.

Artifacts: each formal run preserves bag, runtime parameters, launch log, GT samples/velocity, commands, wheel feedback, sensor reception timestamps, safety diagnostics, compute timing and CPU/RSS observations. `analysis.json`, `geometry_summary.json`, and `audit.json` provide derived evidence and limitations; `environment.json`, source bundle and TEB compatibility/source archives preserve provenance.

Complete: three auxiliary single-instance straight runs also finished (DWB/RPP arrived, TEB aborted), separate from the ten formal cases. Direct compute p95/p99 ms: DWB 15.444/16.359, RPP .158/.214, TEB 51.693/57.261. Final audits confirm thirteen completed measurement records, final command and GT velocity zero, observed wheel/final-command rates <=5rad/s, one shared map hash. Owned Gazebo/Nav2/runner processes have exited; existing containers and dirty user repositories remain preserved. See `REPORT.md` for the final handoff and limitations.
