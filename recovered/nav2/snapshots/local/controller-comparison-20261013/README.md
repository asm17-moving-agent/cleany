# Nav2 controller comparison — 2026-10-13

Five common static-map routes per candidate; no pass/fail score. Position tolerance 15 cm and heading tolerance 10 degrees. All candidates preserve the common planner/costmaps/BT/safety chain.

Wheel limit uses CAD radius 0.0635 m, wheelbase 0.35 m, track 0.6038 m. Conservative common command box vx=0.15, vy=0.10, wz=0.14 gives an inverse-kinematic wheel bound below 5 rad/s. Measured joint maximum is separately reported; a command bound does not prove a physical feedback bound.

Official runs request RTF=1; pilots and accelerated exploratory runs are excluded from comparison.csv. Command sim-time intervals and achieved RTF are included to reveal timing differences. Controllers run concurrently with isolated ROS domains/Gazebo partitions; CPU utilization is under concurrent load. Host CPU values cannot directly rank candidates executed on different hardware. Per-cycle controller compute time is measured on the desktop TEB/RPP runs; local DWB/MPPI timings are supplemental straight-route profiling, kept separate from the 20 official runs. ARM64 and x86_64 hosts differ, so absolute times cannot directly rank all four algorithms.

Smoothness uses GT velocity in the message twist frame, 20 Hz simulation-time interpolation, centered 5-point moving average and two gradients; first/last 0.25 seconds discarded. These are body velocity change metrics, not calibrated passenger comfort or physical acceleration measurements. Stationary time includes intentional safety/yield stops; counts are descriptive.

Static-world contact and minimum true footprint clearance were not instrumented. Existing SafetyProbe clearance is for an injected fixture, which these five routes do not inject. Do not claim collision-free from action success. Gazebo evidence does not prove physical robot performance.

A timed-out/aborted route is included as an outcome; arrival accuracy of such a route is not a successful-arrival score. Startup infrastructure retries are recorded separately. Humble RPP lacks the native translational acceleration limit used by the other candidates; its final speed/yaw are capped by the common guard. No shared velocity smoother was added, so jerk comparisons include that capability difference. TEB uses a recorded ROS 2 compatibility patch and polygon footprint configuration, not an unmodified default.

Five varied cases provide descriptive observations, not a statistically established success rate.

Nav2 status 4 is the controller-stack arrival decision in its estimated pose frame, not a guarantee of the GT arrival tolerance. The DWB chair-gap run had a 17.225 cm GT error despite status 4. `completed` means the experiment reached an outcome, including abort/cancel; it does not mean navigation succeeded.

All four supplemental compute profiles were also run on the same ARM64 host. See profile_compute.csv. Each algorithm follows its own trajectory; scheduler effects and differing state/costmap sequences remain, so these are reference measurements, not a frozen-input microbenchmark.

The raw SafetyProbe `safety_action_sample_counts` can count default zero when no collision state message is observed; it must not be interpreted as observed intervention counts. The remote follow-up report provides separately instrumented safety diagnostics and a conservative projected static-clearance analysis. Physical contact remains unmeasured.
