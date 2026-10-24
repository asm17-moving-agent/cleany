#!/bin/bash
set -e
RUN="/home/ehdrms/workspace/cleany/cleany-data/runs/standard-planner-20261014"
OLD=/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi
source /opt/ros/humble/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/mppi-optimization/install/local_setup.bash
source "$OLD/sol_resume_20261013/relaxed/mppi_ws/install/local_setup.bash"
source "$OLD/measurement_ws/install/local_setup.bash"
export CLEANY_COMPARISON_REPO=/home/ehdrms/.codex/worktrees/mppi-optimization/main
export CLEANY_BENCHMARK_TIMING=1 MPPI_OVERRIDE="$RUN/final_overrides.yaml"
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
exec /usr/bin/python3 "$RUN/run_facility.py" "$@"
