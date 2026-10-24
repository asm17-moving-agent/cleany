#!/bin/bash
set -e
ROOT=/home/ehdrms/workspace/cleany/cleany-data/runs/controller-optimization-20261013/mppi
RUN="$ROOT/sol_resume_20261013/relaxed"
source /opt/ros/humble/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/mppi-optimization/install/local_setup.bash
source "$RUN/mppi_ws/install/local_setup.bash"
source "$ROOT/measurement_ws/install/local_setup.bash"
export CLEANY_COMPARISON_REPO=/home/ehdrms/.codex/worktrees/mppi-optimization/main
export CLEANY_BENCHMARK_TIMING=1
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
exec /usr/bin/python3 "$RUN/run_relaxed.py" --candidate mppi --domain 226 --rtf 1 "$@"
