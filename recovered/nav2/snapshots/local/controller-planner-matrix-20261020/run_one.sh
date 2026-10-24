#!/bin/bash
set -eo pipefail
base=/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment
run=/home/donglee/workspace/cleany/cleany-data/runs/controller-planner-matrix-20261020
source /opt/ros/humble/setup.bash
source /home/donglee/workspace/cleany/controller-comparison-20261013/cleany-data/builds/source/install/local_setup.bash
source "$base/candidate_diverse/install/local_setup.bash"
source /home/donglee/workspace/cleany/controller-comparison-20261013/measurement_ws/install/local_setup.bash
export DWB_INSTALL="$run/builds/dwb/install-navigation"
# This commit differs from the prior baseline only in DWB generator code/docs/tests.
source "$DWB_INSTALL/local_setup.bash"
# Load the exact tuned DWB C++ library built for x86.
export LD_LIBRARY_PATH="$DWB_INSTALL/cleany_navigation/lib:$LD_LIBRARY_PATH"
export CLEANY_COMPARISON_REPO=/home/donglee/workspace/cleany/controller-comparison-20261013/source
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export MPPI_INSTALL=candidate_diverse/install MPPI_EXPERIMENT_MODE=pathseed
export MPPI_CONFIG_OVERRIDE="$base/generalization/common.yaml"
export MPPI_BT_OVERRIDE="$base/generalization/recovery_replan.xml"
unset MPPI_APPROACH_CONFIG MPPI_TEST_HOLD
exec python3 "$run/run_one.py" "$@"
