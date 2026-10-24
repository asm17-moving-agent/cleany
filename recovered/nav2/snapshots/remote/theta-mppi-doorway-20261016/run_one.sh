#!/bin/bash
set -eo pipefail
base=/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment
run=/home/donglee/workspace/cleany/theta-mppi-doorway-20261016
source /opt/ros/humble/setup.bash
source /home/donglee/workspace/cleany/controller-comparison-20261013/cleany-data/builds/source/install/local_setup.bash
source "$base/candidate_diverse/install/local_setup.bash"
source /home/donglee/workspace/cleany/controller-comparison-20261013/measurement_ws/install/local_setup.bash
export CLEANY_COMPARISON_REPO=/home/donglee/workspace/cleany/controller-comparison-20261013/source
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export MPPI_INSTALL=candidate_diverse/install MPPI_EXPERIMENT_MODE=pathseed
export MPPI_CONFIG_OVERRIDE="$base/generalization/common.yaml"
export MPPI_BT_OVERRIDE="$base/generalization/recovery_replan.xml"
unset MPPI_APPROACH_CONFIG MPPI_TEST_HOLD
exec python3 "$run/run_one.py" "$@"
