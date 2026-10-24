#!/bin/bash
set -eo pipefail
exp=/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment
source /opt/ros/humble/setup.bash
source /home/donglee/workspace/cleany/controller-comparison-20261013/cleany-data/builds/source/install/local_setup.bash
source "$exp/${MPPI_INSTALL:-install}/local_setup.bash"
source /home/donglee/workspace/cleany/controller-comparison-20261013/measurement_ws/install/local_setup.bash
export CLEANY_COMPARISON_REPO=/home/donglee/workspace/cleany/controller-comparison-20261013/source
export ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
exec python3 "$exp/run_experiment.py" "$@"
