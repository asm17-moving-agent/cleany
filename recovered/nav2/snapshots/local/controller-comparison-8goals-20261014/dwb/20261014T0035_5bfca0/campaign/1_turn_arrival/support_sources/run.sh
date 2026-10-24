#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/feat-runtime-integration/install/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/install-navigation/setup.bash
source /home/ehdrms/workspace/cleany/cleany-data/builds/controller-opt-dwb/measurement_ws/install/setup.bash
export CLEANY_COMPARISON_REPO=/home/ehdrms/workspace/cleany/controller-opt-dwb
export CLEANY_COMPARISON_COMMIT=44c083abe2ebeb0077879e4419660b15fe4dd91b
export ROS_DOMAIN_ID=225 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export IGN_DISCOVERY_MSG_PORT=22551 IGN_DISCOVERY_SRV_PORT=22552
export PYTHONPATH=/home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/common:${PYTHONPATH:-}
export DWB_PATCH="$1"
shift
python3 /home/ehdrms/workspace/cleany/cleany-data/runs/controller-comparison-8goals-20261014/dwb/20261014T0035_5bfca0/run_relaxed.py --candidate dwb --domain 225 --rtf 1 "$@"
