#!/usr/bin/env bash
set -eo pipefail
exp=/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01
overlay=/home/donglee/workspace/cleany/controller-optimization-20261013/teb/sol61_20261013
source /opt/ros/humble/setup.bash
source /home/donglee/workspace/cleany/controller-comparison-20261013/cleany-data/builds/source/install/setup.bash
source "$overlay/install/setup.bash"
export CLEANY_COMPARISON_REPO=/home/donglee/workspace/cleany/controller-optimization-20261013/teb/source CLEANY_BENCHMARK_TIMING=1
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1 IGN_IP=127.0.0.1 GZ_IP=127.0.0.1
export CLEANY_TEB_OVERRIDES="${3:-}"
if [[ -z "$CLEANY_TEB_OVERRIDES" ]]; then export CLEANY_TEB_OVERRIDES='{}'; fi
python3 "$exp/runner.py" --output "$exp/$1" ${2:+--goal-index "$2"}
