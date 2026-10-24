#!/usr/bin/env bash
set -eo pipefail
unset TEB_PILOT_SIM_LIMIT
export TEB_RELAXED_FREQUENCY=10
exp=/home/donglee/workspace/cleany/controller-optimization-20261013/teb/relaxed_20261013_01
bash "$exp/run.sh" "${1:-final}" '' '{}'
