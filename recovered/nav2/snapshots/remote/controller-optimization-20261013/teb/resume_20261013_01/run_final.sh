#!/usr/bin/env bash
set -eo pipefail
exp=/home/donglee/workspace/cleany/controller-optimization-20261013/teb/resume_20261013_01
bash "$exp/run.sh" "${1:-final}" '' '{"min_obstacle_dist":0.05}'
