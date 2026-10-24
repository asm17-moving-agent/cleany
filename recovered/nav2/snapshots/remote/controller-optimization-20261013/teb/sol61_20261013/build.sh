#!/usr/bin/env bash
set -eo pipefail
exp=/home/donglee/workspace/cleany/controller-optimization-20261013/teb/sol61_20261013
source /opt/ros/humble/setup.bash
source /home/donglee/workspace/cleany/controller-comparison-20261013/cleany-data/builds/source/install/setup.bash
cd "$exp"
colcon build --packages-up-to teb_local_planner cleany_benchmark --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release
