#!/bin/bash
set -eo pipefail
exp=/home/donglee/workspace/cleany/doorway-diagnosis-20261014/desktop-experiment
source /opt/ros/humble/setup.bash
cd "$exp/${1:-candidate_v2}"
colcon --log-base log build --base-paths src --build-base build --install-base install --packages-select nav2_mppi_controller --allow-overriding nav2_mppi_controller --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
