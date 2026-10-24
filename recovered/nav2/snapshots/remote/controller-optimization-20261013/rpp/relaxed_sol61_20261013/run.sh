#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /home/donglee/workspace/cleany/controller-optimization-20261013/rpp-build-v2/install/setup.bash
source /home/donglee/workspace/cleany/controller-comparison-20261013/measurement_ws/install/setup.bash
exec python3 /home/donglee/workspace/cleany/controller-optimization-20261013/rpp/relaxed_sol61_20261013/runner.py "$@"
