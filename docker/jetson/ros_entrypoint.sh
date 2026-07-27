#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /workspace/cleany/ros2_ws/install/setup.bash

exec "$@"
