#!/usr/bin/env bash
set -eo pipefail
root="${CLEANY_DISPLAY_HOME:-$HOME/cleany-display}"
if [[ -f "$root/runtime.env" ]]; then set -a; source "$root/runtime.env"; set +a; fi
source /opt/ros/humble/setup.bash
source "$root/ros2_ws/install/local_setup.bash"
exec python3 -m cleany_display.node --config "$root/config/display.json"
