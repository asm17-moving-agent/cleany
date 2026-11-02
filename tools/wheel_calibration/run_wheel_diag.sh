#!/usr/bin/env bash
# Runner for wheel tracking diagnosis
set -eo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/../.." && pwd)
container=${ROS2_CONTAINER:-ros2-humble}

in_ubuntu=false
if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ ${ID:-} == "ubuntu" && ${VERSION_ID:-} == "22.04" ]]; then
        in_ubuntu=true
    fi
fi

if [[ $in_ubuntu == false ]]; then
    exec distrobox enter --name "$container" --clean-path -- bash -c \
        "cd \"$repo_root\" && bash tools/wheel_calibration/run_wheel_diag.sh \"\$@\"" -- "$@"
fi

export PYTHONNOUSERSITE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_DOMAIN_ID=0

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$repo_root/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$repo_root/ros2_ws/install/setup.bash"
fi

python3 "$script_dir/diagnose_wheel_tracking.py" "$@"
