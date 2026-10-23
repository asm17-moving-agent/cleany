#!/usr/bin/env bash
# Config-driven wheel odometry evaluation with managed hardware-node startup.
set -eo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/../.." && pwd)
container=${ROS2_CONTAINER:-ros2-humble}

in_ubuntu=false
if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 22.04 ]]; then
        in_ubuntu=true
    fi
fi

if [[ $in_ubuntu == false ]]; then
    if ! command -v distrobox >/dev/null 2>&1; then
        echo "Error: distrobox is required on non-Ubuntu 22.04 hosts." >&2
        exit 1
    fi
    exec distrobox enter --name "$container" --clean-path -- \
        bash "$script_dir/run_wheel_only_eval.sh" "$@"
fi

cd "$repo_root"
export PYTHONNOUSERSITE=1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$repo_root/esp32/micro_ros/agent/install/local_setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$repo_root/esp32/micro_ros/agent/install/local_setup.bash"
fi
if [[ -f "$repo_root/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$repo_root/ros2_ws/install/setup.bash"
fi
exec /usr/bin/python3 "$script_dir/run_wheel_only_eval.py" "$@"
