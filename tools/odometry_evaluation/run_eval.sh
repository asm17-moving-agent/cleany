#!/usr/bin/env bash
# Holonomic Odometry Evaluation Runner
# Automatically enters distrobox ros2-humble if on host OS.
set -eo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/../.." && pwd)
container=${ROS2_CONTAINER:-ros2-humble}

# Check if we need to enter distrobox
in_ubuntu=false
if [[ -f /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ ${ID:-} == "ubuntu" && ${VERSION_ID:-} == "22.04" ]]; then
        in_ubuntu=true
    fi
fi

if [[ $in_ubuntu == false ]]; then
    if ! command -v distrobox >/dev/null 2>&1; then
        echo "Error: distrobox is required on non-Ubuntu 22.04 hosts." >&2
        exit 1
    fi
    exec distrobox enter --name "$container" --clean-path -- bash -c \
        "cd \"$repo_root\" && bash tools/odometry_evaluation/run_eval.sh \"\$@\"" -- "$@"
fi

# We are inside the Humble environment
export PYTHONNOUSERSITE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# Check for mock flag
domain=0
for arg in "$@"; do
    if [[ $arg == "--mock" ]]; then
        domain=173
    fi
done
export ROS_DOMAIN_ID=$domain

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "$repo_root/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$repo_root/ros2_ws/install/setup.bash"
fi

python3 "$script_dir/run_odometry_eval.py" "$@"
