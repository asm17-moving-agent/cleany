#!/usr/bin/env bash
# 전방향 odometry 평가 실행기입니다.
# 호스트 OS에서는 distrobox ros2-humble에 자동 진입합니다.
set -eo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$script_dir/../.." && pwd)
container=${ROS2_CONTAINER:-ros2-humble}

# distrobox 진입 필요 여부를 확인합니다.
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

# Humble 환경 안에서 실행합니다.
export PYTHONNOUSERSITE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# mock 옵션을 확인합니다.
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
