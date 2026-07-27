#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTAINER_NAME="${CLEANY_DOCKER_CONTAINER:-cleany-humble-dev}"

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <command> [args...]" >&2
  exit 2
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  "${SCRIPT_DIR}/docker-up-humble.sh" >/dev/null
fi

WORKDIR="${PWD}"
case "${WORKDIR}" in
  "${REPO_ROOT}"|"${REPO_ROOT}"/*) ;;
  *) WORKDIR="${REPO_ROOT}" ;;
esac

DOCKER_ARGS=(exec)
if [[ -t 0 && -t 1 ]]; then
  DOCKER_ARGS+=(-it)
fi
DOCKER_ARGS+=(
  --env IGN_PARTITION="${IGN_PARTITION:-cleany}"
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  --env ROS_HOME=/tmp/cleany-ros
  --workdir "${WORKDIR}"
  "${CONTAINER_NAME}"
  bash -lc 'source /opt/ros/humble/setup.bash; if [[ -f /workspace/cleany/ros2_ws/install/setup.bash ]]; then source /workspace/cleany/ros2_ws/install/setup.bash; fi; exec "$@"' bash
)

docker "${DOCKER_ARGS[@]}" "$@"
