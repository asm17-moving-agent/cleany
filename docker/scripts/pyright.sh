#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER_NAME="${CLEANY_DOCKER_CONTAINER:-cleany-humble-dev}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  "${SCRIPT_DIR}/up.sh" >/dev/null
fi

exec docker exec \
  -i \
  --workdir "${REPO_ROOT}/ros2_ws" \
  "${CONTAINER_NAME}" \
  bash -lc '
    source /opt/ros/humble/setup.bash
    if [[ -f /workspace/cleany/ros2_ws/install/setup.bash ]]; then
      source /workspace/cleany/ros2_ws/install/setup.bash
    fi
    exec pyright-langserver --stdio
  '
