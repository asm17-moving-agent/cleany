#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="${CLEANY_DOCKER_CONTAINER:-cleany-humble-dev}"

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  docker rm -f "${CONTAINER_NAME}" >/dev/null
fi

"${SCRIPT_DIR}/build.sh"
"${SCRIPT_DIR}/up.sh"
