#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CLEANY_DOCKER_CONTAINER:-cleany-humble-dev}"

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  docker rm -f "${CONTAINER_NAME}" >/dev/null
  echo "removed ${CONTAINER_NAME}"
else
  echo "${CONTAINER_NAME} does not exist"
fi
