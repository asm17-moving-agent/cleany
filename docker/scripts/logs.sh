#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CLEANY_DOCKER_CONTAINER:-cleany-humble-dev}"
exec docker logs "$@" "${CONTAINER_NAME}"
