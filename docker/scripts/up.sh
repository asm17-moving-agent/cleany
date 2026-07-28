#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE_NAME="${CLEANY_DOCKER_IMAGE:-cleany:ros2-humble}"
CONTAINER_NAME="${CLEANY_DOCKER_CONTAINER:-cleany-humble-dev}"

if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "${CONTAINER_NAME} is already running"
  exit 0
fi

if docker ps -a --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  docker rm "${CONTAINER_NAME}" >/dev/null
fi

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "Docker image ${IMAGE_NAME} not found; building it now..." >&2
  "${SCRIPT_DIR}/build.sh"
fi

if command -v xhost >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

DOCKER_ARGS=(
  --detach
  --name "${CONTAINER_NAME}"
  --net=host
  --pid=host
  --user "$(id -u):$(id -g)"
  --env HOME=/tmp
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  --env DISPLAY="${DISPLAY:-}"
  --env QT_X11_NO_MITSHM=1
  --volume "${REPO_ROOT}:/workspace/cleany"
  --volume "${REPO_ROOT}:${REPO_ROOT}"
  --workdir "${REPO_ROOT}"
)

if [[ -d /tmp/.X11-unix ]]; then
  DOCKER_ARGS+=(--volume /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

if [[ -e /dev/dri ]]; then
  DOCKER_ARGS+=(--device /dev/dri)
fi

if grep -qi microsoft /proc/version 2>/dev/null; then
  DOCKER_ARGS+=(--env WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}")
  DOCKER_ARGS+=(--env XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}")
  if [[ -d /mnt/wslg ]]; then
    DOCKER_ARGS+=(--volume /mnt/wslg:/mnt/wslg:rw)
  fi
fi

docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}" sleep infinity
