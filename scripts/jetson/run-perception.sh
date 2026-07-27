#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"
source "${SCRIPT_DIR}/artifact-manifest.sh"

require_jetpack_622

ENGINE_PATH="${1:-}"
IMAGE_TOPIC="${CLEANY_IMAGE_TOPIC:-/image_raw}"
ROS_LOCALHOST_ONLY="${CLEANY_ROS_LOCALHOST_ONLY:-1}"

if [[ ! -f "${ENGINE_PATH}" || "${ENGINE_PATH}" != *.engine ]]; then
  echo "Usage: $0 /absolute/path/to/model.engine" >&2
  exit 2
fi

ENGINE_DIR="$(cd "$(dirname "${ENGINE_PATH}")" && pwd)"
ENGINE_NAME="$(basename "${ENGINE_PATH}")"
MANIFEST_PATH="${ENGINE_PATH}.manifest"

if [[ "${ROS_LOCALHOST_ONLY}" != "0" && "${ROS_LOCALHOST_ONLY}" != "1" ]]; then
  echo "CLEANY_ROS_LOCALHOST_ONLY must be 0 or 1." >&2
  exit 2
fi

load_container_identity
export CLEANY_MODEL_SOURCE
CLEANY_MODEL_SOURCE="$(manifest_value model_source "${MANIFEST_PATH}")"
export CLEANY_MODEL_LICENSE
CLEANY_MODEL_LICENSE="$(manifest_value model_license "${MANIFEST_PATH}")"
export CLEANY_ENGINE_PRECISION
CLEANY_ENGINE_PRECISION="$(manifest_value precision "${MANIFEST_PATH}")"
export CLEANY_ENGINE_IMAGE_SIZE
CLEANY_ENGINE_IMAGE_SIZE="$(manifest_value image_size "${MANIFEST_PATH}")"
verify_artifact_manifest "${ENGINE_PATH}" "${MANIFEST_PATH}"

docker run --rm \
  --runtime nvidia \
  --network host \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --pids-limit 512 \
  --user "$(id -u):$(id -g)" \
  --env HOME=/tmp \
  --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  --env ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=256m \
  --volume "${ENGINE_DIR}:/models:ro" \
  "${JETSON_IMAGE}" \
  ros2 run cleany_perception detection_node \
    --ros-args \
    --params-file /workspace/cleany/ros2_ws/install/share/cleany_perception/config/detection.jetson.yaml \
    -p "weights:=/models/${ENGINE_NAME}" \
    -p "image_topic:=${IMAGE_TOPIC}"
