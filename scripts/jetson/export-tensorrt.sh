#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"
source "${SCRIPT_DIR}/artifact-manifest.sh"

require_jetpack_622

MODEL_PATH="${1:-}"
IMAGE_SIZE="${CLEANY_TRT_IMAGE_SIZE:-640}"
WORKSPACE_GIB="${CLEANY_TRT_WORKSPACE_GIB:-2}"

if [[ ! -f "${MODEL_PATH}" || "${MODEL_PATH}" != *.pt ]]; then
  echo "Usage: $0 /absolute/path/to/model.pt" >&2
  exit 2
fi

if [[ ! "${IMAGE_SIZE}" =~ ^[1-9][0-9]*$ ]] \
  || [[ ! "${WORKSPACE_GIB}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Image size and workspace GiB must be positive integers." >&2
  exit 2
fi

MODEL_DIR="$(cd "$(dirname "${MODEL_PATH}")" && pwd)"
MODEL_NAME="$(basename "${MODEL_PATH}")"
ENGINE_NAME="${MODEL_NAME%.pt}.engine"
ENGINE_PATH="${MODEL_DIR}/${ENGINE_NAME}"
MANIFEST_PATH="${ENGINE_PATH}.manifest"
PROVENANCE_PATH="${MODEL_PATH}.provenance"
EXPORT_DIR="$(mktemp -d "${MODEL_DIR}/.cleany-export.XXXXXX")"
trap 'find "${EXPORT_DIR}" -depth -delete' EXIT

verify_model_provenance "${MODEL_PATH}" "${PROVENANCE_PATH}"
cp "${MODEL_PATH}" "${EXPORT_DIR}/${MODEL_NAME}"
load_container_identity

docker run --rm \
  --runtime nvidia \
  --network none \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --pids-limit 512 \
  --user "$(id -u):$(id -g)" \
  --env HOME=/tmp \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --volume "${EXPORT_DIR}:/models" \
  --workdir /models \
  "${JETSON_IMAGE}" \
  yolo export \
    model="/models/${MODEL_NAME}" \
    format=engine \
    imgsz="${IMAGE_SIZE}" \
    batch=1 \
    half=True \
    dynamic=False \
    device=0 \
    workspace="${WORKSPACE_GIB}"

if [[ ! -f "${EXPORT_DIR}/${ENGINE_NAME}" ]]; then
  echo "Ultralytics did not create the expected engine: ${ENGINE_NAME}" >&2
  exit 1
fi

docker run --rm \
  --runtime nvidia \
  --network none \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --pids-limit 512 \
  --user "$(id -u):$(id -g)" \
  --env HOME=/tmp \
  --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --volume "${EXPORT_DIR}:/models:ro" \
  "${JETSON_IMAGE}" \
  python3 -c \
  'import sys; import numpy as np; from ultralytics import YOLO; YOLO(sys.argv[1]).predict(source=np.zeros((int(sys.argv[2]), int(sys.argv[2]), 3), dtype=np.uint8), device=0, verbose=False)' \
  "/models/${ENGINE_NAME}" \
  "${IMAGE_SIZE}"

mv "${EXPORT_DIR}/${ENGINE_NAME}" "${ENGINE_PATH}"
export CLEANY_ENGINE_PRECISION='fp16'
export CLEANY_ENGINE_IMAGE_SIZE="${IMAGE_SIZE}"
write_artifact_manifest "${MODEL_PATH}" "${ENGINE_PATH}" "${MANIFEST_PATH}"
printf 'Created %s\nManifest %s\n' "${ENGINE_PATH}" "${MANIFEST_PATH}"
