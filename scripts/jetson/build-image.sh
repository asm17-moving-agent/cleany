#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
source "${SCRIPT_DIR}/common.sh"

require_jetpack_622

docker build \
  --platform linux/arm64 \
  --file "${REPO_ROOT}/docker/jetson/Dockerfile" \
  --tag "${JETSON_IMAGE}" \
  "${REPO_ROOT}"

load_container_identity
printf 'Built %s (%s)\n' "${CLEANY_IMAGE_REF}" "${CLEANY_IMAGE_ID}"
printf 'Validated TensorRT=%s CUDA=%s PyTorch=%s Ultralytics=%s\n' \
  "${CLEANY_TENSORRT_VERSION}" \
  "${CLEANY_CUDA_VERSION}" \
  "${CLEANY_TORCH_VERSION}" \
  "${CLEANY_ULTRALYTICS_VERSION}"
