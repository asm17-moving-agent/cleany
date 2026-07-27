#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/artifact-manifest.sh"

TEST_DIR="$(mktemp -d)"
trap 'find "${TEST_DIR}" -depth -delete' EXIT

MODEL_PATH="${TEST_DIR}/model.pt"
ENGINE_PATH="${TEST_DIR}/model.engine"
MANIFEST_PATH="${ENGINE_PATH}.manifest"
PROVENANCE_PATH="${MODEL_PATH}.provenance"

printf 'model-v1' > "${MODEL_PATH}"
printf 'engine-v1' > "${ENGINE_PATH}"
cat > "${PROVENANCE_PATH}" <<EOF
schema_version=1
model_file=model.pt
model_sha256=$(sha256_file "${MODEL_PATH}")
source=https://example.invalid/model.pt
license=reviewed-test-license
EOF

verify_model_provenance "${MODEL_PATH}" "${PROVENANCE_PATH}"
printf 'tampered' >> "${MODEL_PATH}"
if verify_model_provenance "${MODEL_PATH}" "${PROVENANCE_PATH}" >/dev/null 2>&1; then
  echo "Tampered model unexpectedly passed provenance verification." >&2
  exit 1
fi
printf 'model-v1' > "${MODEL_PATH}"

export CLEANY_IMAGE_REF='cleany:orin-nx-jp6.2.2'
export CLEANY_IMAGE_ID='sha256:image'
export CLEANY_DEVICE_MODEL='NVIDIA Jetson Orin NX'
export CLEANY_L4T_RELEASE='36.5.0'
export CLEANY_TENSORRT_VERSION='10.3.0'
export CLEANY_CUDA_VERSION='12.6'
export CLEANY_TORCH_VERSION='2.10.0'
export CLEANY_ULTRALYTICS_VERSION='8.4.107'
export CLEANY_ENGINE_PRECISION='fp16'
export CLEANY_ENGINE_IMAGE_SIZE='640'

write_artifact_manifest "${MODEL_PATH}" "${ENGINE_PATH}" "${MANIFEST_PATH}"
verify_artifact_manifest "${ENGINE_PATH}" "${MANIFEST_PATH}"

printf 'tampered' >> "${ENGINE_PATH}"
if verify_artifact_manifest "${ENGINE_PATH}" "${MANIFEST_PATH}" >/dev/null 2>&1; then
  echo "Tampered engine unexpectedly passed manifest verification." >&2
  exit 1
fi

printf 'artifact manifest tests passed\n'
