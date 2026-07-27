#!/usr/bin/env bash
set -euo pipefail

required_artifact_environment() {
  local key
  for key in \
    CLEANY_IMAGE_REF \
    CLEANY_IMAGE_ID \
    CLEANY_DEVICE_MODEL \
    CLEANY_L4T_RELEASE \
    CLEANY_TENSORRT_VERSION \
    CLEANY_CUDA_VERSION \
    CLEANY_TORCH_VERSION \
    CLEANY_ULTRALYTICS_VERSION \
    CLEANY_MODEL_SOURCE \
    CLEANY_MODEL_LICENSE \
    CLEANY_ENGINE_PRECISION \
    CLEANY_ENGINE_IMAGE_SIZE; do
    if [[ -z "${!key:-}" || "${!key}" == *$'\n'* ]]; then
      echo "${key} must be a non-empty single-line value." >&2
      return 1
    fi
  done
}

sha256_file() {
  local path="$1"
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "${path}" | awk '{print $1}'
  else
    shasum -a 256 "${path}" | awk '{print $1}'
  fi
}

manifest_value() {
  local key="$1"
  local manifest_path="$2"
  local count
  count="$(awk -F= -v key="${key}" '$1 == key {count++} END {print count + 0}' "${manifest_path}")"
  if [[ "${count}" != "1" ]]; then
    echo "Manifest key '${key}' must appear exactly once." >&2
    return 1
  fi
  awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print}' "${manifest_path}"
}

verify_model_provenance() {
  local model_path="$1"
  local provenance_path="$2"

  [[ -f "${model_path}" ]] || {
    echo "Model does not exist: ${model_path}" >&2
    return 1
  }
  [[ -f "${provenance_path}" ]] || {
    echo "Model provenance does not exist: ${provenance_path}" >&2
    return 1
  }
  [[ "$(manifest_value schema_version "${provenance_path}")" == "1" ]] || {
    echo "Unsupported model provenance schema." >&2
    return 1
  }
  [[ "$(manifest_value model_file "${provenance_path}")" == "$(basename "${model_path}")" ]] || {
    echo "Model filename does not match its provenance." >&2
    return 1
  }
  [[ "$(manifest_value model_sha256 "${provenance_path}")" == "$(sha256_file "${model_path}")" ]] || {
    echo "Model SHA-256 does not match its provenance." >&2
    return 1
  }

  export CLEANY_MODEL_SOURCE
  CLEANY_MODEL_SOURCE="$(manifest_value source "${provenance_path}")"
  export CLEANY_MODEL_LICENSE
  CLEANY_MODEL_LICENSE="$(manifest_value license "${provenance_path}")"
  [[ -n "${CLEANY_MODEL_SOURCE}" && -n "${CLEANY_MODEL_LICENSE}" ]] || {
    echo "Model source and license must be recorded." >&2
    return 1
  }
}

write_artifact_manifest() {
  local model_path="$1"
  local engine_path="$2"
  local manifest_path="$3"
  local temporary_path="${manifest_path}.tmp.$$"

  required_artifact_environment
  [[ -f "${model_path}" ]] || {
    echo "Model does not exist: ${model_path}" >&2
    return 1
  }
  [[ -f "${engine_path}" ]] || {
    echo "TensorRT engine does not exist: ${engine_path}" >&2
    return 1
  }

  umask 077
  {
    printf 'schema_version=1\n'
    printf 'model_file=%s\n' "$(basename "${model_path}")"
    printf 'model_sha256=%s\n' "$(sha256_file "${model_path}")"
    printf 'model_source=%s\n' "${CLEANY_MODEL_SOURCE}"
    printf 'model_license=%s\n' "${CLEANY_MODEL_LICENSE}"
    printf 'engine_file=%s\n' "$(basename "${engine_path}")"
    printf 'engine_sha256=%s\n' "$(sha256_file "${engine_path}")"
    printf 'image_ref=%s\n' "${CLEANY_IMAGE_REF}"
    printf 'image_id=%s\n' "${CLEANY_IMAGE_ID}"
    printf 'device_model=%s\n' "${CLEANY_DEVICE_MODEL}"
    printf 'l4t_release=%s\n' "${CLEANY_L4T_RELEASE}"
    printf 'tensorrt_version=%s\n' "${CLEANY_TENSORRT_VERSION}"
    printf 'cuda_version=%s\n' "${CLEANY_CUDA_VERSION}"
    printf 'torch_version=%s\n' "${CLEANY_TORCH_VERSION}"
    printf 'ultralytics_version=%s\n' "${CLEANY_ULTRALYTICS_VERSION}"
    printf 'precision=%s\n' "${CLEANY_ENGINE_PRECISION}"
    printf 'image_size=%s\n' "${CLEANY_ENGINE_IMAGE_SIZE}"
  } > "${temporary_path}"
  mv "${temporary_path}" "${manifest_path}"
}

verify_artifact_manifest() {
  local engine_path="$1"
  local manifest_path="$2"

  required_artifact_environment
  [[ -f "${engine_path}" ]] || {
    echo "TensorRT engine does not exist: ${engine_path}" >&2
    return 1
  }
  [[ -f "${manifest_path}" ]] || {
    echo "Artifact manifest does not exist: ${manifest_path}" >&2
    return 1
  }

  local key expected
  [[ -n "$(manifest_value model_source "${manifest_path}")" ]] || return 1
  [[ -n "$(manifest_value model_license "${manifest_path}")" ]] || return 1
  while IFS='|' read -r key expected; do
    if [[ "$(manifest_value "${key}" "${manifest_path}")" != "${expected}" ]]; then
      echo "Artifact manifest mismatch: ${key}" >&2
      return 1
    fi
  done <<EOF
schema_version|1
engine_file|$(basename "${engine_path}")
engine_sha256|$(sha256_file "${engine_path}")
image_ref|${CLEANY_IMAGE_REF}
image_id|${CLEANY_IMAGE_ID}
device_model|${CLEANY_DEVICE_MODEL}
l4t_release|${CLEANY_L4T_RELEASE}
tensorrt_version|${CLEANY_TENSORRT_VERSION}
cuda_version|${CLEANY_CUDA_VERSION}
torch_version|${CLEANY_TORCH_VERSION}
ultralytics_version|${CLEANY_ULTRALYTICS_VERSION}
precision|${CLEANY_ENGINE_PRECISION}
image_size|${CLEANY_ENGINE_IMAGE_SIZE}
EOF
}
