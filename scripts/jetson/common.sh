#!/usr/bin/env bash
set -euo pipefail

JETSON_IMAGE="${CLEANY_JETSON_IMAGE:-cleany:orin-nx-jp6.2.2}"
EXPECTED_TENSORRT_PREFIX='10.3.'
EXPECTED_CUDA_PREFIX='12.6'
EXPECTED_TORCH_PREFIX='2.10.0'
EXPECTED_ULTRALYTICS_VERSION='8.4.107'

load_host_identity() {
  export CLEANY_DEVICE_MODEL
  CLEANY_DEVICE_MODEL="$(tr -d '\0' < /proc/device-tree/model)"
  export CLEANY_L4T_RELEASE
  CLEANY_L4T_RELEASE="$(
    sed -n 's/^# R\([0-9][0-9]*\) (release), REVISION: \([0-9][0-9.]*\).*/\1.\2/p' \
      /etc/nv_tegra_release
  )"
  if [[ -z "${CLEANY_L4T_RELEASE}" ]]; then
    echo "Could not parse the Jetson Linux release." >&2
    return 1
  fi
}

load_container_identity() {
  export CLEANY_IMAGE_REF="${JETSON_IMAGE}"
  export CLEANY_IMAGE_ID
  CLEANY_IMAGE_ID="$(docker image inspect --format '{{.Id}}' "${JETSON_IMAGE}")"

  local stack
  stack="$(
    docker run --rm \
      --runtime nvidia \
      --network none \
      --cap-drop ALL \
      --security-opt no-new-privileges \
      --read-only \
      --tmpfs /tmp:rw,nosuid,nodev,size=64m \
      "${JETSON_IMAGE}" \
      python3 -c \
      'import tensorrt, torch, ultralytics; print("|".join((tensorrt.__version__, str(torch.version.cuda), torch.__version__, ultralytics.__version__, str(torch.cuda.is_available()).lower())))' \
      | tail -n 1
  )"

  local cuda_available
  IFS='|' read -r \
    CLEANY_TENSORRT_VERSION \
    CLEANY_CUDA_VERSION \
    CLEANY_TORCH_VERSION \
    CLEANY_ULTRALYTICS_VERSION \
    cuda_available <<< "${stack}"
  export CLEANY_TENSORRT_VERSION
  export CLEANY_CUDA_VERSION
  export CLEANY_TORCH_VERSION
  export CLEANY_ULTRALYTICS_VERSION

  [[ "${CLEANY_TENSORRT_VERSION}" == "${EXPECTED_TENSORRT_PREFIX}"* ]] || {
    echo "TensorRT ${EXPECTED_TENSORRT_PREFIX}x is required; found ${CLEANY_TENSORRT_VERSION:-unknown}." >&2
    return 1
  }
  [[ "${CLEANY_CUDA_VERSION}" == "${EXPECTED_CUDA_PREFIX}"* ]] || {
    echo "CUDA ${EXPECTED_CUDA_PREFIX}x is required; found ${CLEANY_CUDA_VERSION:-unknown}." >&2
    return 1
  }
  [[ "${CLEANY_TORCH_VERSION}" == "${EXPECTED_TORCH_PREFIX}"* ]] || {
    echo "PyTorch ${EXPECTED_TORCH_PREFIX}x is required; found ${CLEANY_TORCH_VERSION:-unknown}." >&2
    return 1
  }
  [[ "${CLEANY_ULTRALYTICS_VERSION}" == "${EXPECTED_ULTRALYTICS_VERSION}" ]] || {
    echo "Ultralytics ${EXPECTED_ULTRALYTICS_VERSION} is required; found ${CLEANY_ULTRALYTICS_VERSION:-unknown}." >&2
    return 1
  }
  [[ "${cuda_available}" == "true" ]] || {
    echo "PyTorch cannot access the Jetson CUDA device." >&2
    return 1
  }
}

require_jetpack_622() {
  if [[ "$(uname -m)" != "aarch64" ]]; then
    echo "Jetson Orin NX aarch64 host is required." >&2
    return 1
  fi

  if [[ ! -r /proc/device-tree/model ]]; then
    echo "NVIDIA Jetson Orin NX hardware is required." >&2
    return 1
  fi

  local device_model
  device_model="$(tr -d '\0' < /proc/device-tree/model)"
  if [[ "${device_model}" != *"Orin NX"* ]]; then
    echo "NVIDIA Jetson Orin NX hardware is required." >&2
    return 1
  fi

  if [[ ! -r /etc/os-release ]]; then
    echo "/etc/os-release is required." >&2
    return 1
  fi

  source /etc/os-release
  if [[ "${VERSION_ID:-}" != "22.04" ]]; then
    echo "Ubuntu 22.04 is required; found ${VERSION_ID:-unknown}." >&2
    return 1
  fi

  if [[ ! -r /etc/nv_tegra_release ]] \
    || ! grep -Eq '^# R36 \(release\), REVISION: 5\.' /etc/nv_tegra_release; then
    echo "JetPack 6.2.2 / Jetson Linux 36.5 is required." >&2
    return 1
  fi

  if ! docker info >/dev/null 2>&1; then
    echo "Docker and the NVIDIA Container Runtime must be configured." >&2
    return 1
  fi

  if ! docker info --format '{{json .Runtimes}}' | grep -q '"nvidia"'; then
    echo "The Docker NVIDIA runtime is not registered." >&2
    return 1
  fi

  load_host_identity
}
