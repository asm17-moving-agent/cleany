#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required to regenerate the dependency locks." >&2
  exit 1
fi

uv pip compile \
  "${REPO_ROOT}/requirements/jetson-jp622-arm64.in" \
  --output-file "${REPO_ROOT}/requirements/jetson-jp622-arm64.txt" \
  --python-platform aarch64-manylinux_2_35 \
  --python-version 3.10 \
  --managed-python \
  --generate-hashes \
  --only-binary :all: \
  --no-annotate \
  --custom-compile-command './scripts/compile-requirements.sh'

uv pip compile \
  "${REPO_ROOT}/requirements/ros2-humble-dev.in" \
  --output-file "${REPO_ROOT}/requirements/ros2-humble-dev.txt" \
  --python-version 3.10 \
  --managed-python \
  --universal \
  --generate-hashes \
  --only-binary :all: \
  --no-annotate \
  --custom-compile-command './scripts/compile-requirements.sh'
