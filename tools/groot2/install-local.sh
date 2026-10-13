#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/release.sh"

if [[ $# -ne 1 || ! -f $1 ]]; then
  printf 'Usage: %s /path/to/%s\n' "$0" "$groot2_appimage_name" >&2
  exit 2
fi

if [[ $(uname -m) != "$groot2_arch" ]]; then
  printf 'This installer is for the %s VM. Found: %s\n' "$groot2_arch" "$(uname -m)" >&2
  exit 2
fi

source_appimage="$1"
actual_sha256="$(sha256sum -- "$source_appimage" | cut -d ' ' -f 1)"
if [[ $actual_sha256 != "$groot2_appimage_sha256" ]]; then
  printf 'SHA-256 mismatch for %s\n  expected: %s\n  actual:   %s\n' \
    "$source_appimage" "$groot2_appimage_sha256" "$actual_sha256" >&2
  printf 'Download %s from https://www.behaviortree.dev/groot/ again.\n' "$groot2_appimage_name" >&2
  exit 2
fi

mkdir -p -- "$groot2_install_dir"
if [[ -d $groot2_extracted_dir ]]; then
  printf 'Already installed: %s\n' "$groot2_extracted_dir"
  exit 0
fi

working_dir="$(mktemp -d "${groot2_install_dir}/.extract.XXXXXX")"
trap 'rm -rf -- "$working_dir"' EXIT
install -m 755 -- "$source_appimage" "${working_dir}/${groot2_appimage_name}"

# This VM has no libfuse2 for mounting AppImages, so extract once and run AppRun later.
(
  cd -- "$working_dir"
  "./${groot2_appimage_name}" --appimage-extract >/dev/null
)

if [[ ! -x "${working_dir}/squashfs-root/AppRun" ]]; then
  printf 'AppImage extraction did not produce an executable AppRun.\n' >&2
  exit 1
fi

mv -- "${working_dir}/squashfs-root" "$groot2_extracted_dir"
printf 'Installed Groot2 %s at %s\n' "$groot2_version" "$groot2_extracted_dir"
printf 'Run tools/groot2/open-table-tree.sh from a desktop session.\n'
