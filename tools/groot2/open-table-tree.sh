#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${script_dir}/release.sh"
repo_dir="$(cd -- "${script_dir}/../.." && pwd)"
app_run="${groot2_extracted_dir}/AppRun"
tree_xml="${repo_dir}/ros2_ws/src/cleany_skill_executor/docs/groot2_table_cleanup.xml"

if [[ ! -x $app_run ]]; then
  printf 'Groot2 is not installed. Run tools/groot2/install-local.sh with the %s path.\n' "$groot2_appimage_name" >&2
  exit 1
fi

# Groot2 bundles only the Qt xcb platform plugin, so Wayland needs XWayland.
if [[ -z ${DISPLAY:-} ]]; then
  printf 'Groot2 needs an X11 display (DISPLAY). On Wayland, enable XWayland.\n' >&2
  exit 1
fi

printf 'In Groot2, choose "Load Project or File..." and open:\n%s\n' "$tree_xml"
exec "$app_run" "$@"
