#!/usr/bin/env bash
set -eo pipefail
for key in DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_SESSION_TYPE DBUS_SESSION_BUS_ADDRESS; do
  if [[ -n "${!key:-}" ]]; then systemctl --user import-environment "$key"; fi
done
systemctl --user start cleany-display.service cleany-display-kiosk.service
