#!/usr/bin/env bash
set -euo pipefail
root="${CLEANY_DISPLAY_HOME:-$HOME/cleany-display}"
# This installer only owns two named user units and one desktop entry.
mkdir -p "$HOME/.config/systemd/user" "$HOME/.config/autostart"
cat > "$HOME/.config/systemd/user/cleany-display.service" <<EOF
[Unit]
Description=Cleany ROS face display
[Service]
Type=simple
WorkingDirectory=$root
Environment="CLEANY_DISPLAY_HOME=$root"
ExecStart=$root/scripts/start-backend.sh
Restart=on-failure
RestartSec=3
TimeoutStopSec=10
[Install]
WantedBy=default.target
EOF
cat > "$HOME/.config/systemd/user/cleany-display-kiosk.service" <<EOF
[Unit]
Description=Cleany fullscreen face
Wants=cleany-display.service
After=cleany-display.service
PartOf=graphical-session.target
[Service]
Type=simple
WorkingDirectory=$root
Environment="CLEANY_DISPLAY_HOME=$root"
ExecStart=$root/scripts/start-kiosk.sh
Restart=always
RestartSec=3
TimeoutStopSec=10
EOF
cat > "$HOME/.config/autostart/cleany-display.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=Cleany Display
Exec=$root/scripts/start-session.sh
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
systemctl --user daemon-reload
systemctl --user enable --now cleany-display.service
systemctl --user start cleany-display-kiosk.service
