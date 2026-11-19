#!/usr/bin/env bash
set -eo pipefail
root="${CLEANY_DISPLAY_HOME:-$HOME/cleany-display}"
if [[ -f "$root/runtime.env" ]]; then set -a; source "$root/runtime.env"; set +a; fi
url="${CLEANY_DISPLAY_URL:-http://127.0.0.1:8770}"
until curl --silent --fail --max-time 2 "$url/healthz" >/dev/null; do sleep 1; done
if [[ -n "${CLEANY_DISPLAY_BROWSER:-}" ]]; then
  browser="$CLEANY_DISPLAY_BROWSER"
elif [[ -f "$root/browser.path" ]]; then
  browser="$(cat "$root/browser.path")"
else
  browser="$(command -v chromium || command -v chromium-browser)"
fi
export CHROME_CONFIG_HOME="$root/browser-config"
export XDG_CACHE_HOME="$root/browser-cache"
# Yield CPU time while the NVIDIA OpenGL driver waits for graphics work.
export __GL_YIELD="${__GL_YIELD:-USLEEP}"
args=(--kiosk --password-store=basic --disable-infobars --lang=ko --disable-extensions --disable-background-networking --disable-sync --disable-component-update --no-first-run --no-default-browser-check --noerrdialogs --disable-session-crashed-bubble
      --user-data-dir="$root/browser-profile")
if [[ -n "${CLEANY_DISPLAY_DEBUG_PORT:-}" ]]; then
  args+=(--remote-debugging-address=127.0.0.1 --remote-debugging-port="$CLEANY_DISPLAY_DEBUG_PORT")
fi
exec gnome-session-inhibit --app-id org.cleany.Display --reason 'Cleany robot face' --inhibit idle \
  "$browser" "${args[@]}" "$url/"
