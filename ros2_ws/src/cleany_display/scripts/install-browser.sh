#!/usr/bin/env bash
set -euo pipefail
root="${CLEANY_DISPLAY_HOME:-$HOME/cleany-display}"
python3 -m venv "$root/browser-tools"
"$root/browser-tools/bin/pip" install --no-cache-dir 'playwright==1.64.1'
export PLAYWRIGHT_BROWSERS_PATH="$root/browsers"
"$root/browser-tools/bin/python" -m playwright install chromium --no-shell
"$root/browser-tools/bin/python" - <<'PY' > "$root/browser.path"
from playwright.sync_api import sync_playwright
with sync_playwright() as playwright:
    print(playwright.chromium.executable_path)
PY
