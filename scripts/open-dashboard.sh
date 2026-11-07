#!/usr/bin/env bash
# Open the UltraSkelly dashboard full-screen on the Pi's monitor.
#
# Waits until the control panel answers and the desktop is up, then opens the
# dashboard as a full-screen Chromium app window on the desktop session's Wayland
# display. The control panel (ultraskelly-control.service) runs all the time, so the
# window opens while the bot is off too, with the buttons that turn it on. Works from
# the desktop or over SSH; run it in the background (or from a service, as
# ultraskelly-dashboard.service does) to keep it open after logout.
#
#   scripts/open-dashboard.sh [URL]        default: http://localhost/, the control panel
#   scripts/open-dashboard.sh http://localhost:8080/   the bot's own dashboard, without the controls
#   DASHBOARD_WAIT_SECONDS=300 scripts/open-dashboard.sh   (0 waits as long as it takes)
#
# On the Pi's keyboard: F12 opens DevTools, F11 leaves full screen, Alt+F4 closes.
# Not --kiosk: kiosk mode disables DevTools and blames an organisation policy.
set -euo pipefail

url="${1:-http://localhost/}"
wait_seconds="${DASHBOARD_WAIT_SECONDS:-120}"
profile="$HOME/.config/ultraskelly-dashboard"

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

deadline=$((SECONDS + wait_seconds))
until [[ -S $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY ]] && curl -fs --max-time 5 -o /dev/null "$url"; do
  if ((wait_seconds > 0 && SECONDS >= deadline)); then
    if [[ ! -S $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY ]]; then
      echo "No desktop session at $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY after ${wait_seconds}s" >&2
    else
      echo "Nothing answers at $url after ${wait_seconds}s; is ultraskelly-control running (or, for :8080, the bot)?" >&2
    fi
    exit 1
  fi
  sleep 1
done

# Replace an earlier dashboard window instead of opening a second one. Match only
# Chromium's own command line, never a shell that merely mentions the profile.
if pkill -f -- "^[^ ]*chromium[^ ]* .*--user-data-dir=$profile( |$)"; then
  sleep 2
fi
# A power cut leaves the profile marked as crashed; clear it so no restore prompt appears.
preferences="$profile/Default/Preferences"
if [[ -f "$preferences" ]]; then
  sed -i 's/"exit_type":"Crashed"/"exit_type":"Normal"/' "$preferences"
fi
# The window's size whenever it is neither full screen nor maximized: the whole screen. Chromium's
# own default is half of it (945 px on the Pi's 1920 px monitor), under which the dashboard folds
# into its one-column phone layout.
size=$(wlr-randr 2>/dev/null | sed -n 's/^ *\([0-9]\+\)x\([0-9]\+\) px,.*current.*/\1 \2/p' | head -n 1)
# shellcheck disable=SC2086  # The width and height, as two words
python3 - "$preferences" "$url" ${size:-1920 1080} <<'EOF' || echo "Could not set the window's saved size" >&2
import json, sys
from pathlib import Path
from urllib.parse import urlsplit

path, url, width, height = Path(sys.argv[1]), sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
try:
    preferences = json.loads(path.read_text())
except (OSError, ValueError):
    preferences = {}  # None yet (a first start), or cut short by a power cut: Chromium fills in the rest
parts = urlsplit(url)
app = f"{parts.hostname}_{parts.path or '/'}"  # How Chromium names an --app window's saved placement
screen = {"left": 0, "top": 0, "right": width, "bottom": height}
placement = {**screen, **{f"work_area_{side}": value for side, value in screen.items()}, "maximized": True}
preferences.setdefault("browser", {}).setdefault("app_window_placement", {})[app] = placement
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(preferences))
EOF

# --start-fullscreen asks for full screen just after the window appears, and that ask can
# be lost: once on the Pi a first start stayed at Chromium's default 945 px window.
# --start-maximized is part of the window from the start, so it still fills the screen,
# and F11 goes back to maximized, not to half the screen.
exec chromium \
  --app="$url" \
  --start-maximized \
  --start-fullscreen \
  --user-data-dir="$profile" \
  --ozone-platform=wayland \
  --no-first-run \
  --noerrdialogs \
  --disable-infobars \
  --hide-crash-restore-bubble \
  --password-store=basic
