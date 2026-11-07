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

exec chromium \
  --app="$url" \
  --start-fullscreen \
  --user-data-dir="$profile" \
  --ozone-platform=wayland \
  --no-first-run \
  --noerrdialogs \
  --disable-infobars \
  --hide-crash-restore-bubble \
  --password-store=basic
