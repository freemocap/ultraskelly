#!/usr/bin/env bash
# Install the boot services for this checkout: the robot as a system service
# (ultraskelly-bot) and the dashboard window as the desktop user's service
# (ultraskelly-dashboard). Run it on the Pi as the user the robot runs as.
#
#   scripts/install-services.sh             install or update both; leaves start-at-boot as it was
#   scripts/install-services.sh --enable    also start both now and at every boot
#   scripts/install-services.sh --disable   stop both and keep them from starting at boot
#   scripts/install-services.sh --check     report whether both are installed for this checkout; change nothing
#
# The units come from scripts/systemd/ with this checkout's path and user filled in,
# so installing from ~/ultraskelly runs ~/ultraskelly. Safe to re-run: do so after
# moving the checkout or changing a unit. Starting the bot moves the motors.
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
user="$(id -un)"
action="${1:-}"
case "$action" in
  "" | --enable | --disable | --check) ;;
  *) echo "usage: $0 [--enable | --disable | --check]" >&2; exit 2 ;;
esac
if [[ $user == root ]]; then
  echo "Run this as the robot's user, not as root; it uses sudo where it must." >&2
  exit 1
fi
bot_unit=/etc/systemd/system/ultraskelly-bot.service
dashboard_unit=$HOME/.config/systemd/user/ultraskelly-dashboard.service

render() {
  sed -e "s|@USER@|$user|g" -e "s|@REPO@|$repo|g" "$repo/scripts/systemd/$1"
}

state() {
  echo "ultraskelly-bot (system):     $(systemctl is-enabled ultraskelly-bot.service), $(systemctl is-active ultraskelly-bot.service)"
  echo "ultraskelly-dashboard (user): $(systemctl --user is-enabled ultraskelly-dashboard.service), $(systemctl --user is-active ultraskelly-dashboard.service)"
}

if [[ $action == --check ]]; then
  if cmp -s <(render ultraskelly-bot.service) "$bot_unit" &&
    cmp -s <(render ultraskelly-dashboard.service) "$dashboard_unit"; then
    state
    exit 0
  fi
  echo "The services are not installed for $repo; run scripts/install-services.sh" >&2
  exit 1
fi
if [[ ! -x $repo/.venv/bin/python ]]; then
  echo "Warning: no virtual environment at $repo/.venv yet; the bot cannot start until there is one." >&2
fi

# The bot is a system service, so it starts at boot whether or not anyone logs in.
render ultraskelly-bot.service | sudo tee "$bot_unit" >/dev/null
sudo systemctl daemon-reload

# The dashboard window belongs to the desktop session, so it is a user service;
# lingering starts the user's service manager at boot.
if [[ ! -e /var/lib/systemd/linger/$user ]]; then
  sudo loginctl enable-linger "$user"
fi
mkdir -p "$(dirname "$dashboard_unit")"
render ultraskelly-dashboard.service >"$dashboard_unit"
systemctl --user daemon-reload

case "$action" in
  --enable)
    sudo systemctl enable --now ultraskelly-bot.service
    systemctl --user enable --now ultraskelly-dashboard.service
    ;;
  --disable)
    systemctl --user disable --now ultraskelly-dashboard.service
    sudo systemctl disable --now ultraskelly-bot.service
    ;;
esac

state
if [[ -z $action ]] && systemctl --quiet is-active ultraskelly-bot.service; then
  echo "The bot is running; restart it to apply any unit changes: sudo systemctl restart ultraskelly-bot"
fi
