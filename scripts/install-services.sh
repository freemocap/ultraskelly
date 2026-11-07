#!/usr/bin/env bash
# Install the boot services for this checkout: the robot as a system service
# (ultraskelly-bot), the control panel that turns it on and off (ultraskelly-control,
# a system service on port 80) and the dashboard window as the desktop user's
# service (ultraskelly-dashboard), and put the UltraSkelly icon on the desktop: a click
# starts the bot unless it runs and opens the dashboard (scripts/start-ultraskelly.sh).
# Run it on the Pi as the user the robot runs as.
#
#   scripts/install-services.sh             install or update all three and the icon; the bot and the window keep their start-at-boot
#   scripts/install-services.sh --enable    also start the bot and the window now and at every boot
#   scripts/install-services.sh --disable   stop the bot and the window and keep them from starting at boot
#   scripts/install-services.sh --check     report whether all three are installed for this checkout; change nothing
#
# The control panel always runs, now and at every boot, and is restarted by each
# install so it runs this checkout's code: it moves nothing by itself, and it is how
# the bot is turned on and off (http://<the Pi's name>.local/). A polkit rule lets it
# start, stop and restart the bot's service and nothing else
# (/etc/polkit-1/rules.d/50-ultraskelly-control.rules).
#
# Extra bot options go in /etc/default/ultraskelly-bot, for example while the motor
# boards are disconnected:
#   echo 'ULTRASKELLY_BOT_OPTIONS=--simulate-motors' | sudo tee /etc/default/ultraskelly-bot
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
control_unit=/etc/systemd/system/ultraskelly-control.service
control_rule=/etc/polkit-1/rules.d/50-ultraskelly-control.rules
panel_options=/var/lib/ultraskelly-control/bot-options
dashboard_unit=$HOME/.config/systemd/user/ultraskelly-dashboard.service
desktop_entry="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")/ultraskelly.desktop"

render() {
  sed -e "s|@USER@|$user|g" -e "s|@REPO@|$repo|g" "$repo/scripts/systemd/$1"
}

render_desktop_entry() {
  sed -e "s|@REPO@|$repo|g" "$repo/scripts/desktop/ultraskelly.desktop"
}

# polkit may keep its rules where only root can read them.
installed_rule() {
  cat "$control_rule" 2>/dev/null || sudo -n cat "$control_rule" 2>/dev/null
}

# Options from a file that may not exist (yet): none, rather than an error that ends the script.
bot_options() {
  sed -n 's/^ULTRASKELLY_BOT_OPTIONS=//p' /etc/default/ultraskelly-bot 2>/dev/null || true
}

state() {
  local options panel
  options=$(bot_options)
  panel=$(sed -n 's/^ULTRASKELLY_PANEL_OPTIONS=//p' "$panel_options" 2>/dev/null || true)  # Written at its first switch
  echo "ultraskelly-bot (system):     $(systemctl is-enabled ultraskelly-bot.service), $(systemctl is-active ultraskelly-bot.service)${options:+, options $options from /etc/default/ultraskelly-bot}${panel:+, $panel from the control panel}"
  echo "ultraskelly-control (system): $(systemctl is-enabled ultraskelly-control.service), $(systemctl is-active ultraskelly-control.service), http://$(hostname).local/"
  echo "ultraskelly-dashboard (user): $(systemctl --user is-enabled ultraskelly-dashboard.service), $(systemctl --user is-active ultraskelly-dashboard.service)"
  echo "desktop icon:                 $desktop_entry"
}

if [[ $action == --check ]]; then
  if cmp -s <(render ultraskelly-bot.service) "$bot_unit" &&
    cmp -s <(render ultraskelly-control.service) "$control_unit" &&
    cmp -s <(render ultraskelly-control.rules) <(installed_rule) &&
    cmp -s <(render ultraskelly-dashboard.service) "$dashboard_unit" &&
    cmp -s <(render_desktop_entry) "$desktop_entry"; then
    state
    exit 0
  fi
  echo "The services are not installed for $repo; run scripts/install-services.sh" >&2
  exit 1
fi
if [[ ! -x $repo/.venv/bin/python ]]; then
  echo "Warning: no virtual environment at $repo/.venv yet; the bot cannot start until there is one." >&2
fi

# The bot and the control panel are system services, so they start at boot whether
# or not anyone logs in.
render ultraskelly-bot.service | sudo tee "$bot_unit" >/dev/null
render ultraskelly-control.service | sudo tee "$control_unit" >/dev/null
if [[ ! -d $(dirname "$control_rule") ]]; then
  echo "No $(dirname "$control_rule"): install polkitd (scripts/setup-pi.sh does), or the panel cannot turn the bot on and off." >&2
else
  render ultraskelly-control.rules | sudo tee "$control_rule" >/dev/null
  sudo chmod 644 "$control_rule"
fi
sudo systemctl daemon-reload
sudo systemctl enable ultraskelly-control.service
sudo systemctl restart ultraskelly-control.service ||
  echo "The control panel did not start; see journalctl -u ultraskelly-control" >&2

# The dashboard window belongs to the desktop session, so it is a user service;
# lingering starts the user's service manager at boot.
if [[ ! -e /var/lib/systemd/linger/$user ]]; then
  sudo loginctl enable-linger "$user"
fi
mkdir -p "$(dirname "$dashboard_unit")"
render ultraskelly-dashboard.service >"$dashboard_unit"
systemctl --user daemon-reload

# The desktop icon: a click starts the bot unless it runs and opens the dashboard.
mkdir -p "$(dirname "$desktop_entry")"
render_desktop_entry >"$desktop_entry"
chmod 755 "$desktop_entry"  # The desktop's file manager runs a launcher only when it is executable

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
