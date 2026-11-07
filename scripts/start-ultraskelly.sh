#!/usr/bin/env bash
# What the UltraSkelly icon on the Pi's desktop runs (scripts/desktop/ultraskelly.desktop, put there by
# scripts/install-services.sh): start the bot unless it is already running or starting, then open the
# dashboard on the monitor, full screen and in front.
#
# The bot starts through the always-on control panel, as its Turn on button does, so the panel shows it;
# if the panel does not answer, through systemctl, which the panel's polkit rule allows this user. The
# window is the ultraskelly-dashboard user service, restarted so a fresh one opens even when an old one
# was closed or hidden. What it did goes to the journal: journalctl -t ultraskelly-shortcut
set -uo pipefail

say() { logger -t ultraskelly-shortcut -- "$*"; }

state=$(systemctl is-active ultraskelly-bot.service)
if [[ $state == active || $state == activating ]]; then
  say "The bot is already $state"
elif curl -fs --max-time 10 -X POST -H 'Content-Type: application/json' -d '{}' http://localhost/api/bot/start >/dev/null; then
  say "Starting the bot through the control panel"
elif systemctl --no-ask-password --no-block start ultraskelly-bot.service; then
  say "Starting the bot with systemctl: the control panel did not answer"
else
  say "Could not start the bot; see journalctl -u ultraskelly-control and journalctl -u ultraskelly-bot"
fi

# The window opens once the control panel answers, and shows the bot starting until it runs.
systemctl --user restart ultraskelly-dashboard.service
