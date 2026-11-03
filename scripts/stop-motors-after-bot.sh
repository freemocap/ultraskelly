#!/usr/bin/env bash
# Brake the DC motors after the bot service's process ends, however it ended: a clean
# stop, a crash or a watchdog kill. ultraskelly-bot.service runs this as ExecStopPost,
# because a killed bot never runs its own stop commands and the motor board keeps
# driving its last throttle until told otherwise.
#
# Skipped when the bot never started because another program held the robot lock
# (status 75): that program drives the motors. Gives up after 10 s, so an unreachable
# board cannot hold up the next start.
set -uo pipefail

if [[ ${EXIT_CODE:-} == exited && ${EXIT_STATUS:-} == 75 ]]; then
  echo "The bot did not start (another program holds the robot lock); leaving the motors alone."
  exit 0
fi
cd "$(dirname "$0")/.." || exit 1
exec timeout --kill-after=2 10 .venv/bin/python -m scripts.stop_motors
