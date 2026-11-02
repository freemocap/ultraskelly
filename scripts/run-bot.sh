#!/usr/bin/env bash
# Run the robot the way the boot service does: under the shared robot lock, from
# this checkout's virtual environment, without the OpenCV window.
#
#   scripts/run-bot.sh [more bot options, e.g. --record]
#
# Only one program at a time may use the OAK, the Hailo-8 and the motor boards, so
# each holds /run/lock/ultraskelly-robot.lock while it runs (test scripts through
# `flock -n /run/lock/ultraskelly-robot.lock <command>`). If another program holds
# the lock, this names it and exits with status 75 instead of waiting; the service
# tries again later. The bot inherits the lock and holds it until it exits.
set -euo pipefail

repo="$(cd "$(dirname "$0")/.." && pwd)"
python="$repo/.venv/bin/python"
lock=/run/lock/ultraskelly-robot.lock

if [[ ! -x $python ]]; then
  echo "No virtual environment at $repo/.venv; create it as the README describes." >&2
  exit 1
fi

# A read-only descriptor can take the lock whoever created the file.
[[ -e $lock ]] || : >"$lock"
exec 9<"$lock"
if ! flock --nonblock 9; then
  echo "Not starting: another program is using the robot ($lock is held)." >&2
  for pid in $(lslocks --noheadings --raw --output PID,PATH | awk -v lock="$lock" '$2 == lock {print $1}'); do
    echo "  PID $pid: $(ps -o args= -p "$pid" || echo '(exited)')" >&2
  done
  exit 75
fi

cd "$repo"
exec "$python" -m ultraskelly --hardware --no-window "$@"
