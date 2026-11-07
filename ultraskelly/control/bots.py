"""What the control panel turns on and off: the bot's systemd service on the Pi, or a local stand-in.

Both report the same status (BotStatus): one of STATES, since when, how often it restarted on its
own, how its process last ended, and its log, from which `last_error` picks the newest error.
"""
import asyncio
import json
import logging
import re
import time
from collections import deque
from dataclasses import dataclass
from typing import Awaitable, Callable

logger = logging.getLogger(__name__)

# Programs run from an argument list, never through a shell: no word of a command is ever parsed as shell syntax.
launch = asyncio.create_subprocess_exec

# running: up and ready. starting: building its nodes (the OAK boots, the Hailo model loads). stopping: shutting
# down. restarting: it ended and systemd starts it again shortly. waiting: like restarting, but it did not start
# because another program holds the robot lock. off: stopped on purpose. failed: stopped and not coming back by
# itself. missing: no such service on this computer. unknown: the service manager did not answer.
STATES = ("running", "starting", "stopping", "restarting", "waiting", "off", "failed", "missing", "unknown")
LOCK_HELD = 75  # scripts/run-bot.sh's exit status while another program holds the robot lock
VERBS = ("start", "stop", "restart")


@dataclass
class BotStatus:
    state: str
    since: float | None = None  # Unix time of the last change of state
    started: float | None = None  # Unix time it last became ready
    restarts: int = 0  # Automatic restarts since it was last started on purpose
    pid: int | None = None
    exit: dict | None = None  # How its process last ended: {"code": "exited" | "killed" | "dumped", "status": ...}
    result: str | None = None  # systemd's word for how its last run ended: success, exit-code, signal, watchdog, ...
    detail: str = ""  # The manager's own words for the state


class ActionFailed(Exception):
    """The service manager refused or failed a start, stop or restart; the message says why."""


async def run_command(*command: str, timeout: float = 10.0) -> tuple[int, str, str]:
    """Run a program and return its exit status, output and error output (status 124 on a timeout)."""
    process = await launch(*command, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    try:
        out, err = await asyncio.wait_for(process.communicate(), timeout)
    except TimeoutError:
        process.kill()
        await process.wait()
        return 124, "", f"{command[0]} did not answer within {timeout:.0f} s"
    return process.returncode, out.decode("utf-8", "replace"), err.decode("utf-8", "replace")


# --- systemd ------------------------------------------------------------------------------------

SHOW = ("LoadState", "ActiveState", "SubState", "Result", "ExecMainCode", "ExecMainStatus", "MainPID", "NRestarts",
        "ActiveEnterTimestamp", "InactiveEnterTimestamp", "StateChangeTimestamp")
EXIT_CODES = {"1": "exited", "2": "killed", "3": "dumped"}


def parse_show(text: str) -> dict[str, str]:
    """`systemctl show`'s Key=Value lines as a dict."""
    return dict(line.split("=", 1) for line in text.splitlines() if "=" in line)


def unix_time(value: str) -> float | None:
    """A timestamp from `systemctl show --timestamp=unix` (@1730930523), or None when unset."""
    value = value.strip()
    if value.startswith("@"):
        try:
            seconds = float(value[1:])
        except ValueError:
            return None
        return seconds or None
    return None


def status_from(properties: dict[str, str]) -> BotStatus:
    """The bot's status from the unit's properties."""
    get = properties.get
    if get("LoadState") == "not-found":
        return BotStatus(state="missing", detail="not-found")
    active, sub = get("ActiveState", ""), get("SubState", "")
    code, status = EXIT_CODES.get(get("ExecMainCode", "")), get("ExecMainStatus", "")
    exit_info = {"code": code, "status": int(status) if status.isdigit() else status} if code else None
    lock_held = exit_info == {"code": "exited", "status": LOCK_HELD}
    if active in ("active", "reloading"):
        state = "running"
    elif active == "activating":
        state = ("waiting" if lock_held else "restarting") if sub.startswith("auto-restart") else "starting"
    elif active == "deactivating":
        state = "stopping"
    elif active == "failed":
        state = "failed"
    else:
        state = "off"
    restarts = get("NRestarts", "0")
    return BotStatus(
        state=state, since=unix_time(get("StateChangeTimestamp", "")), started=unix_time(get("ActiveEnterTimestamp", "")),
        restarts=int(restarts) if restarts.isdigit() else 0, pid=int(get("MainPID", "0") or 0) or None,
        exit=exit_info, result=get("Result") or None, detail=f"{active}/{sub}".strip("/"),
    )


class SystemdBot:
    """The bot's service, through systemctl and journalctl as the robot's user. Starting, stopping and
    restarting need the polkit rule that allows exactly those for this unit (scripts/systemd/ultraskelly-control.rules)."""

    def __init__(self, unit: str = "ultraskelly-bot.service", *, run=run_command) -> None:
        self.unit = unit
        self.run = run

    async def status(self) -> BotStatus:
        code, out, err = await self.run("systemctl", "show", self.unit, "--timestamp=unix", *(f"--property={name}" for name in SHOW))
        if code != 0:
            raise ActionFailed(err.strip() or f"systemctl show failed ({code})")
        return status_from(parse_show(out))

    async def act(self, verb: str) -> None:
        if verb not in VERBS:
            raise ValueError(verb)
        # --no-block: answer at once; the page follows the start through the status. --no-ask-password: fail at once,
        # with polkit's reason, if the rule is missing, rather than wait for a password nobody can type.
        code, _, err = await self.run("systemctl", "--no-block", "--no-ask-password", verb, self.unit)
        if code != 0:
            raise ActionFailed(err.strip() or f"systemctl {verb} failed ({code})")

    async def journal(self, lines: int = 1000) -> list[dict]:
        """The unit's newest log lines, oldest first: [{"time", "text"}]. Empty if the log cannot be read (the
        robot's user then needs the systemd-journal group)."""
        code, out, _ = await self.run(
            "journalctl", "--unit", self.unit, "--lines", str(lines), "--no-pager", "--output", "json",
            "--output-fields", "MESSAGE", timeout=15,
        )
        return journal_entries(out) if code == 0 else []

    async def close(self) -> None:
        pass


def journal_entries(text: str) -> list[dict]:
    """journalctl's JSON lines as [{"time": Unix seconds, "text": the message}]."""
    entries = []
    for line in text.splitlines():
        try:
            record = json.loads(line)
        except ValueError:
            continue
        message = record.get("MESSAGE")
        if isinstance(message, list):  # Not valid UTF-8: journald sends the bytes
            message = bytes(value for value in message if isinstance(value, int) and 0 <= value < 256).decode("utf-8", "replace")
        if not isinstance(message, str):
            continue
        try:
            when = int(record.get("__REALTIME_TIMESTAMP", "0")) / 1e6
        except ValueError:
            when = 0.0
        entries.append({"time": when, "text": message})
    return entries


# --- The bot's errors in its log ----------------------------------------------------------------

# The bot logs "%(asctime)s %(levelname)s %(name)s: %(message)s" (ultraskelly/__main__.py).
LOG_LINE = re.compile(r"^\d{4}-\d\d-\d\d \d\d:\d\d:\d\d,\d{3} (?P<level>[A-Z]+) (?P<name>[\w.]+): (?P<message>.*)$")
# scripts/run-bot.sh's own reasons for not starting.
NOT_STARTING = ("Not starting:", "No virtual environment at")


def last_error(entries: list[dict]) -> dict | None:
    """The newest error in the bot's log lines (oldest first), or None: {"time", "text", "waiting"}. An error the
    bot logged takes the exception that follows it, if any; `waiting` marks the robot lock held by another program."""
    found = None
    in_traceback = False
    for entry in entries:
        text = entry["text"].rstrip()
        logged = LOG_LINE.match(text)
        if logged:
            in_traceback = False
            if logged["level"] in ("ERROR", "CRITICAL"):
                found = {"time": entry["time"], "text": logged["message"], "waiting": False}
            continue
        if text.startswith("Traceback (most recent call last)"):
            in_traceback = True
            continue
        if in_traceback:
            if not text or text[0].isspace() or text.startswith(("During handling", "The above exception")):
                continue
            in_traceback = False  # The exception itself ends the traceback
            if found is not None and entry["time"] - found["time"] < 2 and "\n" not in found["text"]:
                found = {**found, "text": f"{found['text']}\n{text}", "time": entry["time"]}
            else:
                found = {"time": entry["time"], "text": text, "waiting": False}
            continue
        if text.startswith(NOT_STARTING):
            found = {"time": entry["time"], "text": text, "waiting": text.startswith("Not starting:")}
        elif found is not None and found["waiting"] and text.lstrip().startswith("PID ") and entry["time"] - found["time"] < 2:
            found = {**found, "text": f"{found['text']}\n{text.strip()}"}  # Which program holds the lock
    return found


# --- A local stand-in, for development ----------------------------------------------------------

class LocalBot:
    """A command run as the bot, for developing the panel where there is no systemd: started, stopped and
    restarted like the service, ready once `probe` answers, its output kept as its log. It does not restart
    by itself."""

    def __init__(self, command: list[str], *, probe: Callable[[], Awaitable[bool]], clock=time.time) -> None:
        self.command = command
        self.probe = probe
        self.clock = clock
        self.process: asyncio.subprocess.Process | None = None
        self.state, self.since, self.started = "off", clock(), None
        self.exit: dict | None = None
        self.log: deque[dict] = deque(maxlen=1000)
        self.tasks: set[asyncio.Task] = set()
        self.stopping = False

    def _become(self, state: str) -> None:
        if state != self.state:
            self.state, self.since = state, self.clock()
            if state == "running":
                self.started = self.since

    def _spawn(self, coroutine) -> None:
        task = asyncio.create_task(coroutine)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    async def status(self) -> BotStatus:
        alive = self.process is not None and self.process.returncode is None
        return BotStatus(state=self.state, since=self.since, started=self.started if self.state == "running" else None,
                         pid=self.process.pid if alive else None, exit=self.exit, detail="local stand-in")

    async def act(self, verb: str) -> None:
        if verb not in VERBS:
            raise ValueError(verb)
        if verb in ("stop", "restart"):
            await self._stop()
        if verb in ("start", "restart"):
            await self._start()

    async def _start(self) -> None:
        if self.process is not None and self.process.returncode is None:
            return
        self.stopping, self.exit = False, None
        self._become("starting")
        self.log.append({"time": self.clock(), "text": f"Starting: {' '.join(self.command)}"})
        self.process = await launch(*self.command, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
        self._spawn(self._read(self.process))
        self._spawn(self._until_ready(self.process))

    async def _read(self, process: asyncio.subprocess.Process) -> None:
        async for line in process.stdout:
            self.log.append({"time": self.clock(), "text": line.decode("utf-8", "replace").rstrip()})
        status = await process.wait()
        self.exit = {"code": "exited", "status": status}
        if process is self.process:
            self._become("off" if self.stopping else "failed")
            if not self.stopping:
                self.log.append({"time": self.clock(), "text": f"Exited by itself with status {status}"})

    async def _until_ready(self, process: asyncio.subprocess.Process) -> None:
        while process.returncode is None and self.state == "starting":
            if await self.probe():
                self._become("running")
                return
            await asyncio.sleep(0.25)

    async def _stop(self) -> None:
        process = self.process
        if process is None or process.returncode is not None:
            return
        self.stopping = True
        self._become("stopping")
        process.terminate()
        try:
            await asyncio.wait_for(process.wait(), 10)
        except TimeoutError:
            process.kill()
            await process.wait()
        self._become("off")

    async def journal(self, lines: int = 1000) -> list[dict]:
        return list(self.log)[-lines:]

    async def close(self) -> None:
        await self._stop()
        for task in list(self.tasks):
            task.cancel()
