"""The control panel (ultraskelly/control): the always-on service that turns the bot on and off, restarts it and
switches following, against stand-ins for systemd and for the bot's own routes; its systemd units, its polkit rule
and the Pi screen's window. Nothing here starts a real service or moves a motor."""
import asyncio
import json
import shutil
import socket
import subprocess
import sys
from pathlib import Path

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.control.bots import (
    ActionFailed,
    BotStatus,
    LocalBot,
    SystemdBot,
    journal_entries,
    last_error,
    parse_show,
    status_from,
    unix_time,
)
from ultraskelly.control.options import PANEL_VARIABLE, StartOptions, following_in, read_variable
from ultraskelly.control.server import ControlServer

ROOT = Path(__file__).resolve().parents[1]
SYSTEMD = ROOT / "scripts" / "systemd"
NODE = shutil.which("node")


# --- The switches the bot starts with -------------------------------------------------------------

def test_the_bot_follows_by_default_and_the_panels_switch_wins_over_the_services_options(tmp_path):
    defaults, kept = tmp_path / "ultraskelly-bot", tmp_path / "state" / "bot-options"
    options = StartOptions(kept, bot_defaults=defaults)
    assert options.follows() and options.panel() == [] and options.service() == []
    defaults.write_text('ULTRASKELLY_BOT_OPTIONS="--record --no-follow"\n', encoding="utf-8")
    assert options.service() == ["--record", "--no-follow"] and not options.follows()
    options.set_following(True)  # Makes its directory too, as StateDirectory= does on the Pi
    assert options.follows() and options.panel() == ["--follow"]
    assert kept.read_text(encoding="utf-8").splitlines()[-1] == f"{PANEL_VARIABLE}=--follow"
    options.set_following(False)
    assert not options.follows() and options.panel() == ["--no-follow"]  # Replaced, not added to
    assert [path.name for path in kept.parent.iterdir()] == ["bot-options"]  # No draft left beside it
    assert options.describe() == {"following": False, "panel": ["--no-follow"], "service": ["--record", "--no-follow"],
                                  "file": str(kept)}


def test_environment_files_are_read_as_systemd_reads_them(tmp_path):
    path = tmp_path / "env"
    assert read_variable(path, "X") is None  # No file
    path.write_text("# a note\nX=--a\nY=1\nX='--b  --c'\n", encoding="utf-8")
    assert read_variable(path, "X") == ["--b", "--c"]  # The last assignment, unquoted, split into words
    assert read_variable(path, "Z") is None
    assert following_in(["--no-follow", "--record", "--follow"]) is True and following_in(["--record"]) is None


# --- The bot's service ----------------------------------------------------------------------------

SHOW_RUNNING = """LoadState=loaded
ActiveState=active
SubState=running
Result=success
ExecMainCode=0
ExecMainStatus=0
MainPID=4242
NRestarts=2
ActiveEnterTimestamp=@1762500000
InactiveEnterTimestamp=
StateChangeTimestamp=@1762500000
"""


@pytest.mark.parametrize("properties, state", [
    ({"ActiveState": "active", "SubState": "running"}, "running"),
    ({"ActiveState": "activating", "SubState": "start"}, "starting"),
    ({"ActiveState": "activating", "SubState": "auto-restart", "ExecMainCode": "1", "ExecMainStatus": "1"}, "restarting"),
    ({"ActiveState": "activating", "SubState": "auto-restart", "ExecMainCode": "1", "ExecMainStatus": "75"}, "waiting"),
    ({"ActiveState": "deactivating", "SubState": "stop-sigterm"}, "stopping"),
    ({"ActiveState": "failed", "SubState": "failed", "Result": "exit-code"}, "failed"),
    ({"ActiveState": "inactive", "SubState": "dead"}, "off"),
    ({"LoadState": "not-found", "ActiveState": "inactive"}, "missing"),
])
def test_the_services_state_is_named_as_the_panel_shows_it(properties, state):
    assert status_from({"LoadState": "loaded", **properties}).state == state


def test_the_status_says_since_when_which_process_and_how_the_last_run_ended():
    assert status_from(parse_show(SHOW_RUNNING)) == BotStatus(
        state="running", since=1762500000.0, started=1762500000.0, restarts=2, pid=4242, exit=None, result="success",
        detail="active/running",
    )
    killed = status_from({"LoadState": "loaded", "ActiveState": "failed", "SubState": "failed", "Result": "signal",
                          "ExecMainCode": "2", "ExecMainStatus": "9", "MainPID": "0"})
    assert killed.exit == {"code": "killed", "status": 9} and killed.pid is None and killed.result == "signal"
    assert unix_time("@1762500000") == 1762500000.0 and unix_time("") is None and unix_time("@0") is None


def test_the_service_is_driven_through_systemctl_and_never_waits_for_a_password():
    calls = []

    async def run(*command, timeout=10.0):
        calls.append(command)
        if command[:2] == ("systemctl", "show"):
            return 0, SHOW_RUNNING, ""
        if command[0] == "journalctl":
            return 0, json.dumps({"MESSAGE": "hello", "__REALTIME_TIMESTAMP": "1762500000500000"}) + "\n", ""
        return 1, "", "Failed to restart ultraskelly-bot.service: Interactive authentication required.\n"

    async def scenario():
        bot = SystemdBot(run=run)
        assert (await bot.status()).state == "running"
        with pytest.raises(ActionFailed, match="Interactive authentication required"):
            await bot.act("restart")
        assert calls[-1] == ("systemctl", "--no-block", "--no-ask-password", "restart", "ultraskelly-bot.service")
        with pytest.raises(ValueError):
            await bot.act("enable")
        assert await bot.journal() == [{"time": 1762500000.5, "text": "hello"}]
        assert calls[-1][:3] == ("journalctl", "--unit", "ultraskelly-bot.service")

    asyncio.run(scenario())


# --- The bot's errors, from its log ------------------------------------------------------------------

def log(*lines: str, start: float = 1762500000.0) -> list[dict]:
    return [{"time": start + index * 0.1, "text": line} for index, line in enumerate(lines)]


def test_the_last_error_is_the_newest_the_bot_logged_with_its_exception():
    found = last_error(log(
        "2026-11-07 10:00:00,000 INFO ultraskelly.bot_launcher: Starting",
        "2026-11-07 10:00:01,000 ERROR ultraskelly.core.bot.sensory.oak_camera_node: The OAK stopped",
        "2026-11-07 10:00:02,000 ERROR ultraskelly.bot_launcher: A node crashed",
        "Traceback (most recent call last):",
        '  File "node.py", line 1, in run',
        "RuntimeError: X_LINK_ERROR",
        "2026-11-07 10:00:03,000 INFO ultraskelly.bot_launcher: Still running",
    ))
    assert found["text"] == "A node crashed\nRuntimeError: X_LINK_ERROR" and not found["waiting"]
    assert last_error(log("2026-11-07 10:00:00,000 INFO ultraskelly.bot_launcher: All is well")) is None
    assert last_error([]) is None


def test_a_bot_waiting_for_the_robot_lock_says_which_program_holds_it():
    found = last_error(log(
        "Not starting: another program is using the robot (/run/lock/ultraskelly-robot.lock is held).",
        "  PID 1234: python -m scripts.check_motor_directions",
    ))
    assert found["waiting"] and found["text"].endswith("\nPID 1234: python -m scripts.check_motor_directions")


def test_journal_lines_are_read_even_when_they_are_not_text():
    text = "\n".join([
        json.dumps({"MESSAGE": "plain", "__REALTIME_TIMESTAMP": "1762500000000000"}),
        json.dumps({"MESSAGE": list(b"bytes \xff"), "__REALTIME_TIMESTAMP": "1762500001000000"}),  # Not UTF-8
        json.dumps({"__REALTIME_TIMESTAMP": "1762500002000000"}),  # No message
        "not json",
    ])
    assert journal_entries(text) == [{"time": 1762500000.0, "text": "plain"}, {"time": 1762500001.0, "text": "bytes �"}]


# --- A local stand-in bot, for development -----------------------------------------------------------

async def until(condition, seconds: float = 10.0) -> bool:
    for _ in range(int(seconds / 0.05)):
        if await condition():
            return True
        await asyncio.sleep(0.05)
    return False


async def state_is(bot, state: str) -> bool:
    return (await bot.status()).state == state


async def logged(bot, text: str) -> bool:
    return any(entry["text"] == text for entry in await bot.journal())


def test_a_local_stand_in_starts_runs_and_stops_like_the_service():
    async def scenario():
        ready = asyncio.Event()

        async def probe():
            return ready.is_set()

        bot = LocalBot([sys.executable, "-c", "import time; print('stand-in up', flush=True); time.sleep(60)"], probe=probe)
        await bot.act("start")
        assert (await bot.status()).state == "starting"  # Until its server answers
        ready.set()
        assert await until(lambda: state_is(bot, "running"))
        assert (await bot.status()).pid
        assert await until(lambda: logged(bot, "stand-in up"))  # Its output is its log
        await bot.act("stop")
        assert (await bot.status()).state == "off"
        await bot.close()

    asyncio.run(scenario())


def test_a_local_stand_in_that_exits_by_itself_has_failed():
    async def scenario():
        async def probe():
            return False

        bot = LocalBot([sys.executable, "-c", "raise SystemExit(3)"], probe=probe)
        await bot.act("start")
        assert await until(lambda: state_is(bot, "failed"))
        assert (await bot.status()).exit == {"code": "exited", "status": 3}
        await bot.close()

    asyncio.run(scenario())


# --- The panel's web service -----------------------------------------------------------------------

class FakeService:
    """The bot's systemd service: its state, the commands it was given and its log."""
    unit = "ultraskelly-bot.service"

    def __init__(self, state: str = "running") -> None:
        self.state, self.refuse, self.commands, self.log = state, None, [], []

    async def status(self) -> BotStatus:
        return BotStatus(state=self.state, since=1762500000.0, started=1762500000.0 if self.state == "running" else None)

    async def act(self, verb: str) -> None:
        if self.refuse:
            raise ActionFailed(self.refuse)
        self.commands.append(verb)
        self.state = {"start": "starting", "stop": "off", "restart": "starting"}[verb]

    async def journal(self, lines: int = 1000) -> list[dict]:
        return self.log

    async def close(self) -> None:
        pass


def bot_routes(heard: list, *, live_switch: bool) -> web.Application:
    """The running bot's own routes (ultraskelly/ui/motor_test_routes.py); `heard` keeps what it was sent. Without
    `live_switch`, a bot from before the panel: no /api/follow."""
    async def follow(request):
        body = await request.json()
        heard.append(("follow", body))
        return web.json_response({"ok": True, "following": body["following"]})

    async def tests(request):
        return web.json_response({"enabled": True, "following": True})

    app = web.Application()
    if live_switch:
        app.router.add_post("/api/follow", follow)
    app.router.add_get("/api/motor-test", tests)
    return app


def closed_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def panel(scenario, tmp_path, *, state="running", bot=True, live_switch=True, **options):
    """Run `scenario(client, server, service, heard)` against a control panel for a FakeService, the bot's routes
    served (bot=True) or nothing answering on its port."""
    async def wrapper():
        heard = []
        bot_server = TestServer(bot_routes(heard, live_switch=live_switch), host="127.0.0.1") if bot else None
        if bot_server is not None:
            await bot_server.start_server()
        service = FakeService(state)
        server = ControlServer(
            service, options=StartOptions(tmp_path / "bot-options", bot_defaults=tmp_path / "ultraskelly-bot"),
            bot_port=bot_server.port if bot_server is not None else closed_port(), hostname="ultraskelly-rpi5", **options,
        )
        try:
            async with TestClient(TestServer(server.build_app())) as client:
                await scenario(client, server, service, heard)
        finally:
            if bot_server is not None:
                await bot_server.close()
    asyncio.run(wrapper())


def test_the_panel_serves_the_dashboard_and_says_how_the_bot_is(tmp_path):
    async def scenario(client, server, service, heard):
        page = await (await client.get("/")).text()
        assert 'id="control"' in page and 'id="control-collapse"' in page and 'id="control-rail"' in page
        assert "control-stop" not in page  # The stop-all button was removed at the owner's request, 2026-11-07
        assert (await client.get("/panel.js")).content_type == "text/javascript"
        assert await (await client.get("/api/panel")).json() == {
            "bot_port": server.bot_port, "local": False, "host": "ultraskelly-rpi5", "unit": "ultraskelly-bot.service",
        }
        state = await (await client.get("/api/bot")).json()
        assert state["bot"]["state"] == "running" and state["start"]["following"] is True and state["last_error"] is None
        service.log = log("2026-11-07 10:00:01,000 ERROR ultraskelly.bot_launcher: The OAK stopped")
        service.state = "failed"  # A new state reads the log again at once
        server._status_at = 0.0  # Past the status's half-second reuse
        assert (await (await client.get("/api/bot")).json())["last_error"]["text"] == "The OAK stopped"
    panel(scenario, tmp_path)


def test_start_stop_and_restart_go_to_the_service_and_say_what_they_did(tmp_path):
    async def scenario(client, server, service, heard):
        reply = await client.post("/api/bot/stop", json={})
        assert reply.status == 200 and (await reply.json())["message"] == "Stopping the bot"
        assert (await (await client.get("/api/bot")).json())["bot"]["state"] == "off"  # Not an answer from before
        await client.post("/api/bot/start", json={})
        await client.post("/api/bot/restart", json={})
        assert service.commands == ["stop", "start", "restart"]
        assert (await client.post("/api/bot/enable", json={})).status == 404
        service.refuse = "Interactive authentication required."
        reply = await client.post("/api/bot/start", json={})
        assert reply.status == 502 and "authentication" in (await reply.json())["error"]
        assert (await (await client.get("/api/bot")).json())["action"]["ok"] is False  # Every page shows it
    panel(scenario, tmp_path)


def test_commands_need_json_and_the_robots_own_name(tmp_path):
    async def scenario(client, server, service, heard):
        for host in ("evil.example", "ultraskelly-rpi5.evil.example", "ultraskelly-rpi5.local.evil.example"):
            reply = await client.post("/api/bot/stop", json={}, headers={"Host": host})  # As after DNS rebinding
            assert reply.status == 403, host
        assert (await client.post("/api/bot/stop", data="x=1")).status == 415  # A plain form, as any page could send
        assert service.commands == []
        for host in ("ultraskelly-rpi5", "ultraskelly-rpi5.local", "ULTRASKELLY-RPI5.local:80", "localhost:8000",
                     "192.168.1.20", "[::1]:80", "[fe80::1]"):
            reply = await client.post("/api/bot/restart", json={}, headers={"Host": host})
            assert reply.status == 200, host
    panel(scenario, tmp_path)


def test_following_switches_the_running_bot_at_once_and_holds_for_every_start(tmp_path):
    async def scenario(client, server, service, heard):
        reply = await (await client.post("/api/follow", json={"following": False})).json()
        assert (reply["applied"], reply["message"]) == ("now", "Following off")
        assert heard == [("follow", {"following": False})]
        assert (tmp_path / "bot-options").read_text(encoding="utf-8").endswith(f"{PANEL_VARIABLE}=--no-follow\n")
        assert (await (await client.get("/api/bot")).json())["start"]["following"] is False
        service.state = "off"
        reply = await (await client.post("/api/follow", json={"following": True})).json()
        assert reply["applied"] == "next start" and len(heard) == 1  # No bot to switch now
        for body in ({"following": "no"}, {"following": 1}, {}, ["following"]):
            assert (await client.post("/api/follow", json=body)).status == 400, body
        assert StartOptions(tmp_path / "bot-options").panel() == ["--follow"]  # Unchanged by what was refused
    panel(scenario, tmp_path)


def test_a_bot_without_the_live_switch_is_restarted_to_take_it(tmp_path):
    async def scenario(client, server, service, heard):
        reply = await (await client.post("/api/follow", json={"following": False})).json()
        assert reply["applied"] == "restart" and service.commands == ["restart"]
    panel(scenario, tmp_path, live_switch=False)


def test_following_switched_while_the_bot_starts_reaches_it_once_it_runs(tmp_path):
    async def scenario(client, server, service, heard):
        reply = await (await client.post("/api/follow", json={"following": False})).json()
        assert reply["applied"] == "once started" and heard == []
        service.state = "running"

        async def heard_it():
            return bool(heard)

        assert await until(heard_it, 5.0)
        assert heard == [("follow", {"following": False})]
    panel(scenario, tmp_path, state="starting")


def test_motor_tests_pass_through_to_the_bot_and_say_when_it_is_off(tmp_path):
    async def running(client, server, service, heard):
        assert await (await client.get("/api/motor-test")).json() == {"enabled": True, "following": True}

    async def off(client, server, service, heard):
        reply = await client.get("/api/motor-test")
        assert reply.status == 503 and "off or still starting" in (await reply.json())["error"]

    panel(running, tmp_path)
    panel(off, tmp_path, bot=False, state="off")


def test_the_panel_loads_none_of_the_bots_code():
    """So it answers, and can turn the bot off, while the bot's code is broken."""
    code = ("import sys, ultraskelly.control.__main__; print(sorted(m for m in sys.modules "
            "if m.startswith('ultraskelly.core') or m in ('numpy', 'cv2', 'depthai', 'pydantic')))")
    done = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60, check=True, cwd=ROOT)
    assert done.stdout.strip() == "[]"


# --- Its units and polkit rule, as scripts/install-services.sh installs them ---------------------------------

def render(name: str) -> str:
    return (SYSTEMD / name).read_text(encoding="utf-8").replace("@USER@", "robot").replace("@REPO@", "/home/robot/ultraskelly")


def test_the_bot_starts_with_the_panels_switches_last():
    bot, control = render("ultraskelly-bot.service"), render("ultraskelly-control.service")
    state_directory = next(line.split("=", 1)[1] for line in control.splitlines() if line.startswith("StateDirectory="))
    kept = f"EnvironmentFile=-/var/lib/{state_directory}/bot-options"  # Where the panel writes them (control/__main__.py)
    assert kept in bot and bot.index("EnvironmentFile=-/etc/default/ultraskelly-bot") < bot.index(kept)
    exec_start = next(line for line in bot.splitlines() if line.startswith("ExecStart="))
    assert exec_start.endswith(f"$ULTRASKELLY_BOT_OPTIONS ${PANEL_VARIABLE}")  # Last, so the panel's switch wins


def test_the_panels_service_runs_as_the_robots_user_and_can_gain_no_privilege():
    lines = set(render("ultraskelly-control.service").splitlines())
    assert {"User=robot", "NoNewPrivileges=yes", "AmbientCapabilities=CAP_NET_BIND_SERVICE",
            "CapabilityBoundingSet=CAP_NET_BIND_SERVICE", "ProtectSystem=strict", "Restart=always",
            "WantedBy=multi-user.target"} <= lines
    assert "ExecStart=/home/robot/ultraskelly/.venv/bin/python -m ultraskelly.control --port 80" in lines


def test_the_pi_screen_opens_the_panel_full_screen_and_keeps_devtools():
    opener = (ROOT / "scripts" / "open-dashboard.sh").read_text(encoding="utf-8")
    chromium = opener[opener.index("exec chromium"):].split()
    assert 'url="${1:-http://localhost/}"' in opener and '--app="$url"' in chromium  # The panel, as an app window
    # Full screen is asked for once the window is up; maximized from the start still fills the screen if that is lost.
    assert {"--start-fullscreen", "--start-maximized"} <= set(chromium)
    assert not any(flag.startswith("--kiosk") for flag in chromium)  # Kiosk mode turns DevTools off


def test_the_pi_screen_window_keeps_the_whole_screen_out_of_full_screen(tmp_path):
    # 2026-11-07, the user: "why is the dashboard view in narrow mode?" Out of full screen the window fell back to
    # Chromium's default half screen, 945 px, under which the dashboard folds into its phone layout.
    opener = (ROOT / "scripts" / "open-dashboard.sh").read_text(encoding="utf-8")
    start = opener.index("\n", opener.index("<<'EOF'")) + 1
    sizing = opener[start:opener.index("\nEOF\n", start)]
    preferences = tmp_path / "Default" / "Preferences"
    preferences.parent.mkdir()
    preferences.write_text(json.dumps({
        "browser": {"app_window_placement": {"localhost_/": {"left": 10, "top": 10, "right": 955, "bottom": 1070}}},
        "profile": {"exit_type": "Normal"},
    }))
    whole = {"left": 0, "top": 0, "right": 1920, "bottom": 1080, "work_area_left": 0, "work_area_top": 0,
             "work_area_right": 1920, "work_area_bottom": 1080, "maximized": True}
    size = [sys.executable, "-c", sizing, str(preferences), "http://localhost/", "1920", "1080"]
    subprocess.run(size, check=True, timeout=30)
    saved = json.loads(preferences.read_text())
    assert saved["browser"]["app_window_placement"]["localhost_/"] == whole and saved["profile"] == {"exit_type": "Normal"}
    preferences.unlink()  # A first start: no preferences yet
    subprocess.run(size, check=True, timeout=30)
    assert json.loads(preferences.read_text())["browser"]["app_window_placement"]["localhost_/"] == whole


def test_the_desktop_icon_starts_the_bot_unless_it_runs_and_opens_the_dashboard():
    # 2026-11-07, the user: "a shortcut on the rpi desktop I can click and it starts the dashboard and bot".
    entry = (ROOT / "scripts" / "desktop" / "ultraskelly.desktop").read_text(encoding="utf-8")
    assert {"[Desktop Entry]", "Type=Application", "Terminal=false", "Exec=/home/robot/ultraskelly/scripts/start-ultraskelly.sh",
            "Icon=/home/robot/ultraskelly/scripts/desktop/ultraskelly.svg"} <= set(
        entry.replace("@REPO@", "/home/robot/ultraskelly").splitlines())
    assert (ROOT / "scripts" / "desktop" / "ultraskelly.svg").is_file()
    action = (ROOT / "scripts" / "start-ultraskelly.sh").read_text(encoding="utf-8")
    assert "if [[ $state == active || $state == activating ]]" in action  # A running bot is left alone
    assert "http://localhost/api/bot/start" in action  # Started through the control panel, as its Turn on button does
    assert action.rstrip().endswith("systemctl --user restart ultraskelly-dashboard.service")  # Then a fresh window
    listed = subprocess.run(["git", "ls-files", "-s", "scripts/start-ultraskelly.sh"], cwd=ROOT, capture_output=True, text=True)
    if listed.returncode == 0 and listed.stdout:  # Executable in the checkout the Pi pulls
        assert listed.stdout.startswith("100755")


# The rules file runs in a fresh context with only `polkit` in it, as polkitd runs it: nothing of node's leaks in.
POLKIT = r"""
const fs = require('fs'), vm = require('vm');
const rules = [];
const polkit = {Result: {YES: 'yes', NO: 'no', AUTH_ADMIN: 'auth_admin'}, addRule: rule => rules.push(rule), log: () => {}};
vm.runInNewContext(fs.readFileSync(process.argv[1], 'utf8'), {polkit}, {filename: process.argv[1]});
console.log(JSON.stringify(JSON.parse(process.argv[2]).map(([id, details, user]) => {
  const action = {id, lookup: key => details[key]};
  const subject = {user, local: true, active: false, isInGroup: () => false};
  for (const rule of rules) { const answer = rule(action, subject); if (answer !== undefined) return answer; }
  return null;
})));
"""


@pytest.mark.skipif(NODE is None, reason="needs node to run the polkit rule")
def test_the_polkit_rule_allows_the_panel_exactly_start_stop_and_restart_of_the_bot(tmp_path):
    source = render("ultraskelly-control.rules")
    assert not any(token in source for token in ("=>", "let ", "const ", "`"))  # ES5, which polkit's engine runs
    rule = tmp_path / "50-ultraskelly-control.rules"
    rule.write_text(source, encoding="utf-8")
    manage, bot = "org.freedesktop.systemd1.manage-units", "ultraskelly-bot.service"
    cases = [
        [manage, {"unit": bot, "verb": "start"}, "robot"],
        [manage, {"unit": bot, "verb": "stop"}, "robot"],
        [manage, {"unit": bot, "verb": "restart"}, "robot"],
        [manage, {"unit": bot, "verb": "kill"}, "robot"],
        [manage, {"unit": bot, "verb": "reload-or-restart"}, "robot"],
        [manage, {"unit": "ultraskelly-control.service", "verb": "stop"}, "robot"],
        [manage, {"unit": "ssh.service", "verb": "stop"}, "robot"],
        [manage, {"unit": bot, "verb": "start"}, "someone"],
        ["org.freedesktop.systemd1.manage-unit-files", {}, "robot"],  # enable, disable, mask
        ["org.freedesktop.login1.reboot", {}, "robot"],
    ]
    done = subprocess.run([NODE, "-e", POLKIT, str(rule), json.dumps(cases)], capture_output=True, encoding="utf-8",
                          check=True, timeout=60)
    assert json.loads(done.stdout) == ["yes", "yes", "yes", None, None, None, None, None, None, None]
