"""The control panel's web service: the dashboard page with its control column, and the API behind it.

    GET  /                       the dashboard; finding /api/panel, it shows the control column
    GET  /api/panel              {"bot_port", "local", "unit", "host"}: where the bot's stream is
    GET  /api/bot                {"bot": BotStatus, "last_error", "start": the options it starts with, "action", "time"}
    POST /api/bot/start, /api/bot/stop, /api/bot/restart     {}
    POST /api/follow             {"following": bool}: kept for every start, and switched in the running bot
    POST /api/stop-motors        {}: every motor stops now, and following stays off across restarts
    GET  /api/motor-test, POST /api/motor-test, POST /api/motor-test/stop: passed to the bot

Every POST must send JSON and reach the panel by the robot's own name or address, so another
web page can neither send one nor reach this service through a name of its own (DNS rebinding).
"""
import asyncio
import ipaddress
import logging
import socket
import time
from dataclasses import asdict

from aiohttp import ClientError, ClientSession, ClientTimeout, web

from ultraskelly.control.bots import VERBS, ActionFailed, BotStatus, last_error
from ultraskelly.control.options import StartOptions
from ultraskelly.ui.dashboard_assets import ASSETS

logger = logging.getLogger(__name__)

STATUS_TTL = 0.5  # Seconds one systemctl answer serves every page that asks (each asks once a second)
ERROR_TTL = 5.0  # Seconds before the log is read again, unless the bot's state changes first
BOT_TIMEOUT = ClientTimeout(total=2.0)  # The bot's motor switches answer at once, or the bot is not answering
TEST_TIMEOUT = ClientTimeout(total=60.0)  # A motor test answers once it has finished
UP = ("running", "starting", "restarting", "waiting", "stopping")  # States in which the bot may yet drive a motor


class ControlServer:
    """`bot` is a SystemdBot or a LocalBot; `stop_motors` the command that brakes the DC motors directly, or None
    where there are no motor boards."""

    def __init__(
        self, bot, *, options: StartOptions, bot_port: int = 8080, stop_motors: list[str] | None = None,
        stop_motors_cwd: str | None = None, hostname: str | None = None, local: bool = False, clock=time.time,
    ) -> None:
        self.bot = bot
        self.options = options
        self.bot_port = bot_port
        self.stop_motors_command = stop_motors
        self.stop_motors_cwd = stop_motors_cwd
        self.hostname = hostname or socket.gethostname()
        self.local = local
        self.clock = clock
        self.http: ClientSession | None = None
        self.last_action: dict | None = None  # The newest start, stop, restart, switch or stop from a page
        self._status: BotStatus | None = None
        self._status_at = 0.0
        self._status_lock = asyncio.Lock()
        self._error: dict | None = None
        self._error_at = 0.0
        self._error_key: tuple | None = None
        self._tasks: set[asyncio.Task] = set()

    def build_app(self) -> web.Application:
        app = web.Application(middlewares=[self._guard])
        for path in ASSETS:
            app.router.add_get(path, self._asset)
        app.router.add_get("/api/panel", self._panel)
        app.router.add_get("/api/bot", self._bot_status)
        app.router.add_post("/api/bot/{verb}", self._act)
        app.router.add_post("/api/follow", self._follow)
        app.router.add_post("/api/stop-motors", self._stop_motors)
        app.router.add_get("/api/motor-test", self._to_bot)
        app.router.add_post("/api/motor-test", self._to_bot)
        app.router.add_post("/api/motor-test/stop", self._to_bot)
        app.on_startup.append(self._open)
        app.on_cleanup.append(self._close)
        return app

    # --- Guard, pages ---------------------------------------------------------------------

    @web.middleware
    async def _guard(self, request: web.Request, handler):
        if request.method == "POST":
            if not self.trusted(request.host):
                return web.json_response({"ok": False, "error": "Open the panel by the robot's own name or address"}, status=403)
            if request.content_type != "application/json":
                return web.json_response({"ok": False, "error": "Send the command as JSON"}, status=415)
        return await handler(request)

    def trusted(self, host: str | None) -> bool:
        """Whether a request's Host names this robot: an address, localhost, or this computer's name with any domain."""
        name = (host or "").strip()
        if name.startswith("["):
            name = name[1:name.find("]")] if "]" in name else name[1:]
        elif name.count(":") == 1:
            name = name.split(":")[0]
        try:
            ipaddress.ip_address(name)
            return True
        except ValueError:
            pass
        label = name.split(".")[0].lower()
        return bool(label) and label in ("localhost", self.hostname.split(".")[0].lower())

    async def _asset(self, request: web.Request) -> web.Response:
        path, content_type = ASSETS[request.path]
        return web.Response(body=path.read_bytes(), content_type=content_type, charset="utf-8",
                            headers={"Cache-Control": "no-store"})

    async def _panel(self, request: web.Request) -> web.Response:
        return web.json_response({"bot_port": self.bot_port, "local": self.local, "host": self.hostname,
                                  "unit": getattr(self.bot, "unit", None)})

    # --- Status -----------------------------------------------------------------------------

    async def status(self, *, fresh: bool = False) -> BotStatus:
        async with self._status_lock:
            if fresh or self._status is None or self.clock() - self._status_at > STATUS_TTL:
                try:
                    self._status = await self.bot.status()
                except (ActionFailed, OSError) as error:
                    self._status = BotStatus(state="unknown", detail=str(error))
                self._status_at = self.clock()
            return self._status

    async def last_error(self, status: BotStatus) -> dict | None:
        key = (status.state, status.since)
        if key != self._error_key or self.clock() - self._error_at > ERROR_TTL:
            try:
                self._error = last_error(await self.bot.journal())
            except OSError:
                self._error = None
            self._error_key, self._error_at = key, self.clock()
        return self._error

    async def _bot_status(self, request: web.Request) -> web.Response:
        status = await self.status()
        return web.json_response({
            "bot": asdict(status), "last_error": await self.last_error(status), "start": self.options.describe(),
            "action": self.last_action, "time": self.clock(),
        })

    def _did(self, action: str, ok: bool, message: str) -> dict:
        self.last_action = {"action": action, "ok": ok, "message": message, "time": self.clock()}
        (logger.info if ok else logger.warning)(f"{action}: {message}")
        return self.last_action

    # --- Start, stop, restart ------------------------------------------------------------------

    async def _act(self, request: web.Request) -> web.Response:
        verb = request.match_info["verb"]
        if verb not in VERBS:
            return web.json_response({"ok": False, "error": f"No such command: {verb}"}, status=404)
        try:
            await self.bot.act(verb)
        except ActionFailed as failed:
            return web.json_response({**self._did(verb, False, str(failed)), "error": str(failed)}, status=502)
        self._status_at = 0.0  # The next status shows the change
        return web.json_response(self._did(verb, True, {"start": "Starting the bot", "stop": "Stopping the bot",
                                                         "restart": "Restarting the bot"}[verb]))

    # --- Following --------------------------------------------------------------------------

    async def _follow(self, request: web.Request) -> web.Response:
        try:
            on = (await request.json())["following"]
        except (ValueError, KeyError, TypeError):
            on = None
        if not isinstance(on, bool):
            return web.json_response({"ok": False, "error": 'Send {"following": true} or {"following": false}'}, status=400)
        try:
            self.options.set_following(on)
        except OSError as error:
            message = f"Could not keep the switch in {self.options.path}: {error}"
            return web.json_response({**self._did("follow", False, message), "error": message}, status=500)
        status = await self.status(fresh=True)
        applied = "next start"
        if status.state == "running":
            reply = await self.bot_post("/api/follow", {"following": on})
            if reply is not None and reply[0] == 200:
                applied = "now"
            elif reply is not None and reply[0] == 404:  # A bot from before the live switch: restart it with the new option
                await self.bot.act("restart")
                applied = "restart"
        elif status.state == "starting":  # Started with the old option: switch it once it is up
            self._spawn(self._follow_once_up(on))
            applied = "once started"
        word = "on" if on else "off"
        message = {"now": f"Following {word}", "restart": f"Following {word}: restarting the bot to apply it",
                   "once started": f"Following {word} once the bot has started",
                   "next start": f"Following {word} from the bot's next start"}[applied]
        return web.json_response({**self._did("follow", True, message), "following": on, "applied": applied})

    async def _follow_once_up(self, on: bool) -> None:
        for _ in range(300):  # The bot has 2 min to start (TimeoutStartSec)
            await asyncio.sleep(0.5)
            status = await self.status()
            if status.state == "running":
                if self.options.follows() == on:  # Not switched again meanwhile
                    await self.bot_post("/api/follow", {"following": on})
                return
            if status.state not in ("starting",):
                return

    # --- Stop every motor -----------------------------------------------------------------------

    async def _stop_motors(self, request: web.Request) -> web.Response:
        notes = []
        try:
            self.options.set_following(False)  # Every start from now on holds still, until following is switched on
        except OSError as error:
            notes.append(f"following could not be kept off for the next start ({error})")
        status = await self.status(fresh=True)
        if status.state == "running":
            reply = await self.bot_post("/api/motors/stop", {})
            if reply is not None and reply[0] == 200:
                message = "All motors stopped: the head holds still, the waist and jaw are braked, following is off"
                return web.json_response({**self._did("stop motors", True, message), "how": "held", "notes": notes})
        # The bot did not confirm it (off, starting, not answering, or from before this switch): stop it, then brake
        # the DC motors directly. Its service brakes them again once it has stopped.
        stopped = False
        if status.state in UP:
            try:
                await self.bot.act("stop")
                stopped = True
            except ActionFailed as failed:
                notes.append(f"the bot could not be stopped: {failed}")
        braked, why = await self.brake()
        if not braked:
            notes.append(why)
        done = [*(["the bot is stopping"] if stopped else []), *(["the waist and jaw are braked"] if braked else [])]
        ok = stopped or braked or status.state in ("off", "failed", "missing")
        message = (f"All motors stopped: {' and '.join(done)}" if done else
                   "Nothing was running: the bot is off" if ok else "Could not stop the motors")
        self._status_at = 0.0
        return web.json_response({**self._did("stop motors", ok, message), "how": "stopped" if stopped else "braked",
                                  "notes": notes}, status=200 if ok else 502)

    async def brake(self) -> tuple[bool, str]:
        """Brake every DC motor on the motor board directly (scripts/stop_motors.py); it needs no robot lock."""
        if self.stop_motors_command is None:
            return False, "no motor boards here to brake directly"
        from ultraskelly.control.bots import launch

        try:
            process = await launch(*self.stop_motors_command, cwd=self.stop_motors_cwd,
                                   stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
            output, _ = await asyncio.wait_for(process.communicate(), 12)
        except (OSError, TimeoutError) as error:
            return False, f"the motor board could not be braked directly ({error!r})"
        lines = output.decode("utf-8", "replace").strip().splitlines()
        if process.returncode != 0:
            return False, f"the motor board could not be braked directly: {lines[-1] if lines else process.returncode}"
        return True, lines[-1] if lines else "braked"

    # --- The bot's own routes ------------------------------------------------------------------

    def _bot_url(self, path: str) -> str:
        return f"http://127.0.0.1:{self.bot_port}{path}"

    async def bot_post(self, path: str, payload: dict) -> tuple[int, dict] | None:
        """POST to the bot; (status, reply) or None when it does not answer."""
        try:
            async with self.http.post(self._bot_url(path), json=payload, timeout=BOT_TIMEOUT) as reply:
                body = await reply.json(content_type=None) if reply.content_length != 0 else {}
                return reply.status, body
        except (ClientError, TimeoutError, OSError, ValueError):
            return None

    async def probe(self) -> bool:
        """Whether the bot's server answers (a LocalBot's readiness)."""
        try:
            async with self.http.get(self._bot_url("/api/motor-test"), timeout=ClientTimeout(total=1.0)) as reply:
                return reply.status == 200
        except (ClientError, TimeoutError, OSError):
            return False

    async def _to_bot(self, request: web.Request) -> web.Response:
        post = request.method == "POST"
        try:
            async with self.http.request(
                request.method, self._bot_url(request.path_qs), data=await request.read() if post else None,
                headers={"Content-Type": request.content_type} if post else None, timeout=TEST_TIMEOUT,
            ) as reply:
                return web.Response(body=await reply.read(), status=reply.status, content_type=reply.content_type,
                                    headers={"Cache-Control": "no-store"})
        except (ClientError, TimeoutError, OSError):
            return web.json_response({"ok": False, "error": "The bot is not answering: it is off or still starting"}, status=503)

    # --- Lifetime ---------------------------------------------------------------------------

    def _spawn(self, coroutine) -> None:
        task = asyncio.create_task(coroutine)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    async def _open(self, app: web.Application) -> None:
        self.http = ClientSession()

    async def _close(self, app: web.Application) -> None:
        for task in list(self._tasks):
            task.cancel()
        await self.bot.close()
        await self.http.close()
