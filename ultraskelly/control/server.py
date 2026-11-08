"""The control panel's web service: the dashboard page with its control column, and the API behind it.

    GET  /                       the dashboard; finding /api/panel, it shows the control column
    GET  /api/panel              {"bot_port", "local", "unit", "host"}: where the bot's stream is
    GET  /api/bot                {"bot": BotStatus, "last_error", "start": the options it starts with, "action", "time"}
    POST /api/bot/start, /api/bot/stop, /api/bot/restart     {}
    POST /api/follow             {"following": bool} and/or {"axes": {"pan": bool, ...}} (pan, tilt, roll, waist):
                                 kept for every start, and switched in the running bot
    GET  /api/motor-test, POST /api/motor-test, POST /api/motor-test/stop: passed to the bot
    GET  /api/tuning, POST /api/tuning, POST /api/tuning/reset, POST /api/tuning/save: passed to the bot
    POST /api/align              {}: aligns the OAK to the ZED on the person in front of the robot; passed to the bot

Every POST must send JSON and reach the panel by the robot's own name or address, so another
web page can neither send one nor reach this service through a name of its own (DNS rebinding).
"""
import asyncio
import ipaddress
import json
import logging
import socket
import time
from dataclasses import asdict

from aiohttp import ClientError, ClientSession, ClientTimeout, web

from ultraskelly.control.bots import VERBS, ActionFailed, BotStatus, last_error
from ultraskelly.control.options import AXES, StartOptions
from ultraskelly.ui.dashboard_assets import ASSETS

logger = logging.getLogger(__name__)

FOLLOW_USAGE = 'Send {"following": true or false}, {"axes": {"waist": false}} naming any of pan, tilt, roll and waist, or both'


def follow_change(body) -> tuple[bool | None, dict[str, bool] | None] | None:
    """The switches a POST /api/follow asks for, (following, joints by name), or None when it asks for nothing
    or for anything else: true and false only, known joints only."""
    if not isinstance(body, dict) or not body or set(body) - {"following", "axes"}:
        return None
    on, axes = body.get("following"), body.get("axes")
    if (on is not None and not isinstance(on, bool)) or "following" in body and on is None:
        return None
    if "axes" in body and (not isinstance(axes, dict) or not axes or set(axes) - set(AXES)
                           or not all(isinstance(value, bool) for value in axes.values())):
        return None
    return on, axes


def change_words(on: bool | None, axes: dict[str, bool] | None) -> str:
    """The switches in words: "Following off; waist off", "Pan and tilt on"."""
    def listed(names: list[str]) -> str:
        return names[0] if len(names) == 1 else f"{', '.join(names[:-1])} and {names[-1]}"
    parts = [] if on is None else [f"Following {'on' if on else 'off'}"]
    for state in (True, False):
        names = [name for name in AXES if (axes or {}).get(name) is state]
        if names:
            parts.append(f"{listed(names)} {'on' if state else 'off'}")
    text = "; ".join(parts)
    return text[:1].upper() + text[1:]


STATUS_TTL = 0.5  # Seconds one systemctl answer serves every page that asks (each asks once a second)
ERROR_TTL = 5.0  # Seconds before the log is read again, unless the bot's state changes first
BOT_TIMEOUT = ClientTimeout(total=2.0)  # The bot's motor switches answer at once, or the bot is not answering
TEST_TIMEOUT = ClientTimeout(total=60.0)  # A motor test answers once it has finished


class ControlServer:
    """`bot` is a SystemdBot or a LocalBot."""

    def __init__(
        self, bot, *, options: StartOptions, bot_port: int = 8080, hostname: str | None = None, local: bool = False,
        clock=time.time,
    ) -> None:
        self.bot = bot
        self.options = options
        self.bot_port = bot_port
        self.hostname = hostname or socket.gethostname()
        short = self.hostname.split(".")[0].lower()
        # The names a page may reach the panel by: this computer's own, never another domain's name for it.
        self.names = {"localhost", short, f"{short}.local", self.hostname.lower()}
        if hostname is None:
            self.names.add(socket.getfqdn().lower())
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
        app.router.add_get("/api/motor-test", self._to_bot)
        app.router.add_post("/api/motor-test", self._to_bot)
        app.router.add_post("/api/motor-test/stop", self._to_bot)
        app.router.add_get("/api/tuning", self._to_bot)
        for path in ("/api/tuning", "/api/tuning/reset", "/api/tuning/save", "/api/align"):
            app.router.add_post(path, self._to_bot)
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
        """Whether a request's Host names this robot: an address, localhost, or this computer's own name (alone, with
        .local, or as the computer itself spells it in full). A page that reached the panel through a name of its own
        (DNS rebinding) sends that name, and is refused."""
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
        return name.lower().rstrip(".") in self.names

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
            body = await request.json()
        except ValueError:
            body = None
        change = follow_change(body)
        if change is None:
            return web.json_response({"ok": False, "error": FOLLOW_USAGE}, status=400)
        on, axes = change
        try:
            if on is not None:
                self.options.set_following(on)
            if axes:
                self.options.set_axes(axes)
        except OSError as error:
            message = f"Could not keep the switch in {self.options.path}: {error}"
            return web.json_response({**self._did("follow", False, message), "error": message}, status=500)
        status = await self.status(fresh=True)
        applied = "next start"
        if status.state == "running":
            reply = await self.bot_post("/api/follow", {key: value for key, value in (("following", on), ("axes", axes)) if value is not None})
            if reply is not None and reply[0] == 200:
                applied = "now"
            elif reply is not None and reply[0] in (400, 404):  # A bot from before this switch: restart it with the new options
                await self.bot.act("restart")
                applied = "restart"
        elif status.state == "starting":  # Started with the old options: switch it once it is up
            self._spawn(self._switch_once_up())
            applied = "once started"
        words = change_words(on, axes)
        message = {"now": words, "restart": f"{words}: restarting the bot to apply it",
                   "once started": f"{words} once the bot has started",
                   "next start": f"{words} from the bot's next start"}[applied]
        return web.json_response({**self._did("follow", True, message), "following": self.options.follows(),
                                  "axes": self.options.axes(), "applied": applied})

    async def _switch_once_up(self) -> None:
        for _ in range(300):  # The bot has 2 min to start (TimeoutStartSec)
            await asyncio.sleep(0.5)
            status = await self.status()
            if status.state == "running":  # As switched last, should it have been switched again meanwhile
                await self.bot_post("/api/follow", {"following": self.options.follows(), "axes": self.options.axes()})
                return
            if status.state not in ("starting",):
                return

    # --- The bot's own routes ------------------------------------------------------------------

    def _bot_url(self, path: str) -> str:
        return f"http://127.0.0.1:{self.bot_port}{path}"

    async def bot_post(self, path: str, payload: dict) -> tuple[int, dict] | None:
        """POST to the bot; (status, reply) or None when it does not answer. A reply that is not JSON, such as an
        older bot's 404 for a route it does not have, still gives its status."""
        try:
            async with self.http.post(self._bot_url(path), json=payload, timeout=BOT_TIMEOUT) as reply:
                text = (await reply.read()).decode("utf-8", "replace")
                status = reply.status
        except (ClientError, TimeoutError, OSError):
            return None
        try:
            body = json.loads(text) if text else {}
        except ValueError:
            body = {"text": text}
        return status, body if isinstance(body, dict) else {"reply": body}

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
