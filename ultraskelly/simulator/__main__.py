"""Run with python -m ultraskelly.simulator; serves only on loopback."""
import argparse
import asyncio
import json
import logging
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from pydantic import ValidationError

from .engine import Simulation

STATIC = Path(__file__).with_name("static")
ASSETS = {"/": ("index.html", "text/html"), "/viewer.js": ("viewer.js", "text/javascript"),
          "/style.css": ("style.css", "text/css")}
ASSETS["/hardware.html"] = ("hardware.html", "text/html")
ASSETS["/hardware.svg"] = ("hardware.svg", "image/svg+xml")
for name in ("three.module.js", "three.core.js", "OrbitControls.js"):
    ASSETS[f"/vendor/{name}"] = (f"vendor/{name}", "text/javascript")


class Runtime:
    """One asyncio owner for all controller state; HTTP requests cross via futures."""
    def __init__(self):
        self.loop = asyncio.new_event_loop()
        self.sim = Simulation()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.task = self.loop.create_task(self._pump())
        self.loop.run_forever()
        self.task.cancel()
        self.loop.run_until_complete(asyncio.gather(self.task, return_exceptions=True))
        self.sim.close()
        self.loop.close()

    async def _pump(self):
        while True:
            if self.sim.running:
                await self._advance(2)
            await asyncio.sleep(0.02)

    async def _advance(self, count):
        try:
            await self.sim.step(count)
        except Exception as exc:
            logging.exception("Simulation stopped")
            self.sim.close()
            self.sim.error = str(exc)

    async def dispatch(self, command=None):
        if command is not None:
            if not isinstance(command, dict) or set(command) - {"action", "settings"}:
                raise ValueError("Expected action and optional settings")
            action = command.get("action")
            if action not in ("run", "pause", "step", "reset", "configure"):
                raise ValueError("Unknown action")
            if "settings" in command:
                if action != "configure" or not isinstance(command["settings"], dict):
                    raise ValueError("Settings require configure action")
                self.sim.configure(command["settings"])
            if action == "run":
                if self.sim.error:
                    raise ValueError("Reset the simulator after an error")
                self.sim.running = True
            elif action == "pause":
                self.sim.running = False
            elif action == "step":
                if self.sim.error:
                    raise ValueError("Reset the simulator after an error")
                self.sim.running = False
                await self._advance(1)
            elif action == "reset":
                self.sim.reset()
        return self.sim.snapshot()

    def request(self, command=None):
        future = asyncio.run_coroutine_threadsafe(self.dispatch(command), self.loop)
        return future.result(timeout=5)

    def close(self):
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=5)


def make_server(runtime, port=8765):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, body, content_type="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", content_type + "; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            if self.path == "/api/state":
                self.reply(200, runtime.request())
            elif self.path in ASSETS:
                name, kind = ASSETS[self.path]
                self.reply(200, (STATIC / name).read_bytes(), kind)
            else:
                self.reply(404, {"error": "Not found"})

        def do_POST(self):
            if self.path != "/api/control":
                self.reply(404, {"error": "Not found"})
                return
            allowed = {f"http://127.0.0.1:{self.server.server_port}", f"http://localhost:{self.server.server_port}"}
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 8192:
                    raise ValueError("Invalid request size")
                body = self.rfile.read(size)
                if self.headers.get("Origin") not in allowed | {None}:
                    self.reply(403, {"error": "Local origin required"})
                    return
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("Use application/json")
                command = json.loads(body)
                self.reply(200, runtime.request(command))
            except (ValueError, ValidationError) as exc:
                self.reply(400, {"error": str(exc)})

        def log_message(self, *_):
            pass

    return ThreadingHTTPServer(("127.0.0.1", port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--open", action="store_true", help="Open the viewer in your browser")
    args = parser.parse_args()
    runtime = Runtime()
    try:
        server = make_server(runtime, args.port)
    except OSError as exc:
        parser.exit(1, f"Cannot start local viewer: {exc}\nTry --port 8766.\n")
    runtime.thread.start()
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"UltraSkelly simulator: {url}\nHardware disabled. Ctrl+C to stop.", flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
