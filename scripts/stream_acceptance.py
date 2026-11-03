"""Run FreeMoCap's consumer acceptance checks against a live /stream in headless Chromium.

    python -m scripts.stream_acceptance --stream ws://127.0.0.1:8080/stream
    python -m scripts.stream_acceptance --stream ws://127.0.0.1:8081/stream --people 3  # replay --serve --people 3

Serves scripts/stream_acceptance.html and the vendored @freemocap/client bundle over plain HTTP
from this machine's network address, so the page is not a secure context, opens it in headless
Chromium and prints the page's results as JSON: each check, the live rates, and the image path's
and the protocol's per-frame times. Exits 1 when a check that applies fails.

Chromium gets a throwaway profile, `--password-store=basic` (a desktop session would otherwise ask
for the keyring password on the robot's screen) and no proxy lookups. It needs no robot lock: it
only reads the stream, so whatever serves the stream decides what hardware runs.
"""
import argparse
import asyncio
import json
import shutil
import socket
import subprocess
import sys
import tempfile
from pathlib import Path

import aiohttp
from aiohttp import web

REPO = Path(__file__).resolve().parents[1]
VENDOR = REPO / "ultraskelly" / "ui" / "dashboard" / "vendor"
FILES: dict[str, tuple[Path, str]] = {
    "/": (REPO / "scripts" / "stream_acceptance.html", "text/html"),
    "/stream_acceptance.js": (REPO / "scripts" / "stream_acceptance.js", "text/javascript"),
    "/vendor/freemocap-client.js": (VENDOR / "freemocap-client.js", "text/javascript"),
    "/vendor/freemocap-client.js.map": (VENDOR / "freemocap-client.js.map", "application/json"),
}


def page_app() -> web.Application:
    """The acceptance page and the vendored client, with explicit content types."""
    async def serve(request: web.Request) -> web.Response:
        path, content_type = FILES[request.path]
        return web.Response(body=path.read_bytes(), content_type=content_type, charset="utf-8",
                            headers={"Cache-Control": "no-store"})

    app = web.Application()
    for route in FILES:
        app.router.add_get(route, serve)
    return app


def network_address() -> str:
    """This machine's address on its network (no packet is sent), or 127.0.0.1 without one."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.connect(("192.0.2.1", 9))
            return probe.getsockname()[0]
        except OSError:
            return "127.0.0.1"


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class DevTools:
    """The few Chrome DevTools Protocol calls the checks need, over one page's WebSocket."""

    def __init__(self, websocket: aiohttp.ClientWebSocketResponse) -> None:
        self.websocket = websocket
        self.calls = 0
        self.pending: dict[int, asyncio.Future] = {}
        self.console: list[dict] = []
        self.reading = asyncio.create_task(self._read())

    async def _read(self) -> None:
        async for message in self.websocket:
            data = json.loads(message.data)
            if data.get("id") in self.pending:
                self.pending.pop(data["id"]).set_result(data)
            elif data.get("method") == "Runtime.consoleAPICalled":
                self.console.append({"type": data["params"]["type"], "text": " ".join(
                    str(arg.get("value", arg.get("description", ""))) for arg in data["params"]["args"])})
            elif data.get("method") == "Runtime.exceptionThrown":
                details = data["params"]["exceptionDetails"]
                self.console.append({"type": "exception",
                                     "text": details.get("exception", {}).get("description", details.get("text"))})

    async def call(self, method: str, **params) -> dict:
        self.calls += 1
        future = asyncio.get_running_loop().create_future()
        self.pending[self.calls] = future
        await self.websocket.send_json({"id": self.calls, "method": method, "params": params})
        return await asyncio.wait_for(future, 60)

    async def evaluate(self, expression: str):
        reply = await self.call("Runtime.evaluate", expression=expression, returnByValue=True, awaitPromise=True)
        return reply.get("result", {}).get("result", {}).get("value")


async def run_checks(args: argparse.Namespace) -> dict:
    host = args.page_host or network_address()
    port = args.page_port or free_port()
    runner = web.AppRunner(page_app(), access_log=None)
    await runner.setup()
    await web.TCPSite(runner, host, port).start()
    query = f"stream={args.stream}&seconds={args.seconds}&frames={args.frames}&people={args.people}"
    url = f"http://{host}:{port}/?{query}"
    profile = tempfile.mkdtemp(prefix="stream-acceptance-chromium-")
    debug_port = free_port()
    browser = subprocess.Popen(
        [args.chromium, "--headless=new", f"--remote-debugging-port={debug_port}", f"--user-data-dir={profile}",
         "--password-store=basic", "--no-proxy-server", "--no-first-run", "--no-default-browser-check",
         "about:blank"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    try:
        async with aiohttp.ClientSession() as http:
            version = {}
            for _ in range(150):
                try:
                    async with http.get(f"http://127.0.0.1:{debug_port}/json/version") as response:
                        version = await response.json()
                        break
                except aiohttp.ClientError:
                    await asyncio.sleep(0.1)
            async with http.put(f"http://127.0.0.1:{debug_port}/json/new?about:blank") as response:
                target = await response.json()
            async with http.ws_connect(target["webSocketDebuggerUrl"], max_msg_size=0) as websocket:
                devtools = DevTools(websocket)
                await devtools.call("Runtime.enable")
                await devtools.call("Page.navigate", url=url)
                deadline = asyncio.get_running_loop().time() + args.seconds + 60
                while not await devtools.evaluate("Boolean(window.acceptance && window.acceptance.done)"):
                    if asyncio.get_running_loop().time() > deadline:
                        break
                    await asyncio.sleep(0.5)
                result = json.loads(await devtools.evaluate("JSON.stringify(window.acceptance ?? null)") or "null") or {}
                result["browser"] = version.get("Browser")
                result["console"] = devtools.console
                devtools.reading.cancel()
        return result
    finally:
        browser.terminate()
        try:
            browser.wait(timeout=10)
        except subprocess.TimeoutExpired:
            browser.kill()
        for _ in range(20):  # Chromium's helper processes can still be writing the profile
            shutil.rmtree(profile, ignore_errors=True)
            if not Path(profile).exists():
                break
            await asyncio.sleep(0.25)
        await runner.cleanup()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stream", default="ws://127.0.0.1:8080/stream", help="The /stream WebSocket to check")
    parser.add_argument("--people", type=int, default=0,
                        help="People the feed shows (replay --serve --people N); 0 for a real camera")
    parser.add_argument("--seconds", type=float, default=10.0, help="Length of the live window")
    parser.add_argument("--frames", type=int, default=120, help="Frames to time the image path and protocol on")
    parser.add_argument("--chromium", default="chromium", help="Chromium or Chrome executable")
    parser.add_argument("--page-host", help="Address to serve the page on (default: this machine's network address)")
    parser.add_argument("--page-port", type=int, default=0, help="Port to serve the page on (default: any free port)")
    parser.add_argument("--out", type=Path, help="Also write the results to this JSON file")
    args = parser.parse_args()

    result = asyncio.run(run_checks(args))
    text = json.dumps(result, indent=1)
    if args.out:
        args.out.write_text(text + "\n")
    print(text)
    failed = [name for name, check in result.get("checks", {}).items()
              if not check.get("pass") and check.get("applicable", True)]
    if not result.get("done"):
        failed.append("the page did not finish")
    print(f"\n{'FAILED: ' + ', '.join(failed) if failed else 'All applicable checks passed'}", file=sys.stderr)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
