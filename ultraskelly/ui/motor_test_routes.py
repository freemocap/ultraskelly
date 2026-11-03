"""HTTP commands for dashboard motor tests, beside the read-only /stream.

    GET  /api/motor-test       {"enabled", "limits", "running", "last", "waist_travel"}
    POST /api/motor-test       {"actuator": "pan"|"tilt", "angle": 100}
                               {"actuator": "waist"|"mouth", "throttle": 0.5, "seconds": 0.3}
    POST /api/motor-test/stop  {}

A test's reply comes once it has finished or been stopped: 200 with the test's record,
whose "outcome" is "done", "stopped" or "failed: ...". A test that will not run is answered
at once, with nothing moved: 400 if it is malformed or outside the test limits, 403 if tests
are turned off, 409 if another test is running. Requests are never queued. POSTs must send
JSON, so a page from another site cannot send them without the browser asking first.
"""
from aiohttp import web
from pydantic import ValidationError

from ultraskelly.core.bot.motor.motor_test import MotorTester, MotorTestRefused, MotorTestRequest


def add_motor_test_routes(app: web.Application, tester: MotorTester) -> None:
    async def state(request: web.Request) -> web.Response:
        return web.json_response(tester.state())

    async def run(request: web.Request) -> web.Response:
        if request.content_type != "application/json":
            return web.json_response({"ok": False, "error": "Send the test as JSON"}, status=415)
        try:
            test = MotorTestRequest.model_validate_json(await request.read())
        except ValidationError as error:
            reasons = "; ".join(f"{'.'.join(map(str, problem['loc'])) or 'request'}: {problem['msg']}" for problem in error.errors())
            return web.json_response({"ok": False, "error": reasons}, status=400)
        try:
            record = await tester.run(test)
        except MotorTestRefused as refused:
            status = 409 if refused.busy else 403 if not tester.limits.enabled else 400
            return web.json_response({"ok": False, "error": str(refused)}, status=status)
        return web.json_response({"ok": record["outcome"] in ("done", "stopped"), "test": record})

    async def stop(request: web.Request) -> web.Response:
        if request.content_type != "application/json":
            return web.json_response({"ok": False, "error": "Send the stop as JSON"}, status=415)
        return web.json_response({"ok": True, "stopped": await tester.stop()})

    app.router.add_get("/api/motor-test", state)
    app.router.add_post("/api/motor-test", run)
    app.router.add_post("/api/motor-test/stop", stop)
