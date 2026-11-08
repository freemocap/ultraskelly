"""HTTP commands for dashboard motor tests and the control panel's following switch, beside the read-only /stream.

    GET  /api/motor-test       {"enabled", "limits", "running", "last", "waist_travel", "following"}
    POST /api/motor-test       {"actuator": "pan"|"tilt", "angle": 100}
                               {"actuator": "waist"|"mouth", "throttle": 0.5, "seconds": 0.3}
    POST /api/motor-test/stop  {}
    GET  /api/follow           {"following": true, "axes": {"pan": true, "tilt": true, "roll": true, "waist": true}}
    POST /api/follow           {"following": false}: the head and waist follow the target, or hold still
                               {"axes": {"waist": false}}: which joints follow it (any of pan, tilt, roll, waist)

A test's reply comes once it has finished or been stopped: 200 with the test's record,
whose "outcome" is "done", "stopped" or "failed: ...". A test that will not run is answered
at once, with nothing moved: 400 if it is malformed or outside the test limits, 403 if tests
are turned off, 409 if another test is running. Requests are never queued. POSTs must send
JSON, so a page from another site cannot send them without the browser asking first.

Following and the joints switched here last until the bot stops; the control panel
(ultraskelly/control) also writes them into the options the bot starts with, so a restart keeps them.
"""
from aiohttp import web
from pydantic import BaseModel, ConfigDict, ValidationError, model_validator

from ultraskelly.core.bot.motor.motor_test import MotorTester, MotorTestRefused, MotorTestRequest


class AxesSwitch(BaseModel):
    """Joints switched by name; the ones not named stay as they are."""
    model_config = ConfigDict(extra="forbid", strict=True)  # Known joints, true or false only
    pan: bool | None = None
    tilt: bool | None = None
    roll: bool | None = None
    waist: bool | None = None

    def named(self) -> dict[str, bool]:
        return self.model_dump(exclude_none=True)


class FollowRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)  # true or false only: no "no", 0 or "off"
    following: bool | None = None
    axes: AxesSwitch | None = None

    @model_validator(mode="after")
    def _says_something(self) -> "FollowRequest":
        if self.following is None and (self.axes is None or not self.axes.named()):
            raise ValueError('Send {"following": true or false}, or {"axes": {...}} naming pan, tilt, roll or waist')
        return self


def _reasons(error: ValidationError) -> str:
    return "; ".join(f"{'.'.join(map(str, problem['loc'])) or 'request'}: {problem['msg']}" for problem in error.errors())


def add_motor_test_routes(app: web.Application, tester: MotorTester) -> None:
    async def state(request: web.Request) -> web.Response:
        return web.json_response(tester.state())

    async def run(request: web.Request) -> web.Response:
        if request.content_type != "application/json":
            return web.json_response({"ok": False, "error": "Send the test as JSON"}, status=415)
        try:
            test = MotorTestRequest.model_validate_json(await request.read())
        except ValidationError as error:
            return web.json_response({"ok": False, "error": _reasons(error)}, status=400)
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

    def switches() -> dict:
        orientation = tester.orientation
        return {"following": orientation.following, "axes": orientation.axes.model_dump()}

    async def following(request: web.Request) -> web.Response:
        return web.json_response(switches())

    async def follow(request: web.Request) -> web.Response:
        if request.content_type != "application/json":
            return web.json_response({"ok": False, "error": "Send the switch as JSON"}, status=415)
        try:
            switch = FollowRequest.model_validate_json(await request.read())
        except ValidationError as error:
            return web.json_response({"ok": False, "error": _reasons(error)}, status=400)
        if switch.axes is not None and switch.axes.named():
            tester.orientation.set_axes(**switch.axes.named())
        if switch.following is not None:
            tester.orientation.set_following(switch.following)
        return web.json_response({"ok": True, **switches()})

    app.router.add_get("/api/motor-test", state)
    app.router.add_post("/api/motor-test", run)
    app.router.add_post("/api/motor-test/stop", stop)
    app.router.add_get("/api/follow", following)
    app.router.add_post("/api/follow", follow)
