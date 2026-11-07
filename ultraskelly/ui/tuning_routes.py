"""HTTP routes for tuning the controllers from the dashboard's Tune window, beside the motor tests
(the settings, their checks and the saved file: ultraskelly/core/bot/motor/tuning.py).

    GET  /api/tuning           {"groups": [{"key", "title", "schema", "values", "changed", "unsaved"}],
                                "file", "saved_at", "problems"}
    POST /api/tuning           {"aim": {"deadband": 0.2}, "waist": {"enabled": false}}: set live, all or nothing
    POST /api/tuning/reset     {} every group to its defaults, {"group": "aim"} one group,
                               {"group": "aim", "setting": "deadband"} one setting; live
    POST /api/tuning/save      {}: keep the settings that differ from their defaults for the bot's next start

Each answers with "ok" and the state, as GET gives it: 200, or 400 with the reason when a change
does not fit the controller's settings (nothing changed then), 409 when there is nowhere to save,
415 when it is not JSON, 500 when the file cannot be written. POSTs must send JSON, so a page
from another site cannot send them without the browser asking first.
"""
import json

from aiohttp import web
from pydantic import BaseModel, ConfigDict, ValidationError

from ultraskelly.core.bot.motor.tuning import Tuning, TuningRefused


class ResetRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    group: str | None = None
    setting: str | None = None


class SaveRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


def _reasons(error: ValidationError) -> str:
    return "; ".join(f"{'.'.join(map(str, problem['loc'])) or 'request'}: {problem['msg']}" for problem in error.errors())


async def _body(request: web.Request, model: type[BaseModel] | None = None):
    """The request's JSON, checked by `model` when one is given."""
    if request.content_type != "application/json":
        raise TuningRefused("Send the change as JSON", status=415)
    raw = await request.read()
    try:
        return model.model_validate_json(raw) if model is not None else json.loads(raw)
    except ValidationError as error:
        raise TuningRefused(_reasons(error)) from None
    except ValueError:
        raise TuningRefused("Send the change as JSON") from None


def add_tuning_routes(app: web.Application, tuning: Tuning) -> None:
    def answer(status: int = 200, error: str | None = None, **more) -> web.Response:
        return web.json_response({"ok": error is None, **({"error": error} if error else {}), **more, **tuning.state()},
                                 status=status)

    async def state(request: web.Request) -> web.Response:
        return answer()

    async def change(request: web.Request) -> web.Response:
        try:
            return answer(changed=tuning.set(await _body(request)))
        except TuningRefused as refused:
            return answer(refused.status, str(refused))

    async def reset(request: web.Request) -> web.Response:
        try:
            wanted = await _body(request, ResetRequest)
            return answer(changed=tuning.reset(wanted.group, wanted.setting))
        except TuningRefused as refused:
            return answer(refused.status, str(refused))

    async def save(request: web.Request) -> web.Response:
        try:
            await _body(request, SaveRequest)
            await tuning.save()
        except TuningRefused as refused:
            return answer(refused.status, str(refused))
        except OSError as error:
            return answer(500, f"Could not write {tuning.file}: {error}")
        return answer()

    app.router.add_get("/api/tuning", state)
    app.router.add_post("/api/tuning", change)
    app.router.add_post("/api/tuning/reset", reset)
    app.router.add_post("/api/tuning/save", save)
