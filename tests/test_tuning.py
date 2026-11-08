"""Tuning the controllers from the dashboard's Tune window (ultraskelly/core/bot/motor/tuning.py and
ultraskelly/ui/tuning_routes.py): the groups come from the controllers' own params, so settings added or renamed
appear by themselves; a change is checked by the controller's model and applied, all or nothing, to the running
controllers; saved settings are merged over the launch configuration at the next start; and the routes answer as the
motor tests' do. Moves nothing: the controllers drive simulated actuators."""
import asyncio
import json
import os
import shutil
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Literal

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from pydantic import Field, model_validator

from ultraskelly.__main__bot import launch_config
from ultraskelly.core.bot.base_abcs import NodeParams
from ultraskelly.core.bot.motor import tuning as tuning_module
from ultraskelly.core.bot.motor.tuning import GROUPS, Group, Tuning, TuningRefused, load_tuning, unit_of
from ultraskelly.simulator.engine import Settings, Simulation
from ultraskelly.ui.tuning_routes import add_tuning_routes


class Inner(NodeParams):
    """Its inner loop."""
    gain: float = Field(default=1.0, ge=0.0, le=5.0, description="Share of the error taken each step")


class Example(NodeParams):
    """An example controller's settings, of every kind the Tune window draws."""
    enabled: bool = Field(default=True, description="Run it")
    speed: float = Field(default=150.0, gt=0.0, description="Top servo speed, servo degrees per second")
    count: int = Field(default=5, ge=1, description="Samples in a row")
    direction: Literal[1, -1] = Field(default=1, description="+1: a larger angle turns it right")
    span: tuple[float, float] = Field(default=(10.0, 170.0), description="Servo degrees it may use")
    settle_lag: float = Field(default=0.05, ge=0.0, le=0.5, description="How far it trails its commands")
    angle: float = Field(default=1.0, json_schema_extra={"unit": "rad"})
    decay: bool = Field(default=True, json_schema_extra={"tuning": "start"})
    secret: float = Field(default=1.0, json_schema_extra={"tuning": False})
    motor_channel: int = Field(default=3, ge=0, le=15)
    low: float = Field(default=0.2, ge=0.0, le=1.0)
    high: float = Field(default=0.8, ge=0.0, le=1.0)
    inner: Inner = Field(default_factory=Inner, description="How its inner loop turns")
    wiring: str = Field(default="A")

    @model_validator(mode="after")
    def _ordered(self) -> "Example":
        if self.low > self.high:
            raise ValueError("low must not exceed high")
        return self


EXAMPLE = Group("example", "Example", ("example",), hidden=frozenset({"wiring"}), at_start=frozenset({"count"}))
OTHER = Group("other", "Other", ("other",))


def example_tuning(**options) -> tuple[SimpleNamespace, Tuning]:
    root = SimpleNamespace(example=Example(speed=100.0), other=Example())  # Launched with speed 100, not the class's 150
    return root, Tuning(root, groups=(EXAMPLE, OTHER), **options)


# --- The groups, from the controllers' params --------------------------------------------------------------------

def test_each_group_is_its_controllers_params_less_channels_and_what_runs_only_at_start():
    config = launch_config()
    state = Tuning(config).state()
    assert [group["key"] for group in state["groups"]] == ["aim", "roll", "waist", "jaw"]
    for group, found in zip(GROUPS, state["groups"]):
        params = tuning_module._find(config, group.path)
        expected = [name for name, info in type(params).model_fields.items()
                    if not name.endswith("_channel") and name not in group.hidden]
        assert list(found["schema"]["properties"]) == expected
        assert found["values"] == params.model_dump(mode="json", include=set(expected))
        assert found["changed"] == found["unsaved"] == []
    by_key = {group["key"]: group["schema"]["properties"] for group in state["groups"]}
    assert "enabled" in by_key["roll"] and "enabled" in by_key["waist"]  # The switches stay
    assert "startup_test" not in by_key["jaw"] and "mouth_motor_channel" not in by_key["jaw"]
    assert by_key["waist"]["slow_decay"]["tuning"] == "start"  # Set on the motor as it opens


def test_the_schema_carries_bounds_descriptions_units_and_the_launch_values_as_defaults():
    root, tuning = example_tuning()
    [group, _] = tuning.state()["groups"]
    properties = group["schema"]["properties"]
    assert list(properties) == ["enabled", "speed", "count", "direction", "span", "settle_lag", "angle", "decay", "low",
                                "high", "inner"]  # No channel, nothing marked or listed as not tuned
    assert properties["speed"]["default"] == 100.0  # The launch configuration's, which Reset returns to
    assert properties["speed"]["exclusiveMinimum"] == 0.0 and properties["speed"]["description"].startswith("Top servo")
    assert properties["speed"]["unit"] == "servo°/s"  # As its description says
    assert properties["settle_lag"]["unit"] == "s"  # As its name says
    assert properties["angle"]["unit"] == "rad"  # As it says itself
    assert properties["span"]["unit"] == "servo°" and properties["span"]["prefixItems"] == [{"type": "number"}] * 2
    assert properties["direction"]["enum"] == [1, -1] and "unit" not in properties["direction"]
    assert "unit" not in properties["enabled"]
    assert properties["decay"]["tuning"] == properties["count"]["tuning"] == "start"
    assert properties["inner"]["type"] == "object" and properties["inner"]["description"] == "How its inner loop turns"
    assert properties["inner"]["properties"]["gain"]["maximum"] == 5.0
    assert group["values"]["inner"] == {"gain": 1.0} and group["values"]["span"] == [10.0, 170.0]
    assert "$ref" not in json.dumps(group["schema"])  # Nested settings inline: the page reads no definitions


@pytest.mark.parametrize("name, description, unit", [
    ("servo_speed", "Top servo speed, servo degrees per second, slow enough", "servo°/s"),
    ("max_acceleration", "Degrees per second squared, speeding up and slowing down", "°/s²"),
    ("jerk", "Largest change of the ramp rate per second squared", "/s²"),
    ("max_speed", "Top turning speed of the view, degrees per second", "°/s"),
    ("degrees_per_second", "Estimated torso speed at full drive (18-23 deg/s at 0.75)", "°/s"),
    ("px_per_degree", "Image pixels per degree at the frame's centre", "px/°"),
    ("pan_ratio", "Degrees the view turns per pan servo degree: 0.261 fitted", "°/servo°"),
    ("pan_slack", "Servo degrees of play in the pan linkage", "servo°"),
    ("edge_margin", "Pixels from the picture's left or right edge", "px"),
    ("candidate_gap", "Seconds without a sighting after which", "s"),
    ("gain", "Extra running drive per degree of bearing", "/°"),
    ("ramp_up", "Largest drive increase per second", "/s"),
    ("servo_lag", "How far the camera trails the commands", "s"),
    ("head_settle", "A move starts only once the head has held still this long", "s"),
    ("lock_timeout", "", "s"),
    ("start_angle", "Target bearing from the body that starts a move", "°"),
    ("jump_degrees", "A goal further than this from the current one is a jump", "°"),
    ("smoothing", "Share of each new estimate taken into the goal", None),
    ("min_throttle", "Running drive with the target at start_angle", None),
    ("pan_mode", "The ringing mode: natural frequency (rad/s) and damping ratio", None),
])
def test_a_unit_is_what_the_description_or_the_name_says(name, description, unit):
    assert unit_of(name, description) == unit


# --- Changes: checked, all or nothing, applied in place ----------------------------------------------------------

def test_a_change_is_applied_to_the_running_params_objects_in_place():
    told = []
    root, tuning = example_tuning(on_change=told.append)
    params, inner = root.example, root.example.inner
    changed = tuning.set({"example": {"speed": 80, "span": [20, 160], "inner": {"gain": 2.5}, "direction": -1},
                          "other": {"enabled": False}})
    assert changed == ["example.speed", "example.direction", "example.span", "example.inner.gain", "other.enabled"]
    assert root.example is params and root.example.inner is inner  # The controllers hold these objects
    assert (params.speed, params.span, inner.gain, params.direction, root.other.enabled) == (80.0, (20.0, 160.0), 2.5, -1, False)
    assert isinstance(params.speed, float) and isinstance(params.span, tuple)
    assert told == [{"example", "other"}]
    assert tuning.set({"example": {"speed": 80}}) == [] and len(told) == 1  # Nothing new: nobody is told


@pytest.mark.parametrize("change, reason", [
    ({"example": {"speed": 0}}, "example.speed: Input should be greater than 0"),
    ({"example": {"count": 2.5}}, "example.count: Input should be a valid integer"),
    ({"example": {"speed": "80"}}, "example.speed: Input should be a valid number"),
    ({"example": {"enabled": 1}}, "example.enabled: Input should be a valid boolean"),
    ({"example": {"direction": 2}}, "example.direction: Input should be 1 or -1"),
    ({"example": {"span": [1, 2, 3]}}, "example.span"),
    ({"example": {"low": 0.9}}, "example: Value error, low must not exceed high"),
    ({"example": {"inner": {"gain": 6}}}, "example.inner.gain: Input should be less than or equal to 5"),
    ({"example": {"inner": 3}}, "example.inner: send its settings as an object"),
    ({"example": {"speeed": 80}}, "example.speeed: no such setting"),
    ({"example": {"motor_channel": 4}}, "example.motor_channel: not tuned here"),
    ({"example": {"wiring": "B"}}, "example.wiring: not tuned here"),
    ({"example": {"secret": 2.0}}, "example.secret: not tuned here"),
    ({"arm": {"speed": 80}}, "arm: no such group"),
    ({"example": {}}, "example: send its settings as an object"),
    ([], "Send the changes as"),
    ({"other": {"speed": 90}, "example": {"speed": -1}}, "example.speed"),  # The good half is not applied either
])
def test_a_change_that_does_not_fit_is_refused_with_the_reason_and_nothing_changes(change, reason):
    told = []
    root, tuning = example_tuning(on_change=told.append)
    before = (root.example.model_dump(), root.other.model_dump())
    with pytest.raises(TuningRefused) as refused:
        tuning.set(change)
    assert reason in str(refused.value) and refused.value.status == 400
    assert (root.example.model_dump(), root.other.model_dump()) == before and told == []


def test_reset_returns_a_setting_a_group_or_everything_to_the_launch_values():
    root, tuning = example_tuning()
    tuning.set({"example": {"speed": 80, "inner": {"gain": 2.0}}, "other": {"speed": 90}})
    assert tuning.reset("example", "inner.gain") == ["example.inner.gain"]
    assert root.example.inner.gain == 1.0 and root.example.speed == 80
    state = tuning.state()["groups"]
    assert state[0]["changed"] == ["speed"] and state[1]["changed"] == ["speed"]
    assert tuning.reset("example") == ["example.speed"] and root.example.speed == 100.0  # Its launch value
    assert tuning.reset() == ["other.speed"] and root.other.speed == 150.0
    for wrong, reason in [(("example", "speeed"), "no such setting"), (("arm", None), "no such group"),
                          ((None, "speed"), "Name the group")]:
        with pytest.raises(TuningRefused, match=reason):
            tuning.reset(*wrong)


# --- On the running controllers ------------------------------------------------------------------------------------

def test_the_controllers_take_a_change_at_their_next_step():
    async def scenario():
        sim = Simulation(Settings(scenario="manual", yaw=30, pitch=5))  # Someone well to the side: the waist turns
        tuning = Tuning(SimpleNamespace(orientation=sim.head.params, mouth=sim.mouth.params), on_change=sim.head.retune)
        tuning.set({"waist": {"enabled": False}, "jaw": {"lock_timeout": 0.5}, "aim": {"pan_settle": 0.4}})
        assert sim.head.waist_drive.params.enabled is False and sim.mouth.params.lock_timeout == 0.5
        assert sim.head.head_aim.params.pan_settle == 0.4
        throttles = []
        for _ in range(600):
            await sim.step()
            throttles.append(sim.head.waist_motor.throttle)
        assert set(throttles) <= {0.0, None}  # Off: braked
        tuning.set({"waist": {"enabled": True}})
        for _ in range(600):
            await sim.step()
            throttles.append(sim.head.waist_motor.throttle)
        assert any(throttle not in (0.0, None) for throttle in throttles[600:])  # On again: it turns toward them
        sim.close()
    asyncio.run(scenario())


def test_a_new_gearing_mid_turn_does_not_make_the_head_jump():
    async def jump(retune: bool) -> float:
        sim = Simulation(Settings(scenario="manual", yaw=25, pitch=5))
        tuning = Tuning(SimpleNamespace(orientation=sim.head.params, mouth=sim.mouth.params),
                        on_change=sim.head.retune if retune else None)
        for _ in range(1500):  # Until the head has turned well toward them and still turns
            await sim.step()
            if abs(sim.head.pan_angle - 90) > 12 and not sim.head.head_aim.settled[0]:
                break
        else:
            raise AssertionError("The head never turned")
        before = sim.head.pan_angle
        tuning.set({"aim": {"pan_ratio": sim.head.params.aim.pan_ratio / 2}})
        await sim.step()
        sim.close()
        return abs(sim.head.pan_angle - before)
    assert asyncio.run(jump(True)) < 2.0  # It carries on from the angle sent, as after a motor test
    assert asyncio.run(jump(False)) > 5.0  # Left alone, the new ratio maps its view to another servo angle at once


# --- Saved, and merged at the next start ---------------------------------------------------------------------------

def test_saved_settings_are_merged_over_the_launch_values_at_the_next_start(tmp_path):
    path = tmp_path / "config" / "ultraskelly" / "tuning.json"
    config = launch_config(tuning=path)  # No file yet: the launch values, and nowhere else to look
    assert config.tuning.file == str(path) and config.tuning.saved == {} and config.tuning.problems == []
    tuning = Tuning(config, config.tuning)
    launched = config.orientation.aim.pan_settle
    tuning.set({"aim": {"pan_settle": 0.25, "pan_range": [20, 160]}, "waist": {"enabled": False}})
    asyncio.run(tuning.save())
    saved = json.loads(path.read_text(encoding="utf-8"))
    assert saved["groups"] == {"aim": {"pan_settle": 0.25, "pan_range": [20.0, 160.0]}, "waist": {"enabled": False}}
    assert list(path.parent.iterdir()) == [path]  # Written whole: no draft left beside it

    again = launch_config(tuning=path)
    assert (again.orientation.aim.pan_settle, again.orientation.aim.pan_range, again.orientation.waist.enabled) == (0.25, (20.0, 160.0), False)
    assert again.tuning.defaults["aim"]["pan_settle"] == launched  # What Reset returns to: the launch value
    assert again.tuning.saved_at == saved["saved_at"]
    state = Tuning(again, again.tuning).state()
    assert [group["changed"] for group in state["groups"]] == [["pan_range", "pan_settle"], [], ["enabled"], []]
    assert all(group["unsaved"] == [] for group in state["groups"])  # The next start runs what is running


def test_a_saved_setting_that_no_longer_fits_is_skipped_and_the_rest_apply(tmp_path):
    path = tmp_path / "tuning.json"
    path.write_text(json.dumps({"saved_at": "2026-11-07T21:14:03", "groups": {
        "aim": {"pan_settle": 0.25, "renamed_since": 3, "pan_speed": 9999, "pan_channel": 4},
        "waist": {"run_throttle": 0.1},  # Under the stall: the waist's own rule refuses it
        "gone": {"x": 1},
    }}), encoding="utf-8")
    config = launch_config(tuning=path)
    assert config.orientation.aim.pan_settle == 0.25 and config.tuning.saved == {"aim": {"pan_settle": 0.25}}
    assert config.orientation.aim.pan_speed == launch_config().orientation.aim.pan_speed
    assert config.orientation.waist.run_throttle == launch_config().orientation.waist.run_throttle
    problems = "\n".join(config.tuning.problems)
    for expected in ("aim.renamed_since: no such setting", "aim.pan_speed: Input should be less than or equal to 150",
                     "aim.pan_channel: no such setting", "waist: Value error, Throttles must satisfy", "gone: no such group"):
        assert expected in problems
    assert Tuning(config, config.tuning).state()["problems"] == config.tuning.problems


@pytest.mark.parametrize("text", ["{not json", '["a list"]', '{"groups": 3}'])
def test_an_unreadable_file_starts_the_launch_values_and_says_so(tmp_path, text):
    path = tmp_path / "tuning.json"
    path.write_text(text, encoding="utf-8")
    config = launch_config(tuning=path)
    assert config.orientation == launch_config().orientation and config.mouth == launch_config().mouth
    assert len(config.tuning.problems) == 1 and "none of it applies" in config.tuning.problems[0]
    assert path.read_text(encoding="utf-8") == text  # Left alone until the next save


def test_a_failed_save_leaves_the_old_file_whole(tmp_path, monkeypatch):
    path = tmp_path / "tuning.json"
    config = launch_config(tuning=path)
    tuning = Tuning(config, config.tuning)
    tuning.set({"aim": {"pan_settle": 0.25}})
    asyncio.run(tuning.save())
    old = path.read_text(encoding="utf-8")
    tuning.set({"aim": {"pan_settle": 0.35}})

    def power_cut(*args):
        raise OSError("No space left on device")
    monkeypatch.setattr(tuning_module.os, "replace", power_cut)
    with pytest.raises(OSError):
        asyncio.run(tuning.save())
    assert path.read_text(encoding="utf-8") == old
    assert tuning.state()["groups"][0]["unsaved"] == ["pan_settle"]  # Still not saved


def test_saves_from_two_screens_at_once_are_written_one_after_the_other(tmp_path, monkeypatch):
    """The Pi's screen and a laptop may press Save together: both write the same draft beside the file, so the writes
    must not overlap, or the file renamed into place could hold a mix of the two."""
    config = launch_config(tuning=tmp_path / "tuning.json")
    tuning = Tuning(config, config.tuning)
    tuning.set({"aim": {"pan_settle": 0.25}})
    busy, overlapped, guard, write = [0], [], threading.Lock(), tuning_module.write_whole

    def slow_write(path, text):
        with guard:
            busy[0] += 1
            overlapped.append(busy[0] > 1)
        time.sleep(0.05)
        write(path, text)
        with guard:
            busy[0] -= 1
    monkeypatch.setattr(tuning_module, "write_whole", slow_write)

    async def both():
        await asyncio.gather(tuning.save(), tuning.save())
    asyncio.run(both())
    assert overlapped == [False, False]
    assert json.loads((tmp_path / "tuning.json").read_text(encoding="utf-8"))["groups"] == {"aim": {"pan_settle": 0.25}}
    assert [path.name for path in tmp_path.iterdir()] == ["tuning.json"]


def test_without_a_file_nothing_is_saved():
    root, tuning = example_tuning()
    tuning.set({"example": {"speed": 80}})
    with pytest.raises(TuningRefused, match="Nowhere to save") as refused:
        asyncio.run(tuning.save())
    assert refused.value.status == 409


# --- The routes ----------------------------------------------------------------------------------------------------

def serve(scenario, tuning: Tuning):
    async def wrapper():
        app = web.Application()
        add_tuning_routes(app, tuning)
        async with TestClient(TestServer(app)) as client:
            await scenario(client)
    asyncio.run(wrapper())


def test_the_routes_set_reset_and_save_and_answer_with_the_state(tmp_path):
    config = launch_config(tuning=tmp_path / "tuning.json")
    tuning = Tuning(config, config.tuning)

    async def scenario(client):
        state = await (await client.get("/api/tuning")).json()
        assert state["ok"] and [group["key"] for group in state["groups"]] == ["aim", "roll", "waist", "jaw"]
        assert state["file"] == str(tmp_path / "tuning.json") and state["saved_at"] is None
        reply = await client.post("/api/tuning", json={"aim": {"pan_settle": 0.25}, "jaw": {"lock_timeout": 0.5}})
        body = await reply.json()
        assert reply.status == 200 and body["changed"] == ["aim.pan_settle", "jaw.lock_timeout"]
        assert body["groups"][0]["changed"] == ["pan_settle"] and body["groups"][0]["unsaved"] == ["pan_settle"]
        reply = await client.post("/api/tuning/save", json={})
        assert reply.status == 200 and (await reply.json())["saved_at"]
        assert json.loads((tmp_path / "tuning.json").read_text(encoding="utf-8"))["groups"] == {
            "aim": {"pan_settle": 0.25}, "jaw": {"lock_timeout": 0.5}}
        reply = await client.post("/api/tuning/reset", json={"group": "jaw", "setting": "lock_timeout"})
        body = await reply.json()
        assert body["changed"] == ["jaw.lock_timeout"] and body["groups"][3]["unsaved"] == ["lock_timeout"]
        reply = await client.post("/api/tuning/reset", json={})
        assert (await reply.json())["changed"] == ["aim.pan_settle"] and config.orientation.aim.pan_settle == launch_config().orientation.aim.pan_settle
    serve(scenario, tuning)


def test_the_routes_refuse_what_does_not_fit_with_the_reason_and_the_state_as_it_is():
    root, tuning = example_tuning()

    async def scenario(client):
        for path, payload, status, reason in [
            ("/api/tuning", {"example": {"speed": -1}}, 400, "example.speed: Input should be greater than 0"),
            ("/api/tuning", {"example": {"motor_channel": 1}}, 400, "not tuned here"),
            ("/api/tuning/reset", {"group": "example", "extra": 1}, 400, "extra: Extra inputs are not permitted"),
            ("/api/tuning/reset", {"group": 3}, 400, "group: Input should be a valid string"),
            ("/api/tuning/save", {"now": True}, 400, "Extra inputs are not permitted"),
            ("/api/tuning/save", {}, 409, "Nowhere to save"),
        ]:
            reply = await client.post(path, json=payload)
            body = await reply.json()
            assert reply.status == status and reason in body["error"] and not body["ok"]
            assert body["groups"][0]["values"]["speed"] == 100.0  # The settings as they still are, for the page
        reply = await client.post("/api/tuning", data="example=1")  # A plain form, as any page could send
        assert reply.status == 415
        reply = await client.post("/api/tuning", data="{not json", headers={"Content-Type": "application/json"})
        assert reply.status == 400 and "Send the change as JSON" in (await reply.json())["error"]
    serve(scenario, tuning)


def test_the_replay_stand_in_keeps_its_tuning_where_it_is_told(tmp_path):
    """The control panel's local stand-in (python -m ultraskelly.replay --serve --tuning FILE) loads saved settings
    into its controllers as they run, in place."""
    from ultraskelly.replay import create_controllers
    path = tmp_path / "tuning.json"
    path.write_text(json.dumps({"groups": {"aim": {"pan_settle": 0.4}}}), encoding="utf-8")

    async def scenario():
        bus, orientation, mouth = create_controllers()
        settings = SimpleNamespace(orientation=orientation.params, mouth=mouth.params)
        started = load_tuning(settings, path)
        assert orientation.head_aim.params.pan_settle == 0.4 and orientation.head_aim.params is orientation.params.aim
        assert Tuning(settings, started).state()["groups"][0]["unsaved"] == []
        bus.close()
    asyncio.run(scenario())


def test_the_tuning_file_is_the_bot_users_own():
    assert tuning_module.TUNING_FILE.parts[-3:] == (".config", "ultraskelly", "tuning.json")
    assert str(tuning_module.TUNING_FILE).startswith(os.path.expanduser("~"))


# --- The page: tune.js under node, and the markup it uses ----------------------------------------------------------

ROOT = Path(__file__).resolve().parents[1]
DASHBOARD = ROOT / "ultraskelly" / "ui" / "dashboard"
NODE = shutil.which("node")
needs_node = pytest.mark.skipif(NODE is None, reason="needs node to run the dashboard's JavaScript")


def tune(name: str, *args):
    """tune.js's export `name` called with `args` under node (tests/js/dashboard_calls.mjs)."""
    done = subprocess.run([NODE, str(ROOT / "tests" / "js" / "dashboard_calls.mjs"), str(DASHBOARD),
                           json.dumps([["tune.js", name, list(args)]])], capture_output=True, encoding="utf-8", check=True,
                          timeout=60)
    return json.loads(done.stdout)[0]


@needs_node
def test_the_window_draws_each_setting_by_its_schema():
    root, tuning = example_tuning()
    tuning.set({"example": {"speed": 80, "inner": {"gain": 2.0}}})
    [group, _] = tuning.state()["groups"]
    entries = {entry["key"]: entry for entry in tune("settings", group)}
    assert list(entries) == ["enabled", "speed", "count", "direction", "span", "settle_lag", "angle", "decay", "low", "high",
                             "inner", "inner.gain"]
    assert {key: entry["kind"] for key, entry in entries.items()} == {
        "enabled": "switch", "speed": "number", "count": "number", "direction": "choice", "span": "numbers",
        "settle_lag": "number", "angle": "number", "decay": "switch", "low": "number", "high": "number",
        "inner": "heading", "inner.gain": "number"}
    speed = entries["speed"]  # Above 0, open above: up to a round number three times its default, 100
    assert (speed["min"], speed["max"], speed["step"], speed["value"], speed["fallback"]) == (1, 500, 1, 80, 100)
    assert speed["label"] == "Speed" and speed["unit"] == "servo°/s" and speed["changed"] and speed["unsaved"]
    count = entries["count"]
    assert (count["min"], count["max"], count["step"], count["integer"]) == (1, 20, 1, True)
    assert count["start"] and entries["decay"]["start"] and not speed["start"]
    lag = entries["settle_lag"]
    assert (lag["min"], lag["max"], lag["step"], lag["label"], lag["unit"]) == (0, 0.5, 0.001, "Settle lag", "s")
    assert entries["direction"]["options"] == [1, -1] and entries["span"]["integer"] == [False, False]
    assert entries["inner"]["label"] == "Inner" and entries["inner"]["description"] == "How its inner loop turns"
    assert entries["inner.gain"]["path"] == ["inner", "gain"] and entries["inner.gain"]["changed"]
    assert not entries["enabled"]["changed"] and entries["enabled"]["value"] is True
    assert tune("tipOf", speed) == "Top servo speed, servo degrees per second\nDefault 100 servo°/s"
    assert tune("tipOf", entries["decay"]) == "Default On\nApplies when the bot next starts"


@needs_node
def test_every_setting_the_bot_tunes_has_a_control_within_its_bounds():
    for group in Tuning(launch_config()).state()["groups"]:
        entries = [entry for entry in tune("settings", group) if entry["kind"] != "heading"]
        assert [entry["key"] for entry in entries] == list(tuning_module._flat(group["values"]))
        for entry in entries:
            assert entry["kind"] != "fixed", f"{group['key']}.{entry['key']} has no control"  # A kind the page cannot draw
            assert entry["label"]
            if entry["kind"] == "number":
                assert entry["min"] <= entry["value"] <= entry["max"] and entry["step"] > 0, (group["key"], entry)


@needs_node
@pytest.mark.parametrize("prop, value, expected", [
    ({"type": "number", "minimum": 0, "maximum": 10}, 0.12, {"min": 0, "max": 10, "step": 0.01}),
    ({"type": "number", "exclusiveMinimum": 0, "maximum": 1}, 0.69, {"min": 0.001, "max": 1, "step": 0.001}),
    ({"type": "number", "exclusiveMinimum": 0, "default": 8.1}, 8.1, {"min": 0.01, "max": 25, "step": 0.01}),
    ({"type": "number", "exclusiveMinimum": 0, "default": 150}, 1000, {"min": 1, "max": 2000, "step": 1}),  # Widened to show it
    ({"type": "number", "minimum": 0, "unit": "servo°", "default": 5}, 5, {"min": 0, "max": 180, "step": 0.1}),
    ({"type": "integer", "minimum": 1, "default": 5}, 5, {"min": 1, "max": 20, "step": 1}),
    ({"type": "number", "default": -2}, -2, {"min": -10, "max": 10, "step": 0.01}),  # Open both ways: either side of 0
])
def test_a_slider_spans_the_bounds_or_a_round_number_past_the_default(prop, value, expected):
    assert tune("sliderOf", prop, value) == pytest.approx(expected)


@needs_node
def test_a_kind_the_window_does_not_know_is_shown_as_its_value():
    assert tune("kindOf", {"type": "object", "additionalProperties": {"type": "number"}}, {"a": 1}) == {"kind": "fixed"}
    assert tune("kindOf", {"anyOf": [{"type": "number"}, {"type": "null"}]}, None) == {"kind": "fixed"}


@needs_node
def test_the_status_line_says_what_is_not_saved_or_was_refused():
    def state(unsaved=0, changed=0, file="/home/robot/.config/ultraskelly/tuning.json", saved_at=None, problems=()):
        return {"groups": [{"unsaved": ["x"] * unsaved, "changed": ["x"] * changed}], "file": file, "saved_at": saved_at,
                "problems": list(problems)}
    today = "2026-11-07"
    assert tune("statusOf", state(), None, today) == {
        "text": "", "kind": "", "title": "Saved in /home/robot/.config/ultraskelly/tuning.json", "save": False, "reset": False}
    assert tune("statusOf", state(unsaved=3, changed=3), None, today)["text"] == "3 changes not saved"
    assert tune("statusOf", state(unsaved=1, changed=1), None, today)["save"] is True
    assert tune("statusOf", state(saved_at="2026-11-07T21:14:03", changed=2), None, today)["text"] == "Saved 21:14"
    assert tune("statusOf", state(saved_at="2026-11-06T21:14:03"), None, today)["text"] == "Saved 2026-11-06 21:14"
    assert tune("statusOf", state(file=None, unsaved=2, changed=2), None, today) == {  # Lost at the next start: amber
        "text": "Live only: nowhere to save", "kind": "unsaved", "title": "", "save": False, "reset": True}
    refused = tune("statusOf", state(unsaved=1, problems=["aim.x: no such setting; skipped"]), "aim.deadband: too big", today)
    assert refused["text"] == "aim.deadband: too big · 1 saved setting not applied · 1 change not saved"
    assert refused["kind"] == "refused" and refused["title"].startswith("aim.x: no such setting; skipped\nSaved in")


def test_the_page_has_the_tune_key_and_window_the_scripts_use():
    page = (DASHBOARD / "index.html").read_text(encoding="utf-8")
    for part in ('id="control-tune"', '<dialog id="tune"', 'class="tune-groups"', 'class="tune-status"', 'class="tune-save',
                 'class="tune-reset"', 'class="tune-close"'):
        assert part in page
    assert page.index('id="control-follow"') < page.index('id="control-tune"') < page.index('id="control-parts"')
    script = (DASHBOARD / "panel.js").read_text(encoding="utf-8")
    assert "import { Tune } from '/tune.js'" in script
    assert "this.$('control-tune').after(document.querySelector('.card.tests'))" in script
