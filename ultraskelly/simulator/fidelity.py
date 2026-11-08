"""The simulated head against a recorded identification session, as the fidelity tests judge it.

A session (tests/data/motor-response-<date>, recorded by scripts/record_motor_response.py and measured by
scripts/analyze_motor_response.py) holds every command sent and the camera's turn in every frame, measured
against a fixed reference: yaw (the view turning right is positive), pitch (down is positive) and roll (the
picture's content turning clockwise is positive), in degrees of the camera's own pinhole (camera.json), which
`load` turns into the simulator's (plant.CAMERA_SCALE). `predict` runs a servo's plant over the parts kept out
of its fit (the validation staircases), each free from the rest before it, or over each step of the parts it
was fitted to, each from the view measured just before it (simulator/identify.py's shots). fits.json beside
the session holds each axis's view offset: where the reference frame caught the linkage within its slack.
"""
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ultraskelly.simulator.identify import Shots, part_shots, shots_for, simulate
from ultraskelly.simulator.plant import CAMERA_SCALE, AxisModel

DATA = Path(__file__).resolve().parents[2] / "tests" / "data"
RECORDING = DATA / "motor-response-20261107"
VIEWS = {"pan": "yaw", "tilt": "pitch", "roll": "roll"}  # Each servo's view axis, the same signs as plant.Axis
FITTED_PARTS = ("staircase", "ladder", "one way")
HELD_OUT_PARTS = ("validation",)


@dataclass
class Recording:
    times: np.ndarray  # Each frame's middle-row mid-exposure time, seconds
    views: dict[str, np.ndarray]  # Per servo: its view axis, degrees; nan where the frame was not registered
    ok: np.ndarray
    commands: list[tuple[float, str, float | None]]  # (time sent, actuator, value; None: a released motor)
    segments: list[dict]  # name, start, end
    offsets: dict[str, float]  # Per servo: the view with its linkage at the middle, as fitted


def load(folder: Path = RECORDING) -> Recording:
    data = np.load(folder / "views.npz")
    ok = data["ok"].astype(bool)
    with open(folder / "commands.csv", newline="") as file:
        commands = [(float(r["time"]), r["actuator"], None if r["value"] in ("", "None") else float(r["value"]))
                    for r in csv.DictReader(file)]
    fits = folder / "fits.json"
    offsets = json.loads(fits.read_text())["offsets"] if fits.exists() else {}
    scale = {"yaw": CAMERA_SCALE, "pitch": CAMERA_SCALE, "roll": 1.0}  # Into the simulator's degrees (plant.py)
    views = {servo: np.where(ok, data[view].astype(float) * scale[view], np.nan) for servo, view in VIEWS.items()}
    return Recording(data["times"], views, ok, commands, json.loads((folder / "segments.json").read_text()), offsets)


CROSS_DAY = DATA / "motor-response-20261106"  # The evening before: steps, staircases, ramps and big steps


def every_part(servo: str, recording: Recording) -> tuple[str, ...]:
    """The servo's parts, but the settling at its start."""
    return tuple(sorted({s["name"][len(servo) + 1:] for s in recording.segments
                         if s["name"].startswith(servo + " ") and "settle" not in s["name"]}))


def shots(servo: str, recording: Recording, *, held_out: bool, parts: tuple[str, ...] | None = None) -> Shots:
    """Held out: each part run free from the rest before it. Otherwise each step from the view before it."""
    make = part_shots if held_out else shots_for
    chosen = parts if parts is not None else (HELD_OUT_PARTS if held_out else FITTED_PARTS)
    return make(servo, recording.times, recording.views[servo], recording.ok, recording.commands, recording.segments,
                parts=chosen)


def predict(model: AxisModel, servo: str, recording: Recording, *, held_out: bool,
            parts: tuple[str, ...] | None = None) -> tuple[np.ndarray, np.ndarray]:
    """The plant's view and the measured one at the frames of the servo's held-out (or fitted) parts."""
    batch = shots(servo, recording, held_out=held_out, parts=parts)
    return simulate(model, batch, recording.offsets.get(servo, 0.0)), batch.measured
