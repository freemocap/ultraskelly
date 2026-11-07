"""Replay a recorded motor-response run through the simulated plant and measure how closely it matches.

The recording (tests/data/motor-response-20261106, from scripts/record_motor_response.py) holds every
command sent to the head's servos and the camera's view per frame: yaw (the view turning right is
positive), pitch (down is positive) and roll (the picture's content turning clockwise is positive),
chained from each frame's motion against the one before. `replay` drives a plant Axis with the same
commands at the same times and samples its view at the frame times; `segment_errors` compares the
two over each labelled segment, both taken from where they stood when it began (the chained views
drift over the whole run, not within a segment).
"""
import csv
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ultraskelly.simulator.plant import Axis, AxisModel

RECORDING = Path(__file__).resolve().parents[2] / "tests" / "data" / "motor-response-20261106"
VIEWS = {"pan": "yaw", "tilt": "pitch", "roll": "roll"}  # Each servo's view axis in the recording, same signs as Axis
ALIGN_FRAMES = 4  # Frames at a segment's start whose views it is aligned on


@dataclass
class Recording:
    times: np.ndarray  # Frame capture times, seconds
    views: dict[str, np.ndarray]  # Per servo: its view axis, degrees
    commands: list[tuple[float, str, float]]  # (time sent, actuator, value)
    segments: list[dict]  # name, start, end


def load(folder: Path = RECORDING) -> Recording:
    data = np.load(folder / "views.npz")
    commands = [(float(row["time"]), row["actuator"], float(row["value"])) for row in csv.DictReader(open(folder / "commands.csv"))]
    segments = json.loads((folder / "segments.json").read_text())
    return Recording(data["times"], {servo: data[view] for servo, view in VIEWS.items()}, commands, segments)


def replay(model: AxisModel, servo: str, recording: Recording, dt: float = 0.002) -> np.ndarray:
    """The plant's view of `servo` at every frame time, driven by the recorded commands."""
    commands = [(time, value) for time, actuator, value in recording.commands if actuator == servo]
    times = recording.times
    start = min(times[0], commands[0][0]) if commands else times[0]
    axis = Axis(model, start=commands[0][1] if commands and commands[0][0] <= times[0] else 90.0)
    steps = np.arange(start, times[-1] + dt, dt)
    views = np.empty(len(steps))
    sent = 0
    for i, now in enumerate(steps):
        while sent < len(commands) and commands[sent][0] <= now:
            axis.command(*commands[sent])
            sent += 1
        views[i] = axis.advance(now, dt)
    return np.interp(times, steps, views)


def segment_errors(predicted: np.ndarray, recording: Recording, servo: str) -> list[tuple[dict, np.ndarray]]:
    """Each of the servo's segments with the model's error over it, frame by frame, degrees."""
    measured, times = recording.views[servo], recording.times
    errors = []
    for segment in recording.segments:
        if not segment["name"].startswith(servo + " "):
            continue
        pick = np.flatnonzero((times >= segment["start"]) & (times <= segment["end"]))
        if len(pick) <= ALIGN_FRAMES:
            continue
        head = pick[:ALIGN_FRAMES]
        offset = np.median(measured[head]) - np.median(predicted[head])
        errors.append((segment, predicted[pick] + offset - measured[pick]))
    return errors


def rms(errors: list[tuple[dict, np.ndarray]]) -> float:
    joined = np.concatenate([error for _, error in errors])
    return float(np.sqrt(np.mean(joined ** 2)))
