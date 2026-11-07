"""The open-loop identification session (scripts/record_motor_response.py) on a simulated robot: its plan
keeps every servo inside its window and the session short, and its waist stays near where it started."""
import bisect
import random

import pytest

from scripts import record_motor_response as session_script
from scripts.record_motor_response import (
    JITTER,
    LADDERS,
    WAIST_LIMIT,
    WINDOWS,
    Session,
    camera_settings,
    head_plan,
    plan_seconds,
    waist_plan,
)
from ultraskelly.simulator import plant


class FakeRobot:
    """The simulated waist on a fake clock; the camera reports its turn `latency` seconds late."""

    def __init__(self, sign: int = 1, latency: float = 0.25, silent_after: float = float("inf")) -> None:
        self.waist, self.sign = plant.Waist(plant.WAIST), sign
        self.time, self.latency, self.silent_after = 0.0, latency, silent_after
        self.drive: float | None = 0.0
        self.history: list[tuple[float, float]] = [(0.0, 0.0)]
        self.times: list[float] = [0.0]
        self.commands: list[tuple[float, str, float | None]] = []

    def now(self) -> float:
        return self.time

    def sleep(self, seconds: float) -> None:
        end = self.time + max(0.0, seconds)
        while self.time < end - 1e-12:
            step = min(0.01, end - self.time)
            drive = None if self.drive is None else self.sign * self.drive
            self.waist.advance(drive, step)
            self.time += step
            self.history.append((self.time, self.waist.angle))
            self.times.append(self.time)

    def send(self, name: str, value: float | None) -> None:
        self.commands.append((self.time, name, value))
        if name == "waist":
            self.drive = value
        else:
            assert WINDOWS[name][0] <= value <= WINDOWS[name][1], (name, value)

    def turn(self) -> tuple[float | None, float]:
        if self.time > self.silent_after:
            return None, float("inf")
        index = bisect.bisect_right(self.times, self.time - self.latency) - 1
        return (self.history[index][1] if index >= 0 else 0.0), self.latency


def test_the_head_plan_stays_in_its_windows_and_steps_every_size_both_ways():
    plan = head_plan(random.Random(1))
    for servo, part, steps in plan:
        low, high = WINDOWS[servo]
        assert all(low <= angle <= high for _, angle, _ in steps), (servo, part)
        assert all(0.0 <= wait <= JITTER for wait, _, _ in steps)
    assert max(angle for servo, _, steps in plan if servo == "tilt" for _, angle, _ in steps) <= 130.0
    for servo, _, steps in (entry for entry in plan if entry[1] == "ladder"):
        angles = [90.0] + [angle for _, angle, _ in steps]
        moves = {(round(abs(b - a)), b > a) for a, b in zip(angles, angles[1:])}
        assert all((size, way) in moves for size in LADDERS[servo] for way in (True, False)), servo


def test_the_session_is_short():
    seconds = plan_seconds(head_plan(random.Random(1)), waist_plan(0.8))
    recentring = 26 * 4.0  # At most this much per waist part, from the simulated robot below
    assert seconds + recentring + session_script.NOISE_FLOOR < 10 * 60


@pytest.mark.parametrize("sign", [1, -1])
def test_the_waist_stays_near_its_start_and_finds_its_drive_sign(sign):
    robot = FakeRobot(sign=sign)
    session = Session(send=robot.send, turn=robot.turn, clock=robot)
    session.waist(waist_plan(0.8))
    assert session.waist_ok
    assert len([s for s in session.segments if s["name"].startswith("waist")]) == len(waist_plan(0.8))
    # Braked at the limit as the camera saw it, 0.25 s late at up to 50 degrees a second
    assert max(abs(angle) for _, angle in robot.history) < WAIST_LIMIT + 20.0
    assert abs(robot.waist.angle) < 20.0
    assert session.waist_way == sign  # Learned from the first part's turn
    assert robot.commands[-1] == (robot.commands[-1][0], "waist", 0.0)


def test_a_silent_camera_stops_the_waist_and_skips_its_remaining_parts():
    robot = FakeRobot(silent_after=30.0)
    session = Session(send=robot.send, turn=robot.turn, clock=robot)
    session.waist(waist_plan(0.8))
    assert not session.waist_ok
    assert len(session.segments) < len(waist_plan(0.8))
    stopped = [at for at, name, value in robot.commands if name == "waist" and value == 0.0 and at >= 30.0]
    assert stopped and stopped[0] <= 30.0 + 1.1  # Braked within a second of the last reading going stale
    assert robot.commands[-1][2] == 0.0


def test_manual_exposure_keeps_the_brightness_with_a_short_exposure():
    rows = [(0, 0, 0, 0, 0.016, 400, 120, 4500)] * 15  # Auto-exposure: 16 ms at ISO 400
    settings = camera_settings(rows)
    assert settings["exposure"] == pytest.approx(0.004) and settings["iso"] == 1600  # As bright, a quarter of the blur
    bright = camera_settings([(0, 0, 0, 0, 0.002, 200, 120, 4500)] * 15)
    assert bright["exposure"] == pytest.approx(0.002) and bright["iso"] == 200  # Already short: kept
    dim = camera_settings([(0, 0, 0, 0, 0.033, 1600, 120, 4500)] * 15)
    assert dim["iso"] == 1600 and dim["exposure"] == pytest.approx(0.010)  # As much light as 10 ms allows
