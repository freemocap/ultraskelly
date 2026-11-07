import math

import pytest

from scripts.waist_sound_test import FOCAL_PX, PROFILES, onset_metrics, parse_profile
from ultraskelly.core.bot.motor.simulated_actuators import ActuatorCommand
from ultraskelly.core.bot.motor.waist_drive import WaistDrive, WaistDriveParams


def test_ramps_start_and_stop_at_their_levels():
    profile = parse_profile("ramp:start=0.45,stop=0.35,cruise=0.65")
    assert profile.throttle(0.0) == pytest.approx(0.45)
    assert profile.throttle(0.15) == pytest.approx(0.55)
    assert profile.throttle(0.5) == pytest.approx(0.65)
    assert profile.throttle(0.85) == pytest.approx(0.5)
    assert profile.throttle(0.95) == pytest.approx(0.4)
    assert profile.throttle(1.0) is None  # Released, not braked


def test_named_profiles_keep_their_shapes():
    assert PROFILES["ramp"].throttle(0.15) == pytest.approx(0.375)
    assert PROFILES["ramp"].throttle(0.85) == pytest.approx(0.375)
    current = PROFILES["current"]
    assert current.throttle(0.2) == 0.75
    assert current.throttle(0.6) == 0.0  # Braked like the controller's cooldown
    assert current.throttle(0.95) == 0.75
    assert current.throttle(1.4) == 0.0
    assert PROFILES["slow-55"].throttle(0.05) == 0.75
    assert PROFILES["slow-55"].throttle(0.5) == 0.55


def test_field_changes_are_checked():
    assert parse_profile("slow-55:cruise=0.5,coast=false").coast is False
    assert parse_profile("steady:slow_decay=true").slow_decay is True
    for text in ("missing", "steady:speed=2", "steady:seconds=10", "steady:pwm_hz=5000"):
        with pytest.raises(ValueError):
            parse_profile(text)


def test_onset_reports_where_the_waist_breaks_loose_and_stalls():
    # Throttle ramps 0 to 0.8 and back over 3 s; the camera turns only while it is at least 0.6.
    commands, shifts = [], []
    for step in range(301):
        t = step * 0.01
        commands.append(ActuatorCommand("waist", round(0.8 * min(t, 3.0 - t) / 1.5, 3), t))
    for frame in range(90):
        t = frame / 30
        level = 0.8 * min(t, 3.0 - t) / 1.5
        degrees_per_frame = 20 / 30 if level >= 0.6 else 0.0
        shifts.append((t, FOCAL_PX * math.tan(math.radians(degrees_per_frame)), 0.0, 0.5))
    onset = onset_metrics(shifts, commands, 0.0, 3.0)
    assert onset["moving_from"] == pytest.approx(0.6, abs=0.06)
    assert onset["moving_until"] == pytest.approx(0.6, abs=0.06)
    assert onset_metrics(shifts[:2], commands, 0.0, 3.0) == {}


def test_drive_profile_replays_the_bots_waist_drive():
    params, profile = WaistDriveParams(), PROFILES["drive"]
    drive, bot = WaistDrive(params), []
    for tick in range(len(profile.steps)):
        drive.advance(tick * 0.01)
        bot.append(drive.update(now=tick * 0.01, demand=-30.0 - drive.model.angle))
        drive.sent(bot[-1])
    bench = [profile.throttle(tick * 0.01) for tick in range(len(profile.steps))]
    assert bench == bot and profile.slow_decay == params.slow_decay
    moving = [value for value in bench if value]
    assert moving[0] == params.kick_throttle and None not in bench and bench[-1] == 0.0  # Kicks, runs, then brakes
    assert all(params.run_throttle <= value <= params.max_throttle for value in moving[round(params.kick_seconds / 0.01):])
    assert profile.throttle(profile.seconds + 1) == 0.0 and profile.seconds < 3.0
