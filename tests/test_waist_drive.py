import pytest

from ultraskelly.core.bot.motor.waist_drive import WaistDrive, WaistDriveParams, WaistState

TICK = 0.01  # The orientation controller's period
PARAMS = WaistDriveParams()


def run(drive: WaistDrive, offsets: list, start: float = 0.0) -> list:
    """Feed one head pan offset per tick; return the commands."""
    return [drive.update(now=start + index * TICK, pan_offset=offset) for index, offset in enumerate(offsets)]


def largest_step(commands: list) -> float:
    values = [command for command in commands if command is not None]
    return max(abs(b - a) for a, b in zip(values, values[1:]))


def test_a_move_starts_below_breakaway_and_ramps_without_pulses():
    drive = WaistDrive(PARAMS)
    commands = run(drive, [30.0] * 200)
    wanted = min(PARAMS.max_throttle, PARAMS.min_throttle + PARAMS.gain * (30.0 - PARAMS.start_angle))
    assert commands[0] == -PARAMS.start_throttle  # Positive pan turns the torso with negative drive
    assert all(command < 0 for command in commands)  # Two seconds of drive, never braked or released
    assert largest_step(commands[1:]) <= PARAMS.ramp_up * TICK + 1e-9
    assert commands[-1] == pytest.approx(-wanted)
    assert drive.state is WaistState.TURNING


def test_polarity_follows_the_head():
    assert run(WaistDrive(PARAMS), [-30.0])[0] == PARAMS.start_throttle
    assert run(WaistDrive(PARAMS), [30.0])[0] == -PARAMS.start_throttle


def test_hysteresis_starts_past_start_angle_and_turns_until_stop_angle():
    assert run(WaistDrive(PARAMS), [10.0] * 50) == [0.0] * 50  # Inside start_angle: never starts
    drive = WaistDrive(PARAMS)
    commands = run(drive, [16.0] * 20 + [10.0] * 50)
    assert all(command < 0 for command in commands)  # Keeps turning between the two angles
    assert commands[-1] == pytest.approx(-PARAMS.min_throttle)
    later = run(drive, [4.0] * 100, start=0.7)
    assert later[0] < 0  # Ramps down rather than stopping at once
    assert None in later and later[-1] == 0.0


def test_stopping_ramps_down_then_coasts_then_brakes():
    drive = WaistDrive(PARAMS)
    run(drive, [40.0] * 100)
    commands = run(drive, [None] * 100, start=1.0)  # Target lost
    ramp = [command for command in commands if command not in (None, 0.0)]
    assert ramp and largest_step(ramp) <= PARAMS.ramp_down * TICK + 1e-9
    assert abs(ramp[-1]) > PARAMS.release_throttle
    first_coast = commands.index(None)
    coast = commands[first_coast:].index(0.0)
    assert coast * TICK == pytest.approx(PARAMS.coast_duration, abs=TICK)
    assert all(command == 0.0 for command in commands[first_coast + coast:])
    assert drive.state is WaistState.IDLE


def test_reversing_stops_and_coasts_before_turning_the_other_way():
    drive = WaistDrive(PARAMS)
    commands = run(drive, [30.0] * 50 + [-30.0] * 100)
    signs = [None if command is None else command > 0 for command in commands if command != 0.0]
    first_positive = signs.index(True)
    assert None in signs[:first_positive]  # Released and coasting between the two directions
    assert PARAMS.start_throttle in commands  # The new direction starts low again


def test_a_long_move_rests_then_resumes():
    drive = WaistDrive(PARAMS)
    limit = int(round(PARAMS.max_move_duration / TICK))
    resume = limit + int(round(PARAMS.rest_duration / TICK))
    commands = run(drive, [30.0] * (resume + 10))
    assert all(command < 0 for command in commands[:limit])
    assert drive.state is WaistState.TURNING
    assert commands[resume - 1] == 0.0  # Braked while resting
    assert commands[resume] == -PARAMS.start_throttle


def test_resting_is_reported():
    drive = WaistDrive(PARAMS)
    run(drive, [30.0] * int(round((PARAMS.max_move_duration + 1.0) / TICK)))
    assert drive.state is WaistState.RESTING


def test_inconsistent_settings_are_rejected():
    with pytest.raises(ValueError):
        WaistDriveParams(start_angle=5.0, stop_angle=10.0)
    with pytest.raises(ValueError):
        WaistDriveParams(release_throttle=0.7, min_throttle=0.6)
    with pytest.raises(ValueError):
        WaistDriveParams(start_throttle=0.0)
