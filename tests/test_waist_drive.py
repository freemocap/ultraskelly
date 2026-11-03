import pytest

from ultraskelly.core.bot.motor.waist_drive import WaistDrive, WaistDriveParams, WaistState, s_curve_step

TICK = 0.01  # The orientation controller's period
PARAMS = WaistDriveParams()


def run(drive: WaistDrive, offsets: list, start: float = 0.0) -> list:
    """Feed one head pan offset per tick; return the commands."""
    return [drive.update(now=start + index * TICK, pan_offset=offset) for index, offset in enumerate(offsets)]


def closing(start: float, speed: float, ticks: int, floor: float = 0.0) -> list:
    """Head offsets of a head coming back toward neutral at `speed` degrees per second."""
    return [max(floor, start - speed * index * TICK) for index in range(ticks)]


def steps(values: list) -> list:
    return [b - a for a, b in zip(values, values[1:])]


def test_s_curve_ramps_limit_rate_and_jerk_and_settle_exactly():
    level, rate, levels = 0.15, 0.0, [0.15]
    for _ in range(100):
        level, rate = s_curve_step(level, rate, 0.6, up=1.5, down=2.0, jerk=10.0, dt=TICK)
        levels.append(level)
    rates = [step / TICK for step in steps(levels)]
    assert levels[-1] == 0.6 and max(rates) <= 1.5 + 1e-9 and min(rates) >= 0  # Never overshoots
    assert max(abs(r) for r in steps(rates[:-1])) <= 10.0 * TICK + 1e-9  # Rate changes no faster than the jerk limit
    assert rates[0] <= 10.0 * TICK + 1e-9  # Starts gently


def test_a_move_starts_low_and_eases_up_without_pulses():
    drive = WaistDrive(PARAMS)
    commands = run(drive, closing(30.0, 4.0, 150))  # The head comes back slowly: the waist is moving
    assert commands[0] == -PARAMS.start_throttle  # Positive pan turns the torso with negative drive
    assert all(command < 0 for command in commands)  # One move, never braked or released
    assert max(abs(step) for step in steps(commands)) <= PARAMS.ramp_up * TICK + 1e-9
    assert max(abs(c) for c in commands) <= drive.running_throttle(30.0) + 1e-9  # No breakaway extra: it moved
    assert drive.state is WaistState.TURNING


def test_polarity_follows_the_head():
    assert run(WaistDrive(PARAMS), [-30.0])[0] == PARAMS.start_throttle
    assert run(WaistDrive(PARAMS), [30.0])[0] == -PARAMS.start_throttle


def test_hysteresis_starts_past_start_angle_and_a_stopping_move_does_not_resume():
    assert run(WaistDrive(PARAMS), [10.0] * 50) == [0.0] * 50  # Inside start_angle: never starts
    drive = WaistDrive(PARAMS)
    commands = run(drive, closing(16.0, 10.0, 100, floor=10.0))
    assert all(command < 0 for command in commands)  # Keeps turning between the two angles
    later = run(drive, [4.0] * 5 + [10.0] * 95, start=1.0)  # Centered, then the head drifts back out a little
    assert later[0] < 0  # Eases down rather than stopping at once
    assert None in later and later[-1] == 0.0  # Still stops: no hunting between 5 and 15 degrees
    assert all(command in (None, 0.0) or abs(command) <= abs(later[0]) + 0.02 for command in later)


def test_stopping_ramps_down_then_coasts_then_brakes():
    drive = WaistDrive(PARAMS)
    run(drive, closing(40.0, 10.0, 100))
    commands = run(drive, [None] * 100, start=1.0)  # Target lost
    ramp = [command for command in commands if command not in (None, 0.0)]
    assert ramp and max(abs(step) for step in steps(ramp)) <= PARAMS.ramp_down * TICK + 1e-9
    first_coast = commands.index(None)
    coast = commands[first_coast:].index(0.0)
    assert coast * TICK == pytest.approx(PARAMS.coast_duration, abs=1.5 * TICK)
    assert all(command == 0.0 for command in commands[first_coast + coast:])
    assert drive.state is WaistState.IDLE


def test_a_stuck_waist_gets_breakaway_drive_that_eases_off_once_it_moves():
    drive = WaistDrive(PARAMS)
    stuck = run(drive, [20.0] * 150)  # The head shows no progress: the waist is not turning
    running = drive.running_throttle(20.0)
    assert abs(stuck[-1]) > running + 0.1  # Drive rose past the running level
    assert abs(stuck[-1]) <= PARAMS.breakaway_throttle
    moving = run(drive, closing(20.0, 15.0, 80, floor=6.0), start=1.5)  # It broke loose; the head comes back
    assert abs(moving[-1]) < abs(stuck[-1]) - 0.1  # The extra eases off again


def test_reversing_stops_and_coasts_before_turning_the_other_way():
    drive = WaistDrive(PARAMS)
    commands = run(drive, closing(30.0, 4.0, 50) + [-30.0] * 150)
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
        WaistDriveParams(release_throttle=0.6, min_throttle=0.5)
    with pytest.raises(ValueError):
        WaistDriveParams(max_throttle=0.9)  # Above the breakaway ceiling
    with pytest.raises(ValueError):
        WaistDriveParams(start_throttle=0.0)
