"""The waist's planned moves and its model, on their own: no head, no camera."""
import pytest

from ultraskelly.core.bot.motor.waist_drive import WaistDrive, WaistDriveParams, WaistModel, WaistState

TICK = 0.01  # The orientation controller's period
PARAMS = WaistDriveParams()


def follow(drive: WaistDrive, person: float, ticks: int, start: float = 0.0, **head) -> list:
    """Tick the drive with a still person `person` degrees off the body's front as the waist's own model turns it,
    as a perfect picture would show; `head` passes the head's offset and settled state. Returns the commands."""
    commands = []
    for tick in range(ticks):
        now = start + tick * TICK
        drive.advance(now)
        command = drive.update(now=now, demand=person - drive.model.angle, **head)
        drive.sent(command)
        commands.append(command)
    return commands


def test_the_model_breaks_loose_only_past_breakaway_and_turns_its_own_speed_each_way():
    for drive, moved in ((-0.6, False), (-0.7, True), (0.7, True)):
        model = WaistModel(PARAMS)
        for _ in range(100):
            model.advance(drive, TICK)
        assert (abs(model.angle) > 1.0) is moved
    fast, slow = WaistModel(PARAMS), WaistModel(PARAMS)
    for _ in range(200):
        fast.advance(-0.8, TICK)
        slow.advance(0.8, TICK)
    assert fast.speed == pytest.approx(0.8 * PARAMS.speed_negative, rel=0.01)  # Negative drive: toward positive pan
    assert slow.speed == pytest.approx(-0.8 * PARAMS.speed_positive, rel=0.01)
    assert fast.stopping_distance() == pytest.approx(fast.speed * 0.02, rel=0.2)  # Braked, it stops within a degree


def test_a_move_kicks_runs_through_the_model_and_brakes_where_it_leaves_the_head_its_share():
    drive = WaistDrive(PARAMS)
    commands = follow(drive, 30.0, 300)  # Beyond the head's reach: a move starts at once
    moving = [index for index, command in enumerate(commands) if command]
    assert moving[0] == round(PARAMS.plan_ahead / TICK)  # Decided at once, sent plan_ahead later
    kick = commands[moving[0]:moving[0] + round(PARAMS.kick_seconds / TICK)]
    assert all(command == -PARAMS.kick_throttle for command in kick)  # Positive offset: negative drive
    running = commands[moving[0] + len(kick):moving[-1] + 1]
    assert all(-PARAMS.max_throttle <= command <= -PARAMS.run_throttle for command in running)  # One move, never coasting
    assert commands[-1] == 0.0 and drive.model.speed == 0.0
    assert 30.0 - drive.model.angle == pytest.approx(PARAMS.leave, abs=1.0)  # Stopped by the model, not overshooting


def test_polarity_follows_the_person():
    for person, sign in ((30.0, -1), (-30.0, 1)):
        drive = WaistDrive(PARAMS)
        commands = follow(drive, person, 40)
        assert next(command for command in commands if command) == sign * PARAMS.kick_throttle


def test_mid_ranging_starts_on_a_held_settled_head_offset_or_at_once_past_its_reach():
    hold = round(PARAMS.start_hold / TICK)
    out, inside, beyond = PARAMS.start_offset + 2.0, PARAMS.start_offset - 2.0, PARAMS.start_now + 5.0
    def started(person, ticks, **head):
        drive = WaistDrive(PARAMS)
        follow(drive, person, ticks, **head)
        return drive.phase is not WaistState.IDLE
    assert not started(out, 2 * hold, offset=out, settled=False)  # The head is still turning: wait
    assert not started(inside, 2 * hold, offset=inside, settled=True)  # Inside the band: the head keeps it
    assert not started(out, hold - 2, offset=out, settled=True)  # Not held long enough yet
    assert started(out, hold + 2, offset=out, settled=True)
    assert started(beyond, 2, offset=19.0, settled=False)  # Beyond the head's reach: at once
    assert not started(beyond, 50, offset=19.0, settled=False, planning=True)  # But only once the head has turned


def test_no_move_smaller_than_min_move_and_a_rest_between_moves():
    small = PARAMS.leave + PARAMS.min_move - 0.5
    drive = WaistDrive(PARAMS.model_copy(update={"start_offset": 2.0}))
    assert not any(follow(drive, small, 100, offset=small, settled=True))
    drive = WaistDrive(PARAMS)
    commands = follow(drive, 30.0, 300)
    ended = max(index for index, command in enumerate(commands) if command) * TICK  # The last drive sent
    drive = WaistDrive(PARAMS)
    follow(drive, 30.0, round(ended / TICK) + 1)
    stopped, start = drive.model.angle, ended + TICK
    later = follow(drive, stopped + 25.0, round(0.5 * PARAMS.rest / TICK), start=start)  # Someone new, further on
    assert not any(later)  # Resting, though the person is now well beyond the head's reach
    assert any(follow(drive, stopped + 25.0, 200, start=start + 0.5 * PARAMS.rest))


def test_a_lost_person_brakes_a_move():
    drive = WaistDrive(PARAMS)
    follow(drive, 40.0, 40)
    assert drive.phase is WaistState.TURNING
    commands = []
    for tick in range(60):
        now = 0.4 + tick * TICK
        drive.advance(now)
        commands.append(drive.update(now=now, demand=None))
        drive.sent(commands[-1])
    assert commands[-1] == 0.0 and drive.model.speed == 0.0 and drive.phase is WaistState.IDLE


def test_the_plan_ahead_shows_where_the_body_will_be():
    drive = WaistDrive(PARAMS)
    follow(drive, 30.0, 30)  # A move has started
    now = 29 * TICK
    ahead = drive.ahead(now + PARAMS.plan_ahead)
    later = follow(drive, 30.0, round(PARAMS.plan_ahead / TICK), start=now + TICK)
    assert ahead.angle == pytest.approx(drive.model.angle, abs=0.3)  # The commands queued were the ones sent
    assert any(later)


def test_inconsistent_settings_are_rejected():
    with pytest.raises(ValueError):
        WaistDriveParams(stall=0.7, breakaway=0.65)
    with pytest.raises(ValueError):
        WaistDriveParams(run_throttle=0.4)  # At or below the stall
    with pytest.raises(ValueError):
        WaistDriveParams(run_throttle=0.7, max_throttle=0.6)
    with pytest.raises(ValueError):
        WaistDriveParams(kick_throttle=0.5)  # Under breakaway
