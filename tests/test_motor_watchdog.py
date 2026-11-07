import asyncio
import time

from ultraskelly.core.bot.motor.motor_watchdog import MotorWatchdog
from ultraskelly.core.bot.motor.waist_drive import WaistDrive, WaistDriveParams, WaistState


class Motor:
    def __init__(self, fails: bool = False):
        self.commands, self.fails = [], fails

    @property
    def throttle(self):
        return self.commands[-1][1] if self.commands else None

    @throttle.setter
    def throttle(self, value):
        if self.fails:
            raise OSError(11, "Resource temporarily unavailable")
        self.commands.append((time.monotonic(), value))


def test_it_brakes_once_per_stall_and_only_once_armed():
    now = [0.0]
    waist, mouth = Motor(), Motor()
    watchdog = MotorWatchdog({"waist": waist, "mouth": mouth}, timeout=0.5, clock=lambda: now[0])
    now[0] = 10.0
    assert not watchdog.check()  # Unarmed: the slow startup before the first tick is fine
    watchdog.tick()
    now[0] = 10.4
    assert not watchdog.check()
    now[0] = 10.6
    assert watchdog.check() and not watchdog.check()  # Braked once, not again for the same stall
    assert waist.commands[-1][1] == mouth.commands[-1][1] == 0.0
    watchdog.tick()  # The loop came back
    now[0] = 11.5
    assert watchdog.check() and watchdog.trips == 2  # A second stall brakes again
    watchdog.tick()
    watchdog.disarm()  # A controller shutting down on purpose
    now[0] = 20.0
    assert not watchdog.check()


def test_a_stall_is_an_error_only_when_it_caught_a_motor_driven(caplog):
    # Every start stalls the loop about 0.8 s with the motors at rest (the Pi, 2026-11-07), and the control panel
    # shows the bot's newest error: that stall is a warning, braked all the same.
    now = [0.0]
    waist, mouth = Motor(), Motor()
    watchdog = MotorWatchdog({"waist": waist, "mouth": mouth}, timeout=0.5, clock=lambda: now[0])
    watchdog.tick()
    now[0] = 0.8
    with caplog.at_level("WARNING", logger="ultraskelly.core.bot.motor.motor_watchdog"):
        assert watchdog.check()
    assert [record.levelname for record in caplog.records] == ["WARNING"]
    assert waist.commands[-1][1] == mouth.commands[-1][1] == 0.0
    watchdog.tick()  # Logs that the loop runs again
    caplog.clear()
    waist.throttle = 0.4  # Driven when the next stall comes
    now[0] = 2.0
    with caplog.at_level("WARNING", logger="ultraskelly.core.bot.motor.motor_watchdog"):
        assert watchdog.check()
    assert [record.levelname for record in caplog.records] == ["ERROR"] and "waist driven" in caplog.records[0].message
    assert waist.commands[-1][1] == 0.0


def test_a_motor_that_cannot_be_braked_does_not_stop_the_others():
    now = [0.0]
    broken, mouth = Motor(fails=True), Motor()
    watchdog = MotorWatchdog({"waist": broken, "mouth": mouth}, timeout=0.5, clock=lambda: now[0])
    watchdog.tick()
    now[0] = 1.0
    assert watchdog.check() and mouth.commands[-1][1] == 0.0


def test_its_thread_brakes_the_motors_while_the_event_loop_is_blocked():
    waist = Motor()
    watchdog = MotorWatchdog({"waist": waist}, timeout=0.2, period=0.01)

    async def control_loop():
        for _ in range(10):
            watchdog.tick()
            await asyncio.sleep(0.01)
        stalled_at = time.monotonic()
        time.sleep(0.6)  # A blocking call freezes the loop: no ticks, and no coroutine can run
        return stalled_at

    watchdog.start()
    try:
        stalled_at = asyncio.run(control_loop())
    finally:
        watchdog.stop()
    braked = [at for at, value in waist.commands if value == 0.0]
    assert braked and 0.15 <= braked[0] - stalled_at <= 0.4  # From the watchdog thread, within its timeout


def test_the_waist_drive_starts_over_after_a_stall():
    params = WaistDriveParams()
    drive = WaistDrive(params)
    for tick in range(50):
        drive.update(now=tick * 0.01, pan_offset=30.0)
    assert drive.phase is WaistState.TURNING and drive.level > params.start_throttle
    command = drive.update(now=2.0, pan_offset=30.0)  # 1.5 s without an update: the watchdog braked it
    assert command == -params.start_throttle  # A new move, from rest, not the old drive


def test_the_orientation_loop_ticks_the_watchdog_and_disarms_it_when_stopped():
    from ultraskelly.replay import create_controllers

    async def scenario():
        bus, orientation, mouth = create_controllers()
        watchdog = MotorWatchdog({"waist": orientation.waist_motor, "mouth": mouth.mouth_motor})
        orientation.motor_watchdog = watchdog
        task = asyncio.create_task(orientation.run())
        await asyncio.sleep(0.05)
        assert watchdog.last_tick is not None  # Armed by the loop
        orientation.stop()
        await task
        assert watchdog.last_tick is None and watchdog.trips == 0  # Shutting down on purpose is not a stall
        bus.close()
    asyncio.run(scenario())
