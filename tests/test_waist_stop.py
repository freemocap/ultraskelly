"""A waist at the end of its travel: it turns, the person comes no nearer, so it stops trying that way."""
import asyncio

from ultraskelly.core.bot.motor.waist_drive import WaistDrive, WaistDriveParams
from ultraskelly.simulator.engine import Settings, Simulation

TICK = 0.01


def tick(drive: WaistDrive, now: float, demand: float | None) -> float | None:
    drive.advance(now)
    command = drive.update(now=now, demand=demand)
    drive.sent(command)
    return command


def test_a_waist_that_brings_the_person_no_nearer_brakes_and_tries_that_way_no_more():
    drive = WaistDrive(WaistDriveParams())
    commands = [tick(drive, n * TICK, 30.0) for n in range(600)]  # Stuck: the picture keeps them 30 degrees off
    driven = [n for n, command in enumerate(commands) if command]
    assert driven[0] * TICK < 0.2 and driven[-1] * TICK < 1.3  # It tried for about a second, not the 6 s safety limit
    assert drive.blocked == 1 and not any(commands[200:])  # And not again, though they are still beyond its reach
    start = drive.model.angle  # Someone on the other side, and that way the body turns as its model says
    other = [tick(drive, (600 + n) * TICK, -30.0 - (drive.model.angle - start)) for n in range(300)]
    assert any(command and command > 0 for command in other)  # Positive drive turns toward negative pan
    assert drive.blocked == 0  # Turned the other way: free again


def test_a_waist_that_turns_even_at_half_its_modelled_speed_finishes_its_move():
    for share in (1.0, 0.5):
        drive = WaistDrive(WaistDriveParams())
        commands = [tick(drive, n * TICK, 30.0 - share * drive.model.angle) for n in range(400)]
        assert drive.blocked == 0 and any(commands)


def test_a_motor_test_the_other_way_frees_the_blocked_side():
    drive = WaistDrive(WaistDriveParams())
    drive.blocked = 1
    drive.moved(1)
    assert drive.blocked == 1  # Further toward the stop: still blocked
    drive.moved(-1)
    assert drive.blocked == 0


def test_the_robot_stops_pushing_a_waist_held_at_its_end_stop():
    async def scenario(stop: float) -> tuple[float, int]:
        sim = Simulation(Settings(scenario="manual", yaw=30, pitch=5, waist_stop=stop))  # Beyond the head's reach
        driven = 0
        for _ in range(1200):  # 12 s
            await sim.step()
            driven += bool(sim.head.waist_motor.throttle)
        blocked = sim.head.waist_drive.blocked
        sim.close()
        return driven * TICK, blocked

    seconds, blocked = asyncio.run(scenario(5.0))
    assert 0.0 < seconds < 1.5 and blocked == 1  # One push of about a second against the stop, then no more
    seconds, blocked = asyncio.run(scenario(100.0))
    assert blocked == 0  # With room to turn it turns to them, and nothing is blocked
