import asyncio

import pytest

from ultraskelly.simulator.engine import Simulation, Settings


def settled_pixels(sim):
    """How far off centre a person may stay: a settled head turns again only once its aim is past its resume band."""
    aim = sim.head.params.aim
    return max(aim.pan_resume, aim.tilt_resume) * aim.px_per_degree + 2


@pytest.mark.parametrize("yaw", [-30, 30])  # Inside the OAK's 69-degree view
def test_waist_turns_toward_target_and_head_returns_near_neutral(yaw):
    async def scenario():
        sim = Simulation(Settings(scenario="manual", yaw=yaw, pitch=5))
        await sim.step(2000)
        assert sim.waist * yaw > 0
        assert abs(sim.waist) > 20
        assert abs(sim.head_turn()[0]) <= sim.head.params.waist.start_offset  # The head, not its geared servo
        assert sim.head.waist_motor.throttle == 0
        assert abs(sim.camera()["x"] - 320) <= settled_pixels(sim)
        assert abs(sim.camera()["y"] - 240) <= settled_pixels(sim)
        before = sim.waist
        await sim.step(500)
        assert sim.waist == before
        sim.close()
    asyncio.run(scenario())


def test_waist_turns_in_one_planned_move_kick_run_brake():
    async def scenario():
        sim = Simulation(Settings(scenario="manual", yaw=30, pitch=5))
        drive = sim.head.params.waist
        commands = []
        for _ in range(2000):
            await sim.step()
            commands.append(sim.head.waist_motor.throttle)
        moving = [index for index, command in enumerate(commands) if command not in (None, 0.0)]
        move = commands[moving[0]:moving[-1] + 1]
        assert all(command not in (None, 0.0) for command in move)  # One continuous move: no pulses, no coasting
        kick = round(drive.kick_seconds / sim.dt)
        assert move[:kick] == [-drive.kick_throttle] * kick  # Breaking loose at once, not ramping up to it
        assert all(drive.run_throttle <= abs(command) <= drive.max_throttle for command in move[kick:])
        after = commands[moving[-1] + 1:]
        assert all(command == 0.0 for command in after)  # Braked from its slowest speed, and held
        sim.close()
    asyncio.run(scenario())


def test_a_sticky_waist_still_breaks_loose_and_recenters_in_one_move():
    async def scenario():
        # Breaking loose takes more than the running drive (0.55 would never start it); the kick does it.
        sim = Simulation(Settings(scenario="manual", yaw=25, pitch=5, waist_breakaway=0.72, waist_stall=0.5))
        drive = sim.head.params.waist
        commands = []
        for _ in range(1500):
            await sim.step()
            commands.append(sim.head.waist_motor.throttle)
        moving = [index for index, command in enumerate(commands) if command not in (None, 0.0)]
        move = commands[moving[0]:moving[-1] + 1]
        assert all(command not in (None, 0.0) for command in move)  # One continuous move
        assert max(abs(command) for command in move) == drive.kick_throttle  # The kick got it going
        assert sim.waist > 10 and abs(sim.head_turn()[0]) <= drive.start_offset
        assert commands[-1] == 0.0
        sim.close()
    asyncio.run(scenario())
