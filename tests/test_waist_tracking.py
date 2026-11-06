import asyncio

import pytest

from ultraskelly.simulator.engine import Simulation, Settings


def settled_pixels(sim):
    """How far off centre a person may stay: a still head turns only once its goal is past its deadband."""
    aim = sim.head.params.aim
    return aim.deadband / aim.reach * aim.px_per_degree + 1


@pytest.mark.parametrize("yaw", [-30, 30])  # Inside the OAK's 69-degree view
def test_waist_turns_toward_target_and_head_returns_near_neutral(yaw):
    async def scenario():
        sim = Simulation(Settings(scenario="manual", yaw=yaw, pitch=10))
        await sim.step(2000)
        assert sim.waist * yaw > 0
        assert abs(sim.waist) > 20
        assert abs(sim.pan - 90) <= sim.head.params.waist.start_angle
        assert sim.head.waist_motor.throttle == 0
        assert abs(sim.camera()["x"] - 320) <= settled_pixels(sim)
        assert abs(sim.camera()["y"] - 240) <= settled_pixels(sim)
        before = sim.waist
        await sim.step(500)
        assert sim.waist == before
        sim.close()
    asyncio.run(scenario())


def test_waist_turns_in_one_ramped_move_without_braking():
    async def scenario():
        sim = Simulation(Settings(scenario="manual", yaw=30, pitch=10))
        drive = sim.head.params.waist
        commands = []
        for _ in range(2000):
            await sim.step()
            commands.append(sim.head.waist_motor.throttle)
        moving = [index for index, command in enumerate(commands) if command not in (None, 0.0)]
        move = commands[moving[0]:moving[-1] + 1]
        assert all(command not in (None, 0.0) for command in move)  # One continuous move: no pulses
        steps = [abs(b - a) for a, b in zip(move[1:], move[2:])]
        assert max(steps) <= max(drive.ramp_up, drive.ramp_down) * sim.dt + 1e-9
        assert abs(move[0]) == drive.start_throttle
        after = commands[moving[-1] + 1:]
        assert after[0] is None and after[-1] == 0.0  # Released to coast, then braked at rest
        sim.close()
    asyncio.run(scenario())


def test_a_sticky_waist_still_breaks_loose_and_recenters_in_one_move():
    async def scenario():
        # Breaking loose takes more drive than the running drive at 25 degrees (0.58); turning needs 0.35.
        sim = Simulation(Settings(scenario="manual", yaw=25, pitch=10, waist_breakaway=0.6, waist_stall=0.35))
        drive = sim.head.params.waist
        commands = []
        for _ in range(1500):
            await sim.step()
            commands.append(sim.head.waist_motor.throttle)
        moving = [index for index, command in enumerate(commands) if command not in (None, 0.0)]
        move = commands[moving[0]:moving[-1] + 1]
        assert all(command not in (None, 0.0) for command in move)  # One continuous move
        assert max(abs(command) for command in move) >= 0.6  # The breakaway extra got it going
        assert max(abs(command) for command in move) <= drive.breakaway_throttle
        assert sim.waist > 10 and abs(sim.pan - 90) <= drive.start_angle
        assert commands[-1] == 0.0
        sim.close()
    asyncio.run(scenario())
