import asyncio

import pytest

from ultraskelly.simulator.engine import Simulation, Settings
from ultraskelly.core.bot.motor.orientation_node import WaistControlMode
from ultraskelly.replay import create_controllers


@pytest.mark.parametrize("yaw", [-50, 50])
def test_waist_turns_toward_target_and_head_returns_near_neutral(yaw):
    async def scenario():
        sim = Simulation(Settings(scenario="manual", yaw=yaw, pitch=10))
        await sim.step(2000)
        assert sim.waist * yaw > 0
        assert abs(sim.waist) > 20
        assert abs(sim.pan - 90) <= sim.head.params.pan_center_zone
        assert sim.head.waist_motor.throttle == 0
        assert abs(sim.camera()["x"] - 320) <= 1
        assert abs(sim.camera()["y"] - 240) <= 1
        before = sim.waist
        await sim.step(500)
        assert sim.waist == before
        sim.close()
    asyncio.run(scenario())


def test_centering_retains_timeout_cooldown_and_neutral_stop():
    bus, head, _ = create_controllers()
    now = [0.0]
    head.clock = lambda: now[0]
    head.pan_angle = 120
    drive = lambda: head._calculate_waist_throttle(
        mode=WaistControlMode.CENTERING, target_x=320)
    assert -.8 <= drive() <= -.7
    now[0] = .51
    assert drive() == 0
    now[0] = .8
    assert drive() == 0
    now[0] = .92
    assert drive() < 0
    head.pan_angle = 90
    assert drive() == 0
    bus.close()
