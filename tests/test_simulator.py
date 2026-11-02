import asyncio

import pytest
from pydantic import ValidationError

from ultraskelly.simulator.engine import Settings, Simulation


def test_repeatable_closed_loop_tracking_and_reset():
    async def scenario():
        config = Settings(scenario="manual", yaw=30, pitch=15, head_deadzone=20)
        a, b = Simulation(config), Simulation(config)
        initial = a.camera()
        await a.step(1000)
        await b.step(1000)
        assert a.snapshot() == b.snapshot()
        assert abs(a.camera()['x'] - 320) < abs(initial['x'] - 320)
        assert abs(a.camera()['y'] - 240) < abs(initial['y'] - 240)
        assert abs(a.camera()['x'] - 320) <= 20
        a.reset()
        assert a.tick == 0 and a.pan == 90 and not a.running
        a.close()
        b.close()
    asyncio.run(scenario())


def test_loss_stops_drive_and_memory_stays_bounded():
    async def scenario():
        sim = Simulation(Settings(sensor_hz=100))
        await sim.step(200)
        sim.configure({"visible": False})
        await sim.step(4000)
        assert sim.head.waist_motor.throttle == 0
        assert sim.mouth.mouth_motor.throttle == 0
        assert not sim.last_camera['visible']
        assert len(sim.history) == 600
        assert all(topic.publication.qsize() <= 1 for topic in sim.bus.topics.values())
        sim.close()
    asyncio.run(scenario())


def test_invalid_tuning_is_atomic():
    sim = Simulation()
    before = sim.settings.model_dump()
    for change in [{"head_gain": float('nan')}, {"sensor_hz": 0}, {"typo": 2}]:
        with pytest.raises(ValidationError):
            sim.configure(change)
        assert sim.settings.model_dump() == before
    sim.close()
