import asyncio

from ultraskelly.simulator.engine import Simulation, Settings
from ultraskelly.replay import create_controllers
from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic


def settled_pixels(sim):
    """How far off centre a person may stay: a still head turns only once its goal is past its deadband."""
    aim = sim.head.params.aim
    return aim.deadband / aim.reach * aim.px_per_degree + 1


def test_target_centers_inside_jaw_square():
    async def scenario():
        sim = Simulation(Settings(scenario='manual', yaw=20, pitch=5))  # Within the geared tilt's reach
        await sim.step(2000)
        image = sim.camera()
        assert image['visible']
        assert abs(image['x'] - 320) <= settled_pixels(sim)
        assert abs(image['y'] - 240) <= settled_pixels(sim)
        sim.close()
    asyncio.run(scenario())


def test_jaw_holds_open_across_sensor_gaps_and_releases_on_loss():
    async def scenario():
        sim = Simulation(Settings(scenario='manual', yaw=0, pitch=0, sensor_hz=10))
        await sim.step(100)
        for _ in range(500):
            await sim.step()
            assert sim.mouth.mouth_motor.throttle == -1
            assert sim.jaw == 28
        sim.configure({'visible': False})
        await sim.step(10)
        assert sim.mouth.mouth_motor.throttle == 0
        await sim.step(50)
        assert sim.jaw == 0
        sim.close()
    asyncio.run(scenario())


def test_outside_square_releases_jaw_and_no_data_expires():
    async def scenario():
        bus, head, mouth = create_controllers()
        now = [0.0]
        head.clock = mouth.clock = lambda: now[0]
        async def tick(x):
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=x, y=240))
            await head.step()
            await mouth.step()
        await tick(320)
        assert mouth.current_throttle == -1
        await tick(600)
        assert mouth.current_throttle == 0
        await tick(320)
        now[0] = .1
        await head.step()
        await mouth.step()
        assert mouth.current_throttle == -1
        now[0] = 1.0
        await head.step()
        await mouth.step()
        assert mouth.current_throttle == 0
        assert head.waist_motor.throttle == 0
        bus.close()
    asyncio.run(scenario())
