import asyncio

from ultraskelly.core.bot.motor.orientation_node import WaistControlMode
from ultraskelly.replay import create_controllers, replay


def test_head_deadzone_and_saturation():
    bus, node, _ = create_controllers()
    assert node._update_head_servos(target_x=320, target_y=240) == (True, True)
    assert node.pan_angle == node.tilt_angle == 90
    for _ in range(100):
        node._update_head_servos(target_x=640, target_y=0)
    assert node.pan_angle == 180
    assert node.tilt_angle == 0
    assert node.servo_kit.servo[node.params.pan_channel].angle == 180
    bus.close()


def test_waist_timeout_and_cooldown():
    bus, node, _ = create_controllers()
    now = [100.0]
    node.clock = lambda: now[0]
    node.pan_angle = 160
    throttle = node._calculate_waist_throttle(mode=WaistControlMode.ASSISTING, target_x=640)
    assert -0.8 <= throttle <= -0.7
    now[0] += node.params.waist_max_active_duration + 0.01
    assert node._calculate_waist_throttle(mode=WaistControlMode.ASSISTING, target_x=640) == 0
    now[0] += 0.1
    assert node._calculate_waist_throttle(mode=WaistControlMode.ASSISTING, target_x=640) == 0
    bus.close()


def test_cancellation_stops_both_motors():
    async def scenario():
        bus, orientation, mouth = create_controllers()
        tasks = [asyncio.create_task(node.run()) for node in (orientation, mouth)]
        await asyncio.sleep(0.02)
        orientation.waist_motor.throttle = 0.8
        mouth.mouth_motor.throttle = -1
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        assert orientation.waist_motor.throttle == mouth.mouth_motor.throttle == 0
        bus.close()
    asyncio.run(scenario())


def test_target_replay_exercises_production_coroutines():
    async def scenario():
        states = [state async for state in replay(40, 0.02)]
        assert len(states) == 40
        assert min(s["pan_degrees"] for s in states) < 90
        assert all(0 <= s["pan_degrees"] <= 180 for s in states)
        assert all(abs(s["waist_throttle"]) <= 0.8 for s in states)
        assert states[-1]["target_x"] is None
        assert states[-1]["waist_throttle"] == 0
    asyncio.run(scenario())
