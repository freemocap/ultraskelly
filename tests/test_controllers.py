import asyncio

import pytest

from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic
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


def test_waist_drives_continuously_while_the_head_is_turned():
    async def scenario():
        bus, node, _ = create_controllers()
        now = [100.0]
        node.clock = lambda: now[0]
        node.pan_angle = 160
        commands = []
        for tick in range(200):  # Two seconds; the old controller braked after half of one
            if tick % 10 == 0:
                await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320, y=240))
            await node.step()
            commands.append(node.waist_motor.throttle)
            now[0] += 0.01
        assert all(-0.8 <= command < 0 for command in commands)
        assert commands[0] == -node.params.waist.start_throttle
        # The head never comes back here, so the drive rises to its breakaway ceiling.
        assert commands[-1] == pytest.approx(-node.params.waist.breakaway_throttle)
        bus.close()
    asyncio.run(scenario())


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
        states = [state async for state in replay(40, 0.05)]
        assert len(states) == 40
        assert min(s["pan_degrees"] for s in states) < 90
        assert all(0 <= s["pan_degrees"] <= 180 for s in states)
        assert all(abs(s["waist_throttle"] or 0.0) <= 0.8 for s in states)
        assert states[-1]["target_x"] is None
        assert states[-1]["waist_throttle"] in (None, 0.0)  # Ramped down; coasting or braked
    asyncio.run(scenario())


def test_head_servo_channels_match_the_wiring_and_roll_holds_level():
    # Found 2026-11-06 by wiggling one channel at a time with the user watching.
    bus, node, _ = create_controllers()
    assert node.params.servo_names == {3: "pan", 11: "tilt", 7: "roll"}
    assert node.servo_kit.servo[7].angle == 90.0
    for _ in range(100):
        node._update_head_servos(target_x=640, target_y=0)
    assert (node.servo_kit.servo[3].angle, node.servo_kit.servo[11].angle, node.servo_kit.servo[7].angle) == (180, 0, 90.0)
    bus.close()


def test_without_follow_the_head_and_waist_hold_still_for_a_target():
    async def scenario():
        bus, node, _ = create_controllers()
        node.params.follow = False
        for _ in range(20):
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=640, y=0))
            await node.step()
        assert (node.pan_angle, node.tilt_angle) == (90.0, 90.0)
        assert node.waist_motor.throttle in (None, 0.0)
        bus.close()
    asyncio.run(scenario())


def test_the_robot_launches_with_the_calmer_tuning():
    # 2026-11-06, the user: "turn down the aggressiveness". Gain 0.05 overshot 84 px on the robot.
    from ultraskelly.__main__bot import launch_config

    head = launch_config().orientation
    assert head.head_gain <= 0.012 and head.centering_tolerance >= 20 and head.follow
    assert head.waist.start_angle >= 45 and head.waist.max_throttle <= 0.6
