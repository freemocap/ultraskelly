import asyncio

import pytest

from ultraskelly.core.pubsub.bot_topics import ServoStateTopic, TargetLocationMessage, TargetLocationTopic
from ultraskelly.replay import create_controllers, replay


async def follow(bus, node, x, y, seconds):
    """Step the head on a fake clock for `seconds`, the target at pixel (x, y) in a fresh frame every 30 ms."""
    now = [0.0]
    node.clock = lambda: now[0]
    for tick in range(round(seconds * 100)):
        if tick % 3 == 0:
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=x, y=y, captured=now[0]))
        await node.step()
        now[0] += 0.01


def test_a_centred_target_locks_without_moving_the_head():
    async def scenario():
        bus, node, _ = create_controllers()
        states = bus.get_subscription(ServoStateTopic)
        await follow(bus, node, 320, 240, 1.0)
        state = await states.get()
        assert state.is_locked_x and state.is_locked_y
        assert node.pan_angle == node.tilt_angle == 90
        bus.close()
    asyncio.run(scenario())


def test_a_target_held_at_the_frame_corner_turns_the_head_to_its_end_stops_and_no_further():
    async def scenario():
        bus, node, _ = create_controllers()
        await follow(bus, node, 640, 0, 6.0)
        assert node.pan_angle == 180
        assert node.tilt_angle == pytest.approx(0, abs=1e-3)  # The smoothed goal halves its way down to the stop
        assert node.servo_kit.servo[node.params.pan_channel].angle == 180
        bus.close()
    asyncio.run(scenario())


def test_waist_drives_continuously_while_the_head_is_turned():
    async def scenario():
        bus, node, _ = create_controllers()
        node.params.waist.degrees_per_second = 0.0  # The head holds still instead of counter-rotating
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
    asyncio.run(follow(bus, node, 640, 0, 6.0))
    angles = (node.servo_kit.servo[3].angle, node.servo_kit.servo[11].angle, node.servo_kit.servo[7].angle)
    assert angles == pytest.approx((180, 0, 90.0), abs=1e-3)
    bus.close()
