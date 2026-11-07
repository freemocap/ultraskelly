import asyncio

import pytest

from ultraskelly.core.bot.motor.motor_test import SWEEP_STEP, MotorTester, MotorTestLimits, MotorTestRefused, MotorTestRequest
from ultraskelly.core.bot.motor.mouth_node import MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, create_simulated_motor_nodes
from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

LIMITS = MotorTestLimits(settle_seconds=0.0)


def build(limits=LIMITS):
    recorder, bus = CommandRecorder(), PubSubTopicManager.create()
    orientation, mouth = create_simulated_motor_nodes(
        pubsub=bus, orientation_params=OrientationNodeParams(), mouth_params=MouthNodeParams(), recorder=recorder,
    )
    return MotorTester(orientation=orientation, mouth=mouth, limits=limits), recorder, bus


def values(recorder, actuator):
    return [command.value for command in recorder.history if command.actuator == actuator]


def test_a_servo_test_moves_the_head_and_tracking_resumes_from_there():
    async def scenario():
        tester, recorder, bus = build()
        result = await tester.run(MotorTestRequest(actuator="pan", angle=110))
        assert result["outcome"] == "done" and result["sent"] == 110
        assert values(recorder, "pan")[-1] == 110 and tester.orientation.pan_angle == 110
        assert not tester.orientation.manual and not tester.mouth.manual
        bus.close()
    asyncio.run(scenario())


def test_servo_tests_reach_every_angle_the_servo_takes():
    # 2026-11-07, the owner: from software the range was "super limited (esp in tilt)" (tests were held to 60-120),
    # and "DONT ... REFUSE SERVO ANGLES IF THE MOTOR CAN MAKE THERE THEN ... DO IT".
    async def scenario():
        tester, recorder, bus = build()
        assert tester.state()["limits"]["servo_windows"] == {"pan": [0.0, 180.0], "tilt": [0.0, 180.0], "roll": [0.0, 180.0]}
        # The card's tilt buttons go by where the head goes: a larger tilt angle turns it down (the owner,
        # 2026-11-07: "the -10 tilt moves the head UP and +10 moves it down")
        assert tester.state()["limits"]["directions"] == {"pan": 1, "tilt": 1, "roll": -1}
        tester.orientation.params.roll.servo_speed = 1000.0  # This test is about the range, not the sweep's speed
        tester.orientation.params.aim.max_acceleration = tester.orientation.params.roll.max_acceleration = 1000.0
        for actuator, angle in (("tilt", 180.0), ("tilt", 0.0), ("pan", 180.0), ("pan", 0.0), ("tilt", 90.0),
                                ("roll", 180.0), ("roll", 0.0), ("roll", 90.0)):
            result = await tester.run(MotorTestRequest(actuator=actuator, angle=angle))
            assert result["outcome"] == "done" and values(recorder, actuator)[-1] == angle
        bus.close()
    asyncio.run(scenario())
    with pytest.raises(ValueError):  # Not an angle the servo board's scale has
        MotorTestRequest(actuator="tilt", angle=181)


def test_a_servo_test_sweeps_at_the_heads_own_top_speed_and_a_stop_holds_it():
    # 2026-11-07, the owner: "decrease max pan servo speed". A test jumped the servo at its own full speed; it now turns
    # no faster than following does (pan_servo_speed for pan; the robot launches with 30 degrees a second).
    async def scenario():
        tester, recorder, bus = build()
        tester.orientation.params.aim.pan_servo_speed = 60.0
        result = await tester.run(MotorTestRequest(actuator="pan", angle=120))
        pans = values(recorder, "pan")
        assert result["outcome"] == "done" and pans[-1] == 120 and tester.orientation.pan_angle == 120
        assert max(abs(b - a) for a, b in zip(pans, pans[1:])) <= 60.0 * SWEEP_STEP + 1e-9  # No jump anywhere
        assert result["took"] >= 0.4  # 30 degrees at 60 a second: half a second, and more to speed up and slow down
        steps = [abs(b - a) for a, b in zip(pans, pans[1:])]
        assert steps[0] < max(steps) and steps[-1] < max(steps)  # It speeds up from rest and slows down to the target
        running = asyncio.create_task(tester.run(MotorTestRequest(actuator="pan", angle=30)))
        await asyncio.sleep(0.3)
        assert await tester.stop()
        stopped = await running
        assert stopped["outcome"] == "stopped" and 30 < stopped["sent"] < 120  # Held where the sweep had got to
        assert tester.orientation.pan_angle == stopped["sent"]
        bus.close()
    asyncio.run(scenario())


def test_a_roll_test_sweeps_at_the_rolls_own_speed_and_levelling_carries_on_from_there():
    # 2026-11-07, the owner: "add the roll controls to the robot control panel in the dashboard".
    async def scenario():
        tester, recorder, bus = build()
        roll = tester.orientation.params.roll
        result = await tester.run(MotorTestRequest(actuator="roll", angle=110))
        rolls = values(recorder, "roll")
        assert result["outcome"] == "done" and rolls[-1] == 110 and tester.orientation.roll_angle == 110
        assert max(abs(b - a) for a, b in zip(rolls, rolls[1:])) <= roll.servo_speed * SWEEP_STEP + 1e-9  # Swept
        assert result["took"] >= 20 / roll.servo_speed - 0.05
        await tester.orientation.step()  # Tracking takes over from the test's angle, and with no face eases back
        assert tester.orientation.roll_level.servo == pytest.approx(110, abs=roll.slack)
        bus.close()
    asyncio.run(scenario())


def test_a_waist_pulse_runs_alone_for_its_time_then_brakes():
    async def scenario():
        tester, recorder, bus = build()
        tester.orientation.waist_motor.throttle = -0.6  # Tracking was turning the waist
        tester.mouth.mouth_motor.throttle = -1.0  # and holding the jaw open
        result = await tester.run(MotorTestRequest(actuator="waist", throttle=0.5, seconds=0.1))
        assert result["outcome"] == "done" and 0.08 <= result["took"] < 0.3  # Windows timers may fire 15 ms early
        assert values(recorder, "waist")[-3:] == [0.0, 0.5, 0.0]  # Tracking's drive stopped before the pulse
        assert values(recorder, "mouth")[-1] == 0.0
        assert tester.waist_travel == pytest.approx(0.05, abs=0.01)
        bus.close()
    asyncio.run(scenario())


@pytest.mark.parametrize("request_fields, reason", [
    (dict(actuator="tilt", angle=90, seconds=1), "an angle and nothing else"),
    (dict(actuator="waist", throttle=0.9, seconds=0.2), "at most 0.70"),
    (dict(actuator="waist", throttle=0.5, seconds=2), "at most 0.50 s"),
    (dict(actuator="mouth", throttle=-1.0), "a throttle and seconds"),
])
def test_tests_outside_the_limits_are_refused_without_moving(request_fields, reason):
    tester, recorder, bus = build()
    before = list(recorder.history)
    with pytest.raises(MotorTestRefused, match=reason):
        asyncio.run(tester.run(MotorTestRequest(**request_fields)))
    assert list(recorder.history) == before
    bus.close()


def test_waist_tests_cannot_walk_the_waist_far_one_way():
    async def scenario():
        tester, _, bus = build(MotorTestLimits(settle_seconds=0.0, waist_max_travel=0.12))
        await tester.run(MotorTestRequest(actuator="waist", throttle=0.5, seconds=0.1))
        await tester.run(MotorTestRequest(actuator="waist", throttle=0.5, seconds=0.1))
        with pytest.raises(MotorTestRefused, match="the other way"):
            await tester.run(MotorTestRequest(actuator="waist", throttle=0.5, seconds=0.1))
        await tester.run(MotorTestRequest(actuator="waist", throttle=-0.5, seconds=0.1))  # Back the other way is fine
        bus.close()
    asyncio.run(scenario())


def test_a_second_test_while_one_runs_is_refused_and_stop_cuts_it_short():
    async def scenario():
        tester, recorder, bus = build()
        running = asyncio.create_task(tester.run(MotorTestRequest(actuator="waist", throttle=-0.5, seconds=0.5)))
        await asyncio.sleep(0.05)
        with pytest.raises(MotorTestRefused) as refused:
            await tester.run(MotorTestRequest(actuator="pan", angle=100))
        assert refused.value.busy
        assert tester.state()["running"]["description"] == "waist -0.50 for 0.50 s"
        assert await tester.stop()
        result = await running
        assert result["outcome"] == "stopped" and result["took"] < 0.3
        assert values(recorder, "waist")[-1] == 0.0 and tester.state()["running"] is None
        assert not await tester.stop()  # Nothing left to stop
        bus.close()
    asyncio.run(scenario())


def test_tracking_commands_nothing_during_a_test():
    async def scenario():
        tester, recorder, bus = build()
        running = asyncio.create_task(tester.run(MotorTestRequest(actuator="mouth", throttle=-1.0, seconds=0.2)))
        await asyncio.sleep(0.02)
        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=640, y=0))  # Would turn the head and waist
        await tester.orientation.step()
        await tester.mouth.step()
        assert tester.orientation.pan_angle == 90 and values(recorder, "waist")[-1] == 0.0
        assert values(recorder, "mouth")[-1] == -1.0  # The jaw test keeps its drive
        await running
        bus.close()
    asyncio.run(scenario())


def test_disabled_tests_are_refused():
    tester, _, bus = build(MotorTestLimits(enabled=False))
    with pytest.raises(MotorTestRefused, match="turned off"):
        asyncio.run(tester.run(MotorTestRequest(actuator="pan", angle=90)))
    bus.close()


def test_a_test_on_simulated_motors_says_nothing_moved():
    # 2026-11-06: with --simulate-motors the dashboard said "done" and nothing on the robot moved.
    async def scenario():
        tester, recorder, bus = build()
        tester.simulated = True
        result = await tester.run(MotorTestRequest(actuator="pan", angle=100))
        assert result["outcome"].startswith("simulated, nothing moved") and tester.state()["simulated"]
        assert values(recorder, "pan")[-1] == 100  # Still recorded, as before
        bus.close()
    asyncio.run(scenario())
