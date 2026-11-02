import asyncio

import pytest

from ultraskelly.core.bot.motor.motor_test import MotorTester, MotorTestLimits, MotorTestRefused, MotorTestRequest
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


def test_a_waist_pulse_runs_alone_for_its_time_then_brakes():
    async def scenario():
        tester, recorder, bus = build()
        tester.orientation.waist_motor.throttle = -0.6  # Tracking was turning the waist
        tester.mouth.mouth_motor.throttle = -1.0  # and holding the jaw open
        result = await tester.run(MotorTestRequest(actuator="waist", throttle=0.5, seconds=0.1))
        assert result["outcome"] == "done" and 0.1 <= result["took"] < 0.3
        assert values(recorder, "waist")[-3:] == [0.0, 0.5, 0.0]  # Tracking's drive stopped before the pulse
        assert values(recorder, "mouth")[-1] == 0.0
        assert tester.waist_travel == pytest.approx(0.05, abs=0.01)
        bus.close()
    asyncio.run(scenario())


@pytest.mark.parametrize("request_fields, reason", [
    (dict(actuator="pan", angle=150), "within 30 degrees"),
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
