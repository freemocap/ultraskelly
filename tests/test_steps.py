import asyncio

from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic
from ultraskelly.replay import create_controllers


def test_shared_steps_hold_lock_between_sensor_samples():
    async def scenario():
        bus, head, mouth = create_controllers()
        head.clock = mouth.clock = lambda: 0.0
        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=640, y=0))
        await head.step()
        await mouth.step()
        assert head.pan_angle == 106
        assert head.tilt_angle == 78
        assert mouth.current_throttle == 0
        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320, y=240))
        await head.step()
        await mouth.step()
        assert mouth.mouth_state.value == 'opening'
        assert mouth.current_throttle == -1
        # Empty tick holds the last fresh lock without applying head gain again.
        await head.step()
        await mouth.step()
        assert head.waist_motor.throttle == -head.params.waist.start_throttle  # Head 16 degrees off: a move began
        assert mouth.mouth_state.value == 'opening'
        bus.close()
    asyncio.run(scenario())


def test_mouth_timing_uses_injected_clock():
    async def scenario():
        bus, head, mouth = create_controllers()
        now = [0.0]
        head.clock = mouth.clock = lambda: now[0]
        for t, expected in [(0, 'opening'), (0.21, 'open'), (2.22, 'open')]:
            now[0] = t
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320, y=240))
            await head.step()
            await mouth.step()
            assert mouth.mouth_state.value == expected
        now[0] = 4
        await mouth.step()
        assert mouth.current_throttle == 0
        bus.close()
    asyncio.run(scenario())
