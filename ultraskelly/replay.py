"""Hardware-free target replay through the production controller coroutines.

This observes commanded state; it is not a physical model of the robot.
"""
import argparse
import asyncio
import json
from dataclasses import dataclass, field

from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNode, OrientationNodeParams
from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic, ServoStateTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


@dataclass
class FakeServo:
    angle: float = 90.0


@dataclass
class FakeServoKit:
    servo: list[FakeServo] = field(default_factory=lambda: [FakeServo() for _ in range(16)])


@dataclass
class FakeMotor:
    throttle: float = 0.0


def create_controllers():
    """Inject memory-only actuators without calling either hardware factory."""
    bus = PubSubTopicManager.create()
    orientation = OrientationNode(
        pubsub=bus, params=OrientationNodeParams(), servo_kit=FakeServoKit(),
        waist_motor=FakeMotor(), target_subscription=bus.get_subscription(TargetLocationTopic),
    )
    mouth = MouthNode(
        pubsub=bus, params=MouthNodeParams(), mouth_motor=FakeMotor(),
        head_servo_subscription=bus.get_subscription(ServoStateTopic),
    )
    return bus, orientation, mouth


async def replay(steps: int, interval: float):
    bus, orientation, mouth = create_controllers()
    tasks = [asyncio.create_task(node.run()) for node in (orientation, mouth)]
    try:
        for step in range(steps):
            # Center, left, right, and missing-target phases, repeated for soak runs.
            x = (320, 0, 640, None)[(step // 10) % 4]
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=x, y=240 if x is not None else None))
            await asyncio.sleep(interval)
            for task in tasks:
                if task.done():
                    task.result()
                    raise RuntimeError("Controller exited before replay completed")
            yield {
                "step": step, "target_x": x,
                "pan_degrees": orientation.pan_angle, "tilt_degrees": orientation.tilt_angle,
                "waist_throttle": orientation.waist_motor.throttle,
                "mouth_throttle": mouth.mouth_motor.throttle,
            }
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        bus.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--interval", type=float, default=0.05)
    args = parser.parse_args()
    if args.steps <= 0 or args.interval <= 0:
        parser.error("steps and interval must be positive")

    async def run():
        async for state in replay(args.steps, args.interval):
            print(json.dumps(state))

    asyncio.run(run())


if __name__ == "__main__":
    main()
