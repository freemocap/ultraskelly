"""Hardware-free target replay through the production controller coroutines.

This observes commanded state; it is not a physical model of the robot.
"""
import argparse
import asyncio
import json

from ultraskelly.core.bot.motor.mouth_node import MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, create_simulated_motor_nodes
from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

REPLAY_INTERVAL = 0.05
SERVE_INTERVAL = 0.1  # Slower, so the dashboard's motion is easy to follow


def create_controllers(recorder: CommandRecorder | None = None):
    """Inject memory-only actuators without calling either hardware factory."""
    bus = PubSubTopicManager.create()
    orientation, mouth = create_simulated_motor_nodes(
        pubsub=bus,
        orientation_params=OrientationNodeParams(),
        mouth_params=MouthNodeParams(),
        recorder=recorder or CommandRecorder(),
    )
    return bus, orientation, mouth


def scripted_target(step: int) -> TargetLocationMessage:
    """Center, left, right, and missing-target phases of ten steps each, repeating."""
    x = (320, 0, 640, None)[(step // 10) % 4]
    return TargetLocationMessage(x=x, y=240 if x is not None else None)


async def replay(steps: int, interval: float):
    bus, orientation, mouth = create_controllers()
    tasks = [asyncio.create_task(node.run()) for node in (orientation, mouth)]
    try:
        for step in range(steps):
            target = scripted_target(step)
            await bus.publish(TargetLocationTopic, target)
            await asyncio.sleep(interval)
            for task in tasks:
                if task.done():
                    task.result()
                    raise RuntimeError("Controller exited before replay completed")
            yield {
                "step": step, "target_x": target.x,
                "pan_degrees": orientation.pan_angle, "tilt_degrees": orientation.tilt_angle,
                "waist_throttle": orientation.waist_motor.throttle,
                "mouth_throttle": mouth.mouth_motor.throttle,
            }
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        bus.close()


async def serve(port: int, interval: float) -> None:
    """Loop the scripted targets through the controllers and serve the dashboard on 127.0.0.1."""
    from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams

    recorder = CommandRecorder()
    bus, orientation, mouth = create_controllers(recorder)
    tasks: dict[str, asyncio.Task] = {}

    def node_status() -> list[dict]:
        return [
            {"name": name, "state": "running" if not task.done() else "stopped"}
            for name, task in tasks.items()
        ]

    server = RuntimeServerNode.create(
        pubsub=bus,
        params=RuntimeServerParams(host="127.0.0.1", port=port),
        cameras={},
        recorder=recorder,
        motors_simulated=True,
        node_status=node_status,
    )
    for node in (orientation, mouth, server):
        tasks[type(node).__name__] = asyncio.create_task(node.run())
    print(f"Replay dashboard: http://127.0.0.1:{port} (scripted targets, no hardware). Ctrl+C to stop.", flush=True)
    try:
        step = 0
        while True:
            await bus.publish(TargetLocationTopic, scripted_target(step))
            step += 1
            await asyncio.sleep(interval)
            for task in tasks.values():
                if task.done():
                    task.result()
                    raise RuntimeError("A node exited during the replay")
    finally:
        for task in tasks.values():
            task.cancel()
        await asyncio.gather(*tasks.values(), return_exceptions=True)
        bus.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=80)
    parser.add_argument("--interval", type=float, default=None,
                        help=f"Seconds per target (default {REPLAY_INTERVAL}, or {SERVE_INTERVAL} with --serve)")
    parser.add_argument("--serve", action="store_true",
                        help="Loop the targets and serve the live dashboard instead of printing JSON lines")
    parser.add_argument("--port", type=int, default=8081, help="Dashboard port with --serve")
    args = parser.parse_args()
    interval = args.interval if args.interval is not None else (SERVE_INTERVAL if args.serve else REPLAY_INTERVAL)
    if args.steps <= 0 or interval <= 0:
        parser.error("steps and interval must be positive")

    if args.serve:
        try:
            asyncio.run(serve(args.port, interval))
        except KeyboardInterrupt:
            pass
        return

    async def run():
        async for state in replay(args.steps, interval):
            print(json.dumps(state))

    asyncio.run(run())


if __name__ == "__main__":
    main()
