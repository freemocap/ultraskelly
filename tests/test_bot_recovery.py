"""Fault injection through the real launcher and controllers.

When a board stops answering or the camera goes quiet, the bot must end promptly, with
every motor it can still reach braked, and report itself unhealthy so the boot service's
watchdog covers a stuck shutdown. Brief I2C faults must not end it. The launcher imports
every camera node, so these run where the Pi's camera libraries are installed.
"""
import asyncio
import errno

import numpy as np
import pytest

bot_launcher = pytest.importorskip("ultraskelly.bot_launcher", reason="the launcher imports the Pi's camera libraries")

from ultraskelly.core.bot.base_abcs import Node, NodeParams  # noqa: E402
from ultraskelly.core.bot.motor.mouth_node import MouthNode  # noqa: E402
from ultraskelly.core.bot.motor.orientation_node import OrientationNode  # noqa: E402
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, record_hardware_commands  # noqa: E402
from ultraskelly.core.bot.stall_monitor_node import StallMonitorNode, StallMonitorParams, StreamStalledError  # noqa: E402
from ultraskelly.core.pubsub.bot_topics import (  # noqa: E402
    FrameMessage,
    FrameTopic,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager  # noqa: E402


class FakeBoard:
    """An I2C board whose writes fail with EAGAIN while `dead`, or once every `flaky_every` writes."""

    def __init__(self, flaky_every: int = 0) -> None:
        self.dead = False
        self.flaky_every = flaky_every
        self.writes = 0

    def write(self) -> None:
        self.writes += 1
        if self.dead or (self.flaky_every and self.writes % self.flaky_every == 0):
            raise OSError(errno.EAGAIN, "Resource temporarily unavailable")


class FakeChannel:
    """A servo or DC motor channel on a FakeBoard; `values` holds the commands that reached it."""

    def __init__(self, board: FakeBoard) -> None:
        self.board = board
        self.values: list[float | None] = []

    def _command(self, value: float | None) -> None:
        self.board.write()
        self.values.append(value)

    angle = throttle = property(lambda self: self.values[-1] if self.values else None, _command)


class FakeServoKit:
    def __init__(self, board: FakeBoard) -> None:
        self.servo = [FakeChannel(board) for _ in range(16)]


class FakeCamera(Node):
    """Every 10 ms a frame and a target just off center, until `quiet_after` seconds have passed."""

    params: NodeParams = NodeParams()
    quiet_after: float | None = None

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        started = loop.time()
        while not self.stop_event.is_set():
            if self.quiet_after is None or loop.time() - started < self.quiet_after:
                await self.pubsub.topics[FrameTopic].publish(FrameMessage(frame=np.zeros((2, 2, 3), dtype=np.uint8)))
                await self.pubsub.topics[TargetLocationTopic].publish(TargetLocationMessage(x=330, y=240))
            await asyncio.sleep(0.01)


class Bot:
    """The real launcher, controllers and retrying wrappers, on fake boards and a fake camera."""

    def __init__(self, *, servo_board: FakeBoard, motor_board: FakeBoard, camera_quiet_after: float | None = None):
        config = bot_launcher.LaunchConfig(stall_monitor=StallMonitorParams(
            frame_timeout_seconds=0.3, pose_timeout_seconds=0.3, startup_seconds=1.0, check_interval_seconds=0.02,
        ))
        pubsub = PubSubTopicManager.create()
        self.waist, self.jaw = FakeChannel(motor_board), FakeChannel(motor_board)
        self.orientation = OrientationNode(
            pubsub=pubsub,
            params=config.orientation,
            servo_kit=FakeServoKit(servo_board),
            waist_motor=self.waist,
            target_subscription=pubsub.get_subscription(TargetLocationTopic),
            pan_angle=150.0,  # Head turned well past where the waist drive starts turning the torso
        )
        mouth = MouthNode(
            pubsub=pubsub,
            params=config.mouth,
            mouth_motor=self.jaw,
            head_servo_subscription=pubsub.get_subscription(ServoStateTopic),
        )
        record_hardware_commands(orientation=self.orientation, mouth=mouth, recorder=CommandRecorder(), limits=config.limits)
        self.camera = FakeCamera(pubsub=pubsub, quiet_after=camera_quiet_after)
        self.launcher = bot_launcher.BotLauncher(config=config, pubsub=pubsub)
        self.launcher.nodes = [
            self.orientation,
            mouth,
            self.camera,
            StallMonitorNode.create(pubsub=pubsub, params=config.stall_monitor, watch_poses=False),
        ]

    def assert_motors_were_driven_then_braked(self) -> None:
        assert min(value for value in self.waist.values if value is not None) < 0  # The waist was turning
        assert -1.0 in self.jaw.values  # The jaw was open
        assert self.waist.values[-1] == 0.0 and self.jaw.values[-1] == 0.0


async def run_until(bot: Bot, *, fault_after: float, fault) -> None:
    """Run the bot, inject `fault` after `fault_after` seconds, and await its end (5 s at most)."""
    task = asyncio.create_task(bot.launcher.run())
    await asyncio.sleep(fault_after)
    assert not task.done(), "the bot ended before the fault"
    fault()
    await asyncio.wait_for(task, timeout=5)


def test_brief_i2c_faults_are_retried_without_ending_the_bot():
    bot = Bot(servo_board=FakeBoard(flaky_every=5), motor_board=FakeBoard(flaky_every=5))
    asyncio.run(run_until(bot, fault_after=1.0, fault=bot.camera.stop))  # A normal stop after 1 s of faults
    assert bot.orientation.waist_motor.failed_writes > 0
    bot.assert_motors_were_driven_then_braked()


def test_a_servo_board_that_stops_answering_ends_the_bot_with_the_motors_braked():
    servo_board = FakeBoard()
    bot = Bot(servo_board=servo_board, motor_board=FakeBoard())
    with pytest.raises(OSError):
        asyncio.run(run_until(bot, fault_after=0.5, fault=lambda: setattr(servo_board, "dead", True)))
    assert not bot.launcher.healthy()
    bot.assert_motors_were_driven_then_braked()


def test_a_motor_board_that_stops_answering_ends_the_bot_instead_of_hanging():
    motor_board = FakeBoard()
    bot = Bot(servo_board=FakeBoard(), motor_board=motor_board)
    with pytest.raises(OSError):
        asyncio.run(run_until(bot, fault_after=0.5, fault=lambda: setattr(motor_board, "dead", True)))
    assert not bot.launcher.healthy()  # Its brake writes failed too; the service brakes again after the exit


def test_a_camera_that_goes_quiet_ends_the_bot_with_the_motors_braked():
    bot = Bot(servo_board=FakeBoard(), motor_board=FakeBoard(), camera_quiet_after=0.6)
    with pytest.raises(StreamStalledError, match="No camera frames"):
        asyncio.run(run_until(bot, fault_after=0.5, fault=lambda: None))
    assert not bot.launcher.healthy()
    bot.assert_motors_were_driven_then_braked()
