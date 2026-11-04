"""Ends the bot when a camera or pose stream that was running goes quiet, so the service restarts it.

The camera node raises when its pipeline stops, and the Hailo detector when an inference
times out. A camera that stays connected without sending frames, or a detector that
stops answering, would instead leave the robot running blind: the controllers see no
fresh target and hold still. This node watches both streams and raises when one that was
running stays silent too long; the launcher then stops every node, which brakes the
motors, and the bot exits for the boot service to start it again, reconnecting everything.
That restart is the protection against the historical freeze.

A stream that never started is not a freeze: its camera or detector is missing or failed,
which the launcher reports with how to fix it (core/bot/health.py), and restarting cannot
plug a camera in, while a bot that restarts every 20 seconds is no use for finding out
what is wrong. So a stream is watched only while its source counts as working
(`frames_expected`, `poses_expected`), and only once it has sent something. The cost: a
camera that drops off USB without its node noticing looks like a freeze, so the bot
restarts once, then starts without it and reports it missing.
"""
import asyncio
import logging
import time
from typing import Callable

from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.pubsub.bot_topics import FrameTopic, PoseDataMessage, PoseDataTopic
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)


class StreamStalledError(RuntimeError):
    """A watched stream went silent; the bot should stop and start again."""


class StallMonitorParams(NodeParams):
    """How long a running stream may stay silent before the bot stops."""

    frame_timeout_seconds: float = Field(default=5.0, gt=0.0, description="Longest allowed gap between camera frames")
    pose_timeout_seconds: float = Field(
        default=5.0, gt=0.0, description="Longest allowed gap between pose results for camera frames"
    )
    startup_seconds: float = Field(
        default=20.0, ge=0.0,
        description="How long a started camera may send nothing before it is reported as not streaming (no restart)",
    )
    check_interval_seconds: float = Field(default=0.5, gt=0.0)


class _Stream:
    """When a watched stream last delivered, since it was last expected to."""

    def __init__(self, name: str, timeout: float, expected: Callable[[], bool] | None) -> None:
        self.name, self.timeout, self.expected = name, timeout, expected or (lambda: True)
        self.last: float | None = None

    def check(self, now: float) -> None:
        if not self.expected():
            self.last = None  # Its source is down and reported; it must run again before it can freeze
        elif self.last is not None and now - self.last > self.timeout:
            raise StreamStalledError(f"No {self.name} for {now - self.last:.1f} s")


class StallMonitorNode(Node):
    """Raises StreamStalledError when camera frames, or pose results when watched, stop after running."""

    params: StallMonitorParams
    clock: Callable[[], float] = Field(default_factory=lambda: time.monotonic, exclude=True)
    frame_subscription: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)
    pose_subscription: SkipValidation[TopicSubscriptionQueue | None] = Field(default=None, exclude=True)
    # Whether each stream's source counts as working; None: always
    frames_expected: SkipValidation[Callable[[], bool] | None] = Field(default=None, exclude=True)
    poses_expected: SkipValidation[Callable[[], bool] | None] = Field(default=None, exclude=True)

    @classmethod
    def create(
        cls,
        *,
        pubsub: PubSubTopicManager,
        params: StallMonitorParams,
        watch_poses: bool,
        frames_expected: Callable[[], bool] | None = None,
        poses_expected: Callable[[], bool] | None = None,
    ) -> "StallMonitorNode":
        """Watch camera frames, and with `watch_poses` the pose results reported for them."""
        return cls(
            pubsub=pubsub,
            params=params,
            frame_subscription=pubsub.get_subscription(FrameTopic),
            pose_subscription=pubsub.get_subscription(PoseDataTopic) if watch_poses else None,
            frames_expected=frames_expected,
            poses_expected=poses_expected,
        )

    async def run(self) -> None:
        names = "camera frames" + (" and pose results" if self.pose_subscription is not None else "")
        logger.info(f"Starting StallMonitorNode [restarts the bot when {names} stop after running]")
        frames = _Stream("camera frames", self.params.frame_timeout_seconds, self.frames_expected)
        poses = _Stream("pose results", self.params.pose_timeout_seconds, self.poses_expected)
        while not self.stop_event.is_set():
            now = self.clock()
            while not self.frame_subscription.empty():
                await self.frame_subscription.get()
                frames.last = now
            frames.check(now)
            if self.pose_subscription is not None:
                while not self.pose_subscription.empty():
                    message: PoseDataMessage = await self.pose_subscription.get()
                    if message.frame is not None:  # Not the detector's "camera silent" report
                        poses.last = now
                poses.check(now)
            await asyncio.sleep(self.params.check_interval_seconds)
