"""Ends the bot when its camera or pose detector goes quiet, so the service restarts it.

The camera node raises when its pipeline stops, and the Hailo detector when an inference
times out. A camera that stays connected without sending frames, or a detector that
stops answering, would instead leave the robot running blind: the controllers see no
fresh target and hold still. This node watches both streams and raises when either
stays silent too long; the launcher then stops every node, which brakes the motors, and
the bot exits for the boot service to start it again, reconnecting everything.
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
    """How long each stream may stay silent before the bot stops."""

    frame_timeout_seconds: float = Field(default=5.0, gt=0.0, description="Longest allowed gap between camera frames")
    pose_timeout_seconds: float = Field(
        default=5.0, gt=0.0, description="Longest allowed gap between pose results for camera frames"
    )
    startup_seconds: float = Field(
        default=20.0, ge=0.0, description="Allowed wait for each stream's first message after the nodes start"
    )
    check_interval_seconds: float = Field(default=0.5, gt=0.0)


class StallMonitorNode(Node):
    """Raises StreamStalledError when camera frames, or pose results when watched, stop arriving."""

    params: StallMonitorParams
    clock: Callable[[], float] = Field(default_factory=lambda: time.monotonic, exclude=True)
    frame_subscription: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)
    pose_subscription: SkipValidation[TopicSubscriptionQueue | None] = Field(default=None, exclude=True)

    @classmethod
    def create(
        cls, *, pubsub: PubSubTopicManager, params: StallMonitorParams, watch_poses: bool,
    ) -> "StallMonitorNode":
        """Watch camera frames, and with `watch_poses` the pose results reported for them."""
        return cls(
            pubsub=pubsub,
            params=params,
            frame_subscription=pubsub.get_subscription(FrameTopic),
            pose_subscription=pubsub.get_subscription(PoseDataTopic) if watch_poses else None,
        )

    async def run(self) -> None:
        streams = "camera frames" + (" and pose results" if self.pose_subscription is not None else "")
        logger.info(f"Starting StallMonitorNode [restarts the bot when {streams} stop]")
        started = self.clock()
        last_frame: float | None = None
        last_pose: float | None = None
        while not self.stop_event.is_set():
            now = self.clock()
            while not self.frame_subscription.empty():
                await self.frame_subscription.get()
                last_frame = now
            self._check("camera frames", last_frame, self.params.frame_timeout_seconds, started, now)
            if self.pose_subscription is not None:
                while not self.pose_subscription.empty():
                    message: PoseDataMessage = await self.pose_subscription.get()
                    if message.frame is not None:  # Not the detector's "camera silent" report
                        last_pose = now
                self._check("pose results", last_pose, self.params.pose_timeout_seconds, started, now)
            await asyncio.sleep(self.params.check_interval_seconds)

    def _check(self, stream: str, last: float | None, timeout: float, started: float, now: float) -> None:
        if last is None:
            if now - started > self.params.startup_seconds:
                raise StreamStalledError(f"No {stream} within {self.params.startup_seconds:g} s of starting")
        elif now - last > timeout:
            raise StreamStalledError(f"No {stream} for {now - last:.1f} s")
