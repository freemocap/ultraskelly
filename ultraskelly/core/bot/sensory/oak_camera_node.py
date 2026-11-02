"""Luxonis OAK USB camera (such as the OAK-1) publishing RGB frames to FrameTopic."""
import asyncio
import logging

import numpy as np
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.pubsub.bot_topics import FrameMessage, FrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)


class OakCameraNodeParams(NodeParams):
    """Parameters for OakCameraNode."""

    width: int = Field(default=640, ge=160, le=1920)
    height: int = Field(default=480, ge=120, le=1080)
    fps: float = Field(default=30.0, gt=0.0, le=60.0)
    flip_camera: bool = Field(
        default=False,
        description="Rotate camera image 180 degrees (for upside-down mounting)",
    )


class OakCameraNode(Node):
    """Streams the OAK's color camera as RGB frames, like the Picamera2 nodes did."""

    params: OakCameraNodeParams
    pipeline: SkipValidation[object] = Field(default=None, exclude=True)
    frame_queue: SkipValidation[object] = Field(default=None, exclude=True)

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: OakCameraNodeParams) -> "OakCameraNode":
        """Factory method: connects to the first OAK found and configures its color stream."""
        import depthai as dai

        node = cls(pubsub=pubsub, params=params)
        node.pipeline = dai.Pipeline()  # Connects to and boots the first available OAK
        camera = node.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        node.frame_queue = camera.requestOutput(
            (params.width, params.height), type=dai.ImgFrame.Type.RGB888i, fps=params.fps
        ).createOutputQueue(maxSize=1, blocking=False)
        return node

    async def run(self) -> None:
        """Main camera loop."""
        logger.info(
            f"Starting OakCameraNode [{self.params.width}x{self.params.height} @ {self.params.fps:g} fps, "
            f"flip_camera={self.params.flip_camera}]"
        )
        self.pipeline.start()

        try:
            while not self.stop_event.is_set():
                if not self.pipeline.isRunning():
                    raise RuntimeError("OAK pipeline stopped unexpectedly (camera unplugged?)")

                message = self.frame_queue.tryGet()
                if message is None:
                    await asyncio.sleep(0.002)
                    continue

                frame = message.getFrame()
                if self.params.flip_camera:
                    frame = np.ascontiguousarray(frame[::-1, ::-1])
                await self.pubsub.topics[FrameTopic].publish(FrameMessage(frame=frame))
        finally:
            self.pipeline.stop()
            # Release the USB device so the next restart cycle can connect to it again.
            self.pipeline.getDefaultDevice().close()
            logger.info("OakCameraNode stopped")
