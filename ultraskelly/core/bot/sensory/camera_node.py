import asyncio
import logging

from pydantic import Field, SkipValidation

from ultraskelly import FAIL_ON_IMPORTS

logger = logging.getLogger(__name__)

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.pubsub.bot_topics import FrameMessage, FrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


class VisionNodeParams(NodeParams):
    """Parameters for VisionNode."""

    width: int = Field(default=640, ge=160, le=1920)
    height: int = Field(default=480, ge=120, le=1080)


class VisionNode(Node):
    """Captures frames and publishes them."""

    params: VisionNodeParams
    picam2: SkipValidation[object] = Field(default=None, exclude=True)  # picamera2's Picamera2

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: VisionNodeParams) -> "VisionNode":
        """Factory method to create and initialize VisionNode."""
        from picamera2 import Picamera2  # The Pi camera stack, imported here so the module loads anywhere

        node = cls(pubsub=pubsub, params=params)

        # Initialize camera
        node.picam2 = Picamera2()
        config = node.picam2.create_preview_configuration(
            main={"size": (params.width, params.height), "format": "RGB888"}
        )
        node.picam2.configure(config)

        return node

    async def run(self) -> None:
        """Main vision node loop."""
        logger.info(f"Starting VisionNode [{self.params.width}x{self.params.height}]")
        self.picam2.start()
        await asyncio.sleep(1)

        try:
            while not self.stop_event.is_set():
                frame = self.picam2.capture_array()
                await self.pubsub.topics[FrameTopic].publish(FrameMessage(frame=frame))
                await asyncio.sleep(0.001)
        finally:
            self.picam2.stop()
            logger.info("VisionNode stopped")