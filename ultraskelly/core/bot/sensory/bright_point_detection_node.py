import asyncio
import logging
import time

import numpy as np
from pydantic import Field, SkipValidation, field_validator

from ultraskelly.core.pubsub.bot_topics import TargetLocationMessage, TargetLocationTopic, FrameTopic

logger = logging.getLogger(__name__)

from ultraskelly.core.bot.base_abcs import DetectorNode, NodeParams
from ultraskelly.core.bot.sensory.camera_node import FrameMessage
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


class BrightnessDetectorParams(NodeParams):
    """Parameters for BrightnessDetectorNode."""

    blur_size: int = Field(default=15, ge=1, description="Must be odd")
    threshold: int = Field(default=100, ge=0, le=255)

    @field_validator("blur_size")
    @classmethod
    def validate_blur_size(cls, v: int) -> int:
        """Ensure blur size is odd."""
        if v % 2 == 0:
            raise ValueError("blur_size must be odd")
        return v


class BrightnessDetectorNode(DetectorNode):
    """Detects brightest point in frame with orientation."""

    params: BrightnessDetectorParams
    frame_queue: SkipValidation[object] = Field(default=None, exclude=True)

    @classmethod
    def create(
        cls, *, pubsub: PubSubTopicManager, params: BrightnessDetectorParams
    ) -> "BrightnessDetectorNode":
        """Factory method to create and initialize BrightnessDetectorNode."""
        node = cls(pubsub=pubsub, params=params)
        node.frame_queue = pubsub.topics[FrameTopic].get_subscription()
        return node

    def detect(self, image: np.ndarray) -> tuple[int | None, int | None, float | None]:
        """Find brightest point and its orientation in frame."""
        import cv2  # Here, so the module loads where OpenCV is not installed

        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (self.params.blur_size, self.params.blur_size), 0)

        # Threshold to get bright region
        _, binary = cv2.threshold(blurred, self.params.threshold, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return (None, None, None)

        # Get largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Need at least 5 points to fit ellipse
        if len(largest_contour) < 5:
            # Fall back to centroid only
            M = cv2.moments(largest_contour)
            if M["m00"] == 0:
                return (None, None, None)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy, 0.0)  # No rotation if we can't determine it

        # Fit ellipse to get orientation
        ellipse = cv2.fitEllipse(largest_contour)
        center, axes, angle = ellipse

        # Convert angle: OpenCV gives angle where 0° is horizontal right
        # We want 0° to be vertical up, so rotate by -90°
        rotation_angle = angle - 90.0

        # Normalize to [-90, 90] range
        while rotation_angle > 90:
            rotation_angle -= 180
        while rotation_angle < -90:
            rotation_angle += 180

        return (int(center[0]), int(center[1]), rotation_angle)

    async def run(self) -> None:
        """Main detection loop."""
        logger.info(
            f"Starting BrightnessDetectorNode [blur={self.params.blur_size}, "
            f"threshold={self.params.threshold}]"
        )

        try:
            while not self.stop_event.is_set():
                try:
                    frame_msg: FrameMessage = await asyncio.wait_for(
                        self.frame_queue.get(), timeout=0.1
                    )
                    x, y, angle = self.detect(frame_msg.frame)

                    await self.pubsub.topics[TargetLocationTopic].publish(
                        TargetLocationMessage(x=x, y=y, angle=angle)
                    )
                except asyncio.TimeoutError:
                    continue
        finally:
            logger.info("BrightnessDetectorNode stopped")