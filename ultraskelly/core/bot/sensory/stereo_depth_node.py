"""Depth from the ZED's two eyes: rectify the newest pair, match it, publish depth in millimetres.

The node works at its own rate, at most `max_hz`, always on the newest pair, in a worker
thread. The CPU baseline matcher is OpenCV's semi-global block matching (StereoSGBM,
3-way mode); `stereonet` runs the Hailo Model Zoo's StereoNet on the Hailo-8 instead,
which needs the robot lock like every Hailo user. The depth image is pixel-aligned with
the rectified left image, which travels with it: together they are an RGB-D frame in
the rectified left camera's frame.
"""
import asyncio
import logging
import time
from pathlib import Path
from typing import Callable, Literal, Protocol

import numpy as np
from pydantic import Field, SkipValidation, field_validator

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.sensory.hailo_stereonet import DEFAULT_HEF
from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification, StereoRectifier, zed_rectifier
from ultraskelly.core.pubsub.bot_topics import DepthFrameMessage, DepthFrameTopic, StereoFrameMessage, StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

MAX_MILLIMETRES = np.iinfo(np.uint16).max
# Matches this close to the end of the search range are dropped: where the true disparity
# lies beyond it (the skull a few centimetres in front of the eyes), the matcher settles
# at the limit. On a live VGA pair that pile-up was a fifth of the valid pixels.
EDGE_MARGIN = 4


class StereoDepthNodeParams(NodeParams):
    """Parameters for StereoDepthNode."""

    matcher: Literal["sgbm", "stereonet"] = Field(
        default="sgbm", description="sgbm: OpenCV on the CPU; stereonet: StereoNet on the Hailo-8",
    )
    stereonet_hef: str = Field(default=DEFAULT_HEF, description="StereoNet compiled for the installed HailoRT")
    max_hz: float = Field(default=5.0, gt=0.0, le=30.0, description="Depth frames per second at most")
    num_disparities: int = Field(
        default=64, ge=16, le=256,
        description="Disparities searched, a multiple of 16; 64 at VGA reaches 0.33 m, 128 at HD720 0.34 m",
    )
    block_size: int = Field(default=5, ge=3, le=11, description="Matching window, odd")
    min_depth: float = Field(default=0.3, gt=0.0, description="Nearer depths are dropped, metres")
    max_depth: float = Field(default=8.0, gt=0.0, description="Further depths are dropped, metres")
    calibration: Path | None = Field(default=None, description="Stereolabs calibration file; None finds it by serial")

    @field_validator("num_disparities")
    @classmethod
    def _multiple_of_16(cls, value: int) -> int:
        if value % 16:
            raise ValueError("num_disparities must be a multiple of 16")
        return value

    @field_validator("block_size")
    @classmethod
    def _odd(cls, value: int) -> int:
        if value % 2 == 0:
            raise ValueError("block_size must be odd")
        return value


class DisparityMatcher(Protocol):
    """Finds, for each pixel of a rectified left image, how far left it appears in the right image."""

    name: str
    search_range: float  # Disparities it can find: 0 up to this many pixels
    color: bool  # Whether it matches colour images rather than grey

    def disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Rectified images in; float32 disparity in pixels out, zero or less where unknown."""
        ...

    def close(self) -> None:
        """Release what it holds."""
        ...


class SgbmMatcher:
    """OpenCV's semi-global block matcher, on the CPU."""

    name = "sgbm"
    color = False

    def __init__(self, num_disparities: int, block_size: int) -> None:
        import cv2

        self.search_range = num_disparities
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=num_disparities, blockSize=block_size,
            P1=8 * block_size ** 2, P2=32 * block_size ** 2, disp12MaxDiff=1, uniquenessRatio=10,
            speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        return self.matcher.compute(left, right).astype(np.float32) / 16.0  # Fixed point, 4 fractional bits

    def close(self) -> None:
        pass


def depth_millimetres(
    disparity: np.ndarray, rectification: StereoRectification, min_depth: float, max_depth: float,
    search_range: float = np.inf,
) -> np.ndarray:
    """Depth along the camera's forward axis in whole millimetres; 0 where unknown, out of
    range, or within EDGE_MARGIN pixels of the end of the matcher's search range."""
    with np.errstate(divide="ignore", invalid="ignore"):
        metres = rectification.focal_length * rectification.baseline / disparity
    known = (disparity > 0) & (disparity < search_range - EDGE_MARGIN) & (metres >= min_depth) & (metres <= max_depth)
    return np.where(known, np.minimum(np.rint(metres * 1000.0), MAX_MILLIMETRES), 0).astype(np.uint16)


def rgb_to_grey_with_opencv(image: np.ndarray) -> np.ndarray:
    import cv2

    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


class StereoDepthNode(Node):
    """Turns the newest stereo pair into depth, as often as `max_hz` allows."""

    params: StereoDepthNodeParams
    rectifier: SkipValidation[StereoRectifier | None] = Field(default=None, exclude=True)
    matcher: SkipValidation[DisparityMatcher] = Field(exclude=True)
    to_grey: SkipValidation[Callable[[np.ndarray], np.ndarray]] = Field(default=rgb_to_grey_with_opencv, exclude=True)
    pairs: SkipValidation[TopicSubscriptionQueue] = Field(exclude=True)
    frames_computed: int = Field(default=0, exclude=True)
    seconds_computing: float = Field(default=0.0, exclude=True)

    @classmethod
    def create(
        cls, *, pubsub: PubSubTopicManager, params: StereoDepthNodeParams, eye_size: tuple[int, int],
    ) -> "StereoDepthNode":
        """Factory method: rectification for the ZED's `eye_size` (width, height) and the matcher."""
        rectifier = zed_rectifier(eye_size, params.calibration)
        if params.matcher == "stereonet":
            from ultraskelly.core.bot.sensory.hailo_stereonet import HailoStereoNetMatcher

            matcher = HailoStereoNetMatcher(params.stereonet_hef, eye_size, centre_row=rectifier.rectification.cy)
        else:
            matcher = SgbmMatcher(params.num_disparities, params.block_size)
        return cls(
            pubsub=pubsub, params=params, rectifier=rectifier, matcher=matcher,
            pairs=pubsub.get_subscription(StereoFrameTopic),
        )

    async def run(self) -> None:
        """Depth for the newest pair, then a pause so the rate stays under `max_hz`."""
        logger.info(
            f"Starting StereoDepthNode [{self.matcher.name}, disparities up to {self.matcher.search_range:.0f} px, "
            f"at most {self.params.max_hz:g} Hz]"
        )
        try:
            while not self.stop_event.is_set():
                pair = await self.pairs.get()
                started = time.monotonic()
                depth = await asyncio.to_thread(self.compute, pair)
                elapsed = time.monotonic() - started
                await self.pubsub.topics[DepthFrameTopic].publish(depth)
                self.frames_computed += 1
                self.seconds_computing += elapsed
                await asyncio.sleep(max(0.0, 1.0 / self.params.max_hz - elapsed))
        finally:
            self.matcher.close()
            average = self.seconds_computing / max(self.frames_computed, 1)
            logger.info(f"StereoDepthNode stopped after {self.frames_computed} depth frames, {1000 * average:.0f} ms each")

    def compute(self, pair: StereoFrameMessage) -> DepthFrameMessage:
        """Rectify (unless the pair already is), match and convert to millimetres. Runs in a worker thread."""
        if pair.rectification is not None:
            color, right, rectification = pair.left, pair.right, pair.rectification
        else:
            color = self.rectifier.rectify_eye(0, pair.left)
            # Grey before rectifying, when the matcher works in grey: a third of the remap work.
            right = self.rectifier.rectify_eye(1, pair.right if self.matcher.color else self.to_grey(pair.right))
            rectification = self.rectifier.rectification
        if self.matcher.color:
            disparity = self.matcher.disparity(color, right)
        else:
            disparity = self.matcher.disparity(self.to_grey(color), right if right.ndim == 2 else self.to_grey(right))
        return DepthFrameMessage(
            depth=depth_millimetres(
                disparity, rectification, self.params.min_depth, self.params.max_depth, self.matcher.search_range,
            ),
            color=color,
            rectification=rectification,
            frame_number=pair.frame_number,
            timestamp=pair.timestamp,
            method=self.matcher.name,
        )
