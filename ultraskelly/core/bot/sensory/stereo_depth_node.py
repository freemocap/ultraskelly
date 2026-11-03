"""Depth from the ZED's two eyes: rectify the newest pair, match it, publish depth in millimetres.

The node works at its own rate, at most `max_hz`, always on the newest pair, in a worker
thread. The default matcher is OpenCV's semi-global block matching (StereoSGBM, 3-way
mode) on the CPU, at half the eye size: rectifying straight into 336x188 images from
VGA eyes, it makes a depth frame in about 15 ms of one core. `stereonet` runs the Hailo
Model Zoo's StereoNet on the Hailo-8 instead, at full size; it needs the robot lock like
every Hailo user and holds the chip for about 200 ms a frame, which pose has to wait for.
The depth image is pixel-aligned with the rectified left image, which travels with it:
together they are an RGB-D frame in the rectified left camera's frame. The grey matcher
takes each eye's Y bytes as the camera sent them; only the left eye is converted to
colour, for that image (StereoNet, which matches colour, converts both).
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
from ultraskelly.core.bot.sensory.stereo_mask import ViewMask, view_mask_file
from ultraskelly.core.bot.sensory.stereo_rectification import (
    StereoRectification,
    StereoRectifier,
    find_zed_serial,
    zed_rectifier,
)
from ultraskelly.core.bot.sensory.zed_camera_node import yuyv_to_rgb_with_opencv
from ultraskelly.core.pubsub.bot_topics import DepthFrameMessage, DepthFrameTopic, StereoFrameMessage, StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

MAX_MILLIMETRES = np.iinfo(np.uint16).max
# Matches this close to the end of the search range are dropped: where the true disparity
# lies beyond it (the skull a few centimetres in front of the eyes), the matcher settles
# at the limit. On a live VGA pair that pile-up was a fifth of the valid pixels.
EDGE_MARGIN = 4
REPORT_SECONDS = 30.0  # At most one logged failure this often


class StereoDepthNodeParams(NodeParams):
    """Parameters for StereoDepthNode."""

    matcher: Literal["sgbm", "stereonet"] = Field(
        default="sgbm", description="sgbm: OpenCV on the CPU; stereonet: StereoNet on the Hailo-8",
    )
    stereonet_hef: str = Field(default=DEFAULT_HEF, description="StereoNet compiled for the installed HailoRT")
    max_hz: float | None = Field(
        default=None, gt=0.0, le=30.0,
        description="Depth frames per second at most; None: 10 with sgbm, 1 with stereonet, which holds the "
                    "Hailo-8 about 200 ms a frame while pose waits (pose falls to about 25 fps at 1 Hz)",
    )

    def rate(self) -> float:
        """The depth frames per second aimed for."""
        if self.max_hz is not None:
            return self.max_hz
        return 1.0 if self.matcher == "stereonet" else 10.0
    num_disparities: int = Field(
        default=64, ge=16, le=256,
        description="Disparities searched, in full-size eye pixels, a multiple of 16; 64 at VGA reaches 0.36 m",
    )
    block_size: int = Field(default=5, ge=3, le=11, description="Matching window, odd, in the matched image's pixels")
    scale: float = Field(
        default=0.5, gt=0.0, le=1.0,
        description="Rectify and match at this fraction of the eye size: 0.5 makes 336x188 depth from VGA eyes "
                    "for about a sixth of the full-size work. StereoNet always runs at full size.",
    )
    min_depth: float = Field(default=0.3, gt=0.0, description="Nearer depths are dropped, metres")
    max_depth: float = Field(default=8.0, gt=0.0, description="Further depths are dropped, metres")
    calibration: Path | None = Field(default=None, description="Stereolabs calibration file; None finds it by serial")
    view_mask: Path | None = Field(
        default=None,
        description="Image of where both eyes see the room (white), in the rectified left eye; None uses "
                    "ultraskelly/calibration/zed/SN<serial>-view-mask.png when it exists",
    )

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
    """OpenCV's semi-global block matcher, on the CPU. With `scale` below 1 it matches smaller
    copies of the images (a quarter of the work at 0.5) and returns disparities for the
    full-size image, each repeated over the pixels it stands for."""

    name = "sgbm"
    color = False

    def __init__(self, num_disparities: int, block_size: int, scale: float = 1.0) -> None:
        import cv2

        self._cv2 = cv2
        self.scale = scale
        searched = max(16, 16 * round(num_disparities * scale / 16))
        self.search_range = searched / scale  # In full-size pixels
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=searched, blockSize=block_size,
            P1=8 * block_size ** 2, P2=32 * block_size ** 2, disp12MaxDiff=1, uniquenessRatio=10,
            speckleWindowSize=max(10, round(100 * scale ** 2)), speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        cv2 = self._cv2
        if self.scale == 1.0:
            return self.matcher.compute(left, right).astype(np.float32) / 16.0  # Fixed point, 4 fractional bits
        height, width = left.shape
        small = (round(width * self.scale), round(height * self.scale))
        fixed = self.matcher.compute(*(cv2.resize(image, small, interpolation=cv2.INTER_AREA) for image in (left, right)))
        disparity = fixed.astype(np.float32) / (16.0 * self.scale)
        return cv2.resize(disparity, (width, height), interpolation=cv2.INTER_NEAREST)

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


class StereoDepthNode(Node):
    """Turns the newest stereo pair into depth, as often as `max_hz` allows."""

    params: StereoDepthNodeParams
    rectifier: SkipValidation[StereoRectifier | None] = Field(default=None, exclude=True)
    view_mask: SkipValidation[ViewMask | None] = Field(default=None, exclude=True)
    matcher: SkipValidation[DisparityMatcher] = Field(exclude=True)
    to_rgb: SkipValidation[Callable[[np.ndarray], np.ndarray]] = Field(default=yuyv_to_rgb_with_opencv, exclude=True)
    pairs: SkipValidation[TopicSubscriptionQueue] = Field(exclude=True)
    frames_computed: int = Field(default=0, exclude=True)
    seconds_computing: float = Field(default=0.0, exclude=True)

    @classmethod
    def create(
        cls, *, pubsub: PubSubTopicManager, params: StereoDepthNodeParams, eye_size: tuple[int, int],
    ) -> "StereoDepthNode":
        """Factory method: rectification for the ZED's `eye_size` (width, height) and the matcher."""
        scale = params.scale if params.matcher == "sgbm" else 1.0
        rectifier = zed_rectifier(eye_size, params.calibration, scale)
        shared = rectifier.rectification
        size = (shared.width, shared.height)
        if params.matcher == "stereonet":
            from ultraskelly.core.bot.sensory.hailo_stereonet import HailoStereoNetMatcher

            matcher = HailoStereoNetMatcher(params.stereonet_hef, size, centre_row=shared.cy)
        else:  # The images are already scaled, so the matcher searches proportionally fewer disparities
            matcher = SgbmMatcher(max(16, 16 * round(params.num_disparities * scale / 16)), params.block_size)
        mask_path = params.view_mask
        if mask_path is None and (serial := find_zed_serial()) is not None and view_mask_file(serial).is_file():
            mask_path = view_mask_file(serial)
        view_mask = ViewMask.load(mask_path, size) if mask_path is not None else None
        if view_mask is not None:
            logger.info(f"Depth only where both eyes see the room: {100 * view_mask.valid.mean():.0f}% of the view ({mask_path.name})")
        return cls(
            pubsub=pubsub, params=params, rectifier=rectifier, matcher=matcher, view_mask=view_mask,
            pairs=pubsub.get_subscription(StereoFrameTopic),
        )

    async def run(self) -> None:
        """Depth for the newest pair, then a pause so the rate stays under `max_hz`."""
        rate = self.params.rate()
        logger.info(
            f"Starting StereoDepthNode [{self.matcher.name}, disparities up to {self.matcher.search_range:.0f} px, "
            f"at most {rate:g} Hz]"
        )
        computing: asyncio.Future | None = None
        failures, reported_at = 0, -REPORT_SECONDS
        try:
            while not self.stop_event.is_set():
                pair = await self.pairs.get()
                started = time.monotonic()
                # Shielded, so a cancelled run() still waits for the worker before the matcher is closed.
                computing = asyncio.ensure_future(asyncio.to_thread(self.compute, pair))
                try:
                    depth = await asyncio.shield(computing)
                except Exception:  # Depth is optional: a bad frame is logged, never the end of the bot
                    failures += 1
                    if time.monotonic() - reported_at >= REPORT_SECONDS:
                        logger.exception(f"Depth failed ({failures} frames so far); carrying on")
                        reported_at = time.monotonic()
                else:
                    await self.pubsub.topics[DepthFrameTopic].publish(depth)
                    self.frames_computed += 1
                    self.seconds_computing += time.monotonic() - started
                await asyncio.sleep(max(0.0, 1.0 / rate - (time.monotonic() - started)))
        finally:
            if computing is not None and not computing.done():
                await asyncio.wait({computing})
            self.matcher.close()
            average = self.seconds_computing / max(self.frames_computed, 1)
            logger.info(f"StereoDepthNode stopped after {self.frames_computed} depth frames, {1000 * average:.0f} ms each")

    def compute(self, pair: StereoFrameMessage) -> DepthFrameMessage:
        """Rectify, match and convert to millimetres. Runs in a worker thread."""
        rectification = self.rectifier.rectification
        color = self.rectifier.rectify_eye(0, self.to_rgb(pair.eye(0)))
        if self.matcher.color:
            disparity = self.matcher.disparity(color, self.rectifier.rectify_eye(1, self.to_rgb(pair.eye(1))))
        else:  # The Y bytes are the grey images: rectified as they are, a third of the colour remap's work
            left_grey, right_grey = (self.rectifier.rectify_eye(eye, pair.grey(eye)) for eye in (0, 1))
            if self.view_mask is None:
                disparity = self.matcher.disparity(left_grey, right_grey)
            else:  # Match only the window around the pixels both eyes see
                rows, columns = self.view_mask.window(self.matcher.search_range)
                disparity = np.zeros(left_grey.shape, np.float32)
                disparity[rows, columns] = self.matcher.disparity(
                    np.ascontiguousarray(left_grey[rows, columns]), np.ascontiguousarray(right_grey[rows, columns]),
                )
        if self.view_mask is not None:
            disparity = np.where(self.view_mask.valid, disparity, 0)
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
