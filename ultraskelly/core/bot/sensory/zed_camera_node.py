"""Stereolabs ZED Mini over plain UVC, publishing its two eyes to StereoFrameTopic.

The ZED SDK needs CUDA, so on the Pi the camera is read as an ordinary V4L2 webcam: each
frame holds both eyes side by side in YUYV, left eye first. The modes are fixed; their
names match the ZED SDK's and the sections of Stereolabs' factory calibration files.
Frame times are the kernel's buffer timestamps, on time.monotonic()'s clock like the
OAK's: when the frame's first USB packet arrived, one transfer time before it can be read.
"""
import asyncio
import logging
import threading
import time
from enum import StrEnum
from pathlib import Path
from typing import Callable, Protocol

import numpy as np
from pydantic import Field, SkipValidation, model_validator

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.pubsub.bot_topics import StereoFrameMessage, StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

VIDEO_DEVICES = Path("/sys/class/video4linux")
REOPEN_SECONDS = 1.0  # Between attempts to reopen a camera that stopped streaming
REPORT_SECONDS = 30.0  # At most one warning this often about dropped frames or failed reopening


class ZedResolution(StrEnum):
    """Side-by-side UVC modes, named like the ZED SDK's resolutions."""

    HD2K = "HD2K"
    HD1080 = "HD1080"
    HD720 = "HD720"
    VGA = "VGA"


# Both eyes' total (width, height), and the frame rates the camera offers in that mode.
ZED_MODES: dict[ZedResolution, tuple[tuple[int, int], tuple[int, ...]]] = {
    ZedResolution.HD2K: ((4416, 1242), (15,)),
    ZedResolution.HD1080: ((3840, 1080), (30, 15)),
    ZedResolution.HD720: ((2560, 720), (60, 30, 15)),
    ZedResolution.VGA: ((1344, 376), (100, 60, 30, 15)),
}
NOT_FOUND = (
    "No ZED camera among the video devices. If lsusb lists only its 2b03:f681 HID "
    "interface, flip its USB-C plug at the camera end."
)


class ZedCameraNodeParams(NodeParams):
    """Parameters for ZedCameraNode."""

    resolution: ZedResolution = Field(default=ZedResolution.VGA, description="VGA is 672x376 per eye, HD720 1280x720")
    fps: int = Field(default=30, description="Frames per second; each resolution offers a fixed set")
    device: str | None = Field(default=None, description="V4L2 device such as /dev/video0; None finds the first ZED")

    @model_validator(mode="after")
    def _check_mode(self) -> "ZedCameraNodeParams":
        rates = ZED_MODES[self.resolution][1]
        if self.fps not in rates:
            raise ValueError(f"{self.resolution} runs at {', '.join(map(str, rates))} fps, not {self.fps}")
        return self


def find_zed_device(devices: Path = VIDEO_DEVICES) -> str | None:
    """The first ZED's video capture node, found by the name the camera reports.

    A ZED has two nodes: video capture (index 0) and metadata (index 1).
    """
    for node in sorted(devices.glob("video*"), key=lambda path: (len(path.name), path.name)):
        try:
            name, index = (node / "name").read_text(), (node / "index").read_text().strip()
        except OSError:
            continue
        if "ZED" in name and index == "0":
            return f"/dev/{node.name}"
    return None


def has_lost_rows(yuyv: np.ndarray, row_step: int = 4) -> bool:
    """Whether rows of a (height, width, 2) YUYV frame were lost in the USB transfer.

    They arrive as zeros, which no image contains: black is Y=16 with U=V=128, and even
    full-range black keeps U and V at 128, so at most half of a row's bytes are zero.
    Checks every `row_step`th row.
    """
    rows = yuyv[::row_step].reshape(-1, yuyv.shape[1] * yuyv.shape[2])
    return bool((np.count_nonzero(rows, axis=1) < rows.shape[1] // 10).any())


class FramePeriodCounter:
    """Numbers frames by elapsed camera frame periods, so a dropped frame skips a number."""

    def __init__(self, fps: float) -> None:
        self.period = 1.0 / fps
        self.number = -1
        self.last: float | None = None

    def advance(self, timestamp: float) -> int:
        periods = 1 if self.last is None else max(1, round((timestamp - self.last) / self.period))
        self.number += periods
        self.last = timestamp
        return self.number


class SideBySideSource(Protocol):
    """A camera stream of side-by-side YUYV frames."""

    def read(self) -> tuple[np.ndarray, float] | None:
        """Block for the next frame: its (height, width, 2) YUYV bytes and capture time, or None once the camera stopped."""
        ...

    def close(self) -> None:
        ...


class OpenCvSideBySideSource:
    """A ZED's raw YUYV frames and their kernel timestamps, through OpenCV's V4L2 backend."""

    def __init__(self, capture: object, size: tuple[int, int], timestamp_property: int) -> None:
        self.capture, self.size, self.timestamp_property = capture, size, timestamp_property

    @classmethod
    def open(cls, device: str, size: tuple[int, int], fps: int) -> "OpenCvSideBySideSource":
        import cv2

        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"Could not open the ZED at {device}")
        width, height = size
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"YUYV"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)  # Raw YUYV: each eye is converted to RGB once
        streaming = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if streaming != size:
            capture.release()
            raise RuntimeError(f"The ZED at {device} streams {streaming[0]}x{streaming[1]}, not {width}x{height}")
        logger.info(f"Opened the ZED at {device}: {width}x{height} side by side @ {fps} fps")
        return cls(capture, size, cv2.CAP_PROP_POS_MSEC)

    def read(self) -> tuple[np.ndarray, float] | None:
        ok, frame = self.capture.read()
        if not ok:
            return None
        width, height = self.size
        # The V4L2 buffer timestamp: CLOCK_MONOTONIC under uvcvideo's default clock.
        return frame.reshape(height, width, 2), self.capture.get(self.timestamp_property) / 1000.0

    def close(self) -> None:
        self.capture.release()


def yuyv_to_rgb_with_opencv(yuyv: np.ndarray) -> np.ndarray:
    """One eye's (height, width, 2) YUYV as a contiguous (height, width, 3) RGB image."""
    import cv2

    return cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUYV)


class ZedCameraNode(Node):
    """Streams the ZED's left and right eyes as RGB pairs, reopening the camera if it stops.

    A ZED that stops streaming does not stop the bot: the node retries once a second.
    """

    params: ZedCameraNodeParams
    open_source: SkipValidation[Callable[[], SideBySideSource]] = Field(exclude=True)
    source: SkipValidation[SideBySideSource | None] = Field(default=None, exclude=True)
    to_rgb: SkipValidation[Callable[[np.ndarray], np.ndarray]] = Field(
        default=yuyv_to_rgb_with_opencv, exclude=True
    )
    # Held while a worker thread reads, so the camera is never closed under a read.
    source_lock: SkipValidation[threading.Lock] = Field(default_factory=threading.Lock, exclude=True)
    frames_published: int = Field(default=0, exclude=True)
    frames_lost: int = Field(default=0, exclude=True, description="Frames with rows lost in the USB transfer, not published")

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: ZedCameraNodeParams) -> "ZedCameraNode":
        """Factory method: finds the ZED and opens its side-by-side stream in the configured mode."""
        size = ZED_MODES[params.resolution][0]

        def open_source() -> SideBySideSource:
            device = params.device or find_zed_device()  # Searched again on reopening: the number can change
            if device is None:
                raise RuntimeError(NOT_FOUND)
            return OpenCvSideBySideSource.open(device, size, params.fps)

        node = cls(pubsub=pubsub, params=params, open_source=open_source)
        node.source = open_source()  # Fails at startup, not mid-run, if the camera cannot stream this mode
        return node

    async def run(self) -> None:
        """Main camera loop: reading and conversion run in a worker thread, publishing on the event loop."""
        width, height = ZED_MODES[self.params.resolution][0]
        logger.info(f"Starting ZedCameraNode [{width // 2}x{height} per eye @ {self.params.fps} fps]")
        counter = FramePeriodCounter(self.params.fps)
        reported_lost, reported_at = 0, time.monotonic()
        try:
            while not self.stop_event.is_set():
                if self.source is None:
                    await self._reopen()
                    continue
                pair = await asyncio.to_thread(self._capture, counter)
                if self.frames_lost > reported_lost and time.monotonic() - reported_at >= REPORT_SECONDS:
                    logger.warning(f"Dropped {self.frames_lost - reported_lost} ZED frames with rows lost in USB transfer")
                    reported_lost, reported_at = self.frames_lost, time.monotonic()
                if pair is None:
                    if not self.stop_event.is_set():
                        logger.warning("The ZED stopped streaming; reopening it")
                    await asyncio.to_thread(self._close)
                    continue
                await self.pubsub.topics[StereoFrameTopic].publish(pair)
                self.frames_published += 1
        finally:
            await asyncio.to_thread(self._close)
            logger.info(f"ZedCameraNode stopped after {self.frames_published} frames ({self.frames_lost} lost)")

    def _capture(self, counter: FramePeriodCounter) -> StereoFrameMessage | None:
        """Read until a whole frame arrives and split it into RGB eyes; None once the camera
        stopped or the node is stopping. Runs in a worker thread."""
        with self.source_lock:
            while self.source is not None and not self.stop_event.is_set():
                read = self.source.read()
                if read is None:
                    return None
                yuyv, timestamp = read
                frame_number = counter.advance(timestamp)
                if has_lost_rows(yuyv):
                    self.frames_lost += 1
                    continue
                half = yuyv.shape[1] // 2
                return StereoFrameMessage(
                    left=self.to_rgb(yuyv[:, :half]),
                    right=self.to_rgb(yuyv[:, half:]),
                    frame_number=frame_number,
                    timestamp=timestamp,
                )
        return None

    def _close(self) -> None:
        """Release the camera once no read is in progress."""
        with self.source_lock:
            if self.source is not None:
                self.source.close()
                self.source = None

    async def _reopen(self) -> None:
        """Try to reopen the camera once a second until it streams or the node stops."""
        attempts = 0
        while not self.stop_event.is_set():
            try:
                self.source = await asyncio.to_thread(self.open_source)
                logger.info(f"Reopened the ZED after {attempts} failed attempts")
                return
            except Exception as error:
                if attempts % round(REPORT_SECONDS / REOPEN_SECONDS) == 0:
                    logger.warning(f"Could not reopen the ZED (attempt {attempts + 1}): {error}")
                attempts += 1
            try:
                await asyncio.wait_for(self.stop_event.wait(), timeout=REOPEN_SECONDS)
            except TimeoutError:
                pass
