"""Where the tracked face is in 3D: the OAK's target pixel followed into the ZED's depth.

The pose detector finds the target (the nose by default) in the OAK image. Its ray, carried
into the ZED's rectified left camera by the OAK-ZED alignment, crosses the ZED's depth where
it first passes behind the surface the ZED sees: that is the face. The node publishes the
point in the camera's frame and in the head frame (origin between the eye sockets, x
forward, y to the skull's left, z up), and its distance from the head, with each pose
result, using the newest depth frame taken close enough in time.
"""
import logging
import time
import warnings
from pathlib import Path

import numpy as np
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.sensory.head_frame import nominal_camera_to_head
from ultraskelly.core.bot.sensory.oak_zed_alignment import (
    ALIGNMENT_FILE,
    OAK_FIELD_OF_VIEW,
    OakZedAlignment,
    intrinsics_from_field_of_view,
)
from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.bot_topics import (
    DepthFrameMessage,
    DepthFrameTopic,
    FaceLocationMessage,
    FaceLocationTopic,
    PoseDataMessage,
    PoseDataTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

# Until the alignment is measured: the OAK upright (the bot turns its image), centred between
# the eyes and about 3.5 cm below them, looking the same way as the ZED.
NOMINAL_TRANSLATION = (0.0315, 0.035, 0.0)  # Metres in the ZED's rectified left frame


def nominal_alignment(oak_size: tuple[int, int] = (640, 480)) -> OakZedAlignment:
    return OakZedAlignment(
        oak_camera_matrix=intrinsics_from_field_of_view(oak_size, OAK_FIELD_OF_VIEW), oak_size=oak_size,
        rotation=np.eye(3), translation=np.array(NOMINAL_TRANSLATION),
    )


def depth_along(depth: np.ndarray, columns: np.ndarray, rows: np.ndarray, radius: int) -> np.ndarray:
    """Median known depth (metres) in a (2 radius + 1)-pixel square around each pixel; NaN where none."""
    offsets = np.arange(-radius, radius + 1)
    near_columns = np.clip(columns[:, None, None] + offsets[None, None, :], 0, depth.shape[1] - 1)
    near_rows = np.clip(rows[:, None, None] + offsets[None, :, None], 0, depth.shape[0] - 1)
    values = depth[near_rows, near_columns].reshape(len(columns), offsets.size ** 2).astype(float)  # Also for no pixels
    values[values == 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # A window with no depth gives NaN, as wanted
        return np.nanmedian(values, axis=1) / 1000.0


def first_crossing(
    origin: np.ndarray, direction: np.ndarray, depth: np.ndarray, rectification: StereoRectification,
    distances: np.ndarray, radius: int,
) -> float | None:
    """The distance along the ray where it first passes behind the surface, between samples, or None."""
    points = origin[None, :] + distances[:, None] * direction[None, :]
    z = points[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        columns = rectification.focal_length * points[:, 0] / z + rectification.cx
        rows = rectification.focal_length * points[:, 1] / z + rectification.cy
    inside = (z > 0.05) & (columns >= 0) & (rows >= 0) & (columns <= depth.shape[1] - 1) & (rows <= depth.shape[0] - 1)
    gap = np.full(len(distances), np.nan)
    gap[inside] = depth_along(depth, np.rint(columns[inside]).astype(int), np.rint(rows[inside]).astype(int), radius) - z[inside]
    crossings = np.flatnonzero((gap[:-1] > 0) & (gap[1:] <= 0))  # NaN compares false: unknown depth never crosses
    if crossings.size == 0:
        return None
    first = crossings[0]
    fraction = gap[first] / (gap[first] - gap[first + 1])
    return float(distances[first] + fraction * (distances[first + 1] - distances[first]))


def locate_along_ray(
    origin: np.ndarray, direction: np.ndarray, depth: np.ndarray, rectification: StereoRectification,
    near: float = 0.3, far: float = 6.0, radius: int = 2,
) -> np.ndarray | None:
    """Where the ray first passes behind the surface in a depth image (ZED frame, metres), or None.
    Marches in 2 cm steps, then again in 2 mm steps around the first crossing."""
    coarse = first_crossing(origin, direction, depth, rectification, np.arange(near, far, 0.02), radius)
    if coarse is None:
        return None
    fine = first_crossing(origin, direction, depth, rectification,
                          np.arange(max(near, coarse - 0.04), coarse + 0.04, 0.002), radius)
    return origin + (fine if fine is not None else coarse) * direction


class FaceLocatorParams(NodeParams):
    """Parameters for FaceLocatorNode."""

    alignment: Path | None = Field(
        default=None,
        description="OAK-ZED alignment file; None uses ultraskelly/calibration/oak-zed-alignment.json when it "
                    "exists, else a nominal alignment from the skull's layout",
    )
    max_depth_age: float = Field(default=0.25, gt=0.0, description="Seconds between pose and depth frames, at most")


class FaceLocatorNode(Node):
    """Publishes the tracked face's 3D position with each pose result."""

    params: FaceLocatorParams
    alignment: SkipValidation[OakZedAlignment] = Field(exclude=True)
    alignment_source: str = Field(default="nominal", exclude=True)
    poses: SkipValidation[TopicSubscriptionQueue] = Field(exclude=True)
    depths: SkipValidation[TopicSubscriptionQueue] = Field(exclude=True)
    latest_depth: SkipValidation[DepthFrameMessage | None] = Field(default=None, exclude=True)
    size_warned: bool = Field(default=False, exclude=True)
    calls: int = Field(default=0, exclude=True)
    located: int = Field(default=0, exclude=True)
    seconds_locating: float = Field(default=0.0, exclude=True)

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: FaceLocatorParams) -> "FaceLocatorNode":
        """Factory method: the alignment, measured if a file exists, and the pose and depth subscriptions."""
        path = params.alignment or (ALIGNMENT_FILE if ALIGNMENT_FILE.is_file() else None)
        alignment = OakZedAlignment.load(path) if path is not None else nominal_alignment()
        source = path.name if path is not None else "nominal"
        logger.info(f"Face locator using the {source} OAK-ZED alignment")
        return cls(
            pubsub=pubsub, params=params, alignment=alignment, alignment_source=source,
            poses=pubsub.get_subscription(PoseDataTopic), depths=pubsub.get_subscription(DepthFrameTopic),
        )

    def locate(self, pose: PoseDataMessage) -> FaceLocationMessage:
        """The face for one pose result, from the newest depth frame close enough in time. A result
        without its frame's time, or from an image of another size than the alignment's, has none."""
        target, depth = pose.target, self.latest_depth
        pose_time = pose.frame.timestamp if pose.frame is not None else None
        empty = FaceLocationMessage(pose_timestamp=pose_time, alignment=self.alignment_source)
        if target is None or target.x is None or target.y is None or depth is None:
            return empty
        if pose_time is None or depth.timestamp is None or abs(pose_time - depth.timestamp) > self.params.max_depth_age:
            return empty
        size = (pose.frame.frame.shape[1], pose.frame.frame.shape[0])
        if size != self.alignment.oak_size:
            if not self.size_warned:
                logger.warning(f"Pose frames are {size[0]}x{size[1]} but the OAK-ZED alignment is for "
                               f"{self.alignment.oak_size[0]}x{self.alignment.oak_size[1]}: no face positions")
                self.size_warned = True
            return empty
        origin, direction = self.alignment.ray((target.x, target.y))
        point = locate_along_ray(origin, direction, depth.depth, depth.rectification)
        if point is None:
            return FaceLocationMessage(pose_timestamp=pose_time, depth_timestamp=depth.timestamp,
                                       pixel=(target.x, target.y), alignment=self.alignment_source)
        pose_in_head = nominal_camera_to_head(depth.rectification)
        in_head = np.array(pose_in_head["rotation"]) @ point + np.array(pose_in_head["translation_m"])
        return FaceLocationMessage(
            position_camera=tuple(float(value) for value in point),
            position_head=tuple(float(value) for value in in_head),
            distance=float(np.linalg.norm(in_head)),
            pixel=(target.x, target.y),
            pose_timestamp=pose_time,
            depth_timestamp=depth.timestamp,
            alignment=self.alignment_source,
        )

    async def run(self) -> None:
        logger.info("Starting FaceLocatorNode")
        try:
            while not self.stop_event.is_set():
                pose = await self.poses.get()
                while not self.depths.empty():
                    self.latest_depth = self.depths.get_nowait()
                started = time.perf_counter()
                face = self.locate(pose)
                self.seconds_locating += time.perf_counter() - started
                self.calls += 1
                self.located += face.distance is not None
                await self.pubsub.topics[FaceLocationTopic].publish(face)
        finally:
            average = 1000 * self.seconds_locating / max(self.calls, 1)
            logger.info(f"FaceLocatorNode stopped: a face in {self.located} of {self.calls} pose results, {average:.2f} ms each")
