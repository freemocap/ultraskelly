"""Where the tracked face is in 3D: the OAK's target pixel followed into the ZED's depth.

The pose detector finds the target (the nose by default) in the OAK image. Its ray, carried
into the ZED's rectified left camera by the OAK-ZED alignment, crosses the ZED's depth where
it first passes behind the surface the ZED sees: that is the face. The node publishes the
point in the camera's frame and in the head frame (origin between the eye sockets, x
forward, y to the skull's left, z up), and its distance from the head, with each pose
result, using the newest depth frame taken close enough in time.

The followed person's other keypoints are placed the same way, more cheaply (place_on_depth): each ray is
taken to the face's distance, and the depth the ZED sees where it is there becomes the point's distance, twice
over. The OAK and the ZED sit a few centimetres apart, so the second look lands within a pixel or two.

The dashboard's Align button (align, POST /api/align) fits the OAK's turn from the ZED to the person standing in
front of the robot: for a few seconds it tries small turns and keeps the one that puts the most of their
keypoints on them in the depth (CameraAligner). The turn is kept in LEARNED_FILE and read back at each start.
"""
import asyncio
import json
import logging
import os
import time
import warnings
from dataclasses import replace
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


def place_on_depth(
    origin: np.ndarray, directions: np.ndarray, depth: np.ndarray, rectification: StereoRectification,
    guess: float, radius: int = 2, rounds: int = 2,
) -> np.ndarray:
    """Points (N, 3, ZED frame, metres) on rays from `origin` along `directions` (N, 3) where they meet the depth
    the ZED sees: from `guess` metres along each ray, the depth seen where the ray is becomes how far forward
    the point lies, `rounds` times. NaN rows where the ZED has no depth there."""
    distances = np.full(len(directions), float(guess))
    for _ in range(rounds):
        points = origin[None, :] + distances[:, None] * directions
        z = points[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            columns = rectification.focal_length * points[:, 0] / z + rectification.cx
            rows = rectification.focal_length * points[:, 1] / z + rectification.cy
        inside = (z > 0.05) & (columns >= 0) & (rows >= 0) & (columns <= depth.shape[1] - 1) & (rows <= depth.shape[0] - 1)
        seen = np.full(len(directions), np.nan)
        seen[inside] = depth_along(depth, np.rint(columns[inside]).astype(int), np.rint(rows[inside]).astype(int), radius)
        with np.errstate(divide="ignore", invalid="ignore"):
            distances = np.where(directions[:, 2] > 1e-6, (seen - origin[2]) / directions[:, 2], np.nan)
    return origin[None, :] + distances[:, None] * directions


LEARNED_FILE = Path.home() / ".config" / "ultraskelly" / "oak-zed-learned.json"  # The bot user's own
DROP = 1.0  # Keypoints a frame a turn may score below the best and still be among the best


def turn(yaw: float, pitch: float) -> np.ndarray:
    """A small turn in the ZED's frame (x right, y down, z forward), degrees: yaw turns forward toward the right,
    then pitch turns it up."""
    a, b = np.radians(yaw), np.radians(pitch)
    about_y = np.array([[np.cos(a), 0.0, np.sin(a)], [0.0, 1.0, 0.0], [-np.sin(a), 0.0, np.cos(a)]])
    about_x = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(b), np.sin(b)], [0.0, -np.sin(b), np.cos(b)]])
    return about_x @ about_y


class CameraAligner:
    """Turns the OAK's alignment to fit the person standing in front of the robot, when asked (align).

    The nominal alignment has the OAK in the nose hole facing exactly where the ZED in the eye sockets faces; a
    degree or two either way carries the person's keypoints, along their OAK rays, onto the wall behind them in
    the ZED's depth rather than onto them. So for each frame of the person it tries turns of the OAK about the
    ZED's vertical and horizontal axes within `span` degrees of the current one, and scores each by how many
    keypoints then land on depth near the person's own distance (softly, within about `sigma` metres). Summed
    over the frames, the middle of the best-scoring turns becomes the turn. It is kept in `path` (None: nowhere)
    and read back at the next start."""

    def __init__(self, base: OakZedAlignment, *, span: float = 6.0, step: float = 0.5, sigma: float = 0.3,
                 min_keypoints: int = 8, keypoint_score: float = 0.4, limit: float = 15.0,
                 path: Path | None = LEARNED_FILE) -> None:
        self.base, self.sigma, self.min_keypoints = base, sigma, min_keypoints
        self.keypoint_score, self.limit, self.path = keypoint_score, limit, path
        self.steps = np.arange(-span, span + step / 2, step)  # The turns tried each way, degrees
        self.offsets = np.array([(yaw, pitch) for yaw in self.steps for pitch in self.steps])  # (candidates, 2)
        self.centre = int(np.argmin(np.abs(self.offsets).sum(axis=1)))  # The current turn itself
        self.yaw = self.pitch = 0.0
        if path is not None and path.is_file():
            try:
                kept = json.loads(path.read_text(encoding="utf-8"))
                self.yaw = float(np.clip(kept["yaw_deg"], -limit, limit))
                self.pitch = float(np.clip(kept["pitch_deg"], -limit, limit))
            except (OSError, ValueError, KeyError, TypeError) as error:
                logger.warning(f"Could not read the OAK-ZED turn in {path}: {error}; starting from none")

    def alignment(self) -> OakZedAlignment:
        """The base alignment with the turn."""
        return replace(self.base, rotation=turn(self.yaw, self.pitch) @ self.base.rotation)

    def describe(self) -> str:
        return f"turned {self.yaw:+.1f} yaw, {self.pitch:+.1f} pitch degrees"

    def score(self, keypoints: np.ndarray, depth: np.ndarray, rectification: StereoRectification) -> np.ndarray | None:
        """For one frame of the person (17 rows of OAK x, y and score) and a depth frame from close in time, how
        many keypoints land on the person with each candidate turn; None when too few keypoints can tell."""
        sure = keypoints[:, 2] >= self.keypoint_score
        if sure.sum() < self.min_keypoints:
            return None
        pixels = np.column_stack([keypoints[sure, :2], np.ones(int(sure.sum()))])
        rays = np.linalg.solve(self.base.oak_camera_matrix, pixels.T).T
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)  # In the OAK camera's frame
        origin = self.base.translation
        # The person's distance: their keypoints placed on the depth with the current turn, the middle one
        placed = place_on_depth(origin, rays @ (turn(self.yaw, self.pitch) @ self.base.rotation).T, depth, rectification, 2.0)
        distances = np.linalg.norm(placed - origin, axis=1)
        distances = distances[np.isfinite(distances)]
        if len(distances) < self.min_keypoints:
            return None
        distance = float(np.median(distances))
        turns = np.array([turn(self.yaw + yaw, self.pitch + pitch) @ self.base.rotation for yaw, pitch in self.offsets])
        points = origin + distance * np.einsum("cij,nj->cni", turns, rays)  # (candidates, keypoints, 3) in the ZED frame
        z = points[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            columns = rectification.focal_length * points[..., 0] / z + rectification.cx
            rows = rectification.focal_length * points[..., 1] / z + rectification.cy
        inside = (z > 0.05) & (columns >= 0) & (rows >= 0) & (columns <= depth.shape[1] - 1) & (rows <= depth.shape[0] - 1)
        seen = np.full(z.shape, np.nan)
        seen[inside] = depth_along(depth, np.rint(columns[inside]).astype(int), np.rint(rows[inside]).astype(int), 1)
        return np.nan_to_num(np.exp(-((seen - z) / self.sigma) ** 2)).sum(axis=1)  # Keypoints on the person, softly

    def adopt(self, totals: np.ndarray, frames: int) -> dict:
        """Take the middle of the best turns from scores summed over `frames`, keep it, and say what changed.

        The best turns are those that put every keypoint the person shows on them; their middle is the turn
        only where the person's outline bounds them both ways. Arms and shoulders near both sides bound yaw;
        pitch is often open one way (the top of the head is well above the eyes, and the floor at the feet is as
        far away as the person), and an axis whose best turns run to the end of the search is left as it was."""
        grid = (totals / frames).reshape(len(self.steps), len(self.steps))  # Keypoints a frame: yaw down, pitch across
        before = float(totals[self.centre] / frames)
        row, column = np.unravel_index(int(np.argmax(grid)), grid.shape)
        after = float(grid[row, column])

        def middle(values: np.ndarray, at: int) -> float | None:
            """The middle of the run of turns around `at` that stays within DROP keypoints of it, in degrees;
            None when the run reaches the end of the search, so the person does not bound it that way."""
            low, high = at, at
            while low > 0 and values[low - 1] >= values[at] - DROP:
                low -= 1
            while high < len(values) - 1 and values[high + 1] >= values[at] - DROP:
                high += 1
            return None if low == 0 or high == len(values) - 1 else float(np.interp((low + high) / 2, np.arange(len(self.steps)), self.steps))

        middles = (middle(grid[:, column], row), middle(grid[row, :], column))
        aim = np.array([0.0 if value is None else value for value in middles])
        unbounded = [name for name, value in zip(("yaw", "pitch"), middles) if value is None]
        note = f"; {' and '.join(unbounded)} left as it was: you do not bound it" if unbounded else ""
        if after - before < 0.25 or not aim.any():  # Less than a quarter of a keypoint a frame better: leave it
            return {"ok": True, "changed": False, "yaw": self.yaw, "pitch": self.pitch, "frames": frames,
                    "before": round(float(before), 1), "after": round(float(before), 1),
                    "message": f"Already aligned: {before:.1f} keypoints a frame on you; {self.describe()}{note}"}
        self.yaw = float(np.clip(self.yaw + aim[0], -self.limit, self.limit))
        self.pitch = float(np.clip(self.pitch + aim[1], -self.limit, self.limit))
        self.save()
        return {"ok": True, "changed": True, "yaw": self.yaw, "pitch": self.pitch, "frames": frames,
                "before": round(float(before), 1), "after": round(float(after), 1),
                "message": f"Aligned: {after:.1f} keypoints a frame on you, {before:.1f} before; {self.describe()}{note}"}

    def save(self) -> None:
        """Keep the turn: written beside its file, then renamed over it, so a power cut leaves one whole."""
        if self.path is None:
            return
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            draft = self.path.with_name(f".{self.path.name}.new")
            kept = {"yaw_deg": round(self.yaw, 4), "pitch_deg": round(self.pitch, 4)}
            draft.write_text(json.dumps(kept, indent=2) + "\n", encoding="utf-8")
            os.replace(draft, self.path)
        except OSError as error:
            logger.warning(f"Could not keep the OAK-ZED turn in {self.path}: {error}")


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
    keypoint_score: float = Field(
        default=0.4, ge=0.0, le=1.0, description="A keypoint the pose detector is less sure of is left out of the 3D skeleton",
    )
    skeleton_guess: float = Field(
        default=2.0, gt=0.0, description="Metres the skeleton's rays start from when the face itself has no depth",
    )
    align_seconds: float = Field(
        default=2.5, gt=0.0, le=30.0, description="Seconds of the person in front of it an alignment (align) watches",
    )
    learned_file: Path | None = Field(
        default=LEARNED_FILE, description="Where the OAK's turn from aligning is kept across restarts; None keeps it nowhere",
    )


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
    aligner: SkipValidation[CameraAligner | None] = Field(default=None, exclude=True)
    base_source: str = Field(default="nominal", exclude=True)
    latest_pose: SkipValidation[PoseDataMessage | None] = Field(default=None, exclude=True)
    aligning: bool = Field(default=False, exclude=True)

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: FaceLocatorParams) -> "FaceLocatorNode":
        """Factory method: the alignment, measured if a file exists, turned as aligning last left it, and the pose
        and depth subscriptions."""
        path = params.alignment or (ALIGNMENT_FILE if ALIGNMENT_FILE.is_file() else None)
        alignment = OakZedAlignment.load(path) if path is not None else nominal_alignment()
        source = path.name if path is not None else "nominal"
        node = cls(
            pubsub=pubsub, params=params, alignment=alignment, alignment_source=source, base_source=source,
            poses=pubsub.get_subscription(PoseDataTopic), depths=pubsub.get_subscription(DepthFrameTopic),
        )
        node.aligner = CameraAligner(alignment, keypoint_score=params.keypoint_score, path=params.learned_file)
        node.adopt()
        logger.info(f"Face locator using the {node.alignment_source} OAK-ZED alignment")
        return node

    def adopt(self) -> None:
        """Place with the aligner's alignment from now on."""
        self.alignment = self.aligner.alignment()
        turned = self.aligner.yaw or self.aligner.pitch
        self.alignment_source = f"{self.base_source}, {self.aligner.describe()}" if turned else self.base_source

    def usable(self, pose: PoseDataMessage | None) -> tuple[np.ndarray, DepthFrameMessage] | None:
        """The followed person's keypoints in a pose result, and the depth frame from close enough in time."""
        depth = self.latest_depth
        if (pose is None or pose.keypoints is None or pose.target_person is None
                or not 0 <= pose.target_person < len(pose.keypoints) or depth is None or pose.frame is None
                or pose.frame.timestamp is None or depth.timestamp is None
                or abs(pose.frame.timestamp - depth.timestamp) > self.params.max_depth_age):
            return None
        return np.asarray(pose.keypoints[pose.target_person], dtype=float), depth

    async def align(self) -> dict:
        """Turn the OAK's alignment to fit the person the head follows now: watch them for align_seconds,
        score turns on each frame (off the event loop), take the best, and keep it. The answer says how it went."""
        if self.aligning:
            return {"ok": False, "busy": True, "message": "Already aligning"}
        self.aligning = True
        try:
            totals, frames, seen = None, 0, None
            deadline = time.monotonic() + self.params.align_seconds
            while time.monotonic() < deadline:
                pose = self.latest_pose
                found = self.usable(pose) if pose is not seen else None
                seen = pose
                if found is not None:
                    scores = await asyncio.to_thread(self.aligner.score, found[0], found[1].depth, found[1].rectification)
                    if scores is not None:
                        totals, frames = (scores if totals is None else totals + scores), frames + 1
                await asyncio.sleep(0.03)
            if frames < 5:
                return {"ok": False, "frames": frames,
                        "message": f"Not aligned: saw enough of you in {frames} frames, 5 needed. Stand in front of it, "
                                   "followed, with your whole upper body in view"}
            result = self.aligner.adopt(totals, frames)
            self.adopt()
            logger.info(result["message"])
            return result
        finally:
            self.aligning = False

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
        pose_in_head = nominal_camera_to_head(depth.rectification)
        rotation, translation = np.array(pose_in_head["rotation"]), np.array(pose_in_head["translation_m"])
        skeleton = self.skeleton(pose, depth, rotation, translation,
                                 self.params.skeleton_guess if point is None else float(np.linalg.norm(point - origin)))
        if point is None:
            return FaceLocationMessage(pose_timestamp=pose_time, depth_timestamp=depth.timestamp,
                                       pixel=(target.x, target.y), alignment=self.alignment_source, skeleton_head=skeleton)
        in_head = rotation @ point + translation
        return FaceLocationMessage(
            position_camera=tuple(float(value) for value in point),
            position_head=tuple(float(value) for value in in_head),
            distance=float(np.linalg.norm(in_head)),
            pixel=(target.x, target.y),
            pose_timestamp=pose_time,
            depth_timestamp=depth.timestamp,
            alignment=self.alignment_source,
            skeleton_head=skeleton,
        )

    def skeleton(self, pose: PoseDataMessage, depth: DepthFrameMessage, rotation: np.ndarray, translation: np.ndarray,
                 guess: float) -> tuple[tuple[float, float, float] | None, ...] | None:
        """The followed person's keypoints in the head frame, each None where the detector was unsure of it or
        the ZED has no depth there; None without a followed person."""
        if pose.keypoints is None or pose.target_person is None or not 0 <= pose.target_person < len(pose.keypoints):
            return None
        keypoints = np.asarray(pose.keypoints[pose.target_person], dtype=float)  # (17, 3): x, y, score
        origin = self.alignment.ray((0.0, 0.0))[0]
        directions = np.array([self.alignment.ray((x, y))[1] for x, y, _ in keypoints])
        points = place_on_depth(origin, directions, depth.depth, depth.rectification, guess)
        in_head = points @ rotation.T + translation
        return tuple(
            tuple(float(value) for value in point) if score >= self.params.keypoint_score and np.all(np.isfinite(point)) else None
            for (_, _, score), point in zip(keypoints, in_head)
        )

    async def run(self) -> None:
        logger.info("Starting FaceLocatorNode")
        try:
            while not self.stop_event.is_set():
                pose = await self.poses.get()
                self.latest_pose = pose  # For align, which reads the newest as it goes
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
