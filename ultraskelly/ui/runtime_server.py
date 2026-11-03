"""Live telemetry for the dashboard, streamed the way FreeMoCap streams to its browser.

`/stream` is one WebSocket of self-describing CBOR messages (freemocap-stream).
FreeMoCap `frame` messages carry each camera's JPEG with the poses detected in that
very image (a stereo camera's two images travel together in one message);
`robot_state` messages carry the bot's snapshot. The browser decodes them
with @freemocap/client and draws the poses and guides itself. The stream is read-only
and newest-wins: a slow browser receives fewer, newer messages and never blocks the
bot's event loop. A viewer may say what it draws (display feedback); it then receives
only that, and a feed nobody draws is not encoded (docs/stream.md). Motor values are the
commands sent, not measured positions.
Dashboard motor tests are separate commands with their own replies; see motor_test_routes.
"""
import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np
from aiohttp import WSMsgType, web
from freemocap_stream import (
    OVERLAY_2D,
    Latest,
    PayloadImage,
    StreamConnection,
    channel_block,
    connection_group,
    encode_frame_message,
    encode_image_payload,
    encode_message,
    tracker_observation,
    uncalibrated_camera,
)
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.motor.motor_test import MotorTester
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder
from ultraskelly.core.bot.sensory.pose_common import SKELETON_CONNECTIONS, CocoKeypoint
from ultraskelly.core.pubsub.bot_topics import (
    DepthFrameMessage,
    FaceLocationMessage,
    FaceLocationTopic,
    FrameMessage,
    ImuTopic,
    PoseDataMessage,
    PoseDataTopic,
    ServoStateMessage,
    ServoStateTopic,
    StereoFrameMessage,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC, TopicSubscriptionQueue
from ultraskelly.ui.depth_stream import encode_depth_frame, nominal_camera_to_head
from ultraskelly.ui.imu_stream import encode_imu_batch
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.motor_test_routes import add_motor_test_routes

logger = logging.getLogger(__name__)

DASHBOARD = Path(__file__).with_name("dashboard")
SHARED = Path(__file__).parents[1] / "simulator" / "static"  # Skeleton, style and vendored Three.js
# Explicit content types: Windows can map .js to text/plain, which browsers refuse for modules.
ASSETS: dict[str, tuple[Path, str]] = {
    "/": (DASHBOARD / "index.html", "text/html"),
    "/dashboard.js": (DASHBOARD / "dashboard.js", "text/javascript"),
    "/dashboard.css": (DASHBOARD / "dashboard.css", "text/css"),
    "/scene.js": (DASHBOARD / "scene.js", "text/javascript"),
    "/tiles.js": (DASHBOARD / "tiles.js", "text/javascript"),
    "/depth.js": (DASHBOARD / "depth.js", "text/javascript"),
    "/depth3d.js": (DASHBOARD / "depth3d.js", "text/javascript"),
    "/gaze.js": (DASHBOARD / "gaze.js", "text/javascript"),
    "/timeseries.js": (DASHBOARD / "timeseries.js", "text/javascript"),
    "/depth-worker.js": (DASHBOARD / "depth-worker.js", "text/javascript"),
    "/colormap.js": (DASHBOARD / "colormap.js", "text/javascript"),
    "/motor-test.js": (DASHBOARD / "motor-test.js", "text/javascript"),
    "/style.css": (SHARED / "style.css", "text/css"),
    "/skeleton.js": (SHARED / "skeleton.js", "text/javascript"),
    **{
        f"/vendor/{name}": (SHARED / "vendor" / name, "text/javascript")
        for name in ("three.module.js", "three.core.js", "OrbitControls.js")
    },
    # @freemocap/client's browser bundle, vendored by scripts/update-freemocap-client.sh.
    "/vendor/freemocap-client.js": (DASHBOARD / "vendor" / "freemocap-client.js", "text/javascript"),
    "/vendor/freemocap-client.js.map": (DASHBOARD / "vendor" / "freemocap-client.js.map", "application/json"),
}


class RuntimeServerParams(NodeParams):
    """Parameters for RuntimeServerNode."""

    host: str | None = Field(
        default=None,
        description="Interface to serve on; None serves every IPv4 and IPv6 interface, so other computers can connect",
    )
    port: int = Field(default=8080, ge=1, le=65535)
    state_hz: float = Field(default=20.0, gt=0.0, le=60.0, description="robot_state messages per second")
    jpeg_quality: int = Field(default=80, ge=10, le=100)
    depth_previews: bool = Field(
        default=False,
        description="Also send each depth frame as frame-message images, cameras <name>_rectified and <name>_depth "
                    "(depth drawn in colour); depth_frame carries both, so they are off by default",
    )


DEPTH_NEAR, DEPTH_FAR = 300, 5000  # Millimetres drawn red and blue


def depth_to_bgr(depth: np.ndarray) -> np.ndarray:
    """Draw a uint16 millimetre depth image in colour, BGR: red at 0.3 m, through yellow and
    green, to blue at 5 m and beyond; black where unknown."""
    import cv2

    span = DEPTH_FAR - DEPTH_NEAR  # Nearer than DEPTH_NEAR saturates at 255, red
    steps = cv2.convertScaleAbs(np.minimum(depth, DEPTH_FAR), alpha=-255 / span, beta=255 * DEPTH_FAR / span)
    bgr = cv2.applyColorMap(steps, cv2.COLORMAP_TURBO)
    # Blacken unknown pixels with a mask: 0.2 ms here, where numpy's bgr[depth == 0] = 0 takes 5.
    return cv2.bitwise_and(bgr, bgr, mask=cv2.compare(depth, 0, cv2.CMP_NE))


def depth_to_rgb(depth: np.ndarray) -> np.ndarray:
    """`depth_to_bgr` in RGB order."""
    import cv2

    return cv2.cvtColor(depth_to_bgr(depth), cv2.COLOR_BGR2RGB)


def encode_jpeg_with_opencv(frame: np.ndarray, quality: int) -> bytes:
    """Encode an RGB frame as JPEG; a 2-D uint16 depth image is drawn in colour first."""
    import cv2

    if frame.ndim == 2 and frame.dtype == np.uint16:
        bgr = depth_to_bgr(frame)
    else:
        bgr = cv2.cvtColor(np.ascontiguousarray(frame[..., :3]), cv2.COLOR_RGB2BGR)
    ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return jpeg.tobytes()


POSE_TRACKER = "yolov8_pose"
KEYPOINT_NAMES = tuple(keypoint.name.lower() for keypoint in CocoKeypoint)
KEYPOINT_COLUMNS = ("x", "y", "visibility")
BODY = connection_group(
    "body", [(KEYPOINT_NAMES[start], KEYPOINT_NAMES[end]) for start, end in SKELETON_CONNECTIONS], "#3ddc84"
)
POSE_STALL_SECONDS = 0.5  # Without a pose this long, a posed camera streams its frames bare


def camera_images(message: FrameMessage | StereoFrameMessage | DepthFrameMessage) -> list[np.ndarray]:
    """The images a camera message carries: one, a stereo pair's left then right, or a
    depth frame's rectified left image then its depth."""
    if isinstance(message, StereoFrameMessage):
        return [message.left, message.right]
    if isinstance(message, DepthFrameMessage):
        return [message.color, message.depth]
    return [message.frame]


def encode_camera_frame(
    cameras: list[tuple[str, int, np.ndarray]],
    *,
    frame_number: int,
    timestamp: float,
    poses: PoseDataMessage | None,
    encode_jpeg: Callable[[np.ndarray, int], bytes],
    jpeg_quality: int,
) -> bytes:
    """One FreeMoCap frame message: the JPEG of each (camera ID, index, image) captured
    together, and each person's keypoints in pixels of the first image. Pure CPU work,
    safe to run in a worker thread."""
    camera, _, image = cameras[0]
    height, width = image.shape[:2]
    trackers = person_trackers(camera, (width, height), poses)
    images = [
        PayloadImage(camera, index, image.shape[1], image.shape[0], encode_jpeg(image, jpeg_quality))
        for camera, index, image in cameras
    ]
    return encode_frame_message(
        frame_number=frame_number,
        timestamp=timestamp,
        cameras=[uncalibrated_camera(image.camera_id, image.camera_index, (image.width, image.height))
                 for image in images],
        trackers=trackers,
        image=encode_image_payload(frame_number, images),
    )


FOLLOWED = "target"  # Tracker ID of the person the head follows; the others are person-<row>
TARGET_CHANNEL = "ultraskelly.target"
TARGET_COLUMNS = ("x", "y", "body_angle_deg")


def person_trackers(camera: str, image_size: tuple[int, int], poses: PoseDataMessage | None) -> list[dict]:
    """One tracker observation per person, with their keypoints in pixels of `camera`'s image.

    The person the head follows is `target` and also carries the target it steers toward, as an
    `ultraskelly.target` channel; everyone else is `person-<row>` by their row in this frame's
    detections. The IDs name roles in this frame only: YOLOv8 pose does not track people.
    """
    if poses is None or poses.keypoints is None:
        return []
    target = poses.target
    trackers = []
    for row, person in enumerate(poses.keypoints):
        channels = [channel_block(OVERLAY_2D, KEYPOINT_COLUMNS, person, names=KEYPOINT_NAMES,
                                  camera_id=camera, image_size=image_size)]
        followed = row == poses.target_person and target is not None and None not in (target.x, target.y)
        if followed:
            angle = np.nan if target.angle is None else target.angle  # NaN marks a missing value
            channels.append(channel_block(TARGET_CHANNEL, TARGET_COLUMNS, [[target.x, target.y, angle]],
                                          names=["target"], camera_id=camera, image_size=image_size))
        trackers.append(tracker_observation(FOLLOWED if followed else f"person-{row}", channels,
                                            detector_type=POSE_TRACKER, connection_groups=[BODY]))
    return trackers


STATE_KIND, DEPTH_KIND, IMU_KIND = "robot_state", "depth_frame", "imu"


@dataclass(frozen=True)
class Display:
    """What a viewer draws, from its display feedback: each camera ID with the size it draws
    it at (None when it gave none usable), and the application kinds it wants."""

    sizes: dict[str, tuple[int, int] | None]
    kinds: frozenset[str]


def display_feedback(text: str) -> Display | None:
    """What a viewer's display feedback says it draws (docs/stream.md, "What a viewer draws");
    None for any other text, which changes nothing."""
    try:
        message = json.loads(text)
    except ValueError:
        return None
    if not isinstance(message, dict) or not isinstance(message.get("displayImageSizes"), dict):
        return None

    def size(drawn: object) -> tuple[int, int] | None:
        if not isinstance(drawn, dict):
            return None
        width, height = drawn.get("width"), drawn.get("height")
        usable = all(isinstance(side, (int, float)) and math.isfinite(side) and side >= 1 for side in (width, height))
        return (round(width), round(height)) if usable else None

    kinds = message.get("kinds")
    return Display(
        sizes={camera: size(drawn) for camera, drawn in message["displayImageSizes"].items()},
        kinds=frozenset(kind for kind in kinds if isinstance(kind, str)) if isinstance(kinds, list) else frozenset(),
    )


@dataclass(eq=False)
class Viewer:
    """One /stream connection and what it draws: None until it says, which is everything."""

    connection: StreamConnection
    display: Display | None = None

    def wants(self, cameras: tuple[str, ...] = (), kind: str | None = None) -> bool:
        """Whether the viewer draws one of `cameras` or wants `kind` messages; robot_state always."""
        if self.display is None or kind == STATE_KIND or kind in self.display.kinds:
            return True
        return any(camera in self.display.sizes for camera in cameras)


class JpegCache:
    """Encodes each image once, however many messages carry it. A camera frame's own image is
    encoded through FrameMessage.jpeg_bytes, so the recorder keeps the same JPEG."""

    def __init__(
        self,
        encode_jpeg: Callable[[np.ndarray, int], bytes],
        frame: FrameMessage | StereoFrameMessage | DepthFrameMessage | None = None,
    ) -> None:
        self.encode_jpeg = encode_jpeg
        self.frame = frame if isinstance(frame, FrameMessage) else None
        self.encoded: dict[int, tuple[np.ndarray, bytes]] = {}

    def encode(self, image: np.ndarray, quality: int) -> bytes:
        if self.frame is not None and image is self.frame.frame:
            return self.frame.jpeg_bytes(quality, self.encode_jpeg)
        if id(image) not in self.encoded:
            self.encoded[id(image)] = (image, self.encode_jpeg(image, quality))  # Holding the image keeps its id
        return self.encoded[id(image)][1]


@dataclass(frozen=True)
class TrackingOverlay:
    """Where the head steers the target and how wide its jaw zone is; the dashboard draws both."""

    aim: tuple[int, int] = (320, 240)
    jaw_zone: int = 100


@dataclass
class CameraFeed:
    """One camera topic: its frame statistics and its stream of encoded frame messages.
    A stereo camera's frame messages carry both of its images."""

    subscription: TopicSubscriptionQueue
    name: str = "camera"
    index: int = 0  # The first image's camera index; a stereo pair's right image has the next
    views: tuple[str, ...] = ("camera",)  # Camera IDs in image order: the name, or <name>_left and <name>_right
    overlay: TrackingOverlay | None = None
    frames_seen: int = 0
    fps: float = 0.0
    last_frame_at: float | None = None
    shape: tuple[int, ...] | None = None
    stream: Latest = field(default_factory=Latest)
    # The newest frame (with the poses found in it) waiting to be encoded, newest-wins.
    pending: tuple[FrameMessage | StereoFrameMessage | DepthFrameMessage, PoseDataMessage | None] | None = None
    encoder: asyncio.Task | None = None  # The task encoding this feed's frames, while one runs
    encoded: int = 0
    last_pose_at: float | None = None


@dataclass
class Received:
    """A topic message and when this server received it."""

    message: object
    at: float


class RuntimeServerNode(Node):
    """Streams live bot state and camera video to browsers."""

    params: RuntimeServerParams
    feeds: SkipValidation[dict[str, CameraFeed]] = Field(default_factory=dict, exclude=True)
    subscriptions: SkipValidation[dict[str, TopicSubscriptionQueue]] = Field(default_factory=dict, exclude=True)
    latest: SkipValidation[dict[str, Received]] = Field(default_factory=dict, exclude=True)
    rates: dict[str, float] = Field(default_factory=dict, exclude=True)
    recorder: SkipValidation[CommandRecorder | None] = Field(default=None, exclude=True)
    motors_simulated: bool = Field(default=False)
    node_status: SkipValidation[Callable[[], list[dict]]] = Field(default=lambda: [], exclude=True)
    encode_jpeg: SkipValidation[Callable[[np.ndarray, int], bytes]] = Field(
        default=encode_jpeg_with_opencv, exclude=True
    )
    pose_camera: str | None = Field(default=None, description="Camera whose frames the pose detector reads")
    state_stream: SkipValidation[Latest] = Field(default_factory=Latest, exclude=True)
    viewers: SkipValidation[list[Viewer]] = Field(default_factory=list, exclude=True)
    # Set while any viewer is connected
    watched: SkipValidation[asyncio.Event] = Field(default_factory=asyncio.Event, exclude=True)
    state_sequence: int = Field(default=0, exclude=True)
    depth_stream: SkipValidation[Latest] = Field(default_factory=Latest, exclude=True)  # depth_frame messages
    depth_sequence: int = Field(default=0, exclude=True)
    imu_stream: SkipValidation[Latest] = Field(default_factory=Latest, exclude=True)  # imu messages
    imu_sequence: int = Field(default=0, exclude=True)
    motor_tests: SkipValidation[MotorTester | None] = Field(default=None, exclude=True)

    @property
    def clients(self) -> int:
        """How many viewers are connected."""
        return len(self.viewers)

    @classmethod
    def create(
        cls,
        *,
        pubsub: PubSubTopicManager,
        params: RuntimeServerParams,
        cameras: dict[str, type[PubSubTopicABC]],
        recorder: CommandRecorder | None,
        motors_simulated: bool,
        node_status: Callable[[], list[dict]],
        overlays: dict[str, TrackingOverlay] | None = None,
        pose_camera: str | None = None,
        encode_jpeg: Callable[[np.ndarray, int], bytes] = encode_jpeg_with_opencv,
        motor_tests: MotorTester | None = None,
        stereo_cameras: dict[str, type[PubSubTopicABC]] | None = None,
        depth_cameras: dict[str, type[PubSubTopicABC]] | None = None,
    ) -> "RuntimeServerNode":
        """Factory method: subscribes to bot state and each camera's frame topic.

        Cameras named in `overlays` report the aim point and jaw zone the dashboard draws.
        `pose_camera`'s frames travel with the poses detected in them. With `motor_tests`,
        the dashboard can run motor tests through /api/motor-test. A stereo camera's two
        images travel together in one frame message, as cameras `<name>_left` and `<name>_right`;
        a depth frame's as `<name>_rectified` and, with `depth_previews`, `<name>_depth`, the
        depth drawn in colour.
        """
        node = cls(
            pubsub=pubsub,
            params=params,
            recorder=recorder,
            motors_simulated=motors_simulated,
            node_status=node_status,
            encode_jpeg=encode_jpeg,
            pose_camera=pose_camera,
            motor_tests=motor_tests,
        )
        overlays = overlays or {}
        feeds = [(name, topic, (name,)) for name, topic in cameras.items()]
        feeds += [(name, topic, (f"{name}_left", f"{name}_right")) for name, topic in (stereo_cameras or {}).items()]
        for name, topic in (depth_cameras or {}).items():
            views = (f"{name}_rectified", f"{name}_depth") if params.depth_previews else (f"{name}_rectified",)
            feeds.append((f"{name}_depth", topic, views))
        index = 0
        for name, topic, views in feeds:
            node.feeds[name] = CameraFeed(
                subscription=pubsub.get_subscription(topic), name=name, index=index, views=views,
                overlay=overlays.get(name),
            )
            index += len(views)
        node.subscriptions = {
            "servo_state": pubsub.get_subscription(ServoStateTopic),
            "target": pubsub.get_subscription(TargetLocationTopic),
            "pose": pubsub.get_subscription(PoseDataTopic),
            "face": pubsub.get_subscription(FaceLocationTopic),
            "imu": pubsub.get_subscription(ImuTopic),
        }
        return node

    def build_app(self) -> web.Application:
        """The aiohttp application, separate from serving so tests can host it themselves."""
        app = web.Application()
        for path in ASSETS:
            app.router.add_get(path, self._asset)
        app.router.add_get("/stream", self._stream)
        if self.motor_tests is not None:
            add_motor_test_routes(app, self.motor_tests)
        return app

    def robot_state(self) -> dict:
        """The `robot_state` payload. Ages are seconds since this server received a value.
        Poses are not here: they travel in frame messages, with the image they came from."""
        now = time.monotonic()
        state: dict = {
            "time": now,
            "motors": {
                "simulated": self.motors_simulated,
                "commands": {
                    name: {"value": command.value, "age": now - command.at}
                    for name, command in (self.recorder.latest.items() if self.recorder else [])
                },
            },
            "servo_state": None,
            "target": None,
            "cameras": {
                camera: {
                    "fps": round(feed.fps, 1),
                    "age": None if feed.last_frame_at is None else now - feed.last_frame_at,
                    "width": feed.shape[1] if feed.shape else None,
                    "height": feed.shape[0] if feed.shape else None,
                }
                for feed in self.feeds.values()
                for camera in feed.views
            },
            "overlays": {
                name: {"aim": list(feed.overlay.aim), "jaw_zone": feed.overlay.jaw_zone}
                for name, feed in self.feeds.items()
                if feed.overlay is not None
            },
            "rates": {name: round(hz, 1) for name, hz in self.rates.items()},
            "nodes": self.node_status(),
            "motor_test": None if self.motor_tests is None else self.motor_tests.state(),
        }
        if servo := self.latest.get("servo_state"):
            message: ServoStateMessage = servo.message
            state["servo_state"] = {
                "pan": message.pan_angle,
                "tilt": message.tilt_angle,
                "roll": message.roll_angle,
                "locked_x": message.is_locked_x,
                "locked_y": message.is_locked_y,
                "age": now - servo.at,
            }
        if target := self.latest.get("target"):
            message: TargetLocationMessage = target.message
            state["target"] = {"x": message.x, "y": message.y, "angle": message.angle, "age": now - target.at}
        if face := self.latest.get("face"):
            message: FaceLocationMessage = face.message
            state["face"] = {
                "position_head": None if message.position_head is None else list(message.position_head),
                "position_camera": None if message.position_camera is None else list(message.position_camera),
                "distance": message.distance,
                "pixel": None if message.pixel is None else list(message.pixel),
                "alignment": message.alignment,
                "age": now - face.at,
            }
        return state

    async def _asset(self, request: web.Request) -> web.Response:
        path, content_type = ASSETS[request.path]
        return web.Response(
            body=path.read_bytes(), content_type=content_type, charset="utf-8",
            headers={"Cache-Control": "no-store"},
        )

    async def _stream(self, request: web.Request) -> web.WebSocketResponse:
        """One viewer of the CBOR stream: robot state, and the newest frame of each view it shows."""
        # No permessage-deflate: JPEG and zlib data do not shrink, so deflating them would only cost the Pi
        websocket = web.WebSocketResponse(heartbeat=10, compress=False)
        await websocket.prepare(request)
        viewer = Viewer(StreamConnection(websocket.send_bytes, websocket.send_str))
        self.viewers.append(viewer)
        self.watched.set()
        streams = [
            (self.state_stream, (), STATE_KIND), (self.imu_stream, (), IMU_KIND), (self.depth_stream, (), DEPTH_KIND),
            *((feed.stream, feed.views, None) for feed in self.feeds.values()),
        ]
        sending = [asyncio.create_task(self._send(viewer, stream, cameras, kind)) for stream, cameras, kind in streams]
        self._kick_all()
        try:
            async for message in websocket:
                if message.type == WSMsgType.TEXT:
                    text = await viewer.connection.receive_text(message.data)  # Answers the client's ping
                    if text is not None and (display := display_feedback(text)) is not None:
                        viewer.display = display
                        self._kick_all()  # A camera just asked for need not wait for its next frame
                elif message.type == WSMsgType.ERROR:
                    break
        except ConnectionResetError:
            pass
        finally:
            self.viewers.remove(viewer)
            if not self.viewers:
                self.watched.clear()
            for task in sending:
                task.cancel()
            await asyncio.gather(*sending, return_exceptions=True)
        return websocket

    @staticmethod
    async def _send(viewer: Viewer, stream: Latest, cameras: tuple[str, ...], kind: str | None) -> None:
        """Send the stream's newest message, now and whenever it changes, while the viewer draws one
        of its `cameras` or wants its `kind`. New feedback applies from the next message, so no send
        is cut short."""
        version = 0
        while True:
            version, message = await stream.after(version)
            if viewer.wants(cameras, kind):
                await viewer.connection.send(message)

    def _shown(self, cameras: tuple[str, ...] = (), kind: str | None = None) -> bool:
        """Whether any viewer draws one of `cameras` or wants `kind` messages."""
        return any(viewer.wants(cameras, kind) for viewer in self.viewers)

    def _wanted(self, feed: CameraFeed) -> tuple[bool, bool]:
        """Whether any viewer shows the feed's pending frame as a frame message, and as a depth_frame.
        A depth frame travels as a frame message only with depth_previews."""
        depth = isinstance(feed.pending[0], DepthFrameMessage)
        frames = (self.params.depth_previews or not depth) and self._shown(feed.views)
        return frames, depth and self._shown(kind=DEPTH_KIND)

    def _kick(self, feed: CameraFeed) -> None:
        """Start encoding the feed's pending frame, unless that is under way or nobody shows it."""
        if feed.pending is not None and feed.encoder is None and any(self._wanted(feed)):
            feed.encoder = asyncio.create_task(self._encode(feed))

    def _kick_all(self) -> None:
        for feed in self.feeds.values():
            self._kick(feed)

    def _encode_depth_frame(self, frame: DepthFrameMessage, sequence: int, jpegs: "JpegCache") -> bytes:
        """The `depth_frame` message for a depth frame (docs/depth-stream.md). Runs in a worker thread."""
        return encode_depth_frame(
            frame, sequence=sequence, color_jpeg=jpegs.encode(frame.color, self.params.jpeg_quality),
            camera_to_head=nominal_camera_to_head(frame.rectification),
        )

    async def _encode(self, feed: CameraFeed) -> None:
        """Encode the newest pending frame as each viewer draws it, repeating while newer ones arrive."""
        try:
            while feed.pending is not None and any(wanted := self._wanted(feed)):
                frames_shown, depth_shown = wanted
                (camera_frame, poses), feed.pending = feed.pending, None
                feed.encoded += 1
                # Each image once: a depth frame's colour for both its messages, a camera frame's for the recorder too
                jpegs = JpegCache(self.encode_jpeg, camera_frame)
                if frames_shown:
                    cameras = [
                        (camera, feed.index + offset, image)
                        for offset, (camera, image) in enumerate(zip(feed.views, camera_images(camera_frame)))
                    ]
                    message = await asyncio.to_thread(
                        encode_camera_frame, cameras,
                        frame_number=(camera_frame.frame_number if camera_frame.frame_number is not None
                                      else feed.encoded),
                        timestamp=camera_frame.timestamp if camera_frame.timestamp is not None else time.monotonic(),
                        poses=poses, encode_jpeg=jpegs.encode, jpeg_quality=self.params.jpeg_quality,
                    )
                    feed.stream.publish(message)
                if depth_shown:  # The depth itself, for depth tiles and 3D views
                    self.depth_sequence += 1
                    self.depth_stream.publish(
                        await asyncio.to_thread(self._encode_depth_frame, camera_frame, self.depth_sequence, jpegs)
                    )
        except Exception:
            logger.exception("Frame encoding failed")
        finally:
            feed.encoder = None

    async def _take(self, name: str, subscription: TopicSubscriptionQueue) -> None:
        """Keep a topic's newest message and its rate; a pose also offers the frame it was found in."""
        while True:
            message = await subscription.get()
            now = time.monotonic()
            previous = self.latest.get(name)
            self.latest[name] = Received(message=message, at=now)
            if previous is not None and now > previous.at:
                rate = self.rates.get(name, 0.0)
                self.rates[name] = rate + 0.1 * (1 / (now - previous.at) - rate)
            if name == "imu" and self._shown(kind=IMU_KIND):  # Every batch, for time series
                self.imu_sequence += 1
                self.imu_stream.publish(encode_imu_batch(message, sequence=self.imu_sequence))
            if name == "pose" and message.frame is not None and self.pose_camera in self.feeds:
                posed = self.feeds[self.pose_camera]
                posed.last_pose_at = now
                posed.pending = (message.frame, message)
                self._kick(posed)

    async def _take_frames(self, feed: CameraFeed) -> None:
        """Count a camera's frames and offer each one for encoding."""
        while True:
            camera_frame = await feed.subscription.get()
            now = time.monotonic()
            if feed.last_frame_at is not None and now > feed.last_frame_at:
                feed.fps += 0.1 * (1 / (now - feed.last_frame_at) - feed.fps)
            feed.last_frame_at, feed.shape = now, camera_images(camera_frame)[0].shape
            feed.frames_seen += 1
            # A posed camera's frames come with their poses, unless the detector has stalled.
            poses_flowing = feed.last_pose_at is not None and now - feed.last_pose_at < POSE_STALL_SECONDS
            if feed.name != self.pose_camera or not poses_flowing:
                feed.pending = (camera_frame, None)
                self._kick(feed)

    async def _publish_state(self) -> None:
        """Publish robot state at state_hz while anyone watches."""
        while True:
            await self.watched.wait()
            now = time.monotonic()
            self.state_sequence += 1
            self.state_stream.publish(
                encode_message(STATE_KIND, self.robot_state(), timestamp=now, sequence=self.state_sequence)
            )
            await asyncio.sleep(1 / self.params.state_hz)

    async def pump(self) -> None:
        """Take each topic's messages as they arrive into the latest state; while anyone watches,
        encode the newest frames someone shows and publish robot state. Runs until stopped; a
        part that fails stops it with its error."""
        parts = [
            *(asyncio.create_task(self._take(name, subscription)) for name, subscription in self.subscriptions.items()),
            *(asyncio.create_task(self._take_frames(feed)) for feed in self.feeds.values()),
            asyncio.create_task(self._publish_state()),
        ]
        stopped = asyncio.create_task(self.stop_event.wait())
        try:
            done, _ = await asyncio.wait([stopped, *parts], return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        finally:
            tasks = [stopped, *parts, *(feed.encoder for feed in self.feeds.values() if feed.encoder is not None)]
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def run(self) -> None:
        """Serve until stopped."""
        runner = web.AppRunner(self.build_app(), access_log=None)
        await runner.setup()
        site = web.TCPSite(runner, self.params.host, self.params.port)
        await site.start()
        logger.info(
            f"Runtime server on port {self.params.port}, host {self.params.host or 'all interfaces'} "
            f"({'with motor tests' if self.motor_tests is not None and self.motor_tests.limits.enabled else 'read-only'})"
        )
        try:
            await self.pump()
        finally:
            self.stop_event.set()
            if self.motor_tests is not None:
                await self.motor_tests.stop()
            await runner.cleanup()
            logger.info("RuntimeServerNode stopped")
