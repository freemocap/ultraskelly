"""Live telemetry for the dashboard, streamed the way FreeMoCap streams to its browser.

`/stream` is one WebSocket of self-describing CBOR messages (freemocap-stream).
FreeMoCap `frame` messages carry each camera's JPEG with the poses detected in that
very image; `robot_state` messages carry the bot's snapshot. The browser decodes them
with @freemocap/client and draws the poses and guides itself. The stream is read-only
and newest-wins: a slow browser receives fewer, newer messages and never blocks the
bot's event loop. Motor values are the commands sent, not measured positions.
"""
import asyncio
import logging
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
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder
from ultraskelly.core.bot.sensory.pose_common import SKELETON_CONNECTIONS, CocoKeypoint
from ultraskelly.core.pubsub.bot_topics import (
    FrameMessage,
    PoseDataTopic,
    ServoStateMessage,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC, TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

DASHBOARD = Path(__file__).with_name("dashboard")
SHARED = Path(__file__).parents[1] / "simulator" / "static"  # Skeleton, style and vendored Three.js
# Explicit content types: Windows can map .js to text/plain, which browsers refuse for modules.
ASSETS: dict[str, tuple[Path, str]] = {
    "/": (DASHBOARD / "index.html", "text/html"),
    "/dashboard.js": (DASHBOARD / "dashboard.js", "text/javascript"),
    "/dashboard.css": (DASHBOARD / "dashboard.css", "text/css"),
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


def encode_jpeg_with_opencv(frame: np.ndarray, quality: int) -> bytes:
    """Encode an RGB frame as JPEG."""
    import cv2

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


def encode_camera_frame(
    camera: str,
    index: int,
    image: np.ndarray,
    *,
    frame_number: int,
    timestamp: float,
    poses: np.ndarray | None,
    encode_jpeg: Callable[[np.ndarray, int], bytes],
    jpeg_quality: int,
) -> bytes:
    """One FreeMoCap frame message: this camera's JPEG and each person's keypoints, in
    pixels of that image. Pure CPU work, safe to run in a worker thread."""
    height, width = image.shape[:2]
    trackers = [
        tracker_observation(
            POSE_TRACKER,
            [channel_block(OVERLAY_2D, KEYPOINT_COLUMNS, person, names=KEYPOINT_NAMES,
                           camera_id=camera, image_size=(width, height))],
            connection_groups=[BODY],
        )
        for person in (poses if poses is not None else ())
    ]
    jpeg = encode_jpeg(image, jpeg_quality)
    return encode_frame_message(
        frame_number=frame_number,
        timestamp=timestamp,
        cameras=[uncalibrated_camera(camera, index, (width, height))],
        trackers=trackers,
        image=encode_image_payload(frame_number, [PayloadImage(camera, index, width, height, jpeg)]),
    )


@dataclass(frozen=True)
class TrackingOverlay:
    """Where the head steers the target and how wide its jaw zone is; the dashboard draws both."""

    aim: tuple[int, int] = (320, 240)
    jaw_zone: int = 100


@dataclass
class CameraFeed:
    """One camera topic: its frame statistics and its stream of encoded frame messages."""

    subscription: TopicSubscriptionQueue
    name: str = "camera"
    index: int = 0
    overlay: TrackingOverlay | None = None
    frames_seen: int = 0
    fps: float = 0.0
    last_frame_at: float | None = None
    shape: tuple[int, ...] | None = None
    stream: Latest = field(default_factory=Latest)
    # The newest frame (with the poses found in it) waiting to be encoded, newest-wins.
    pending: tuple[FrameMessage, np.ndarray | None] | None = None
    encoding: bool = False
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
    clients: int = Field(default=0, exclude=True)
    state_sequence: int = Field(default=0, exclude=True)
    state_published_at: float = Field(default=0.0, exclude=True)

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
    ) -> "RuntimeServerNode":
        """Factory method: subscribes to bot state and each camera's frame topic.

        Cameras named in `overlays` report the aim point and jaw zone the dashboard draws.
        `pose_camera`'s frames travel with the poses detected in them.
        """
        node = cls(
            pubsub=pubsub,
            params=params,
            recorder=recorder,
            motors_simulated=motors_simulated,
            node_status=node_status,
            encode_jpeg=encode_jpeg,
            pose_camera=pose_camera,
        )
        overlays = overlays or {}
        node.feeds = {
            name: CameraFeed(
                subscription=pubsub.get_subscription(topic), name=name, index=index, overlay=overlays.get(name)
            )
            for index, (name, topic) in enumerate(cameras.items())
        }
        node.subscriptions = {
            "servo_state": pubsub.get_subscription(ServoStateTopic),
            "target": pubsub.get_subscription(TargetLocationTopic),
            "pose": pubsub.get_subscription(PoseDataTopic),
        }
        return node

    def build_app(self) -> web.Application:
        """The aiohttp application, separate from serving so tests can host it themselves."""
        app = web.Application()
        for path in ASSETS:
            app.router.add_get(path, self._asset)
        app.router.add_get("/stream", self._stream)
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
                name: {
                    "fps": round(feed.fps, 1),
                    "age": None if feed.last_frame_at is None else now - feed.last_frame_at,
                    "width": feed.shape[1] if feed.shape else None,
                    "height": feed.shape[0] if feed.shape else None,
                }
                for name, feed in self.feeds.items()
            },
            "overlays": {
                name: {"aim": list(feed.overlay.aim), "jaw_zone": feed.overlay.jaw_zone}
                for name, feed in self.feeds.items()
                if feed.overlay is not None
            },
            "rates": {name: round(hz, 1) for name, hz in self.rates.items()},
            "nodes": self.node_status(),
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
        return state

    async def _asset(self, request: web.Request) -> web.Response:
        path, content_type = ASSETS[request.path]
        return web.Response(
            body=path.read_bytes(), content_type=content_type, charset="utf-8",
            headers={"Cache-Control": "no-store"},
        )

    async def _stream(self, request: web.Request) -> web.WebSocketResponse:
        """One client of the CBOR stream: robot state and every camera's newest frame."""
        websocket = web.WebSocketResponse(heartbeat=10)
        await websocket.prepare(request)
        connection = StreamConnection(websocket.send_bytes, websocket.send_str)
        self.clients += 1
        sending = asyncio.create_task(
            connection.follow(self.state_stream, *(feed.stream for feed in self.feeds.values()))
        )
        try:
            async for message in websocket:
                if message.type == WSMsgType.TEXT:
                    await connection.receive_text(message.data)  # Answers the client's ping
                elif message.type == WSMsgType.ERROR:
                    break
        except ConnectionResetError:
            pass
        finally:
            self.clients -= 1
            sending.cancel()
            await asyncio.gather(sending, return_exceptions=True)
        return websocket

    async def _encode(self, feed: CameraFeed) -> None:
        """Encode the newest pending frame message, repeating while newer ones arrive."""
        feed.encoding = True
        try:
            while feed.pending is not None and self.clients > 0:
                (camera_frame, poses), feed.pending = feed.pending, None
                feed.encoded += 1
                message = await asyncio.to_thread(
                    encode_camera_frame, feed.name, feed.index, camera_frame.frame,
                    frame_number=camera_frame.frame_number if camera_frame.frame_number is not None else feed.encoded,
                    timestamp=camera_frame.timestamp if camera_frame.timestamp is not None else time.monotonic(),
                    poses=poses, encode_jpeg=self.encode_jpeg, jpeg_quality=self.params.jpeg_quality,
                )
                feed.stream.publish(message)
        except Exception:
            logger.exception("Frame encoding failed")
        finally:
            feed.encoding = False

    async def pump(self) -> None:
        """Drain subscriptions into the latest state; while anyone watches, encode the
        newest frames and publish robot state."""
        while not self.stop_event.is_set():
            now = time.monotonic()
            for name, subscription in self.subscriptions.items():
                while not subscription.empty():
                    message = subscription.get_nowait()
                    previous = self.latest.get(name)
                    self.latest[name] = Received(message=message, at=now)
                    if previous is not None and now > previous.at:
                        rate = self.rates.get(name, 0.0)
                        self.rates[name] = rate + 0.1 * (1 / (now - previous.at) - rate)
                    if name == "pose" and message.frame is not None and self.pose_camera in self.feeds:
                        posed = self.feeds[self.pose_camera]
                        posed.last_pose_at = now
                        posed.pending = (message.frame, message.keypoints)
            for feed in self.feeds.values():
                while not feed.subscription.empty():
                    camera_frame = feed.subscription.get_nowait()
                    if feed.last_frame_at is not None and now > feed.last_frame_at:
                        feed.fps += 0.1 * (1 / (now - feed.last_frame_at) - feed.fps)
                    feed.last_frame_at, feed.shape = now, camera_frame.frame.shape
                    feed.frames_seen += 1
                    # A posed camera's frames come with their poses, unless the detector has stalled.
                    poses_flowing = feed.last_pose_at is not None and now - feed.last_pose_at < POSE_STALL_SECONDS
                    if feed.name != self.pose_camera or not poses_flowing:
                        feed.pending = (camera_frame, None)
                if feed.pending is not None and self.clients > 0 and not feed.encoding:
                    asyncio.create_task(self._encode(feed))
            if self.clients > 0 and now - self.state_published_at >= 1 / self.params.state_hz:
                self.state_published_at, self.state_sequence = now, self.state_sequence + 1
                self.state_stream.publish(
                    encode_message("robot_state", self.robot_state(), timestamp=now, sequence=self.state_sequence)
                )
            await asyncio.sleep(0.005)

    async def run(self) -> None:
        """Serve until stopped."""
        runner = web.AppRunner(self.build_app(), access_log=None)
        await runner.setup()
        site = web.TCPSite(runner, self.params.host, self.params.port)
        await site.start()
        logger.info(f"Runtime server on port {self.params.port}, host {self.params.host or 'all interfaces'} (read-only)")
        try:
            await self.pump()
        finally:
            self.stop_event.set()
            await runner.cleanup()
            logger.info("RuntimeServerNode stopped")
