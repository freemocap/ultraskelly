"""Live telemetry for the dashboard, streamed the way FreeMoCap streams to its browser.

`/stream` is one WebSocket of self-describing CBOR messages (freemocap-stream):
FreeMoCap `frame` messages carry each camera's JPEG with the poses detected in
that very image, and `robot_state` messages carry the bot's snapshot. JSON state
and MJPEG video are also served. Every endpoint is read-only. A slow browser only
skips snapshots or frames; it never blocks the bot's event loop. Motor values are
the commands sent, not measured positions.
"""
import asyncio
import json
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
    PoseDataMessage,
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
    state_hz: float = Field(default=20.0, gt=0.0, le=60.0, description="WebSocket snapshots per second")
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


def draw_overlay_with_opencv(frame: np.ndarray, **overlay) -> np.ndarray:
    """Draw the pose, target and centering guides on a copy of an RGB frame."""
    from ultraskelly.ui.overlay import draw_tracking_overlay

    return draw_tracking_overlay(frame, **overlay)


@dataclass(frozen=True)
class TrackingOverlay:
    """Where the head steers the target and how wide its jaw zone is, drawn on a camera's video."""

    aim: tuple[int, int] = (320, 240)
    jaw_zone: int = 100


@dataclass
class CameraFeed:
    """Latest frame of one camera topic and its JPEG, encoded only while someone watches."""

    subscription: TopicSubscriptionQueue
    name: str = "camera"
    index: int = 0
    overlay: TrackingOverlay | None = None
    # /stream: the newest encoded frame message, and the frame (with its poses) waiting for encoding.
    stream: Latest = field(default_factory=Latest)
    stream_pending: tuple[FrameMessage, np.ndarray | None] | None = None
    stream_encoding: bool = False
    stream_frames: int = 0
    last_pose_at: float | None = None
    clients: int = 0
    frames_seen: int = 0
    fps: float = 0.0
    last_frame_at: float | None = None
    shape: tuple[int, ...] | None = None
    pending: np.ndarray | None = None
    encoding: bool = False
    jpeg: bytes | None = None
    jpeg_sequence: int = -1
    updated: asyncio.Condition = field(default_factory=asyncio.Condition)


@dataclass
class Received:
    """A topic message and when this server received it."""

    message: object
    at: float


class RuntimeServerNode(Node):
    """Serves live bot state and camera video to browsers."""

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
    draw_overlay: SkipValidation[Callable[..., np.ndarray]] = Field(default=draw_overlay_with_opencv, exclude=True)
    pose_camera: str | None = Field(default=None, description="Camera whose frames the pose detector reads")
    state_stream: SkipValidation[Latest] = Field(default_factory=Latest, exclude=True)
    stream_clients: int = Field(default=0, exclude=True)
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
        draw_overlay: Callable[..., np.ndarray] = draw_overlay_with_opencv,
    ) -> "RuntimeServerNode":
        """Factory method: subscribes to bot state and each camera's frame topic.

        Cameras named in `overlays` get the pose, target and centering guides drawn on their video.
        On `/stream`, `pose_camera`'s frames travel with the poses detected in them.
        """
        node = cls(
            pubsub=pubsub,
            params=params,
            recorder=recorder,
            motors_simulated=motors_simulated,
            node_status=node_status,
            encode_jpeg=encode_jpeg,
            draw_overlay=draw_overlay,
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
        app.router.add_get("/api/state", self._state)
        app.router.add_get("/ws", self._websocket)
        app.router.add_get("/video/{camera}.mjpg", self._video)
        app.router.add_get("/stream", self._stream)
        return app

    def robot_state(self) -> dict:
        """The `robot_state` message: the snapshot without poses, which travel in frame
        messages with their image, plus where each camera's guides are drawn."""
        state = self.snapshot()
        del state["pose"]
        state["overlays"] = {
            name: {"aim": list(feed.overlay.aim), "jaw_zone": feed.overlay.jaw_zone}
            for name, feed in self.feeds.items()
            if feed.overlay is not None
        }
        return state

    async def _stream(self, request: web.Request) -> web.WebSocketResponse:
        """One client of the CBOR stream: robot state and every camera's newest frame."""
        websocket = web.WebSocketResponse(heartbeat=10)
        await websocket.prepare(request)
        connection = StreamConnection(websocket.send_bytes, websocket.send_str)
        self.stream_clients += 1
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
            self.stream_clients -= 1
            sending.cancel()
            await asyncio.gather(sending, return_exceptions=True)
        return websocket

    def snapshot(self) -> dict:
        """Current bot state as JSON-ready values. Ages are seconds since this server received them."""
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
            "pose": None,
            "cameras": {
                name: {
                    "fps": round(feed.fps, 1),
                    "age": None if feed.last_frame_at is None else now - feed.last_frame_at,
                    "width": feed.shape[1] if feed.shape else None,
                    "height": feed.shape[0] if feed.shape else None,
                }
                for name, feed in self.feeds.items()
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
        if pose := self.latest.get("pose"):
            message: PoseDataMessage = pose.message
            people = message.keypoints if message.keypoints is not None else np.empty((0, 17, 3))
            state["pose"] = {
                "keypoints": np.round(people.astype(float), 1).tolist(),
                "scores": [] if message.scores is None else np.round(np.asarray(message.scores, float), 3).tolist(),
                "age": now - pose.at,
            }
        return state

    async def _asset(self, request: web.Request) -> web.Response:
        path, content_type = ASSETS[request.path]
        return web.Response(
            body=path.read_bytes(), content_type=content_type, charset="utf-8",
            headers={"Cache-Control": "no-store"},
        )

    async def _state(self, request: web.Request) -> web.Response:
        return web.json_response(self.snapshot(), headers={"Cache-Control": "no-store"})

    async def _websocket(self, request: web.Request) -> web.WebSocketResponse:
        websocket = web.WebSocketResponse(heartbeat=10)
        await websocket.prepare(request)
        # Read incoming frames so close and ping messages are handled; their content is ignored.
        reader = asyncio.create_task(self._discard_messages(websocket))
        try:
            while not websocket.closed and not reader.done() and not self.stop_event.is_set():
                await websocket.send_str(json.dumps(self.snapshot(), allow_nan=False))
                await asyncio.sleep(1 / self.params.state_hz)
        except ConnectionResetError:
            pass
        finally:
            reader.cancel()
        return websocket

    @staticmethod
    async def _discard_messages(websocket: web.WebSocketResponse) -> None:
        async for message in websocket:
            if message.type == WSMsgType.ERROR:
                break

    async def _video(self, request: web.Request) -> web.StreamResponse:
        feed = self.feeds.get(request.match_info["camera"])
        if feed is None:
            raise web.HTTPNotFound(text="Unknown camera")
        response = web.StreamResponse(
            headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame", "Cache-Control": "no-store"}
        )
        feed.clients += 1  # Counted before headers go out, so the next frame is encoded
        sequence = -1
        try:
            await response.prepare(request)
            while not self.stop_event.is_set():
                async with feed.updated:
                    await feed.updated.wait_for(lambda: feed.jpeg_sequence > sequence or self.stop_event.is_set())
                if self.stop_event.is_set():
                    break
                jpeg, sequence = feed.jpeg, feed.jpeg_sequence
                await response.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n"
                )
        except ConnectionResetError:
            pass
        finally:
            feed.clients -= 1
        return response

    def _overlay_for(self, feed: CameraFeed) -> dict | None:
        """The latest pose and target to draw on this camera's next frame, read on the event loop."""
        if feed.overlay is None:
            return None
        pose, target = self.latest.get("pose"), self.latest.get("target")
        target_message: TargetLocationMessage | None = target.message if target else None
        has_target = target_message is not None and target_message.x is not None and target_message.y is not None
        return {
            "keypoints": pose.message.keypoints if pose else None,
            "target": (target_message.x, target_message.y) if has_target else None,
            "aim": feed.overlay.aim,
            "jaw_zone": feed.overlay.jaw_zone,
        }

    def _render_jpeg(self, frame: np.ndarray, overlay: dict | None) -> bytes:
        """Draw the overlay, if any, on a copy of the frame, then encode. Runs in a worker thread."""
        if overlay is not None:
            frame = self.draw_overlay(frame, **overlay)
        return self.encode_jpeg(frame, self.params.jpeg_quality)

    async def _encode(self, feed: CameraFeed) -> None:
        """Encode the newest pending frame, repeating while newer frames arrive."""
        feed.encoding = True
        try:
            while feed.pending is not None and feed.clients > 0:
                frame, feed.pending = feed.pending, None
                jpeg = await asyncio.to_thread(self._render_jpeg, frame, self._overlay_for(feed))
                async with feed.updated:
                    feed.jpeg, feed.jpeg_sequence = jpeg, feed.jpeg_sequence + 1
                    feed.updated.notify_all()
        except Exception:
            logger.exception("JPEG encoding failed")
        finally:
            feed.encoding = False

    async def _stream_frames(self, feed: CameraFeed) -> None:
        """Encode the newest pending frame message for /stream, repeating while newer ones arrive."""
        feed.stream_encoding = True
        try:
            while feed.stream_pending is not None and self.stream_clients > 0:
                (camera_frame, poses), feed.stream_pending = feed.stream_pending, None
                feed.stream_frames += 1
                message = await asyncio.to_thread(
                    encode_camera_frame, feed.name, feed.index, camera_frame.frame,
                    frame_number=camera_frame.frame_number if camera_frame.frame_number is not None else feed.stream_frames,
                    timestamp=camera_frame.timestamp if camera_frame.timestamp is not None else time.monotonic(),
                    poses=poses, encode_jpeg=self.encode_jpeg, jpeg_quality=self.params.jpeg_quality,
                )
                feed.stream.publish(message)
        except Exception:
            logger.exception("Stream frame encoding failed")
        finally:
            feed.stream_encoding = False

    async def pump(self) -> None:
        """Drain subscriptions into the latest state and start encoding for watched cameras."""
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
                        posed.stream_pending = (message.frame, message.keypoints)
            for feed in self.feeds.values():
                while not feed.subscription.empty():
                    camera_frame = feed.subscription.get_nowait()
                    frame = camera_frame.frame
                    if feed.last_frame_at is not None and now > feed.last_frame_at:
                        feed.fps += 0.1 * (1 / (now - feed.last_frame_at) - feed.fps)
                    feed.last_frame_at, feed.shape = now, frame.shape
                    feed.frames_seen += 1
                    if feed.clients > 0:
                        feed.pending = frame
                    # A posed camera's frames come with their poses, unless the detector has stalled.
                    poses_flowing = feed.last_pose_at is not None and now - feed.last_pose_at < POSE_STALL_SECONDS
                    if feed.name != self.pose_camera or not poses_flowing:
                        feed.stream_pending = (camera_frame, None)
                if feed.pending is not None and feed.clients > 0 and not feed.encoding:
                    asyncio.create_task(self._encode(feed))
                if feed.stream_pending is not None and self.stream_clients > 0 and not feed.stream_encoding:
                    asyncio.create_task(self._stream_frames(feed))
            if self.stream_clients > 0 and now - self.state_published_at >= 1 / self.params.state_hz:
                self.state_published_at, self.state_sequence = now, self.state_sequence + 1
                self.state_stream.publish(
                    encode_message("robot_state", self.robot_state(), timestamp=now, sequence=self.state_sequence)
                )
            await asyncio.sleep(0.005)

    async def _wake_video_clients(self) -> None:
        for feed in self.feeds.values():
            async with feed.updated:
                feed.updated.notify_all()

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
            await self._wake_video_clients()
            await runner.cleanup()
            logger.info("RuntimeServerNode stopped")
