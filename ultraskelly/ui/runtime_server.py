"""Live telemetry for the dashboard: JSON bot state over a WebSocket and MJPEG camera video.

Every endpoint is read-only. A slow browser only skips snapshots or frames; it never
blocks the bot's event loop. Motor values are the commands sent, not measured positions.
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
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder
from ultraskelly.core.pubsub.bot_topics import (
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


@dataclass
class CameraFeed:
    """Latest frame of one camera topic and its JPEG, encoded only while someone watches."""

    subscription: TopicSubscriptionQueue
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
        encode_jpeg: Callable[[np.ndarray, int], bytes] = encode_jpeg_with_opencv,
    ) -> "RuntimeServerNode":
        """Factory method: subscribes to bot state and each camera's frame topic."""
        node = cls(
            pubsub=pubsub,
            params=params,
            recorder=recorder,
            motors_simulated=motors_simulated,
            node_status=node_status,
            encode_jpeg=encode_jpeg,
        )
        node.feeds = {name: CameraFeed(subscription=pubsub.get_subscription(topic)) for name, topic in cameras.items()}
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
        return app

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

    async def _encode(self, feed: CameraFeed) -> None:
        """Encode the newest pending frame, repeating while newer frames arrive."""
        feed.encoding = True
        try:
            while feed.pending is not None and feed.clients > 0:
                frame, feed.pending = feed.pending, None
                jpeg = await asyncio.to_thread(self.encode_jpeg, frame, self.params.jpeg_quality)
                async with feed.updated:
                    feed.jpeg, feed.jpeg_sequence = jpeg, feed.jpeg_sequence + 1
                    feed.updated.notify_all()
        except Exception:
            logger.exception("JPEG encoding failed")
        finally:
            feed.encoding = False

    async def pump(self) -> None:
        """Drain subscriptions into the latest state and start JPEG encoding for watched cameras."""
        while not self.stop_event.is_set():
            now = time.monotonic()
            for name, subscription in self.subscriptions.items():
                while not subscription.empty():
                    previous = self.latest.get(name)
                    self.latest[name] = Received(message=subscription.get_nowait(), at=now)
                    if previous is not None and now > previous.at:
                        rate = self.rates.get(name, 0.0)
                        self.rates[name] = rate + 0.1 * (1 / (now - previous.at) - rate)
            for feed in self.feeds.values():
                while not feed.subscription.empty():
                    frame = feed.subscription.get_nowait().frame
                    if feed.last_frame_at is not None and now > feed.last_frame_at:
                        feed.fps += 0.1 * (1 / (now - feed.last_frame_at) - feed.fps)
                    feed.last_frame_at, feed.shape = now, frame.shape
                    feed.frames_seen += 1
                    if feed.clients > 0:
                        feed.pending = frame
                if feed.pending is not None and feed.clients > 0 and not feed.encoding:
                    asyncio.create_task(self._encode(feed))
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
