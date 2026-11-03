"""Record a run to disk: the camera video, the poses found in it, and every motor command.

Each run gets its own folder under `RunRecorderParams.folder`:

    <camera>.avi         every camera frame the bot saw (after any flip), Motion JPEG: the same
                         JPEG bytes the stream sends, so each frame is encoded once
    <camera>_frames.csv  per video frame: its index, the camera's frame number, capture and arrival times
    poses.jsonl          per detector result: the frame it came from and each person's keypoints
    targets.csv          every target the detector published; empty x and y when nobody was seen
    servo_state.csv      the head controller's published angles and lock flags
    commands.csv         every change in a commanded servo angle or motor throttle
    audio.wav            optional microphone audio
    run.json             launch configuration, timing, and counts of what was written or dropped

Times are seconds on time.monotonic()'s clock, like the camera's capture times, so the
files line up with each other and with the audio (run.json gives its first sample's
time). Motor values are commands, not measured positions. A thread does the writing
behind a bounded queue: a slow disk drops items, counted in run.json, instead of
slowing the bot.
"""
import asyncio
import csv
import json
import logging
import queue
import subprocess
import threading
import time
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path
from typing import Callable, Protocol

import numpy as np
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.motor.simulated_actuators import ActuatorCommand, CommandRecorder
from ultraskelly.core.bot.recording.microphone import Microphone, find_microphone
from ultraskelly.core.bot.recording.mjpeg_avi import MjpegAviWriter
from ultraskelly.core.pubsub.bot_topics import (
    FrameMessage,
    PoseDataMessage,
    PoseDataTopic,
    ServoStateTopic,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC, TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

DEFAULT_JPEG_QUALITY = 80  # As the stream's; see RunRecorderParams.jpeg_quality


class VideoWriter(Protocol):
    def write(self, jpeg: bytes) -> None: ...

    def release(self) -> None: ...


def open_mjpeg_writer(path: Path, size: tuple[int, int], fps: float) -> VideoWriter:
    """A Motion JPEG AVI of `size` (width, height) that stores each frame's JPEG bytes as they are."""
    return MjpegAviWriter(path, size, fps)


def pose_record(message: PoseDataMessage, received: float) -> dict:
    """One detector result as JSON: the frame it came from, and each person's score, box and keypoints (x, y, score)."""
    frame = message.frame
    people = []
    if message.keypoints is not None:
        count = len(message.keypoints)
        scores = message.scores if message.scores is not None else [None] * count
        boxes = message.boxes if message.boxes is not None else [None] * count
        for keypoints, score, box in zip(message.keypoints, scores, boxes):
            people.append({
                "score": None if score is None else round(float(score), 4),
                "box": None if box is None else np.round(np.asarray(box, dtype=float), 1).tolist(),
                "keypoints": np.round(np.asarray(keypoints, dtype=float), 2).tolist(),
            })
    return {
        "received": received,
        "frame_number": None if frame is None else frame.frame_number,
        "captured": None if frame is None else frame.timestamp,
        "people": people,
    }


def git_commit() -> str | None:
    """The checkout's commit, to tie a recording to the code that made it."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=Path(__file__).parent, capture_output=True, text=True, timeout=5
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() or None


class RunRecorderParams(NodeParams):
    """Parameters for RunRecorderNode."""

    folder: Path = Field(
        default_factory=lambda: Path.home() / "ultraskelly-recordings", description="Each run gets a new folder in here"
    )
    jpeg_quality: int | None = Field(
        default=None, ge=10, le=100,
        description="None: the stream's JPEG quality, so the stream and the recording share one encoding of "
                    "each frame; another value encodes the recording separately",
    )
    nominal_fps: float = Field(
        default=30.0, gt=0.0, description="Frame rate in the video's header; the frames table has the real times"
    )
    microphone: str | None = Field(
        default=None, description="Part of a sound card's name to record audio from; None records no audio"
    )
    queue_size: int = Field(default=60, ge=1, description="Items waiting for the disk before new ones are dropped")


class RunRecorderNode(Node):
    """Writes the run's camera frames, detections and motor commands to disk."""

    params: RunRecorderParams
    camera: str = "camera"
    configuration: dict = Field(default_factory=dict, description="The launch configuration, saved with the run")
    frames: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)
    subscriptions: SkipValidation[dict[str, TopicSubscriptionQueue]] = Field(default_factory=dict, exclude=True)
    commands: SkipValidation[CommandRecorder | None] = Field(default=None, exclude=True)
    open_writer: SkipValidation[Callable[..., VideoWriter]] = Field(default=open_mjpeg_writer, exclude=True)
    # Encodes a frame that came without the camera's own JPEG; FrameMessage.jpeg_bytes shares the result.
    encode_jpeg: SkipValidation[Callable[[np.ndarray, int], bytes]] = Field(exclude=True)
    jpeg_quality: int = DEFAULT_JPEG_QUALITY
    folder: Path | None = None
    started: datetime | None = None
    started_at: float | None = None
    counts: dict[str, int] = Field(default_factory=dict, description="Items written, by kind; written by the writer thread")
    dropped: dict[str, int] = Field(default_factory=dict, description="Items dropped because the disk fell behind")
    frames_missed: int = Field(default=0, description="Camera frame numbers that never reached the recorder")
    failure: str | None = Field(default=None, description="Why writing stopped early, if it did")
    pending: SkipValidation[queue.Queue | None] = Field(default=None, exclude=True)
    writer: SkipValidation[threading.Thread | None] = Field(default=None, exclude=True)
    microphone: SkipValidation[Microphone | None] = Field(default=None, exclude=True)

    @classmethod
    def create(
        cls,
        *,
        pubsub: PubSubTopicManager,
        params: RunRecorderParams,
        camera: str,
        frame_topic: type[PubSubTopicABC],
        commands: CommandRecorder | None,
        encode_jpeg: Callable[[np.ndarray, int], bytes],
        stream_jpeg_quality: int = DEFAULT_JPEG_QUALITY,
        configuration: dict | None = None,
        open_writer: Callable[..., VideoWriter] = open_mjpeg_writer,
    ) -> "RunRecorderNode":
        """Factory method: subscribes to the camera's frames, the detections and the head's state.

        `encode_jpeg` and `stream_jpeg_quality` are the stream's, so a frame both keep is encoded once.
        """
        node = cls(
            pubsub=pubsub,
            params=params,
            camera=camera,
            configuration=configuration or {},
            commands=commands,
            open_writer=open_writer,
            encode_jpeg=encode_jpeg,
            jpeg_quality=stream_jpeg_quality if params.jpeg_quality is None else params.jpeg_quality,
        )
        node.frames = pubsub.get_subscription(frame_topic)
        node.subscriptions = {
            "pose": pubsub.get_subscription(PoseDataTopic),
            "target": pubsub.get_subscription(TargetLocationTopic),
            "servo_state": pubsub.get_subscription(ServoStateTopic),
        }
        return node

    def _enqueue(self, kind: str, received: float, message: object) -> None:
        if self.failure is not None:
            self.dropped[kind] = self.dropped.get(kind, 0) + 1
            return
        try:
            self.pending.put_nowait((kind, received, message))
        except queue.Full:
            self.dropped[kind] = self.dropped.get(kind, 0) + 1

    def _on_command(self, command: ActuatorCommand) -> None:
        self._enqueue("command", command.at, command)

    def _drain(self) -> None:
        now = time.monotonic()
        while not self.frames.empty():
            self._enqueue("frame", now, self.frames.get_nowait())
        for kind, subscription in self.subscriptions.items():
            while not subscription.empty():
                self._enqueue(kind, now, subscription.get_nowait())

    async def _forward(self, kind: str, subscription: TopicSubscriptionQueue) -> None:
        """Queue each message for the writer as it arrives."""
        while True:
            message = await subscription.get()
            self._enqueue(kind, time.monotonic(), message)

    def _write_all(self) -> None:
        """The writer thread. A failure ends the recording but never the run: later items
        are dropped, and the queue keeps draining so shutdown cannot wait on a full one."""
        try:
            self._write_files()
        except Exception as error:
            self.failure = repr(error)
            logger.exception("Run recording failed; the bot keeps running without it")
            while self.pending.get() is not None:
                pass

    def _write_files(self) -> None:
        """One file per kind, written until the None that ends the run."""
        with ExitStack() as files:
            def table(name: str, header: tuple[str, ...]) -> "csv._writer":
                rows = csv.writer(files.enter_context(open(self.folder / name, "w", newline="")))
                rows.writerow(header)
                return rows

            frames = table(f"{self.camera}_frames.csv", ("index", "frame_number", "captured", "received"))
            targets = table("targets.csv", ("received", "x", "y", "angle"))
            servo = table("servo_state.csv", ("received", "pan", "tilt", "locked_x", "locked_y"))
            commands = table("commands.csv", ("time", "actuator", "value"))
            poses = files.enter_context(open(self.folder / "poses.jsonl", "w"))
            video = None
            previous_number = None
            while (item := self.pending.get()) is not None:
                kind, received, message = item
                if kind == "frame":
                    frame: FrameMessage = message
                    if video is None:
                        height, width = frame.frame.shape[:2]
                        video = self.open_writer(
                            self.folder / f"{self.camera}.avi", (width, height), self.params.nominal_fps,
                        )
                        files.callback(video.release)
                    video.write(frame.jpeg_bytes(self.jpeg_quality, self.encode_jpeg))
                    frames.writerow((self.counts.get("frame", 0), message.frame_number, message.timestamp, received))
                    if previous_number is not None and message.frame_number is not None:
                        self.frames_missed += max(0, message.frame_number - previous_number - 1)
                    previous_number = message.frame_number
                elif kind == "pose":
                    poses.write(json.dumps(pose_record(message, received)) + "\n")
                elif kind == "target":
                    targets.writerow((received, message.x, message.y, message.angle))
                elif kind == "servo_state":
                    servo.writerow((received, message.pan_angle, message.tilt_angle, message.is_locked_x, message.is_locked_y))
                elif kind == "command":
                    commands.writerow((message.at, message.actuator, message.value))
                self.counts[kind] = self.counts.get(kind, 0) + 1

    def _start(self) -> None:
        self.started, self.started_at = datetime.now(), time.monotonic()
        self.folder = self.params.folder / f"run-{self.started:%Y%m%d-%H%M%S}"
        self.folder.mkdir(parents=True)
        self.pending = queue.Queue(maxsize=self.params.queue_size)
        if self.params.microphone:
            try:
                self.microphone = Microphone(find_microphone(self.params.microphone), self.folder / "audio.wav")
            except (ValueError, OSError) as error:
                logger.warning(f"Recording without audio: {error}")
        if self.commands is not None:
            for command in self.commands.latest.values():  # Commands sent before the run began, such as the head's start
                self._on_command(command)
            self.commands.listeners.append(self._on_command)
        self.writer = threading.Thread(target=self._write_all, name="run-recorder", daemon=True)
        self.writer.start()

    def _finish(self) -> None:
        if self.commands is not None and self._on_command in self.commands.listeners:
            self.commands.listeners.remove(self._on_command)
        self.pending.put(None)  # Waits for room: everything queued so far still gets written
        self.writer.join()
        audio_start = self.microphone.stop() if self.microphone is not None else None
        summary = {
            "started": self.started.isoformat(),
            "started_monotonic": self.started_at,
            "stopped_monotonic": time.monotonic(),
            "camera": self.camera,
            "written": self.counts,
            "dropped": self.dropped,
            "frames_missed": self.frames_missed,
            "failure": self.failure,
            "audio_first_sample_monotonic": audio_start,
            "git_commit": git_commit(),
            "configuration": self.configuration,
        }
        (self.folder / "run.json").write_text(json.dumps(summary, indent=2, default=str))

    async def run(self) -> None:
        """Record until stopped, then write everything still queued."""
        self._start()
        logger.info(f"Recording this run to {self.folder}")
        forwarders = [
            asyncio.create_task(self._forward(kind, subscription))
            for kind, subscription in [("frame", self.frames), *self.subscriptions.items()]
        ]
        try:
            await self.stop_event.wait()
        finally:
            for forwarder in forwarders:
                forwarder.cancel()
            await asyncio.gather(*forwarders, return_exceptions=True)
            self._drain()
            await asyncio.to_thread(self._finish)
            logger.info(f"RunRecorderNode stopped; saved {self.counts.get('frame', 0)} frames to {self.folder}")
