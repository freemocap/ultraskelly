"""Record a run to disk: the camera video, the poses found in it, and every motor command.

Each run gets its own folder under `RunRecorderParams.folder`:

    <camera>.avi         every camera frame the bot saw (after any flip), Motion JPEG
    <camera>_frames.csv  per video frame: its index, the camera's frame number, capture and arrival times
    poses.jsonl          per detector result: the frame it came from and each person's keypoints
    targets.csv          every target the detector published; empty x and y when nobody was seen
    servo_state.csv      the head controller's published angles and lock flags
    commands.csv         every change in a commanded servo angle or motor throttle
    imu.csv              with the ZED, every IMU sample: host and IMU times, acceleration (m/s²) and
                         angular velocity (°/s) along the IMU's own axes, its temperature (°C)
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
from ultraskelly.core.pubsub.bot_topics import (
    ImuTopic,
    PoseDataMessage,
    PoseDataTopic,
    ServoStateTopic,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC, TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

IMU_HEADER = "time,imu_time,ax,ay,az,gx,gy,gz,temperature"
IMU_FORMAT = ("%.6f", "%.6f", "%.4f", "%.4f", "%.4f", "%.3f", "%.3f", "%.3f", "%.2f")


class VideoWriter(Protocol):
    def write(self, frame: np.ndarray) -> None: ...

    def release(self) -> None: ...


def open_mjpeg_writer(path: Path, size: tuple[int, int], fps: float, quality: int) -> VideoWriter:
    """An OpenCV Motion JPEG writer taking BGR frames of `size` (width, height)."""
    import cv2

    writer = cv2.VideoWriter(str(path), cv2.VideoWriter.fourcc(*"MJPG"), fps, size)
    if not writer.isOpened():
        raise RuntimeError(f"Could not open {path} for writing")
    writer.set(cv2.VIDEOWRITER_PROP_QUALITY, quality)  # Ignored by backends without the property
    return writer


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
    jpeg_quality: int = Field(default=90, ge=10, le=100)
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
    folder: Path | None = None
    started: datetime | None = None
    started_at: float | None = None
    counts: dict[str, int] = Field(default_factory=dict, description="Items written, by kind; written by the writer thread")
    dropped: dict[str, int] = Field(default_factory=dict, description="Items dropped because the disk fell behind")
    frames_missed: int = Field(default=0, description="Camera frame numbers that never reached the recorder")
    imu_samples_missed: int = Field(default=0, description="IMU samples published that never reached the recorder")
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
        configuration: dict | None = None,
        open_writer: Callable[..., VideoWriter] = open_mjpeg_writer,
    ) -> "RunRecorderNode":
        """Factory method: subscribes to the camera's frames, the detections and the head's state."""
        node = cls(
            pubsub=pubsub,
            params=params,
            camera=camera,
            configuration=configuration or {},
            commands=commands,
            open_writer=open_writer,
        )
        node.frames = pubsub.get_subscription(frame_topic)
        node.subscriptions = {
            "pose": pubsub.get_subscription(PoseDataTopic),
            "target": pubsub.get_subscription(TargetLocationTopic),
            "servo_state": pubsub.get_subscription(ServoStateTopic),
            "imu": pubsub.get_subscription(ImuTopic),
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
            video = imu = None
            previous_number = next_imu_sample = None
            while (item := self.pending.get()) is not None:
                kind, received, message = item
                if kind == "frame":
                    if video is None:
                        height, width = message.frame.shape[:2]
                        video = self.open_writer(
                            self.folder / f"{self.camera}.avi", (width, height), self.params.nominal_fps,
                            self.params.jpeg_quality,
                        )
                        files.callback(video.release)
                    video.write(np.ascontiguousarray(message.frame[..., 2::-1]))  # RGB to BGR
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
                elif kind == "imu":
                    if imu is None:  # Only runs with the ZED's IMU get the file
                        imu = files.enter_context(open(self.folder / "imu.csv", "w", newline=""))
                        imu.write(IMU_HEADER + "\n")
                    if next_imu_sample is not None:
                        self.imu_samples_missed += max(0, message.first_sample - next_imu_sample)
                    next_imu_sample = message.first_sample + len(message.timestamps)
                    temperature = np.nan if message.temperature is None else message.temperature
                    np.savetxt(imu, np.column_stack([
                        message.timestamps, message.imu_times, message.acceleration, message.angular_velocity,
                        np.full(len(message.timestamps), temperature),
                    ]), fmt=IMU_FORMAT, delimiter=",")
                    self.counts["imu_sample"] = self.counts.get("imu_sample", 0) + len(message.timestamps)
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
            "imu_samples_missed": self.imu_samples_missed,
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
        try:
            while not self.stop_event.is_set():
                self._drain()
                await asyncio.sleep(0.005)
        finally:
            self._drain()
            await asyncio.to_thread(self._finish)
            logger.info(f"RunRecorderNode stopped; saved {self.counts.get('frame', 0)} frames to {self.folder}")
