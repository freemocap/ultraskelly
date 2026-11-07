import asyncio
import csv
import json
import shutil
import threading

import numpy as np

from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder
from ultraskelly.core.bot.recording import run_recorder_node
from ultraskelly.core.bot.recording.run_recorder_node import RunRecorderNode, RunRecorderParams, prune_runs
from ultraskelly.core.pubsub.bot_topics import (
    FrameMessage,
    FrameTopic,
    ImuMessage,
    ImuTopic,
    PoseDataMessage,
    PoseDataTopic,
    ServoStateMessage,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


class FakeVideo:
    """Keeps the JPEG bytes a video writer was given."""

    def __init__(self, path, size, fps, gate=None):
        self.path, self.size, self.frames, self.released, self.gate = path, size, [], False, gate

    def write(self, jpeg):
        if self.gate is not None:
            self.gate.wait()  # A disk that has stalled
        self.frames.append(jpeg)

    def release(self):
        self.released = True


class FakeEncoder:
    """Stands in for the stream's JPEG encoder: names the frame number it was given, counting calls."""

    def __init__(self):
        self.calls = []

    def __call__(self, image, quality):
        self.calls.append((int(image[0, 0, 0]), quality))
        return b"JPEG%d@%d" % (int(image[0, 0, 0]), quality)


def frame(number, **fields):
    image = np.zeros((4, 6, 3), dtype=np.uint8)
    image[..., 0] = number  # The red channel carries the frame number
    return FrameMessage(frame=image, frame_number=number, timestamp=100.0 + number, **fields)


def rows(path):
    with open(path, newline="") as file:
        return list(csv.DictReader(file))


def make_recorder(bus, tmp_path, commands, videos, *, gate=None, queue_size=60, encoder=None, **params):
    def open_writer(*args):
        videos.append(FakeVideo(*args, gate=gate))
        return videos[-1]

    return RunRecorderNode.create(
        pubsub=bus,
        params=RunRecorderParams(folder=tmp_path, queue_size=queue_size, **params),
        camera="oak",
        frame_topic=FrameTopic,
        commands=commands,
        encode_jpeg=encoder or FakeEncoder(),
        stream_jpeg_quality=80,
        configuration={"note": "test run"},
        open_writer=open_writer,
    )


def test_a_run_is_saved_with_its_frames_detections_and_commands(tmp_path):
    videos = []

    async def scenario():
        bus, commands = PubSubTopicManager.create(), CommandRecorder()
        commands.record("pan", 90.0)  # Sent before the run starts, like the head's starting angle
        node = make_recorder(bus, tmp_path, commands, videos)
        task = asyncio.create_task(node.run())
        await asyncio.sleep(0.02)
        for number in (5, 6, 8):  # Frame 7 never reaches the recorder
            message = frame(number)
            await bus.publish(FrameTopic, message)
            await asyncio.sleep(0.02)
            if number == 6:
                await bus.publish(PoseDataTopic, PoseDataMessage(
                    keypoints=np.ones((1, 17, 3)), scores=np.array([0.9]), boxes=[np.array([1, 2, 3, 4])], frame=message,
                ))
                await bus.publish(TargetLocationTopic, TargetLocationMessage(x=3, y=2))
                await bus.publish(ServoStateTopic, ServoStateMessage(
                    pan_angle=95.0, tilt_angle=90.0, roll_angle=90.0,
                    is_locked_x=True, is_locked_y=False, is_locked_roll=False,
                ))
                commands.record("waist", 0.7)
                await asyncio.sleep(0.02)
        node.stop()
        await task
        bus.close()
        return node.folder

    folder = asyncio.run(scenario())

    [video] = videos
    assert video.path == folder / "oak.avi" and video.size == (6, 4) and video.released
    assert video.frames == [b"JPEG5@80", b"JPEG6@80", b"JPEG8@80"]  # At the stream's quality
    assert [(r["index"], r["frame_number"], r["captured"]) for r in rows(folder / "oak_frames.csv")] == [
        ("0", "5", "105.0"), ("1", "6", "106.0"), ("2", "8", "108.0"),
    ]
    [pose] = [json.loads(line) for line in (folder / "poses.jsonl").read_text().splitlines()]
    assert (pose["frame_number"], pose["captured"], pose["people"][0]["score"]) == (6, 106.0, 0.9)
    assert np.array(pose["people"][0]["keypoints"]).shape == (17, 3)
    assert [(r["x"], r["y"]) for r in rows(folder / "targets.csv")] == [("3", "2")]
    assert [(r["pan"], r["locked_x"]) for r in rows(folder / "servo_state.csv")] == [("95.0", "True")]
    assert [(r["actuator"], r["value"]) for r in rows(folder / "commands.csv")] == [("pan", "90.0"), ("waist", "0.7")]
    run = json.loads((folder / "run.json").read_text())
    assert run["written"]["frame"] == 3 and run["dropped"] == {} and run["frames_missed"] == 1
    assert run["configuration"] == {"note": "test run"} and run["camera"] == "oak"


def record_frames(tmp_path, frames, **options):
    """Publish `frames` to a recorder and return its video and encoder once it has stopped."""
    videos, encoder = [], options.pop("encoder", FakeEncoder())

    async def scenario():
        bus = PubSubTopicManager.create()
        node = make_recorder(bus, tmp_path, None, videos, encoder=encoder, **options)
        task = asyncio.create_task(node.run())
        await asyncio.sleep(0.02)
        for message in frames:
            await bus.publish(FrameTopic, message)
            await asyncio.sleep(0.02)
        node.stop()
        await task
        bus.close()

    asyncio.run(scenario())
    return videos[0], encoder


def test_a_frame_the_stream_already_encoded_is_not_encoded_again(tmp_path):
    encoder = FakeEncoder()
    shown = frame(5)
    shown.jpeg_bytes(80, encoder)  # As the stream does for a watching dashboard
    video, _ = record_frames(tmp_path, [shown, frame(6)], encoder=encoder)
    assert video.frames == [b"JPEG5@80", b"JPEG6@80"]
    assert encoder.calls == [(5, 80), (6, 80)]  # Frame 5 once, for both


def test_a_cameras_own_jpeg_is_recorded_as_it_came(tmp_path):
    video, encoder = record_frames(tmp_path, [frame(5, jpeg=b"from the camera")])
    assert video.frames == [b"from the camera"] and encoder.calls == []


def test_a_recording_quality_of_its_own_encodes_separately(tmp_path):
    video, encoder = record_frames(tmp_path, [frame(5)], jpeg_quality=95)
    assert video.frames == [b"JPEG5@95"]


def imu_batch(first_sample, count=4):
    times = 200.0 + 0.00126 * np.arange(first_sample, first_sample + count)
    acceleration = np.tile(np.array([0.0, -9.8, 0.5], np.float32), (count, 1))
    return ImuMessage(
        timestamps=times, imu_times=times - 50.0, acceleration=acceleration,
        angular_velocity=np.ones((count, 3), np.float32), first_sample=first_sample, temperature=52.5,
    )


def test_every_imu_sample_is_saved_and_missed_batches_are_counted(tmp_path):
    async def scenario():
        bus = PubSubTopicManager.create()
        node = make_recorder(bus, tmp_path, None, [])
        task = asyncio.create_task(node.run())
        for first in (0, 4, 12):  # The batch of samples 8 to 11 never reaches the recorder
            await bus.publish(ImuTopic, imu_batch(first))
            await asyncio.sleep(0.02)
        node.stop()
        await task
        bus.close()
        return node.folder

    folder = asyncio.run(scenario())

    saved = rows(folder / "imu.csv")
    assert len(saved) == 12 and list(saved[0]) == ["time", "imu_time", "ax", "ay", "az", "gx", "gy", "gz", "temperature"]
    assert (saved[0]["time"], saved[0]["imu_time"], saved[0]["ay"], saved[0]["temperature"]) == (
        "200.000000", "150.000000", "-9.8000", "52.50",
    )
    assert saved[8]["time"] == "200.015120"  # Sample 12
    run = json.loads((folder / "run.json").read_text())
    assert run["written"]["imu"] == 3 and run["written"]["imu_sample"] == 12 and run["imu_samples_missed"] == 4


def test_a_stalled_disk_drops_frames_instead_of_slowing_the_bot(tmp_path):
    videos, gate = [], threading.Event()

    async def scenario():
        bus = PubSubTopicManager.create()
        node = make_recorder(bus, tmp_path, None, videos, gate=gate, queue_size=2)
        task = asyncio.create_task(node.run())
        for number in range(10):
            await asyncio.wait_for(bus.publish(FrameTopic, frame(number)), timeout=0.1)
            await asyncio.sleep(0.01)
        gate.set()  # The disk catches up
        node.stop()
        await task
        bus.close()
        return node

    node = asyncio.run(scenario())

    assert node.dropped["frame"] > 0
    assert node.counts["frame"] + node.dropped["frame"] == 10
    assert len(videos[0].frames) == node.counts["frame"]


def test_a_failed_recording_never_stops_or_hangs_the_run(tmp_path):
    def broken_writer(*args):
        raise RuntimeError("no codec")

    async def scenario():
        bus = PubSubTopicManager.create()
        node = RunRecorderNode.create(
            pubsub=bus, params=RunRecorderParams(folder=tmp_path, queue_size=2), camera="oak",
            frame_topic=FrameTopic, commands=None, encode_jpeg=FakeEncoder(), open_writer=broken_writer,
        )
        task = asyncio.create_task(node.run())
        for number in range(10):
            await bus.publish(FrameTopic, frame(number))
            await asyncio.sleep(0.01)
        node.stop()
        await asyncio.wait_for(task, timeout=2)
        bus.close()
        return node

    node = asyncio.run(scenario())

    run = json.loads((node.folder / "run.json").read_text())
    assert "no codec" in run["failure"] and run["dropped"]["frame"] > 0


def test_command_listeners_hear_each_change_once():
    commands, heard = CommandRecorder(), []
    commands.listeners.append(heard.append)

    commands.record("waist", 0.7)
    commands.record("waist", 0.7)  # A repeat is not a change
    commands.record("waist", None)

    assert [(c.actuator, c.value) for c in heard] == [("waist", 0.7), ("waist", None)]


def test_the_oldest_runs_are_deleted_to_keep_the_recordings_in_budget(tmp_path):
    for name, size in (("run-20261101-120000", 1000), ("run-20261102-120000", 1000), ("run-20261103-120000", 1000),
                       ("motor-response-20261101-120000", 5000)):  # Not a run folder: never deleted
        (tmp_path / name).mkdir()
        (tmp_path / name / "oak.avi").write_bytes(b"x" * size)

    deleted = prune_runs(tmp_path, keep_bytes=2500)

    assert deleted == [("run-20261101-120000", 1000)]
    assert sorted(path.name for path in tmp_path.iterdir()) == [
        "motor-response-20261101-120000", "run-20261102-120000", "run-20261103-120000"]
    assert prune_runs(tmp_path / "missing", keep_bytes=0) == []


def test_a_nearly_full_disk_stops_the_video_but_not_the_tables(tmp_path, monkeypatch):
    monkeypatch.setattr(run_recorder_node.shutil, "disk_usage", lambda path: shutil._ntuple_diskusage(100e9, 99e9, 1e9))
    videos = []

    async def scenario():
        bus = PubSubTopicManager.create()
        node = make_recorder(bus, tmp_path, None, videos, min_free_gb=20.0)
        task = asyncio.create_task(node.run())
        await asyncio.sleep(0.02)
        for number in (1, 2, 3):
            await bus.publish(FrameTopic, frame(number))
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=320.0, y=240.0))
            await asyncio.sleep(0.02)
        node.stop()
        await task
        bus.close()
        return node

    node = asyncio.run(scenario())

    assert videos == [] and node.dropped["frame_low_disk"] == 3
    assert len(rows(node.folder / "targets.csv")) == 3
