import asyncio
import io
import struct
import time
from pathlib import Path

import cbor2
import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.bot.sensory import zed_imu_node, zed_imu_reader
from ultraskelly.core.bot.sensory.zed_imu_node import (
    TICK,
    TICKS_PER_SAMPLE,
    ImuClock,
    ImuSamples,
    ReaderProcess,
    ZedImuNode,
    ZedImuNodeParams,
    parse_reports,
)
from ultraskelly.core.bot.sensory.zed_imu_reader import HEADER, REPORT_SIZE, hid_set_feature, stream
from ultraskelly.core.pubsub.bot_topics import ImuMessage, ImuTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.imu_stream import encode_imu_batch
from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams

# A report the ZED Mini on the Pi sent on 2026-11-03, the camera at rest
CAPTURED = bytes.fromhex(
    "00002215000000000000e8ff1a002c00f6f4cd0b28fe000000000000801401010101010101010101010101010101"
    "010101010101010101010101010101010101"
)


def report(ticks: int, *, valid: bool = True, acceleration=(0, 0, 4096), angular_velocity=(0, 0, 0)) -> bytes:
    data = bytearray(REPORT_SIZE)
    data[1] = 0 if valid else 1
    struct.pack_into("<Q3h3h", data, 2, ticks, *angular_velocity, *acceleration)
    struct.pack_into("<h", data, 28, 2500)
    return bytes(data)


def samples(ticks) -> ImuSamples:
    ticks = np.asarray(ticks, np.int64)
    return ImuSamples(
        ticks=ticks, acceleration=np.zeros((len(ticks), 3), np.float32),
        angular_velocity=np.zeros((len(ticks), 3), np.float32), temperature=np.zeros(len(ticks), np.float32),
    )


def make_node(start_reader=None, **params) -> ZedImuNode:
    return ZedImuNode(pubsub=PubSubTopicManager.create(), params=ZedImuNodeParams(**params), start_reader=start_reader)


def test_reports_are_read_with_zed_open_captures_layout_and_scales():
    read = parse_reports(CAPTURED + report(5442, valid=False) + report(5474))

    assert read.ticks.tolist() == [5410, 5474]  # The sample marked not valid is skipped
    assert read.acceleration[0] == pytest.approx([-6.7745, 7.2419, -1.1315], abs=1e-3)  # m/s², gravity included
    assert read.angular_velocity[0] == pytest.approx([-0.7324, 0.7935, 1.3428], abs=1e-3)  # Degrees per second
    assert read.temperature[0] == pytest.approx(52.48)
    assert hid_set_feature(2) == 0xC0024806  # Linux's HIDIOCSFEATURE(2)


def test_the_imus_slow_clock_is_mapped_onto_host_time_and_its_rate_measured():
    rng = np.random.default_rng(1)
    drift = 0.00777  # The IMU's clock ran this much slow on the Pi
    ticks = 7_000_000 + TICKS_PER_SAMPLE * np.arange(24_000)  # 30 s at 800 Hz by the IMU's clock
    host = 5000.0 + ticks * TICK * (1 + drift)
    arrival = host + rng.exponential(0.0006, len(ticks))  # Each report a little after its sample
    arrival[9000:9060] += 0.05  # A reader stalled for 50 ms
    available = np.maximum.accumulate(arrival)  # Reports are read in order
    node, start = make_node(), 0
    for read_at in np.arange(host[0], available[-1] + 0.04, 0.02):  # A read every 20 ms takes what has arrived
        end = int(np.searchsorted(available, read_at, side="right"))
        if end > start:
            node.gather(samples(ticks[start:end]), read_at)
            start = end
    batch = node.take()

    assert len(batch.timestamps) == len(ticks) and batch.first_sample == 0 and node.lost == 0
    error = batch.timestamps - host
    assert np.abs(error).max() < 0.001  # The first second waited for the clock to be fitted
    assert np.abs(error[host - host[0] > 2.0]).max() < 0.0005
    assert np.all(np.diff(batch.timestamps) >= 0)
    assert node.sample_rate() == pytest.approx(800 / (1 + drift), abs=0.05)
    assert batch.imu_times == pytest.approx(ticks * TICK)


def test_lost_samples_are_counted_and_a_restarted_count_restarts_the_clock():
    node = make_node()

    def feed(first_tick, count, start):  # Reads every 20 ms, each report arriving as it was taken
        ticks = first_tick + TICKS_PER_SAMPLE * np.arange(count)
        for index in range(0, count, 16):
            chunk = ticks[index:index + 16]
            node.gather(samples(chunk), start + (chunk[-1] - first_tick) * TICK + 0.0002)

    feed(0, 1600, 10.0)  # Two seconds
    node.gather(samples([51264, 51296]), 12.003)  # Ticks 51200 and 51232 never arrived
    lost = node.lost
    feed(0, 1600, 13.0)  # The camera reset, and its count with it
    batch = node.take()

    assert lost == 2 and node.lost == 2 and node.last_tick == 1599 * TICKS_PER_SAMPLE
    assert len(batch.timestamps) == 3202 and np.all(np.diff(batch.timestamps[:1602]) >= 0)
    assert batch.timestamps[-1] == pytest.approx(13.0 + 1599 * TICKS_PER_SAMPLE * TICK, abs=0.001)


class FakeHid:
    """An IMU device that returns given reads, then nothing."""

    def __init__(self, reads):
        self.reads, self.streaming = list(reads), []

    def set_streaming(self, on):
        self.streaming.append(on)

    def read(self, timeout):
        if self.reads:
            return self.reads.pop(0), time.monotonic()
        time.sleep(0.005)
        return b"", time.monotonic()

    def close(self):
        pass


def test_the_reader_writes_each_read_with_its_time_and_restarts_a_quiet_stream(monkeypatch):
    monkeypatch.setattr(zed_imu_reader, "READ_SECONDS", 0.0)
    monkeypatch.setattr(zed_imu_reader, "QUIET_SECONDS", 0.05)
    device, out = FakeHid([report(0) + report(32), report(64)]), io.BytesIO()
    started = time.monotonic()
    stream(device, out, lambda: time.monotonic() - started > 0.2)
    written = out.getvalue()

    first_at, first_count = HEADER.unpack_from(written, 0)
    second_at, second_count = HEADER.unpack_from(written, HEADER.size + 2 * REPORT_SIZE)
    assert (first_count, second_count) == (2, 1) and started <= first_at <= second_at
    assert len(written) == 2 * HEADER.size + 3 * REPORT_SIZE
    assert written[HEADER.size:HEADER.size + REPORT_SIZE] == report(0)
    assert device.streaming[0] is True and device.streaming.count(True) >= 2  # Started, then again when quiet


def test_the_node_reads_the_readers_pipe_until_it_ends():
    async def scenario():
        pipe = asyncio.StreamReader()
        pipe.feed_data(HEADER.pack(5.0, 2) + report(0) + report(32) + HEADER.pack(5.02, 1) + report(64))
        pipe.feed_data(HEADER.pack(5.04, 3)[:5])  # The reader died mid-header
        pipe.feed_eof()

        class Process:
            stdout, returncode = pipe, 1

        reader = ReaderProcess(Process())
        return [await reader.read() for _ in range(3)]

    first, second, end = asyncio.run(scenario())

    assert first == (report(0) + report(32), 5.0) and second == (report(64), 5.02) and end is None


class FakeReader:
    """A reader process's reads, then its end."""

    def __init__(self, reads):
        self.reads, self.closed = list(reads), False

    async def read(self):
        await asyncio.sleep(0.005)
        return self.reads.pop(0) if self.reads else None

    async def close(self):
        self.closed = True


def test_the_node_publishes_every_sample_and_starts_a_stopped_reader_again(monkeypatch):
    monkeypatch.setattr(zed_imu_node, "REOPEN_SECONDS", 0.01)
    monkeypatch.setattr(zed_imu_node, "FIT_BLOCKS", 2)
    burst = lambda first: (b"".join(report(first + TICKS_PER_SAMPLE * index) for index in range(4)), time.monotonic())
    readers = [FakeReader([burst(0), burst(128)]), OSError("No such device"), FakeReader([burst(256), burst(384)])]
    started = []

    async def start_reader():
        started.append(readers[len(started)] if len(started) < len(readers) else FakeReader([]))
        if isinstance(started[-1], Exception):
            raise started[-1]
        return started[-1]

    async def scenario():
        node = make_node(start_reader, batch_hz=100)
        node.clock = ImuClock(block=0.004)  # Fitted within these few reads
        batches = node.pubsub.get_subscription(ImuTopic)
        running = asyncio.create_task(node.run())
        received = []
        while sum(len(batch.timestamps) for batch in received) < 16:
            received.append(await asyncio.wait_for(batches.get(), timeout=5))
        node.stop()
        await running
        node.pubsub.close()
        return received

    received = asyncio.run(scenario())

    assert [batch.first_sample for batch in received] == list(np.cumsum([0] + [len(b.timestamps) for b in received])[:-1])
    assert np.concatenate([batch.imu_times for batch in received]) == pytest.approx(TICK * TICKS_PER_SAMPLE * np.arange(16))
    assert started[:3] == readers and readers[0].closed and readers[2].closed
    with pytest.raises(OSError, match="udev rule"):
        ZedImuNode.create(pubsub=PubSubTopicManager.create(), params=ZedImuNodeParams(device=Path("/dev/no-such-imu")))


def batch(first_sample=40) -> ImuMessage:
    times = 100.0 + np.array([0.0, 0.00126, 0.00252])
    return ImuMessage(
        timestamps=times, imu_times=times - 99.5, acceleration=np.array([[0.0, -9.8, 0.1]] * 3, np.float32),
        angular_velocity=np.array([[1.0, 2.0, 3.0]] * 3, np.float32), first_sample=first_sample, lost=1,
        temperature=52.5, sample_rate=793.8,
    )


def test_a_batch_streams_as_an_imu_message_with_every_sample_in_a_channel_block():
    message = cbor2.loads(encode_imu_batch(batch(), sequence=7))

    assert (message["kind"], message["sequence"], message["first_sample"], message["count"]) == ("imu", 7, 40, 3)
    assert message["start_time"] == 100.0 and message["timestamp"] == pytest.approx(100.00252)
    assert message["axes"].startswith("sensor") and message["units"]["angular_velocity"] == "deg/s"
    assert (message["sample_rate"], message["lost"], message["temperature"]) == (793.8, 1, 52.5)
    block = message["samples"]
    assert (block["kind"], block["columns"]) == ("ultraskelly.imu", ["time", "ax", "ay", "az", "gx", "gy", "gz"])
    rows = np.frombuffer(block["data"], "<f4").reshape(-1, 7)
    assert rows[:, 0] == pytest.approx([0.0, 0.00126, 0.00252], abs=1e-7)  # Seconds after start_time
    assert rows[1, 1:] == pytest.approx([0.0, -9.8, 0.1, 1.0, 2.0, 3.0])


def test_the_runtime_server_streams_each_imu_batch():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = RuntimeServerNode.create(
            pubsub=bus, params=RuntimeServerParams(state_hz=50), cameras={}, recorder=None,
            motors_simulated=True, node_status=lambda: [],
        )
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await client.ws_connect("/stream")
            while server.clients == 0:
                await asyncio.sleep(0.005)
            received = []
            for first in (0, 3):
                await bus.publish(ImuTopic, batch(first_sample=first))
                while len(received) <= first // 3:
                    message = cbor2.loads((await asyncio.wait_for(socket.receive(), timeout=5)).data)
                    if message["kind"] == "imu":
                        received.append(message)
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return received

    received = asyncio.run(scenario())

    assert [(message["sequence"], message["first_sample"]) for message in received] == [(1, 0), (2, 3)]
