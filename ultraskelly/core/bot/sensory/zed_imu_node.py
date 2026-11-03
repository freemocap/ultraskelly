"""The ZED Mini's IMU (accelerometer and gyroscope) over Linux hidraw, publishing batches to ImuTopic.

The camera's microcontroller reports the IMU on its own USB HID interface (2b03:f681), which
needs no ZED SDK. The report layout, the feature report that starts the stream and the scale
factors are those of Stereolabs' zed-open-capture (include/sensorcapture_def.hpp). Each
64-byte input report holds one sample and the IMU's tick count, 39.0625 µs a tick. A reader
process (zed_imu_reader.py) takes the reports as they come and pipes them here. The ticks say
800 samples a second, but on this camera they ran 0.78% slow against time.monotonic(), so
`ImuClock` maps them onto that clock, the cameras' clock, from the reads that came soonest.
Values are raw, in the IMU's own axes, with no bias or scale calibration. For display and
recording only: nothing steers by them.
"""
import asyncio
import logging
import os
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Awaitable, Callable, Protocol

import numpy as np
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.sensory.zed_imu_reader import HEADER, REPORT_SIZE
from ultraskelly.core.pubsub.bot_topics import ImuMessage, ImuTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

IMU_DEVICE = Path("/dev/zed-mini-imu")  # The name scripts/pi-system's udev rule gives it
HIDRAW_DEVICES = Path("/sys/class/hidraw")
HID_ID = "0003:00002B03:0000F681"  # USB bus, Stereolabs, the ZED Mini's HID interface
NOT_FOUND = (
    "No ZED Mini IMU among the hidraw devices. Its HID interface (USB 2b03:f681) shows in lsusb "
    "with the camera; scripts/setup-pi.sh installs the udev rule that lets the bot open it."
)

TICK = 39062.5e-9  # Seconds per IMU tick
TICKS_PER_SAMPLE = 32  # 800 samples a second by the IMU's clock
ACCELERATION_SCALE = 9.8189 * 8.0 / 32768.0  # m/s² per count: ±8 g
ANGULAR_VELOCITY_SCALE = 1000.0 / 32768.0  # Degrees per second per count: ±1000 °/s
TEMPERATURE_SCALE = 0.01  # °C per count

REOPEN_SECONDS = 1.0  # Before starting a reader again
REPORT_SECONDS = 30.0
FIT_BLOCKS = 4  # Whole blocks of reads before the clock map is fitted: a second, which the first samples wait

# The fields of zed-open-capture's packed RawData this node uses, at their offsets.
REPORT = np.dtype({
    "names": ["imu_not_valid", "ticks", "angular_velocity", "acceleration", "temperature"],
    "formats": ["u1", "<u8", ("<i2", 3), ("<i2", 3), "<i2"],
    "offsets": [1, 2, 10, 16, 28],
    "itemsize": REPORT_SIZE,
})


@dataclass(frozen=True, eq=False)
class ImuSamples:
    """Samples from a run of reports, oldest first."""

    ticks: np.ndarray  # (N,) int64, the IMU's clock
    acceleration: np.ndarray  # (N, 3) float32 m/s²
    angular_velocity: np.ndarray  # (N, 3) float32 °/s
    temperature: np.ndarray  # (N,) float32 °C, the IMU's


def parse_reports(data: bytes) -> ImuSamples:
    """The valid samples in concatenated 64-byte reports."""
    reports = np.frombuffer(data, REPORT, count=len(data) // REPORT_SIZE)
    reports = reports[reports["imu_not_valid"] == 0]
    return ImuSamples(
        ticks=reports["ticks"].astype(np.int64),
        acceleration=(reports["acceleration"] * ACCELERATION_SCALE).astype(np.float32),
        angular_velocity=(reports["angular_velocity"] * ANGULAR_VELOCITY_SCALE).astype(np.float32),
        temperature=(reports["temperature"] * TEMPERATURE_SCALE).astype(np.float32),
    )


class ImuClock:
    """Maps IMU time onto time.monotonic()'s clock.

    A report reaches the host from a fraction of a millisecond to a few milliseconds after its
    sample, and the IMU's clock runs at its own rate. So the map is a line through the soonest
    arrivals: in each `block` seconds of IMU time, the read whose arrival time minus IMU time
    is least, once a first fit's slope is taken out, over the last `window` seconds.
    """

    def __init__(self, block: float = 0.25, window: float = 30.0) -> None:
        self.block, self.window = block, window
        self.reads: deque[tuple[float, float]] = deque()  # (IMU time, arrival minus IMU time) of each read
        self.reference = 0.0  # The IMU time the line is anchored at
        self.offset = float("inf")  # Host time minus IMU time at the reference
        self.drift = 0.0  # Host seconds gained per IMU second: how much slower the IMU's clock runs
        self.fitted = False  # True once FIT_BLOCKS whole blocks have measured the drift

    def add(self, imu_time: float, arrival: float) -> None:
        """A read's newest IMU time and the host time it was read at, no earlier than its arrival."""
        if self.reads and int(imu_time // self.block) != int(self.reads[-1][0] // self.block):
            self._fit()  # The previous block is complete
        lag = arrival - imu_time
        self.reads.append((imu_time, lag))
        while imu_time - self.reads[0][0] > self.window:
            self.reads.popleft()
        if not self.fitted and lag < self.offset:  # Until fitted, the soonest read so far
            self.reference, self.offset = imu_time, lag

    def _fit(self) -> None:
        times, lags = np.array(self.reads).T
        blocks = (times // self.block).astype(np.int64)
        if blocks[-1] - blocks[0] < FIT_BLOCKS:
            return
        reference = times[-1]
        rough = np.polyfit(times - reference, lags, 1)[0]  # Else a block's first reads would always look soonest
        order = np.lexsort((lags - rough * (times - reference), blocks))
        soonest = order[np.unique(blocks[order], return_index=True)[1]][1:]  # Not the oldest block: it is partial
        if len(soonest) < 2:  # Blocks without a read leave too few
            return
        drift, offset = np.polyfit(times[soonest] - reference, lags[soonest], 1)
        self.reference, self.offset, self.drift, self.fitted = float(reference), float(offset), float(drift), True

    def host_time(self, imu_time: np.ndarray) -> np.ndarray:
        return imu_time + self.offset + self.drift * (imu_time - self.reference)


class ImuReads(Protocol):
    async def read(self) -> tuple[bytes, float] | None: ...

    async def close(self) -> None: ...


class ReaderProcess:
    """The reader process (zed_imu_reader.py) and its pipe of reads."""

    def __init__(self, process: asyncio.subprocess.Process) -> None:
        self.process = process

    @classmethod
    async def start(cls, path: Path) -> "ReaderProcess":
        """Its warnings and errors go to the bot's own stderr."""
        return cls(await asyncio.create_subprocess_exec(
            sys.executable, "-m", "ultraskelly.core.bot.sensory.zed_imu_reader", str(path),
            stdin=asyncio.subprocess.DEVNULL, stdout=asyncio.subprocess.PIPE,
        ))

    async def read(self) -> tuple[bytes, float] | None:
        """The next read's reports and the time it was made; None once the reader has stopped."""
        try:
            read_at, count = HEADER.unpack(await self.process.stdout.readexactly(HEADER.size))
            return await self.process.stdout.readexactly(count * REPORT_SIZE), read_at
        except asyncio.IncompleteReadError:
            return None

    async def close(self) -> None:
        if self.process.returncode is None:
            self.process.terminate()  # It stops the IMU's stream as it goes
            try:
                await asyncio.wait_for(self.process.wait(), timeout=2.0)
            except asyncio.TimeoutError:
                self.process.kill()
                await self.process.wait()


def find_zed_imu() -> Path | None:
    """The IMU's device: the udev rule's /dev/zed-mini-imu, else its hidraw node."""
    if IMU_DEVICE.exists():
        return IMU_DEVICE
    for device in sorted(HIDRAW_DEVICES.glob("hidraw*")):
        try:
            if f"HID_ID={HID_ID}" in (device / "device" / "uevent").read_text():
                return Path("/dev") / device.name
        except OSError:
            continue
    return None


def openable_zed_imu(device: Path | None) -> Path:
    """`device`, else the IMU found, checked for this user to open."""
    path = device or find_zed_imu()
    if path is None:
        raise OSError(NOT_FOUND)
    if not os.access(path, os.R_OK | os.W_OK):
        raise OSError(f"This user may not open {path}; scripts/setup-pi.sh installs the udev rule that allows it")
    return path


class ZedImuNodeParams(NodeParams):
    """Parameters for ZedImuNode."""

    device: Path | None = Field(default=None, description="The IMU's hidraw device; None finds it")
    batch_hz: float = Field(
        default=20.0, gt=0.0, le=100.0,
        description="Batches a second at most, each with every sample since the last. A batch ends with a read "
                    "of the reader process, every 20 ms, so 20 gives about 15.",
    )


class ZedImuNode(Node):
    """Publishes the ZED Mini's IMU samples in batches, starting its reader again if it stops."""

    params: ZedImuNodeParams
    start_reader: SkipValidation[Callable[[], Awaitable[ImuReads]]] = Field(exclude=True)
    clock: SkipValidation[ImuClock] = Field(default_factory=ImuClock, exclude=True)
    waiting: SkipValidation[list[ImuSamples]] = Field(default_factory=list, exclude=True)  # Read before the clock was fitted
    # (host times, IMU times, acceleration, angular velocity, temperature) mapped, not yet published
    gathered: SkipValidation[list[tuple[np.ndarray, ...]]] = Field(default_factory=list, exclude=True)
    samples: int = Field(default=0, exclude=True, description="Samples published")
    lost: int = Field(default=0, exclude=True, description="Samples the IMU took that never arrived")
    last_tick: int | None = Field(default=None, exclude=True)
    last_time: float = Field(default=float("-inf"), exclude=True)

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: ZedImuNodeParams) -> "ZedImuNode":
        """Factory method: finds the IMU, so a missing one fails at startup; its reader starts with the node."""
        openable_zed_imu(params.device)
        return cls(pubsub=pubsub, params=params,
                   start_reader=lambda: ReaderProcess.start(openable_zed_imu(params.device)))

    def sample_rate(self) -> float | None:
        """Samples per second by time.monotonic()'s clock, once the IMU's drift is measured."""
        if not self.clock.fitted:
            return None
        return 1.0 / (TICKS_PER_SAMPLE * TICK * (1.0 + self.clock.drift))

    def summary(self) -> str:
        rate = self.sample_rate()
        timing = "rate not yet measured" if rate is None else (
            f"{rate:.1f} a second by the host's clock (the IMU's runs {100 * self.clock.drift:.2f}% slow)"
        )
        return f"{self.samples} IMU samples, {timing}, {self.lost} lost"

    async def run(self) -> None:
        """Map each read the reader sends and publish a batch every 1 / batch_hz seconds."""
        logger.info(f"Starting ZedImuNode [{self.params.batch_hz:g} batches a second]")
        published_at = reported_at = time.monotonic()
        warned_at = float("-inf")
        try:
            while not self.stop_event.is_set():
                try:
                    reader = await self.start_reader()
                except OSError as error:
                    if time.monotonic() - warned_at >= REPORT_SECONDS:
                        logger.warning(f"Cannot read the ZED's IMU ({error}); trying every {REOPEN_SECONDS:g} s")
                        warned_at = time.monotonic()
                    await asyncio.sleep(REOPEN_SECONDS)
                    continue
                try:
                    while not self.stop_event.is_set() and (read := await reader.read()) is not None:
                        self.gather(parse_reports(read[0]), read[1])
                        now = time.monotonic()
                        if now - published_at >= 1 / self.params.batch_hz:
                            published_at = now
                            if (batch := self.take()) is not None:
                                await self.pubsub.topics[ImuTopic].publish(batch)
                        if now - reported_at >= REPORT_SECONDS:
                            reported_at = now
                            logger.info(self.summary())
                    if (batch := self.take()) is not None:  # What the last reads brought
                        await self.pubsub.topics[ImuTopic].publish(batch)
                finally:
                    await reader.close()
                if not self.stop_event.is_set():
                    if time.monotonic() - warned_at >= REPORT_SECONDS:
                        logger.warning(f"The ZED's IMU reader stopped; starting it again in {REOPEN_SECONDS:g} s")
                        warned_at = time.monotonic()
                    await asyncio.sleep(REOPEN_SECONDS)
        finally:
            logger.info(f"ZedImuNode stopped: {self.summary()}")

    def take(self) -> ImuMessage | None:
        """Everything mapped since the last batch, or None."""
        gathered, self.gathered = self.gathered, []
        if not gathered:
            return None
        host, imu_time, acceleration, angular_velocity, temperature = (np.concatenate(part) for part in zip(*gathered))
        batch = ImuMessage(
            timestamps=host, imu_times=imu_time, acceleration=acceleration, angular_velocity=angular_velocity,
            temperature=float(temperature[-1]), first_sample=self.samples, lost=self.lost,
            sample_rate=self.sample_rate(),
        )
        self.samples += len(host)
        return batch

    def gather(self, samples: ImuSamples, arrival: float) -> None:
        """Map samples read at `arrival` onto host time and keep them for the next batch. Until the
        IMU's drift is measured, a second in, they wait, so the first are mapped as well as the rest."""
        ticks = samples.ticks
        if len(ticks) == 0:
            return
        previous = ticks[0] - TICKS_PER_SAMPLE if self.last_tick is None else self.last_tick
        steps = np.diff(ticks, prepend=previous)
        if steps.min() < 0:  # The IMU's count started over (the camera reset): so does the clock map
            logger.warning("The ZED's IMU clock started over")
            self._map_waiting()
            self.clock, self.last_time = ImuClock(), float("-inf")
            steps = np.full(len(ticks), TICKS_PER_SAMPLE)
        self.lost += int(np.maximum(np.rint(steps / TICKS_PER_SAMPLE) - 1, 0).sum())
        self.last_tick = int(ticks[-1])
        self.clock.add(float(ticks[-1] * TICK), arrival)  # The newest report waited least, so it says most
        self.waiting.append(samples)
        if self.clock.fitted:
            self._map_waiting()

    def _map_waiting(self) -> None:
        for samples in self.waiting:
            imu_time = samples.ticks * TICK
            host = np.maximum.accumulate(np.concatenate(([self.last_time], self.clock.host_time(imu_time))))[1:]
            self.last_time = float(host[-1])
            self.gathered.append((host, imu_time, samples.acceleration, samples.angular_velocity, samples.temperature))
        self.waiting.clear()
