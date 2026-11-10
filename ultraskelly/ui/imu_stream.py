"""The `imu` stream message: a batch of IMU samples for time-series views (docs/stream.md).

Each batch holds every sample since the last, packed as a FreeMoCap channel block: one row per
sample, float32 columns `time` (seconds after `start_time`), `ax`, `ay`, `az` (m/s²) and `gx`,
`gy`, `gz` (degrees per second). `start_time` is float64 seconds on time.monotonic()'s clock,
the clock of every other time on the stream.
"""
import numpy as np
from freemocap_sdk.stream import channel_block, encode_message

from ultraskelly.core.pubsub.bot_topics import ImuMessage

IMU_KIND = "imu"
IMU_CHANNEL = "ultraskelly.imu"
COLUMNS = ("time", "ax", "ay", "az", "gx", "gy", "gz")
AXES = "sensor: the IMU's own x, y, z, not yet placed in the head frame"


def encode_imu_batch(message: ImuMessage, *, sequence: int, source: str = "zed_mini") -> bytes:
    """One `imu` message for a batch of samples."""
    start = float(message.timestamps[0])
    rows = np.column_stack([message.timestamps - start, message.acceleration, message.angular_velocity])
    return encode_message(IMU_KIND, {
        "source": source,
        "axes": AXES,
        "start_time": start,
        "first_sample": message.first_sample,
        "count": len(rows),
        "samples": channel_block(IMU_CHANNEL, COLUMNS, rows),
        "units": {"time": "s", "acceleration": "m/s^2", "angular_velocity": "deg/s", "temperature": "degC"},
        "sample_rate": message.sample_rate,
        "lost": message.lost,
        "temperature": message.temperature,
    }, timestamp=float(message.timestamps[-1]), sequence=sequence)
