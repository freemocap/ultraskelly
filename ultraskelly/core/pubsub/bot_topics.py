import threading
from collections.abc import Callable

import numpy as np
from pydantic import ConfigDict, PrivateAttr

from ultraskelly.core.pubsub.pubsub_abcs import TopicMessageABC, create_topic


class FrameMessage(TopicMessageABC):
    """Raw frame from camera."""
    frame: np.ndarray
    frame_number: int | None = None  # The camera's frame counter, when it provides one
    timestamp: float | None = None  # Capture time in seconds on time.monotonic()'s clock, when known
    jpeg: bytes | None = None  # The frame as JPEG when the camera encodes it itself, as an OAK's hardware encoder can
    _jpegs: dict[int, bytes] = PrivateAttr(default_factory=dict)  # Encoded here, by quality
    _jpeg_lock: threading.Lock = PrivateAttr(default_factory=threading.Lock)

    def jpeg_bytes(self, quality: int, encode: Callable[[np.ndarray, int], bytes]) -> bytes:
        """The frame as JPEG: the camera's own when it sent one, else encoded at `quality` once,
        whichever consumer asks first, and kept for the rest (the stream and the recorder)."""
        if self.jpeg is not None:
            return self.jpeg
        with self._jpeg_lock:
            if quality not in self._jpegs:
                self._jpegs[quality] = encode(self.frame, quality)
            return self._jpegs[quality]

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


class TargetLocationMessage(TopicMessageABC):
    """Generic target location with orientation."""
    x: int | None = None
    y: int | None = None
    angle: float | None = None  # Rotation angle in degrees

    model_config = ConfigDict(frozen=True)


class ServoStateMessage(TopicMessageABC):
    """Motor node output."""
    pan_angle: float
    tilt_angle: float
    roll_angle: float
    is_locked_x: bool
    is_locked_y: bool
    is_locked_roll: bool

    model_config = ConfigDict(frozen=True)


class PoseDataMessage(TopicMessageABC):
    """Full pose detection data for visualization."""
    keypoints: np.ndarray | None = None  # Shape: (num_people, 17, 3)
    scores: np.ndarray | None = None  # Shape: (num_people,)
    boxes: list[np.ndarray] | None = None
    frame: FrameMessage | None = None  # The camera frame these poses were detected in, so viewers can pair them

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )



# Create topics using the factory - they auto-register
FrameTopic = create_topic(FrameMessage)
TargetLocationTopic = create_topic(TargetLocationMessage)
ServoStateTopic = create_topic(ServoStateMessage)
PoseDataTopic = create_topic(PoseDataMessage)
