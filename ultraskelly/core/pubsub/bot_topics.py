import threading
from collections.abc import Callable

import numpy as np
from pydantic import ConfigDict, PrivateAttr

from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
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


class StereoFrameMessage(TopicMessageABC):
    """A stereo camera's two images, captured together (the ZED Mini in the skull's eye sockets)."""
    left: np.ndarray  # RGB (height, width, 3): the skeleton's left eye, on the camera's left looking out
    right: np.ndarray  # RGB, the same size: the skeleton's right eye
    frame_number: int | None = None  # Counts camera frame periods, so a dropped frame skips a number
    timestamp: float | None = None  # Capture time in seconds on time.monotonic()'s clock, when known
    # The camera both images now share, when they were rectified; None for the camera's own images.
    rectification: StereoRectification | None = None

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


class DepthFrameMessage(TopicMessageABC):
    """Depth from the stereo camera, pixel-aligned with its rectified left image."""
    depth: np.ndarray  # uint16 (height, width): millimetres along the camera's forward axis; 0 = unknown
    color: np.ndarray  # RGB (height, width, 3): the rectified left image the depth belongs to
    rectification: StereoRectification  # The pinhole camera both share, and the baseline
    frame_number: int | None = None  # The stereo pair's frame number
    timestamp: float | None = None  # The stereo pair's capture time, on time.monotonic()'s clock
    method: str = ""  # How disparity was found: "sgbm" (OpenCV on the CPU) or "stereonet" (Hailo-8)

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


class FaceLocationMessage(TopicMessageABC):
    """Where the tracked face is in 3D, found from the OAK's target pixel and the ZED's depth.
    Positions are None when no face was found (no target, no recent depth, or no surface on its ray)."""
    position_camera: tuple[float, float, float] | None = None  # Metres, the ZED's rectified left camera frame
    position_head: tuple[float, float, float] | None = None  # Metres, the head frame (x forward, y left, z up)
    distance: float | None = None  # Metres from the head frame's origin, between the eye sockets
    pixel: tuple[int, int] | None = None  # The target pixel in the OAK image
    pose_timestamp: float | None = None  # The OAK frame's capture time
    depth_timestamp: float | None = None  # The depth frame's capture time
    alignment: str = "nominal"  # Which OAK-ZED alignment placed it: its file name, or "nominal"

    model_config = ConfigDict(frozen=True)


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
    target: TargetLocationMessage | None = None  # The target published from these same detections
    target_person: int | None = None  # Row of `keypoints` the head follows; None when it follows nobody

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )



# Create topics using the factory - they auto-register
FrameTopic = create_topic(FrameMessage)
StereoFrameTopic = create_topic(StereoFrameMessage)
DepthFrameTopic = create_topic(DepthFrameMessage)
FaceLocationTopic = create_topic(FaceLocationMessage)
TargetLocationTopic = create_topic(TargetLocationMessage)
ServoStateTopic = create_topic(ServoStateMessage)
PoseDataTopic = create_topic(PoseDataMessage)
