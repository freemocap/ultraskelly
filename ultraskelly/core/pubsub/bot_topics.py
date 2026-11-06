import threading
from collections.abc import Callable

import numpy as np
from pydantic import BaseModel, ConfigDict, PrivateAttr

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
    """A stereo camera's two eyes, captured together, as the camera sends them: one YUYV frame
    with the left eye's half first (the ZED Mini in the skull's eye sockets; its left eye is the
    skeleton's left, on the camera's left looking out). Each consumer converts only what it
    needs: `grey` is an eye's Y bytes as they are, with no colour conversion; colour comes
    from `eye` with OpenCV (`yuyv_to_rgb_with_opencv`, or COLOR_YUV2BGR_YUYV for JPEG)."""
    yuyv: np.ndarray  # (height, 2 * width, 2) uint8: each pixel's Y, then alternately U and V
    frame_number: int | None = None  # Counts camera frame periods, so a dropped frame skips a number
    timestamp: float | None = None  # Capture time in seconds on time.monotonic()'s clock, when known

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )

    @property
    def eye_size(self) -> tuple[int, int]:
        """One eye's (width, height)."""
        return self.yuyv.shape[1] // 2, self.yuyv.shape[0]

    def eye(self, index: int) -> np.ndarray:
        """Eye `index` (0 left, 1 right) as YUYV, (height, width, 2): a view, not a copy."""
        width = self.yuyv.shape[1] // 2
        return self.yuyv[:, index * width:(index + 1) * width]

    def grey(self, index: int) -> np.ndarray:
        """Eye `index`'s brightness, (height, width) uint8: its Y bytes, copied out."""
        return np.ascontiguousarray(self.eye(index)[..., 0])


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


class ImuMessage(TopicMessageABC):
    """A batch of IMU samples, oldest first: the ZED Mini's accelerometer and gyroscope, raw and in
    the IMU's own axes (not the head frame). For display and recording; nothing steers by them."""
    timestamps: np.ndarray  # (N,) float64 seconds on time.monotonic()'s clock
    imu_times: np.ndarray  # (N,) float64 seconds by the IMU's own clock, which runs at its own rate
    acceleration: np.ndarray  # (N, 3) float32 m/s² along the IMU's x, y, z; gravity included
    angular_velocity: np.ndarray  # (N, 3) float32 degrees per second about the IMU's x, y, z
    first_sample: int  # The first sample's number since the IMU node started: a jump means missed batches
    lost: int = 0  # Samples the IMU took that never arrived, so far
    temperature: float | None = None  # °C, the IMU's own, at the last sample
    sample_rate: float | None = None  # Samples per second by time.monotonic()'s clock, once measured

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


class TargetLocationMessage(TopicMessageABC):
    """Generic target location with orientation."""
    x: int | None = None
    y: int | None = None
    angle: float | None = None  # Rotation angle in degrees
    captured: float | None = None  # When the camera captured the frame it came from, on time.monotonic()'s clock

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


class FaceObservation(BaseModel):
    """One face the OAK's own chip found and measured in one of its frames. Pixels are the frame's."""
    box: tuple[float, float, float, float]  # x1, y1, x2, y2
    score: float  # YuNet's confidence, 0 to 1
    landmarks: np.ndarray  # (5, 2) x, y: right eye, left eye, nose, right and left mouth corner (the person's own sides)
    head_pose: tuple[float, float, float] | None = None  # Yaw, pitch, roll in degrees, as the head pose network gives them
    gaze: tuple[float, float, float] | None = None  # Unit gaze direction in the gaze network's axes (docs/stream.md)
    mesh: np.ndarray | None = None  # (468, 3) MediaPipe face landmarks: x, y and relative depth z, in pixels

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


class HandObservation(BaseModel):
    """One hand the OAK's own chip found in one of its frames. Pixels are the frame's."""
    palm_box: tuple[float, float, float, float]  # x1, y1, x2, y2 of the palm the detector found
    palm_score: float  # The palm detector's confidence, 0 to 1
    points: np.ndarray  # (21, 3) MediaPipe hand landmarks: x, y and relative depth z, in pixels
    presence: float  # The hand landmarker's confidence that a hand is there, 0 to 1
    handedness: float  # The hand landmarker's handedness score, 0 to 1; above 0.5 it calls the hand a right one

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


class OakResultsMessage(TopicMessageABC):
    """What the models on the OAK's own chip found in one OAK frame: its faces, and on the frames
    they are looked for in, its hands."""
    faces: list[FaceObservation]
    hands: list[HandObservation] | None = None  # None: not looked for in this frame; []: none found
    image_size: tuple[int, int]  # (width, height) of the frame the pixels belong to
    frame_number: int | None = None  # The OAK frame's number, as in its FrameMessage
    timestamp: float | None = None  # That frame's capture time, on time.monotonic()'s clock

    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        frozen=True
    )


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
OakResultsTopic = create_topic(OakResultsMessage)
ImuTopic = create_topic(ImuMessage)
TargetLocationTopic = create_topic(TargetLocationMessage)
ServoStateTopic = create_topic(ServoStateMessage)
PoseDataTopic = create_topic(PoseDataMessage)
