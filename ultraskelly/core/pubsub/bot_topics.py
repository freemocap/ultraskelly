import numpy as np
from pydantic import ConfigDict

from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.pubsub_abcs import TopicMessageABC, create_topic


class FrameMessage(TopicMessageABC):
    """Raw frame from camera."""
    frame: np.ndarray
    frame_number: int | None = None  # The camera's frame counter, when it provides one
    timestamp: float | None = None  # Capture time in seconds on time.monotonic()'s clock, when known

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
StereoFrameTopic = create_topic(StereoFrameMessage)
TargetLocationTopic = create_topic(TargetLocationMessage)
ServoStateTopic = create_topic(ServoStateMessage)
PoseDataTopic = create_topic(PoseDataMessage)
