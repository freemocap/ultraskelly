import numpy as np
from pydantic import ConfigDict

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
TargetLocationTopic = create_topic(TargetLocationMessage)
ServoStateTopic = create_topic(ServoStateMessage)
PoseDataTopic = create_topic(PoseDataMessage)
