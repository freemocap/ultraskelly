import zlib

import cbor2
import numpy as np
import pytest

from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.bot_topics import DepthFrameMessage
from ultraskelly.ui.depth_stream import encode_depth_frame, nominal_camera_to_head

LEVEL = StereoRectification(width=672, height=376, focal_length=340.0, cx=331.0, cy=193.0, baseline=0.063,
                            left_rotation=((1, 0, 0), (0, 1, 0), (0, 0, 1)))


def wall_at(metres: float) -> DepthFrameMessage:
    """A flat wall facing the camera, with a hole of unknown depth in its corner."""
    depth = np.full((376, 672), int(metres * 1000), np.uint16)
    depth[:50, :50] = 0
    return DepthFrameMessage(depth=depth, color=np.zeros((376, 672, 3), np.uint8), rectification=LEVEL,
                             frame_number=77, timestamp=12.25, method="sgbm")


def decode_points(message: dict) -> np.ndarray:
    """Every known depth pixel as an (x, y, z) point in metres in the camera frame, as a viewer would."""
    depth, camera = message["depth"], message["camera"]
    values = np.frombuffer(zlib.decompress(depth["data"]), "<u2").reshape(depth["height"], depth["width"])
    rows, columns = np.nonzero(values != depth["invalid"])
    z = values[rows, columns] / 1000.0
    u, v = columns * depth["step"], rows * depth["step"]  # Camera pixels the samples came from
    return np.stack([(u - camera["cx"]) * z / camera["fx"], (v - camera["cy"]) * z / camera["fy"], z], axis=1)


def test_a_depth_frame_carries_depth_colour_intrinsics_and_pose():
    encoded = encode_depth_frame(wall_at(1.5), color_jpeg=b"JPEG", sequence=3,
                                 camera_to_head=nominal_camera_to_head(LEVEL))
    message = cbor2.loads(encoded)

    assert (message["kind"], message["version"], message["sequence"], message["timestamp"]) == ("depth_frame", 0, 3, 12.25)
    assert (message["frame_number"], message["method"]) == (77, "sgbm")
    assert message["camera"] == {"id": "zed_rectified", "width": 672, "height": 376, "fx": 340.0, "fy": 340.0,
                                 "cx": 331.0, "cy": 193.0, "baseline_m": 0.063,
                                 "axes": "x image right, y image down, z forward"}
    depth = message["depth"]
    assert (depth["width"], depth["height"], depth["step"], depth["units"], depth["encoding"], depth["invalid"],
            depth["compression"]) == (336, 188, 2, "mm", "uint16le", 0, "zlib")
    assert len(depth["data"]) < 336 * 188 * 2 / 10  # A flat wall compresses well
    assert message["color"] == {"encoding": "jpeg", "width": 672, "height": 376, "data": b"JPEG"}

    points = decode_points(message)
    assert len(points) == 336 * 188 - 25 * 25  # The hole stays unknown
    assert points[:, 2] == pytest.approx(1.5)
    corner = points[np.argmax(points[:, 0])]  # Rightmost point: image column 670
    assert corner[0] == pytest.approx((670 - 331.0) * 1.5 / 340.0)


def test_the_nominal_pose_puts_the_camera_in_the_left_eye_socket_looking_forward():
    pose = nominal_camera_to_head(LEVEL)
    rotation, translation = np.array(pose["rotation"]), np.array(pose["translation_m"])

    assert rotation @ [0, 0, 1] == pytest.approx([1, 0, 0])  # The optical axis points out of the face
    assert rotation @ [1, 0, 0] == pytest.approx([0, -1, 0])  # Image right is the skull's right
    assert rotation @ [0, 1, 0] == pytest.approx([0, 0, -1])  # Image down is down
    assert translation == pytest.approx([0, 0.0315, 0])  # Half a baseline to the skull's left
    assert pose["status"] == "nominal"
    # A point 1 m straight ahead of the left eye sits 1 m in front of the face, level with the eyes.
    assert rotation @ [0, 0, 1.0] + translation == pytest.approx([1.0, 0.0315, 0.0])
