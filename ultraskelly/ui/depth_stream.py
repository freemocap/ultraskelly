"""The `depth_frame` message: one RGB-D frame from the ZED, complete in itself.

FreeMoCap's image payload carries only JPEGs, so 16-bit depth travels as an
application message kind of its own on `/stream`, beside the frame messages. Each
message holds everything a viewer needs to draw a depth image, a point cloud or a mesh:
the depth in millimetres (zlib-compressed little-endian uint16, every `step`th pixel),
the rectified left image it belongs to, that image's pinhole intrinsics, and the
camera's pose in the head frame. docs/depth-stream.md describes it for consumers.
"""
import zlib

import numpy as np
from freemocap_stream import encode_message

from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.bot_topics import DepthFrameMessage

DEPTH_KIND = "depth_frame"
CAMERA_ID = "zed_rectified"
HEAD_FRAME = "H: origin midway between the eye-socket centres, x forward, y to the skull's left, z up"
# Optical axes (x image right, y image down, z out of the lens) in H, for a camera that
# looks straight out of the face: right is -y, down is -z, forward is +x.
OPTICAL_TO_HEAD = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def nominal_camera_to_head(rectification: StereoRectification) -> dict:
    """The rectified left camera's pose in the head frame, assuming the ZED sits level with its
    lenses centred in the eye sockets: the left lens at the left socket, half a baseline from
    the origin. The camera bracket's design will replace this with the designed pose."""
    rectified_to_left = np.array(rectification.left_rotation).T  # Rectified frame to the left camera's own
    rotation = OPTICAL_TO_HEAD @ rectified_to_left
    return {
        "rotation": [[round(float(value), 6) for value in row] for row in rotation],
        "translation_m": [0.0, rectification.baseline / 2, 0.0],
        "head_frame": HEAD_FRAME,
        "status": "nominal",
    }


def encode_depth_frame(
    message: DepthFrameMessage,
    *,
    color_jpeg: bytes,
    sequence: int,
    step: int = 2,
    compression_level: int = 1,
    camera_to_head: dict | None = None,
) -> bytes:
    """One `depth_frame` message. `color_jpeg` is the JPEG of `message.color`; `step` keeps
    every step-th depth pixel in each direction, exact values with no averaging across edges."""
    shared = message.rectification
    depth = np.ascontiguousarray(message.depth[::step, ::step], dtype="<u2")
    color_height, color_width = message.color.shape[:2]
    timestamp = message.timestamp if message.timestamp is not None else 0.0
    return encode_message(
        DEPTH_KIND,
        {
            "frame_number": message.frame_number,
            "method": message.method,
            "camera": {
                "id": CAMERA_ID,
                "width": shared.width,
                "height": shared.height,
                "fx": shared.focal_length,
                "fy": shared.focal_length,
                "cx": shared.cx,
                "cy": shared.cy,
                "baseline_m": shared.baseline,
                "axes": "x image right, y image down, z forward",
            },
            "depth": {
                "width": depth.shape[1],
                "height": depth.shape[0],
                "step": step,
                "units": "mm",
                "encoding": "uint16le",
                "invalid": 0,
                "compression": "zlib",
                "data": zlib.compress(depth.tobytes(), compression_level),
            },
            "color": {"encoding": "jpeg", "width": color_width, "height": color_height, "data": color_jpeg},
            "camera_to_head": camera_to_head,
        },
        timestamp=timestamp,
        sequence=sequence,
    )
