import asyncio
import logging
from dataclasses import dataclass

import numpy as np
from pydantic import Field, SkipValidation
from picamera2 import CompletedRequest, Picamera2
from picamera2.devices.imx500 import IMX500, NetworkIntrinsics
from picamera2.devices.imx500.postprocess_highernet import postprocess_higherhrnet
from libcamera import Transform  # Import Transform for camera flipping

from ultraskelly import FAIL_ON_IMPORTS
from ultraskelly.core.pubsub.bot_topics import PoseDataMessage, TargetLocationTopic, PoseDataTopic, FrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.core.bot.base_abcs import DetectorNode, NodeParams
from ultraskelly.core.bot.motor.head_node import TargetLocationMessage
from ultraskelly.core.bot.sensory.camera_node import FrameMessage
from ultraskelly.core.bot.sensory.pose_common import (  # re-exported for existing importers
    SKELETON_CONNECTIONS,
    CocoKeypoint,
    select_target,
)

logger = logging.getLogger(__name__)


class PoseDetectorParams(NodeParams):
    """Parameters for PoseDetectorNode."""
    model_path: str = Field(
        default="/usr/share/imx500-models/imx500_network_higherhrnet_coco.rpk",
        description="Path to IMX500 pose estimation model",
    )
    target_keypoint: CocoKeypoint = Field(
        default=CocoKeypoint.NOSE,
        description="Body part to track"
    )
    detection_threshold: float = Field(
        default=0.3,
        ge=0.0,
        le=1.0,
        description="Minimum confidence for person detection"
    )
    keypoint_threshold: float = Field(
        default=0.3,
        ge=0.0,
        le=1.0,
        description="Minimum confidence for keypoint detection"
    )
    inference_rate: int = Field(
        default=10,
        ge=1,
        le=30,
        description="Frames per second for inference"
    )
    width: int = Field(default=640, ge=160, le=1920)
    height: int = Field(default=480, ge=120, le=1080)
    flip_camera: bool = Field(
        default=True,  # Set to True for upside-down mounted camera
        description="Rotate camera image 180 degrees (for upside-down mounting)"
    )


class PoseDetectorNode(DetectorNode):
    """Detects human poses using IMX500 and tracks specific body part."""
    params: PoseDetectorParams
    imx500: SkipValidation[IMX500] = Field(default=None, exclude=True)
    picam2: SkipValidation[Picamera2] = Field(default=None, exclude=True)

    # Latest detection results (shared between callback and coroutine)
    latest_keypoints: np.ndarray | None = Field(default=None, exclude=True)
    latest_scores: np.ndarray | None = Field(default=None, exclude=True)
    latest_boxes: list[np.ndarray] | None = Field(default=None, exclude=True)

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: PoseDetectorParams) -> "PoseDetectorNode":
        """Factory method to create and initialize PoseDetectorNode."""
        node = cls(pubsub=pubsub, params=params)

        # Initialize IMX500 before Picamera2
        node.imx500 = IMX500(params.model_path)
        intrinsics = node.imx500.network_intrinsics

        if not intrinsics:
            intrinsics = NetworkIntrinsics()
            intrinsics.task = "pose estimation"
        elif intrinsics.task != "pose estimation":
            raise ValueError(f"Model is not a pose estimation task: {intrinsics.task}")

        intrinsics.inference_rate = params.inference_rate
        intrinsics.update_with_defaults()

        # Initialize camera
        node.picam2 = Picamera2(node.imx500.camera_num)

        # Create transform for 180-degree rotation if camera is upside down
        transform = Transform(hflip=params.flip_camera, vflip=params.flip_camera)

        # Create configuration using Picamera2's create_preview_configuration
        # This applies the transform to ALL streams including the lores stream used for inference
        config = node.picam2.create_preview_configuration(
            main={"size": (params.width, params.height)},
            lores={"size": (params.width, params.height)},  # Ensure lores matches for inference
            controls={"FrameRate": params.inference_rate},
            buffer_count=12,
            transform=transform  # This applies to both main and lores streams
        )

        node.picam2.configure(config)
        node.picam2.pre_callback = node._pose_callback

        return node

    def _pose_callback(self, request: CompletedRequest) -> None:
        """Callback to process pose detection results (runs in picamera2 thread).

        Since the image is now flipped BEFORE inference, the keypoints are already
        in the correct coordinate space and don't need transformation.
        """
        metadata = request.get_metadata()
        np_outputs = self.imx500.get_outputs(metadata=metadata, add_batch=True)

        if np_outputs is None:
            return

        keypoints, scores, boxes = postprocess_higherhrnet(
            outputs=np_outputs,
            img_size=(self.params.height, self.params.width),
            img_w_pad=(0, 0),
            img_h_pad=(0, 0),
            detection_threshold=self.params.detection_threshold,
            network_postprocess=True,
        )

        # Store results (thread-safe update)
        if scores is not None and len(scores) > 0:
            # Reshape keypoints to (num_people, 17, 3) where 3 = [x, y, confidence]
            self.latest_keypoints = np.reshape(
                np.stack(keypoints, axis=0), (len(scores), 17, 3)
            )
            # No coordinate transformation needed - inference was done on flipped image
            self.latest_boxes = [np.array(b) for b in boxes]
            self.latest_scores = np.array(scores)
        else:
            self.latest_keypoints = None
            self.latest_scores = None
            self.latest_boxes = None

    def detect(self) -> tuple[int | None, int | None, float | None]:
        """Extract target keypoint from latest detection results."""
        return select_target(
            keypoints=self.latest_keypoints,
            scores=self.latest_scores,
            target_keypoint=self.params.target_keypoint,
            keypoint_threshold=self.params.keypoint_threshold,
        )

    async def run(self) -> None:
        """Main detection loop."""
        logger.info(
            f"Starting PoseDetectorNode [target={self.params.target_keypoint.name}, "
            f"threshold={self.params.detection_threshold}, "
            f"flip_camera={self.params.flip_camera}]"
        )

        self.imx500.show_network_fw_progress_bar()
        self.picam2.start(show_preview=False)
        self.imx500.set_auto_aspect_ratio()

        await asyncio.sleep(1)

        try:
            while not self.stop_event.is_set():
                # Extract target from latest detection - using abstract detect method
                x, y, angle = self.detect()  # Frame comes from callback

                await self.pubsub.topics[TargetLocationTopic].publish(
                    TargetLocationMessage(x=x, y=y, angle=angle)
                )

                # Publish full pose data for visualization
                await self.pubsub.topics[PoseDataTopic].publish(
                    PoseDataMessage(
                        keypoints=self.latest_keypoints,
                        scores=self.latest_scores,
                        boxes=self.latest_boxes,
                    )
                )

                # Publish frames for UI
                # The frame is already flipped by the hardware transform
                frame = self.picam2.capture_array()
                await self.pubsub.topics[FrameTopic].publish(FrameMessage(frame=frame))

                await asyncio.sleep(0.01)  # 100 Hz publishing rate
        finally:
            self.picam2.stop()
            logger.info("PoseDetectorNode stopped")