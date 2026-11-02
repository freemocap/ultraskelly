"""Human pose detection with YOLOv8 pose on the Hailo-8 AI HAT+, for frames from any camera node."""
import asyncio
import contextlib
import logging

import cv2
import numpy as np
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import DetectorNode, NodeParams
from ultraskelly.core.bot.sensory.pose_common import CocoKeypoint, select_target
from ultraskelly.core.bot.sensory.yolov8_pose import Letterbox, PoseDetections, decode_yolov8_pose
from ultraskelly.core.pubsub.bot_topics import (
    FrameMessage,
    FrameTopic,
    PoseDataMessage,
    PoseDataTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

LETTERBOX_GRAY = 114  # Padding value YOLO models were trained with


class HailoPoseDetectorParams(NodeParams):
    """Parameters for HailoPoseDetectorNode."""

    hef_path: str = Field(
        default="/usr/share/hailo-models/yolov8s_pose_h8.hef",
        description="YOLOv8 pose model compiled for the Hailo-8 (installed by hailo-all)",
    )
    target_keypoint: CocoKeypoint = Field(
        default=CocoKeypoint.NOSE,
        description="Body part to track"
    )
    detection_threshold: float = Field(
        default=0.5,
        ge=0.0,
        le=1.0,
        description="Minimum confidence for person detection"
    )
    keypoint_threshold: float = Field(
        default=0.4,
        ge=0.0,
        le=1.0,
        description="Minimum confidence for keypoint detection"
    )
    iou_threshold: float = Field(
        default=0.65,
        ge=0.0,
        le=1.0,
        description="Box overlap above which two detections count as the same person"
    )
    max_people: int = Field(default=10, ge=1, description="Most people reported per frame")
    frame_timeout_seconds: float = Field(
        default=1.0,
        gt=0.0,
        description="Report no target when no camera frame arrives for this long"
    )


class HailoPoseDetectorNode(DetectorNode):
    """Runs YOLOv8 pose on the Hailo-8 for each new camera frame and tracks one body part."""

    params: HailoPoseDetectorParams
    frame_subscription: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)

    # HailoRT objects. The device and model are entered as context managers so they are torn
    # down in reverse order; releasing the device while the model lives segfaults HailoRT 4.23.
    hailo_resources: SkipValidation[contextlib.ExitStack] = Field(default=None, exclude=True)
    configured_model: SkipValidation[object] = Field(default=None, exclude=True)
    bindings: SkipValidation[object] = Field(default=None, exclude=True)
    input_size: int = Field(default=640, exclude=True)
    input_buffer: SkipValidation[np.ndarray] = Field(default=None, exclude=True)
    output_buffers: SkipValidation[dict[str, np.ndarray]] = Field(default_factory=dict, exclude=True)

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: HailoPoseDetectorParams) -> "HailoPoseDetectorNode":
        """Factory method: loads the model onto the Hailo-8 and subscribes to camera frames."""
        import hailo_platform as hp

        node = cls(pubsub=pubsub, params=params)
        node.frame_subscription = pubsub.get_subscription(FrameTopic)
        node.hailo_resources = contextlib.ExitStack()

        try:
            device_params = hp.VDevice.create_params()
            # The scheduler activates the model on demand and lets other processes share the chip.
            device_params.scheduling_algorithm = hp.HailoSchedulingAlgorithm.ROUND_ROBIN
            vdevice = node.hailo_resources.enter_context(hp.VDevice(device_params))

            model = vdevice.create_infer_model(params.hef_path)
            model.set_batch_size(1)
            model.input().set_format_type(hp.FormatType.UINT8)
            for output in model.outputs:
                output.set_format_type(hp.FormatType.FLOAT32)
            node.configured_model = node.hailo_resources.enter_context(model.configure())

            node.input_size = model.input().shape[1]  # Square input, (height, width, 3)
            node.input_buffer = np.empty(model.input().shape, dtype=np.uint8)
            node.output_buffers = {
                name: np.empty(model.output(name).shape, dtype=np.float32) for name in model.output_names
            }
            node.bindings = node.configured_model.create_bindings()
            node.bindings.input().set_buffer(node.input_buffer)
            for name, buffer in node.output_buffers.items():
                node.bindings.output(name).set_buffer(buffer)
        except BaseException:
            node._release_hailo()
            raise

        return node

    def _infer(self, frame: np.ndarray) -> PoseDetections:
        """Letterbox an RGB frame into the model input, run the Hailo-8, and decode people."""
        height, width = frame.shape[:2]
        letterbox = Letterbox.fit(width=width, height=height, size=self.input_size)
        rgb = frame[..., :3]
        if (letterbox.resized_width, letterbox.resized_height) != (width, height):
            rgb = cv2.resize(
                np.ascontiguousarray(rgb),
                (letterbox.resized_width, letterbox.resized_height),
                interpolation=cv2.INTER_LINEAR,
            )

        self.input_buffer[:] = LETTERBOX_GRAY
        self.input_buffer[
            letterbox.pad_y:letterbox.pad_y + letterbox.resized_height,
            letterbox.pad_x:letterbox.pad_x + letterbox.resized_width,
        ] = rgb
        self.configured_model.run([self.bindings], 1000)

        return decode_yolov8_pose(
            list(self.output_buffers.values()),
            input_size=self.input_size,
            letterbox=letterbox,
            score_threshold=self.params.detection_threshold,
            iou_threshold=self.params.iou_threshold,
            max_people=self.params.max_people,
        )

    def _select_target(self, detections: PoseDetections) -> tuple[int | None, int | None, float | None]:
        return select_target(
            keypoints=detections.keypoints,
            scores=detections.scores,
            target_keypoint=self.params.target_keypoint,
            keypoint_threshold=self.params.keypoint_threshold,
        )

    def detect(self, image: np.ndarray) -> tuple[int | None, int | None, float | None]:
        """Detect poses in one RGB image and return the tracked keypoint and body angle."""
        return self._select_target(self._infer(image))

    async def _publish(self, detections: PoseDetections | None) -> None:
        """Publish the target and full pose data; None or no people means no target."""
        if detections is None or detections.count == 0:
            await self.pubsub.topics[TargetLocationTopic].publish(TargetLocationMessage())
            await self.pubsub.topics[PoseDataTopic].publish(PoseDataMessage())
            return

        x, y, angle = self._select_target(detections)
        await self.pubsub.topics[TargetLocationTopic].publish(TargetLocationMessage(x=x, y=y, angle=angle))
        await self.pubsub.topics[PoseDataTopic].publish(
            PoseDataMessage(
                keypoints=detections.keypoints,
                scores=detections.scores,
                boxes=list(detections.boxes),
            )
        )

    def _release_hailo(self) -> None:
        """Shut down the model, then the device, then drop every reference to them."""
        if self.hailo_resources is not None:
            self.hailo_resources.close()
        self.bindings = None
        self.configured_model = None
        self.hailo_resources = None

    async def run(self) -> None:
        """Main detection loop: one inference per new camera frame."""
        logger.info(
            f"Starting HailoPoseDetectorNode [target={self.params.target_keypoint.name}, "
            f"threshold={self.params.detection_threshold}, model={self.params.hef_path}]"
        )
        inference: asyncio.Task | None = None

        try:
            while not self.stop_event.is_set():
                try:
                    frame_message: FrameMessage = await asyncio.wait_for(
                        self.frame_subscription.get(), timeout=self.params.frame_timeout_seconds
                    )
                except asyncio.TimeoutError:
                    await self._publish(None)  # Camera silent: don't chase a stale target
                    continue

                # Inference blocks for ~15 ms, so run it off the event loop. Shield it so a
                # cancelled run() still waits for the thread before releasing the Hailo-8.
                inference = asyncio.ensure_future(asyncio.to_thread(self._infer, frame_message.frame))
                detections = await asyncio.shield(inference)
                await self._publish(detections)
        finally:
            if inference is not None and not inference.done():
                await asyncio.wait({inference})
            self._release_hailo()
            logger.info("HailoPoseDetectorNode stopped")
