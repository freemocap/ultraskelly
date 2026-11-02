import asyncio
import logging

from pydantic import BaseModel, Field, SkipValidation, ConfigDict

from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams, OrientationNode
from ultraskelly.core.bot.motor.waist_node import WaistNode, WaistNodeParams

logger = logging.getLogger(__name__)
from ultraskelly.core.bot.base_abcs import DetectorType, Node, NodeParams
from ultraskelly.core.bot.motor.head_node import HeadNode, HeadNodeParams
from ultraskelly.core.bot.sensory.bright_point_detection_node import (
    BrightnessDetectorNode,
    BrightnessDetectorParams,
)
from ultraskelly.core.bot.sensory.camera_node import VisionNode, VisionNodeParams
from ultraskelly.core.bot.sensory.hailo_pose_node import (
    HailoPoseDetectorNode,
    HailoPoseDetectorParams,
)
from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
from ultraskelly.core.bot.sensory.pose_detection_node import (
    PoseDetectorNode,
    PoseDetectorParams,
)
from ultraskelly.ui.cv2_ui import UINode, UINodeParams
from ultraskelly.core.pubsub.pubsub_manager import (
    PubSubTopicManager,
)


class LaunchConfig(NodeParams):
    """Declarative launch configuration."""

    detector_type: DetectorType = Field(
        default=DetectorType.POSE, description="Which detector to use"
    )
    vision: VisionNodeParams = Field(default_factory=VisionNodeParams)
    brightness_detector: BrightnessDetectorParams = Field(
        default_factory=BrightnessDetectorParams
    )
    pose_detector: PoseDetectorParams = Field(default_factory=PoseDetectorParams)
    oak_camera: OakCameraNodeParams = Field(default_factory=OakCameraNodeParams)
    hailo_pose_detector: HailoPoseDetectorParams = Field(default_factory=HailoPoseDetectorParams)
    # head: HeadNodeParams = Field(default_factory=HeadNodeParams)
    mouth: MouthNodeParams = Field(default_factory=MouthNodeParams)
    # waist: WaistNodeParams = Field(default_factory=WaistNodeParams)
    orientation: OrientationNodeParams = Field(default_factory=OrientationNodeParams)
    ui: UINodeParams = Field(default_factory=UINodeParams)


class BotLauncher(BaseModel):
    """Main launcher for the bot system."""
    model_config = ConfigDict(arbitrary_types_allowed=True)
    config: LaunchConfig
    pubsub: SkipValidation[PubSubTopicManager]
    nodes: list[Node] = Field(default_factory=list)
    tasks: list[asyncio.Task] = Field(default_factory=list, exclude=True)

    def _create_nodes(self) -> None:
        """Create all nodes from config."""
        self.nodes.clear()
        
        # Create motor and UI nodes
        # self.nodes.append(HeadNode.create(pubsub=self.pubsub, params=self.config.head))
        # self.nodes.append(WaistNode.create(pubsub=self.pubsub, params=self.config.waist))
        self.nodes.append(OrientationNode.create(pubsub=self.pubsub, params=self.config.orientation))
        self.nodes.append(MouthNode.create(pubsub=self.pubsub, params=self.config.mouth))
        self.nodes.append(UINode.create(pubsub=self.pubsub, params=self.config.ui))

        # Create detector based on type
        if self.config.detector_type == DetectorType.BRIGHTNESS:
            self.nodes.append(
                VisionNode.create(pubsub=self.pubsub, params=self.config.vision)
            )
            self.nodes.append(
                BrightnessDetectorNode.create(
                    pubsub=self.pubsub, params=self.config.brightness_detector
                )
            )
        elif self.config.detector_type == DetectorType.POSE:
            self.nodes.append(
                PoseDetectorNode.create(pubsub=self.pubsub, params=self.config.pose_detector)
            )
        elif self.config.detector_type == DetectorType.HAILO_POSE:
            self.nodes.append(
                OakCameraNode.create(pubsub=self.pubsub, params=self.config.oak_camera)
            )
            self.nodes.append(
                HailoPoseDetectorNode.create(
                    pubsub=self.pubsub, params=self.config.hailo_pose_detector
                )
            )
        else:
            raise ValueError(f"Unknown detector type: {self.config.detector_type}")

        logger.info(f"Created {len(self.nodes)} nodes")

    @classmethod
    def from_config(cls, config: LaunchConfig) -> "BotLauncher":
        """Instantiate launcher and nodes based on config."""
        logger.info("Creating nodes from launch config...")

        launcher = cls(config=config, pubsub=PubSubTopicManager.create())
        launcher._create_nodes()
        
        return launcher

    async def _shutdown_nodes(self) -> None:
        """Stop all nodes and clean up tasks."""
        logger.info("Shutting down nodes...")
        
        # Stop all nodes
        for node in self.nodes:
            node.stop()

        # Cancel all tasks
        for task in self.tasks:
            task.cancel()

        # Wait for tasks to finish cancelling
        await asyncio.gather(*self.tasks, return_exceptions=True)
        
        # Clear task list
        self.tasks.clear()
        
        # Explicitly close camera resources for PoseDetectorNode
        for node in self.nodes:
            if hasattr(node, 'picam2') and node.picam2 is not None:
                logger.info(f"Closing camera for {node.__class__.__name__}")
                node.picam2.close()
                node.picam2 = None
        
        # Clear nodes to release resources
        self.nodes.clear()

        logger.info("All nodes stopped and resources released")

    async def _start_nodes(self) -> None:
        """Start all nodes as async tasks."""
        logger.info("Starting nodes...")
        
        for node in self.nodes:
            task = asyncio.create_task(node.run())
            self.tasks.append(task)
        
        logger.info(f"Started {len(self.tasks)} node tasks")

    async def run(self) -> None:
        """Run all nodes until any one stops or crashes, then stop the rest.

        A crashed node's exception is re-raised so failures surface instead of being hidden.
        """
        logger.info("=" * 60)
        logger.info("LAUNCHING TARGET TRACKER WITH SOPHISTICATED PUBSUB")
        logger.info("=" * 60)

        try:
            await self._start_nodes()
            done, _ = await asyncio.wait(self.tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()  # Re-raises if the node crashed
            logger.info("A node stopped; stopping the others...")
        finally:
            await self._shutdown_nodes()
            self.pubsub.close()
            logger.info("All nodes stopped and pubsub closed")
