import asyncio
import logging

from pydantic import BaseModel, Field, SkipValidation, ConfigDict

from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
from ultraskelly.core.bot.motor.motor_test import MotorTester, MotorTestLimits
from ultraskelly.core.bot.motor.motor_watchdog import MotorWatchdog
from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams, OrientationNode
from ultraskelly.core.bot.motor.simulated_actuators import (
    CommandRecorder,
    create_simulated_motor_nodes,
    record_hardware_commands,
)
from ultraskelly.core.bot.recording.run_recorder_node import RunRecorderNode, RunRecorderParams
from ultraskelly.core.bot.stall_monitor_node import StallMonitorNode, StallMonitorParams

logger = logging.getLogger(__name__)
from ultraskelly.core.bot.base_abcs import DetectorType, Node, NodeParams
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
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNode, StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZED_MODES, ZedCameraNode, ZedCameraNodeParams
from ultraskelly.ui.cv2_ui import UINode, UINodeParams
from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams, TrackingOverlay
from ultraskelly.core.pubsub.bot_topics import DepthFrameTopic, FrameTopic, StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC
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
    runtime_server: RuntimeServerParams = Field(default_factory=RuntimeServerParams)
    simulate_motors: bool = Field(
        default=False,
        description="Drive memory-only servo and motor stand-ins instead of the motor boards",
    )
    show_window: bool = Field(
        default=True,
        description="Show the OpenCV camera window; needs a desktop session. The dashboard works either way.",
    )
    limits: ActuatorLimits = Field(
        default_factory=ActuatorLimits,
        description="Hard limits every servo angle and motor throttle is clamped to, simulated or real",
    )
    motor_tests: MotorTestLimits = Field(
        default_factory=MotorTestLimits,
        description="What the dashboard's motor tests may do, within the hard limits",
    )
    motor_pwm_hz: float = Field(
        default=1600.0, ge=24.0, le=1600.0,
        description="PWM frequency of the DC motor board, one clock for the waist and jaw; 1600 asks for its highest, about 1.5 kHz",
    )
    recording: RunRecorderParams | None = Field(
        default=None,
        description="Save the run's camera video, detections and motor commands to disk; None saves nothing",
    )
    zed_camera: ZedCameraNodeParams | None = Field(
        default=None,
        description="Also capture the ZED Mini stereo camera and stream its two eyes; None runs without it",
    )
    stereo_depth: StereoDepthNodeParams | None = Field(
        default=None,
        description="With zed_camera, also compute depth from its pair; None computes none",
    )
    tracking: bool = Field(
        default=True,
        description="Run the tracking camera, detector and motors; False runs only the ZED and the dashboard",
    )
    stall_monitor: StallMonitorParams = Field(
        default_factory=StallMonitorParams,
        description="Stop the bot, for its service to restart it, when camera frames or pose results stop",
    )


class BotLauncher(BaseModel):
    """Main launcher for the bot system."""
    model_config = ConfigDict(arbitrary_types_allowed=True)
    config: LaunchConfig
    pubsub: SkipValidation[PubSubTopicManager]
    nodes: list[Node] = Field(default_factory=list)
    tasks: list[asyncio.Task] = Field(default_factory=list, exclude=True)
    # Commands sent to the motors, simulated or real; None until nodes are created.
    command_recorder: SkipValidation[CommandRecorder | None] = Field(default=None, exclude=True)
    motor_tester: SkipValidation[MotorTester | None] = Field(default=None, exclude=True)
    # Brakes the DC motors from its own thread if the control loop stalls; None until nodes are created.
    motor_watchdog: SkipValidation[MotorWatchdog | None] = Field(default=None, exclude=True)
    shutting_down: bool = Field(default=False, exclude=True)

    def healthy(self) -> bool:
        """True until the bot starts shutting down; the boot service's watchdog is fed only while it holds."""
        return not self.shutting_down

    def _create_nodes(self) -> None:
        """Create all nodes from config."""
        self.nodes.clear()
        if not self.config.tracking:
            self._create_zed_only_nodes()
            return
        # The ZED first: an optional camera that fails to start stops the launch before
        # the motors, the OAK or the Hailo are opened.
        stereo_feeds = self._create_stereo_nodes()

        # Create motor and UI nodes
        # self.nodes.append(HeadNode.create(pubsub=self.pubsub, params=self.config.head))
        # self.nodes.append(WaistNode.create(pubsub=self.pubsub, params=self.config.waist))
        self.command_recorder = CommandRecorder()
        if self.config.simulate_motors:
            logger.info("Simulating motors: commands are recorded, the motor boards are not touched")
            orientation, mouth = create_simulated_motor_nodes(
                pubsub=self.pubsub,
                orientation_params=self.config.orientation,
                mouth_params=self.config.mouth,
                recorder=self.command_recorder,
                limits=self.config.limits,
            )
            self.nodes.extend([orientation, mouth])
        else:
            pwm = self.config.motor_pwm_hz
            orientation = OrientationNode.create(pubsub=self.pubsub, params=self.config.orientation, pwm_frequency=pwm)
            mouth = MouthNode.create(pubsub=self.pubsub, params=self.config.mouth, pwm_frequency=pwm)
            record_hardware_commands(
                orientation=orientation, mouth=mouth, recorder=self.command_recorder, limits=self.config.limits,
            )
            if self.config.mouth.startup_test:
                mouth.run_startup_test()  # After wrapping, so the limits apply
            self.nodes.extend([orientation, mouth])
        self.motor_tester = MotorTester(orientation=orientation, mouth=mouth, limits=self.config.motor_tests)
        self.motor_watchdog = MotorWatchdog({"waist": orientation.waist_motor, "mouth": mouth.mouth_motor})
        orientation.motor_watchdog = self.motor_watchdog
        if self.config.show_window:
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

        camera_name = "oak" if self.config.detector_type == DetectorType.HAILO_POSE else "camera"
        aim = self.config.orientation
        self.nodes.append(
            RuntimeServerNode.create(
                pubsub=self.pubsub,
                params=self.config.runtime_server,
                cameras={camera_name: FrameTopic},
                **stereo_feeds,
                recorder=self.command_recorder,
                motors_simulated=self.config.simulate_motors,
                node_status=self.node_status,
                overlays={
                    camera_name: TrackingOverlay(aim=(aim.target_x, aim.target_y), jaw_zone=aim.head_deadzone)
                },
                # The Hailo detector reports which OAK frame each pose came from.
                pose_camera=camera_name if self.config.detector_type == DetectorType.HAILO_POSE else None,
                motor_tests=self.motor_tester,
            )
        )
        if self.config.recording is not None:
            self.nodes.append(
                RunRecorderNode.create(
                    pubsub=self.pubsub,
                    params=self.config.recording,
                    camera=camera_name,
                    frame_topic=FrameTopic,
                    commands=self.command_recorder,
                    configuration=self.config.model_dump(mode="json"),
                )
            )
        self.nodes.append(
            StallMonitorNode.create(
                pubsub=self.pubsub,
                params=self.config.stall_monitor,
                # Only the Hailo detector reports the frame each pose result belongs to.
                watch_poses=self.config.detector_type == DetectorType.HAILO_POSE,
            )
        )

        logger.info(f"Created {len(self.nodes)} nodes")

    def _create_stereo_nodes(self) -> dict[str, dict[str, type[PubSubTopicABC]]]:
        """The ZED Mini's nodes, when configured: capture, and depth if asked for.
        Returns the dashboard's stereo and depth camera feeds."""
        zed = self.config.zed_camera
        if zed is None:
            return {}
        self.nodes.append(ZedCameraNode.create(pubsub=self.pubsub, params=zed))
        feeds = {"stereo_cameras": {"zed": StereoFrameTopic}}
        if self.config.stereo_depth is not None:
            width, height = ZED_MODES[zed.resolution][0]
            self.nodes.append(StereoDepthNode.create(
                pubsub=self.pubsub, params=self.config.stereo_depth, eye_size=(width // 2, height),
            ))
            feeds["depth_cameras"] = {"zed": DepthFrameTopic}
        return feeds

    def _create_zed_only_nodes(self) -> None:
        """Only the ZED Mini and the dashboard: no OAK, Hailo or motors, so no robot lock is needed."""
        self.command_recorder = None
        stereo_feeds = self._create_stereo_nodes()
        self.nodes.append(
            RuntimeServerNode.create(
                pubsub=self.pubsub,
                params=self.config.runtime_server,
                cameras={},
                **stereo_feeds,
                recorder=None,
                motors_simulated=True,
                node_status=self.node_status,
            )
        )
        logger.info(f"Created {len(self.nodes)} nodes without tracking or motors")

    def node_status(self) -> list[dict]:
        """Each node's name and state: starting, running, stopped, or crashed with its error."""
        status = []
        for index, node in enumerate(self.nodes):
            entry = {"name": type(node).__name__, "state": "starting"}
            if index < len(self.tasks):
                task = self.tasks[index]
                if not task.done():
                    entry["state"] = "running"
                elif not task.cancelled() and task.exception() is not None:
                    entry.update(state="crashed", error=repr(task.exception()))
                else:
                    entry["state"] = "stopped"
            status.append(entry)
        return status

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
        self.shutting_down = True

        # Stop all nodes
        for node in self.nodes:
            node.stop()

        # Cancel all tasks
        for task in self.tasks:
            task.cancel()

        # Wait for tasks to finish cancelling
        await asyncio.gather(*self.tasks, return_exceptions=True)
        if self.motor_watchdog is not None:
            self.motor_watchdog.stop()
        
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
        if self.motor_watchdog is not None:
            self.motor_watchdog.start()  # Armed by the control loop's first tick

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
