"""Builds the bot from its launch configuration and runs it with whatever hardware works.

Every part of the robot is tried: the OAK camera and its models, the Hailo-8, the ZED Mini
and its IMU, depth, the servo board and the Motor HAT. A part that is missing or fails is
reported with how to fix it (core/bot/health.py, sent to the dashboard as `health`
messages) and the bot runs on without it: motors without cameras, cameras without motors,
no Hailo means no tracking. A missing board's actuators are stand-ins that send nothing
(core/bot/motor/motor_boards.py). Nodes are of two kinds. A camera, detector or depth
node that stops or crashes takes only its part away; it is reported, and the rest run on.
Any other node stopping or crashing ends the bot, which brakes the motors, for its service
to start it again; the stall monitor raises for a stream that was running and went quiet.
"""
import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Callable, TypeVar

from pydantic import BaseModel, Field, SkipValidation, ConfigDict

from ultraskelly.core.bot.diagnostics import Diagnostics, Sources, judge_servo_board
from ultraskelly.core.bot.health import (
    COMPONENTS,
    Health,
    Probes,
    Status,
    diagnose_board,
    diagnose_hailo,
    diagnose_oak,
    diagnose_zed,
    diagnose_zed_imu,
)
from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
from ultraskelly.core.bot.motor.motor_boards import (
    MOTOR_BOARD_ADDRESS,
    SERVO_BOARD_ADDRESS,
    build_motor_nodes,
    open_motor_board,
    open_servo_board,
)
from ultraskelly.core.bot.motor.motor_test import MotorTester, MotorTestLimits
from ultraskelly.core.bot.motor.motor_watchdog import MotorWatchdog
from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams, OrientationNode
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, create_simulated_motor_nodes
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
from ultraskelly.core.bot.sensory.oak_models import missing_models
from ultraskelly.core.bot.sensory.pose_detection_node import (
    PoseDetectorNode,
    PoseDetectorParams,
)
from ultraskelly.core.bot.sensory.face_locator_node import FaceLocatorNode, FaceLocatorParams
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNode, StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZED_MODES, ZedCameraNode, ZedCameraNodeParams
from ultraskelly.core.bot.sensory.zed_imu_node import ZedImuNode, ZedImuNodeParams
from ultraskelly.ui.cv2_ui import UINode, UINodeParams
from ultraskelly.ui.runtime_server import (
    RuntimeServerNode, RuntimeServerParams, TrackingOverlay, encode_jpeg_with_opencv,
)
from ultraskelly.core.pubsub.bot_topics import DepthFrameTopic, FrameTopic, StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC
from ultraskelly.core.pubsub.pubsub_manager import (
    PubSubTopicManager,
)

Part = TypeVar("Part")
Diagnosis = Callable[[BaseException], tuple[Status, str, str | None]]
STREAMING_WINDOW = 3.0  # Seconds without new data before a camera or the IMU counts as not streaming
HEALTH_INTERVAL = 1.0  # Seconds between checks of how each part is doing
DIAGNOSTICS_INTERVAL = 5.0  # Seconds between the live hardware checks (core/bot/diagnostics.py)


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
        default=False,
        description="Also show the OpenCV camera window (`--window`), which needs a desktop session; "
                    "the dashboard works either way.",
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
    zed_imu: ZedImuNodeParams | None = Field(
        default=None,
        description="With zed_camera, also stream its IMU for display and recording (never control); None reads none",
    )
    face_locator: FaceLocatorParams = Field(
        default_factory=FaceLocatorParams,
        description="With ZED depth and pose tracking, where the tracked face is in 3D",
    )
    tracking: bool = Field(
        default=True,
        description="Run the tracking camera, detector and motors; False runs only the ZED and the dashboard",
    )
    stall_monitor: StallMonitorParams = Field(
        default_factory=StallMonitorParams,
        description="Stop the bot, for its service to restart it, when camera frames or pose results stop",
    )


@dataclass
class Hardware:
    """How the launcher opens each part of the robot; each raises if its part is missing or fails.
    Tests pass fakes."""

    oak_camera: Callable[..., Node] = OakCameraNode.create
    hailo_pose: Callable[..., Node] = HailoPoseDetectorNode.create
    zed_camera: Callable[..., Node] = ZedCameraNode.create  # Also takes wait=True: start without the camera
    zed_imu: Callable[..., Node] = ZedImuNode.create  # Also takes wait=True
    stereo_depth: Callable[..., Node] = StereoDepthNode.create
    servo_board: Callable[[], object] = open_servo_board
    motor_board: Callable[[float], object] = open_motor_board
    probes: Probes = field(default_factory=Probes)
    sources: Sources = field(default_factory=Sources)  # Where the live hardware checks look


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
    health: SkipValidation[Health] = Field(default_factory=Health, exclude=True)
    hardware: SkipValidation[Hardware] = Field(default_factory=Hardware, exclude=True)
    # Nodes whose stop or crash takes away only their part of the robot: id(node) -> component
    components_of: dict[int, str] = Field(default_factory=dict, exclude=True)
    diagnoses: SkipValidation[dict[str, Diagnosis]] = Field(default_factory=dict, exclude=True)
    # Called every HEALTH_INTERVAL while the bot runs, to keep each part's status current
    watches: SkipValidation[list[Callable[[], None]]] = Field(default_factory=list, exclude=True)
    diagnostics: SkipValidation[Diagnostics | None] = Field(default=None, exclude=True)
    controllers: SkipValidation[tuple[OrientationNode, MouthNode] | None] = Field(default=None, exclude=True)

    def healthy(self) -> bool:
        """True until the bot starts shutting down; the boot service's watchdog is fed only while it holds."""
        return not self.shutting_down

    def _attempt(self, component: str, open_part: Callable[[], Part], diagnose: Diagnosis, working: str) -> Part | None:
        """Open one part of the robot and report it. A part that is missing or fails is reported
        with how to fix it and None returned, so the bot runs on without it."""
        self.diagnoses[component] = diagnose
        try:
            part = open_part()
        except Exception as error:
            status, message, hint = diagnose(error)
            self.health.report(component, status, message, hint, detail={"error": f"{type(error).__name__}: {error}"})
            logger.error(f"Running without the {COMPONENTS[component][0]}: {message} ({type(error).__name__}: {error})")
            return None
        self.health.report(component, Status.OK, working)
        return part

    def _add_part(self, node: Node, component: str) -> None:
        """A node whose stop or crash is reported against `component` instead of ending the bot."""
        self.nodes.append(node)
        self.components_of[id(node)] = component

    def _create_nodes(self) -> None:
        """Create all nodes from config."""
        self.nodes.clear()
        self.diagnostics = Diagnostics(self.hardware.sources)
        if not self.config.tracking:
            self._create_zed_only_nodes()
            return
        stereo_feeds = self._create_stereo_nodes()

        # Create motor and UI nodes
        # self.nodes.append(HeadNode.create(pubsub=self.pubsub, params=self.config.head))
        # self.nodes.append(WaistNode.create(pubsub=self.pubsub, params=self.config.waist))
        orientation, mouth = self._create_motor_nodes()
        self.controllers = (orientation, mouth)
        self.nodes.extend([orientation, mouth])
        self.motor_tester = MotorTester(
            orientation=orientation, mouth=mouth, limits=self.config.motor_tests, unavailable=self._unavailable,
        )
        self.motor_watchdog = MotorWatchdog({"waist": orientation.waist_motor, "mouth": mouth.mouth_motor})
        orientation.motor_watchdog = self.motor_watchdog
        if self.config.show_window:
            self.nodes.append(UINode.create(pubsub=self.pubsub, params=self.config.ui))

        # Create detector based on type
        oak = None
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
            oak = self._create_tracking_nodes()
        else:
            raise ValueError(f"Unknown detector type: {self.config.detector_type}")

        hailo_pose = self.config.detector_type == DetectorType.HAILO_POSE
        camera_name = "oak" if hailo_pose else "camera"
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
                pose_camera=camera_name if hailo_pose else None,
                motor_tests=self.motor_tester,
                # The OAK's own chip finds faces in the same frames.
                # Only while the OAK runs its models: without them no frame would ever get its results.
                results_camera=camera_name if oak is not None and oak.assembler is not None else None,
                results_for=oak.results_for if oak is not None else None,
                oak_status=oak.status if oak is not None else None,
                health=self.health,
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
                    # The stream's encoder and quality: a frame both keep is encoded once.
                    encode_jpeg=encode_jpeg_with_opencv,
                    stream_jpeg_quality=self.config.runtime_server.jpeg_quality,
                )
            )
        self.nodes.append(
            StallMonitorNode.create(
                pubsub=self.pubsub,
                params=self.config.stall_monitor,
                # Only the Hailo detector reports the frame each pose result belongs to.
                watch_poses=hailo_pose,
                # A stream is watched only while its source works; a missing one is reported, not restarted.
                frames_expected=(lambda: self.health.ok("oak_camera")) if hailo_pose else None,
                poses_expected=(
                    (lambda: self.health.ok("oak_camera") and self.health.ok("hailo")) if hailo_pose else None
                ),
            )
        )

        logger.info(f"Created {len(self.nodes)} nodes")

    def _create_motor_nodes(self) -> tuple[OrientationNode, MouthNode]:
        """Both controllers, on the boards that answer; simulated with --simulate-motors."""
        self.command_recorder = CommandRecorder()
        config = self.config
        if config.simulate_motors:
            logger.info("Simulating motors: commands are recorded, the motor boards are not touched")
            for board in ("servo_board", "motor_board"):
                self.health.report(
                    board, Status.OFF, "Simulated (--simulate-motors): commands are recorded, not sent",
                    "Leave out --simulate-motors (for the service, in /etc/default/ultraskelly-bot) "
                    "to drive the boards.",
                )
            return create_simulated_motor_nodes(
                pubsub=self.pubsub,
                orientation_params=config.orientation,
                mouth_params=config.mouth,
                recorder=self.command_recorder,
                limits=config.limits,
            )
        probes = self.hardware.probes
        servo_kit = self._attempt(
            "servo_board", self.hardware.servo_board,
            lambda error: diagnose_board("servo board", SERVO_BOARD_ADDRESS, error, probes,
                                         "Until then the head's pan and tilt are recorded, not sent."),
            "Answers; the head's pan and tilt are sent to it",
        )
        motor_kit = self._attempt(
            "motor_board", lambda: self.hardware.motor_board(config.motor_pwm_hz),
            lambda error: diagnose_board("Motor HAT", MOTOR_BOARD_ADDRESS, error, probes,
                                         "Until then the waist and jaw are recorded, not sent."),
            "Answers; the waist and jaw are sent to it",
        )
        return build_motor_nodes(
            pubsub=self.pubsub,
            orientation_params=config.orientation,
            mouth_params=config.mouth,
            recorder=self.command_recorder,
            limits=config.limits,
            servo_kit=servo_kit,
            motor_kit=motor_kit,
        )

    def _unavailable(self, actuator: str) -> str | None:
        """Why a motor test on `actuator` cannot reach a board now, or None."""
        component = self.health.get("servo_board" if actuator in ("pan", "tilt") else "motor_board")
        if component is None or component.status not in (Status.MISSING, Status.ERROR):
            return None
        return f"{component.message}, so nothing would move. {component.hint}"

    def _create_tracking_nodes(self) -> OakCameraNode | None:
        """The OAK camera and the Hailo-8 pose detector, each if it starts; returns the OAK's node."""
        probes = self.hardware.probes
        oak = self._attempt(
            "oak_camera", lambda: self.hardware.oak_camera(pubsub=self.pubsub, params=self.config.oak_camera),
            lambda error: diagnose_oak(error, probes), "Streaming",
        )
        if oak is not None:
            self._add_part(oak, "oak_camera")
            self.watches.append(self._watch_oak(oak))
        self._report_oak_models(oak)
        hailo = self._attempt(
            "hailo", lambda: self.hardware.hailo_pose(pubsub=self.pubsub, params=self.config.hailo_pose_detector),
            lambda error: diagnose_hailo(error, probes), "Finds people in the OAK's frames",
        )
        if hailo is not None:
            self._add_part(hailo, "hailo")
        return oak

    def _report_oak_models(self, oak: OakCameraNode | None) -> None:
        models = self.config.oak_camera.models
        if models is None:
            self.health.report("oak_models", Status.OFF, "Turned off (--no-oak-models)")
            return
        fetch = ("Run .venv/bin/python -m scripts.fetch_oak_models with the internet on (scripts/setup-pi.sh does "
                 "it too), then restart the bot.")
        if oak is not None:
            if oak.models_error:
                self.health.report("oak_models", Status.MISSING, oak.models_error, fetch)
            else:
                self.health.report("oak_models", Status.OK, "Running on the OAK's chip")
            return
        missing = missing_models(models.model_cache)
        if missing:
            self.health.report("oak_models", Status.MISSING, f"Not downloaded: {', '.join(missing)}", fetch)
        else:
            self.health.report("oak_models", Status.OK, "All downloaded; they run once the OAK is plugged in")

    def _watch_oak(self, oak: OakCameraNode) -> Callable[[], None]:
        """While the OAK node runs: streaming, or started but sending no frames (reported, not restarted)."""
        started = time.monotonic()
        quiet_after = self.config.stall_monitor.startup_seconds

        def watch() -> None:
            current = self.health.get("oak_camera")
            if current is None or current.status not in (Status.OK, Status.DEGRADED):
                return  # Stopped or crashed: reported when its node ended
            if oak.status()["rates"].get("frames", 0.0) > 0:
                self.health.report("oak_camera", Status.OK, "Streaming")
            elif time.monotonic() - started > quiet_after:
                self.health.report("oak_camera", Status.DEGRADED, "Started, but sends no frames",
                                   "Unplug the OAK, wait 5 s and plug it back in; if it stays quiet, restart the bot.")
        return watch

    def _create_stereo_nodes(self) -> dict[str, dict[str, type[PubSubTopicABC]]]:
        """The ZED Mini's nodes, when configured: capture, its IMU, and depth if asked for. The camera and
        the IMU start even when missing, and pick them up once they appear. Returns the dashboard's
        stereo and depth camera feeds."""
        zed = self.config.zed_camera
        if zed is None:
            return {}
        probes = self.hardware.probes
        camera = self._attempt(
            "zed_camera", lambda: self.hardware.zed_camera(pubsub=self.pubsub, params=zed),
            lambda error: diagnose_zed(error, probes), "Streaming both eyes",
        )
        if camera is None:  # It tries once a second until the camera streams
            camera = self.hardware.zed_camera(pubsub=self.pubsub, params=zed, wait=True)
        self._add_part(camera, "zed_camera")
        self.watches.append(self._watch_streaming(
            "zed_camera", lambda: camera.frames_published if camera.source is not None else None,
            "Streaming both eyes", lambda: diagnose_zed(RuntimeError("it does not stream"), probes),
        ))
        if self.config.zed_imu is not None:
            imu_params = self.config.zed_imu
            imu = self._attempt(
                "zed_imu", lambda: self.hardware.zed_imu(pubsub=self.pubsub, params=imu_params),
                lambda error: diagnose_zed_imu(error, probes), "Reading the accelerometer and gyroscope",
            )
            if imu is None:  # Its reader tries again every few seconds
                imu = self.hardware.zed_imu(pubsub=self.pubsub, params=imu_params, wait=True)
            self._add_part(imu, "zed_imu")
            self.watches.append(self._watch_streaming(
                "zed_imu", lambda: imu.samples, "Reading the accelerometer and gyroscope",
                lambda: diagnose_zed_imu(OSError("no samples arrive"), probes),
            ))
        feeds = {"stereo_cameras": {"zed": StereoFrameTopic}}
        if self.config.stereo_depth is not None:
            depth_params = self.config.stereo_depth
            width, height = ZED_MODES[zed.resolution][0]
            depth = self._attempt(
                "depth", lambda: self.hardware.stereo_depth(
                    pubsub=self.pubsub, params=depth_params, eye_size=(width // 2, height)),
                lambda error: self._diagnose_depth(error), f"Depth from the ZED's eyes ({depth_params.matcher})",
            )
            if depth is not None:
                self._add_part(depth, "depth")
                feeds["depth_cameras"] = {"zed": DepthFrameTopic}
                self.watches.append(self._watch_depth(depth_params.matcher))
            if self.config.tracking:
                # It uses only pose results that carry the OAK frame they came from (its size and time),
                # so any detector of OAK images feeds it and the others are ignored.
                self.nodes.append(FaceLocatorNode.create(pubsub=self.pubsub, params=self.config.face_locator))
        return feeds

    def _watch_streaming(
        self, component: str, count: Callable[[], int | None], working: str,
        diagnose: Callable[[], tuple[Status, str, str | None]],
    ) -> Callable[[], None]:
        """A part whose node picks it up by itself: working while `count` (None: not open) keeps
        rising, else diagnosed from what is plugged in. For its first STREAMING_WINDOW the status
        from startup stands, while the node opens it."""
        state = {"count": count(), "rose_at": None, "first": None}

        def watch() -> None:
            now, latest = time.monotonic(), count()
            state["first"] = state["first"] or now
            if latest is not None and state["count"] is not None and latest > state["count"]:
                state["rose_at"] = now
            state["count"] = latest
            if state["rose_at"] is not None and now - state["rose_at"] <= STREAMING_WINDOW:
                self.health.report(component, Status.OK, working)
            elif now - state["first"] > STREAMING_WINDOW:
                self.health.report(component, *diagnose())
        return watch

    def _diagnose_depth(self, error: BaseException) -> tuple[Status, str, str | None]:
        if self.config.stereo_depth.matcher == "stereonet" and not self.hardware.probes.hailo_device():
            return (Status.MISSING, "StereoNet runs on the Hailo-8, which is missing",
                    "Seat the AI HAT+ (see Hailo-8), or run with --depth, which computes depth on the CPU.")
        return (Status.ERROR, f"Depth did not start: {error}", None)

    def _watch_depth(self, matcher: str) -> Callable[[], None]:
        def watch() -> None:
            if self.health.ok("zed_camera"):
                self.health.report("depth", Status.OK, f"Depth from the ZED's eyes ({matcher})")
            else:
                self.health.report("depth", Status.DEGRADED, "Waiting for the ZED Mini's images",
                                   "Depth needs the ZED Mini; see its line.")
        return watch

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
                health=self.health,
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
    def from_config(cls, config: LaunchConfig, hardware: Hardware | None = None) -> "BotLauncher":
        """Instantiate launcher and nodes based on config, on `hardware` (the robot's when None)."""
        logger.info("Creating nodes from launch config...")

        launcher = cls(config=config, pubsub=PubSubTopicManager.create(), hardware=hardware or Hardware())
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

    def _report_stopped(self, component: str, task: asyncio.Task) -> None:
        """A camera, detector or depth node ended: report why, against its part, and run on."""
        if task.cancelled():
            return
        error = task.exception() or RuntimeError("it stopped")
        diagnose = self.diagnoses.get(component, lambda error: (Status.ERROR, str(error), None))
        status, message, hint = diagnose(error)
        self.health.report(component, status, message, hint, detail={"error": f"{type(error).__name__}: {error}"})
        logger.error(f"The {COMPONENTS[component][0]} stopped; running on without it: {message}")

    async def _watch_health(self) -> None:
        """Keep each part's status current while the bot runs."""
        while True:
            for watch in self.watches:
                watch()
            await asyncio.sleep(HEALTH_INTERVAL)

    async def _watch_diagnostics(self) -> None:
        """The live hardware checks, every DIAGNOSTICS_INTERVAL, into the health messages. Best effort:
        a check that fails is logged and skipped, never a reason to stop the bot."""
        failures = 0
        while True:
            try:
                await self._diagnose()
            except Exception:
                failures += 1
                if failures == 1 or failures % 100 == 0:
                    logger.exception(f"A hardware check failed ({failures} so far); the others go on")
            await asyncio.sleep(DIAGNOSTICS_INTERVAL)

    async def _diagnose(self) -> None:
        driven = self.controllers is not None and self.health.ok("servo_board")  # The real board, in use
        params = self.config.orientation
        head = {"pan": params.pan_channel, "tilt": params.tilt_channel} if driven else {}
        latest = self.command_recorder.latest if self.command_recorder is not None else {}
        commanded = {name: latest[name].value for name in head if name in latest}
        result = await asyncio.to_thread(self.diagnostics.collect, head_channels=head, commanded=commanded)
        watchdog = self.motor_watchdog
        result["motor_watchdog"] = None if watchdog is None else {"trips": watchdog.trips,
                                                                  "tripped_at": watchdog.tripped_at}
        if self.controllers is not None:
            orientation, mouth = self.controllers
            result["retried_writes"] = {
                name: getattr(motor, "failed_writes", None)
                for name, motor in (("waist", orientation.waist_motor), ("mouth", mouth.mouth_motor))
            }
        self.health.set_diagnostics(result)
        if driven and (judged := judge_servo_board(result["pca9685"])) is not None:
            self.health.report("servo_board", *judged)

    async def run(self) -> None:
        """Run all nodes until one that is not a camera, detector or depth node stops or crashes,
        then stop the rest. Those three report their end against their part and the rest run on.

        A crashed node's exception is re-raised so failures surface instead of being hidden.
        """
        logger.info("=" * 60)
        logger.info("LAUNCHING TARGET TRACKER WITH SOPHISTICATED PUBSUB")
        logger.info("=" * 60)

        watcher = checks = None
        try:
            await self._start_nodes()
            watcher = asyncio.create_task(self._watch_health())
            checks = asyncio.create_task(self._watch_diagnostics()) if self.diagnostics is not None else None
            pending = {*self.tasks, watcher, *([checks] if checks is not None else [])}
            while True:
                done, pending = await asyncio.wait(pending, return_when=asyncio.FIRST_COMPLETED)
                core_ended = False
                for task in done:
                    if task in (watcher, checks):
                        task.result()  # Re-raises; they never return
                        core_ended = True
                        continue
                    component = self.components_of.get(id(self.nodes[self.tasks.index(task)]))
                    if component is None:
                        task.result()  # Re-raises if the node crashed
                        core_ended = True
                    else:
                        self._report_stopped(component, task)
                if core_ended:
                    logger.info("A node stopped; stopping the others...")
                    break
        finally:
            for task in (watcher, checks):
                if task is not None:
                    task.cancel()
                    await asyncio.gather(task, return_exceptions=True)
            await self._shutdown_nodes()
            if self.diagnostics is not None:
                self.diagnostics.close()
            self.pubsub.close()
            logger.info("All nodes stopped and pubsub closed")
