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

Every few seconds the bot brings back each part whose node is not running once it can
start: the OAK once it shows on USB, the Hailo-8 once /dev/hailo0 is there, depth once the
ZED Mini streams, and a camera or IMU node that crashed. The motor boards are set up, and the
head centred, only when the bot starts, so a board that answers later is reported with a
restart to use it.
"""
import asyncio
import logging
import time
from dataclasses import dataclass, field, replace
from typing import Callable, TypeVar

from pydantic import BaseModel, Field, SkipValidation, ConfigDict

from ultraskelly.core.bot.diagnostics import Diagnostics, Sources, judge_power, judge_servo_board
from ultraskelly.core.bot.health import (
    COMPONENTS,
    Finding,
    Health,
    Probes,
    Status,
    diagnose_board,
    diagnose_hailo,
    diagnose_oak,
    diagnose_zed,
    diagnose_zed_imu,
    head_finding,
    imu_clock_finding,
    oak_on_usb,
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
from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.sensory.hailo_pose_node import (
    HailoPoseDetectorNode,
    HailoPoseDetectorParams,
)
from ultraskelly.core.bot.sensory.oak_camera_node import OakCameraNode, OakCameraNodeParams
from ultraskelly.core.bot.sensory.oak_models import missing_models
from ultraskelly.core.bot.sensory.face_locator_node import FaceLocatorNode, FaceLocatorParams
from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNode, StereoDepthNodeParams
from ultraskelly.core.bot.sensory.zed_camera_node import ZED_MODES, ZedCameraNode, ZedCameraNodeParams
from ultraskelly.core.bot.sensory.zed_imu_node import ZedImuNode, ZedImuNodeParams
from ultraskelly.ui.runtime_server import (
    RuntimeServerNode, RuntimeServerParams, TrackingOverlay, encode_jpeg_with_opencv,
)
from ultraskelly.core.pubsub.bot_topics import DepthFrameTopic, FrameTopic, StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_abcs import PubSubTopicABC
from ultraskelly.core.pubsub.pubsub_manager import (
    PubSubTopicManager,
)

Part = TypeVar("Part")
Diagnosis = Callable[[BaseException], Finding]
STREAMING_WINDOW = 3.0  # Seconds without new data before a camera or the IMU counts as not streaming
HEALTH_INTERVAL = 1.0  # Seconds between checks of how each part is doing
DIAGNOSTICS_INTERVAL = 5.0  # Seconds between the live hardware checks (core/bot/diagnostics.py)
RECOVERY_INTERVAL = 5.0  # Seconds between looks for parts to bring back
RETRY_LIMIT = 300.0  # The longest wait, in seconds, between tries at a part that is there but fails to start
STEADY_SECONDS = 60.0  # How long a part brought back must run before its waits start over
BOARD_MISSES = 2  # Checks in a row a board the bot drives must miss before it counts as gone
MODELS_RECHECK = 30.0  # Seconds between reads of the OAK model cache while the OAK is out
OAK_FEED = "oak"  # The dashboard's name for the OAK's camera


class LaunchConfig(NodeParams):
    """Declarative launch configuration."""

    oak_camera: OakCameraNodeParams = Field(default_factory=OakCameraNodeParams)
    hailo_pose_detector: HailoPoseDetectorParams = Field(default_factory=HailoPoseDetectorParams)
    mouth: MouthNodeParams = Field(default_factory=MouthNodeParams)
    orientation: OrientationNodeParams = Field(default_factory=OrientationNodeParams)
    runtime_server: RuntimeServerParams = Field(default_factory=RuntimeServerParams)
    simulate_motors: bool = Field(
        default=False,
        description="Drive memory-only servo and motor stand-ins instead of the motor boards",
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


@dataclass
class Revival:
    """How the launcher brings a part back while it runs: once `present` finds it, `open` makes its
    node, in a worker thread, and `started` tells the rest of the bot. A part that is there but
    fails to start, or stops again within STEADY_SECONDS, is tried again after a wait that doubles,
    from RECOVERY_INTERVAL to RETRY_LIMIT."""

    component: str
    present: Callable[[], bool]
    open: Callable[[], Node]
    working: str | None  # Its message once it runs; None when its watch reports it
    started: Callable[[Node], None] = lambda node: None
    failures: int = 0  # Tries since it last ran for STEADY_SECONDS
    next_try: float = 0.0  # On time.monotonic()'s clock
    launched_at: float = 0.0  # When the last node it made started


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
    # The OAK's node while there is one, and when it was made
    oak: SkipValidation[OakCameraNode | None] = Field(default=None, exclude=True)
    oak_started: float = Field(default=0.0, exclude=True)
    # The OAK models not downloaded, read every MODELS_RECHECK seconds while the OAK is out (with it, the OAK says)
    models_missing: dict[str, str] | None = Field(default=None, exclude=True)
    models_checked_at: float = Field(default=float("-inf"), exclude=True)
    # Each part's node, the newest when one was brought back; every task's node
    parts: SkipValidation[dict[str, Node]] = Field(default_factory=dict, exclude=True)
    task_nodes: SkipValidation[dict[asyncio.Task, Node]] = Field(default_factory=dict, exclude=True)
    revivals: SkipValidation[list[Revival]] = Field(default_factory=list, exclude=True)
    ended: SkipValidation[asyncio.Queue | None] = Field(default=None, exclude=True)  # Tasks as they end, for run()
    server: SkipValidation[RuntimeServerNode | None] = Field(default=None, exclude=True)
    # Whether each motor board's actuators reach it (else stand-ins record them), and checks it missed in a row
    driven: dict[str, bool] = Field(default_factory=dict, exclude=True)
    board_misses: dict[str, int] = Field(default_factory=dict, exclude=True)
    check_errors_logged: str = Field(default="", exclude=True)  # The live checks' errors last logged

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
            finding = diagnose(error)
            self.health.report(component, *finding, detail={"error": f"{type(error).__name__}: {error}"})
            logger.error(f"Running without the {COMPONENTS[component][0]}: {finding.message} "
                         f"({type(error).__name__}: {error})")
            return None
        self.health.report(component, Status.OK, working)
        return part

    def _add_part(self, node: Node, component: str) -> None:
        """A node whose stop or crash is reported against `component` instead of ending the bot."""
        self.nodes.append(node)
        self.components_of[id(node)] = component
        self.parts[component] = node

    def _create_nodes(self) -> None:
        """Create all nodes from config."""
        self.nodes.clear()
        self.diagnostics = Diagnostics(self.hardware.sources)
        if not self.config.tracking:
            self._create_zed_only_nodes()
            return
        stereo_feeds = self._create_stereo_nodes()

        orientation, mouth = self._create_motor_nodes()
        self.controllers = (orientation, mouth)
        self.nodes.extend([orientation, mouth])
        self.motor_tester = MotorTester(
            orientation=orientation, mouth=mouth, limits=self.config.motor_tests, unavailable=self._unavailable,
            simulated=self.config.simulate_motors,
        )
        if not self.config.orientation.follow:
            logger.info("Not following the target (--no-follow): the head and waist move only for motor tests")
        self.motor_watchdog = MotorWatchdog({"waist": orientation.waist_motor, "mouth": mouth.mouth_motor})
        orientation.motor_watchdog = self.motor_watchdog
        self.watches.append(self._watch_head(orientation))

        oak = self._create_tracking_nodes()
        aim = self.config.orientation
        self.server = RuntimeServerNode.create(
            pubsub=self.pubsub,
            params=self.config.runtime_server,
            cameras={OAK_FEED: FrameTopic},
            **stereo_feeds,
            recorder=self.command_recorder,
            motors_simulated=self.config.simulate_motors,
            node_status=self.node_status,
            overlays={
                OAK_FEED: TrackingOverlay(aim=(aim.target_x, aim.target_y), jaw_zone=aim.head_deadzone)
            },
            # The Hailo detector reports which OAK frame each pose came from.
            pose_camera=OAK_FEED,
            motor_tests=self.motor_tester,
            # The OAK's own chip finds faces in the same frames.
            # Only while the OAK runs its models: without them no frame would ever get its results.
            # An OAK brought back later sets it (_oak_started); its results and status follow the newest OAK.
            results_camera=OAK_FEED if oak is not None and oak.assembler is not None else None,
            results_for=self._oak_results,
            oak_status=self._oak_status,
            health=self.health,
        )
        self.nodes.append(self.server)
        if self.config.recording is not None:
            self.nodes.append(
                RunRecorderNode.create(
                    pubsub=self.pubsub,
                    params=self.config.recording,
                    camera=OAK_FEED,
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
                # The Hailo detector reports the frame each pose result belongs to.
                watch_poses=True,
                # A stream is watched only while its source works; a missing one is reported, not restarted.
                frames_expected=lambda: self.health.ok("oak_camera"),
                poses_expected=lambda: self.health.ok("oak_camera") and self.health.ok("hailo"),
            )
        )

        logger.info(f"Created {len(self.nodes)} nodes")

    def _create_motor_nodes(self) -> tuple[OrientationNode, MouthNode]:
        """Both controllers, on the boards that answer; simulated with --simulate-motors."""
        self.command_recorder = CommandRecorder()
        config = self.config
        if config.simulate_motors:
            self.driven = {"servo_board": False, "motor_board": False}
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
        self.driven = {"servo_board": servo_kit is not None, "motor_board": motor_kit is not None}
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
        """Why a motor test on `actuator` cannot reach a board now, or None; simulated tests run."""
        board = "servo_board" if actuator in ("pan", "tilt", "roll") else "motor_board"
        component = self.health.get(board)
        if self.config.simulate_motors or component is None:
            return None
        if self.driven.get(board) and component.status not in (Status.MISSING, Status.ERROR):
            return None
        return f"{component.message}, so nothing would move. {component.restart or component.hint}"

    def _create_tracking_nodes(self) -> OakCameraNode | None:
        """The OAK camera and the Hailo-8 pose detector, each if it starts, and each brought back once
        it can start; returns the OAK's node. Neither is searched for when its probe finds it
        missing: depthai would look for 11 s."""
        probes = self.hardware.probes

        def open_oak() -> Node:
            if not oak_on_usb(probes.usb()):
                raise LookupError("no OAK (03e7) device on USB")
            return self.hardware.oak_camera(pubsub=self.pubsub, params=self.config.oak_camera)

        def open_hailo() -> Node:
            if not probes.hailo_device():
                raise LookupError("no /dev/hailo0")
            return self.hardware.hailo_pose(pubsub=self.pubsub, params=self.config.hailo_pose_detector)

        oak = self._attempt("oak_camera", open_oak, lambda error: diagnose_oak(error, probes), "Streaming")
        if oak is not None:
            self._add_part(oak, "oak_camera")
            self._oak_started(oak)
        self.revivals.append(Revival("oak_camera", lambda: oak_on_usb(probes.usb()), open_oak, "Streaming",
                                     self._oak_started))
        self.watches.append(self._watch_oak)
        self._report_oak_models()
        self.watches.append(self._report_oak_models)
        hailo = self._attempt(
            "hailo", open_hailo, lambda error: diagnose_hailo(error, probes), "Finds people in the OAK's frames",
        )
        if hailo is not None:
            self._add_part(hailo, "hailo")
        self.revivals.append(Revival("hailo", probes.hailo_device, open_hailo, "Finds people in the OAK's frames"))
        self._watch_hailo()
        self.watches.append(self._watch_hailo)
        return oak

    def _oak_started(self, oak: OakCameraNode) -> None:
        """A new OAK node: the dashboard's results and the OAK's status follow it."""
        self.oak, self.oak_started = oak, time.monotonic()
        if self.server is not None and oak.assembler is not None:
            self.server.results_camera = OAK_FEED

    def _oak_results(self, frame_number: int) -> object | None:
        """What the OAK of the moment found in a frame, for the dashboard."""
        return self.oak.results_for(frame_number) if self.oak is not None else None

    def _oak_status(self) -> dict | None:
        """The OAK's chip and model rates while it runs, else None."""
        return self.oak.status() if self.oak is not None and self.health.ok("oak_camera") else None

    def _streaming(self, component: str) -> bool:
        """Whether `component` fully works: ok, not just started."""
        current = self.health.get(component)
        return current is not None and current.status == Status.OK

    def _report_oak_models(self) -> None:
        """The OAK's own models: off, not downloaded, running, or waiting for the OAK."""
        models = self.config.oak_camera.models
        if models is None:
            self.health.report("oak_models", Status.OFF, "Turned off (--no-oak-models)")
            return
        fetch = "Run .venv/bin/python -m scripts.fetch_oak_models with the internet on (scripts/setup-pi.sh does it too)"
        if self.oak is not None and self.health.ok("oak_camera"):
            if self.oak.models_error:  # This OAK started without them: it loads them when it starts again
                self.health.report("oak_models", Status.MISSING, self.oak.models_error,
                                   f"{fetch}, then unplug the OAK and plug it back in, or restart the bot.")
            elif self._streaming("oak_camera"):
                self.health.report("oak_models", Status.OK, "Running on the OAK's chip")
            else:
                self.health.report("oak_models", Status.DEGRADED, "Loaded on the OAK; waiting for its frames",
                                   None, "oak_camera")
            return
        if time.monotonic() - self.models_checked_at >= MODELS_RECHECK:  # Fetched meanwhile, or gone
            self.models_missing, self.models_checked_at = missing_models(models.model_cache), time.monotonic()
        if self.models_missing:
            self.health.report("oak_models", Status.MISSING, f"Not downloaded: {', '.join(self.models_missing)}",
                               f"{fetch}; the OAK loads them once it is plugged in.")
        else:
            self.health.report("oak_models", Status.DEGRADED, "Downloaded; they run once the OAK streams",
                               None, "oak_camera")

    def _watch_oak(self) -> None:
        """While the OAK node runs: streaming, or started but sending no frames (reported, not restarted)."""
        current = self.health.get("oak_camera")
        if self.oak is None or current is None or current.status not in (Status.OK, Status.DEGRADED):
            return  # Missing, or stopped or crashed: reported when its node ended
        if self.oak.status()["rates"].get("frames", 0.0) > 0:
            self.health.report("oak_camera", Status.OK, "Streaming")
        elif time.monotonic() - self.oak_started > self.config.stall_monitor.startup_seconds:
            self.health.report("oak_camera", Status.DEGRADED, "Started, but sends no frames",
                               "Unplug the OAK, wait 5 s and plug it back in; if it stays quiet, restart the bot.")

    def _watch_hailo(self) -> None:
        """The Hailo-8 finds people in the OAK's frames, so it waits while the OAK sends none."""
        if not self.health.ok("hailo"):
            return  # Missing or failed: its own report stands
        if self._streaming("oak_camera"):
            self.health.report("hailo", Status.OK, "Finds people in the OAK's frames")
        else:
            self.health.report("hailo", Status.DEGRADED, "Waiting for the OAK camera's frames", None, "oak_camera")

    def _create_stereo_nodes(self) -> dict[str, dict[str, type[PubSubTopicABC]]]:
        """The ZED Mini's nodes, when configured: capture, its IMU, and depth if asked for. The camera and
        the IMU start even when missing, and pick them up once they appear; depth reads its calibration
        from the camera, so it starts once the camera streams. Returns the dashboard's stereo and depth
        camera feeds."""
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
        # Should its node crash, a new one starts waiting for the camera
        self.revivals.append(Revival(
            "zed_camera", lambda: True, lambda: self.hardware.zed_camera(pubsub=self.pubsub, params=zed, wait=True), None,
        ))
        self.watches.append(self._watch_streaming(
            "zed_camera", lambda node: node.frames_published if node.source is not None else None,
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
            self.revivals.append(Revival(
                "zed_imu", lambda: True, lambda: self.hardware.zed_imu(pubsub=self.pubsub, params=imu_params, wait=True),
                None,
            ))
            self.watches.append(self._watch_streaming(
                "zed_imu", lambda node: node.samples, "Reading the accelerometer and gyroscope",
                lambda: diagnose_zed_imu(OSError("no samples arrive"), probes),
                check=lambda node: imu_clock_finding(node.clock_error(time.monotonic())),
            ))
        feeds = {"stereo_cameras": {"zed": StereoFrameTopic}}
        if self.config.stereo_depth is not None:
            depth_params = self.config.stereo_depth
            width, height = ZED_MODES[zed.resolution][0]
            working = f"Depth from the ZED's eyes ({depth_params.matcher})"

            def open_depth() -> Node:
                return self.hardware.stereo_depth(pubsub=self.pubsub, params=depth_params, eye_size=(width // 2, height))

            def depth_possible() -> bool:  # The camera streams; StereoNet also needs the Hailo-8
                return self._streaming("zed_camera") and (depth_params.matcher != "stereonet" or probes.hailo_device())

            self.diagnoses["depth"] = self._diagnose_depth
            if self._streaming("zed_camera"):
                depth = self._attempt("depth", open_depth, self._diagnose_depth, working)
                if depth is not None:
                    self._add_part(depth, "depth")
            else:
                self.health.report("depth", Status.DEGRADED, "Starts once the ZED Mini streams", None, "zed_camera")
            self.revivals.append(Revival("depth", depth_possible, open_depth, working))
            feeds["depth_cameras"] = {"zed": DepthFrameTopic}  # Also before depth starts: the server waits for it
            self.watches.append(self._watch_depth(depth_params.matcher))
            if self.config.tracking:
                # It uses only pose results that carry the OAK frame they came from (its size and time),
                # so any detector of OAK images feeds it and the others are ignored.
                self.nodes.append(FaceLocatorNode.create(pubsub=self.pubsub, params=self.config.face_locator))
        return feeds

    def _watch_head(self, orientation: OrientationNode) -> Callable[[], None]:
        """The head's aim: following, or an axis held at the end of its range while following someone. Following
        is the node's live switch (--no-follow at start, then the control panel), not the start option."""
        def watch() -> None:
            self.health.report("head", *head_finding(
                follow=orientation.following, pinned=orientation.pinned(orientation.clock()),
            ))
        return watch

    def _watch_streaming(
        self, component: str, count: Callable[[Node], int | None], working: str, diagnose: Callable[[], Finding],
        check: Callable[[Node], Finding | None] | None = None,
    ) -> Callable[[], None]:
        """A part whose node picks it up by itself: working while `count` of its node (None: not open)
        keeps rising, else diagnosed from what is plugged in. For each node's first STREAMING_WINDOW
        the status it had stands, while the node opens it. While it streams, `check` may still find
        something wrong with what it sends."""
        state = {"node": None, "count": None, "rose_at": None, "first": None}

        def watch() -> None:
            now, node = time.monotonic(), self.parts.get(component)
            latest = count(node) if node is not None else None
            if node is not state["node"]:  # The first look, or a node brought back: count afresh
                state.update(node=node, count=latest, rose_at=None, first=now)
            if latest is not None and state["count"] is not None and latest > state["count"]:
                state["rose_at"] = now
            state["count"] = latest
            if state["rose_at"] is not None and now - state["rose_at"] <= STREAMING_WINDOW:
                finding = check(node) if check is not None else None
                self.health.report(component, *(finding or (Status.OK, working)))
            elif now - state["first"] > STREAMING_WINDOW:
                self.health.report(component, *diagnose())
        return watch

    def _diagnose_depth(self, error: BaseException) -> Finding:
        if self.config.stereo_depth.matcher == "stereonet" and not self.hardware.probes.hailo_device():
            return Finding(Status.MISSING, "StereoNet runs on the Hailo-8, which is missing",
                           "Seat the AI HAT+ (see Hailo-8), or run with --depth, which computes depth on the CPU.",
                           "hailo")
        return Finding(Status.ERROR, f"Depth did not start: {error}")

    def _watch_depth(self, matcher: str) -> Callable[[], None]:
        """Depth while it runs, else what it waits for; one that failed to start keeps its report."""
        def watch() -> None:
            if self._part_running("depth"):
                if self.health.ok("zed_camera"):
                    self.health.report("depth", Status.OK, f"Depth from the ZED's eyes ({matcher})")
                else:
                    self.health.report("depth", Status.DEGRADED, "Waiting for the ZED Mini's images", None, "zed_camera")
            elif not self._streaming("zed_camera"):
                self.health.report("depth", Status.DEGRADED, "Starts once the ZED Mini streams", None, "zed_camera")
            elif matcher == "stereonet" and not self.hardware.probes.hailo_device():
                self.health.report("depth", *self._diagnose_depth(LookupError("no /dev/hailo0")))
        return watch

    def _create_zed_only_nodes(self) -> None:
        """Only the ZED Mini and the dashboard: no OAK, Hailo or motors, so no robot lock is needed,
        and the live checks leave the motor boards' I2C bus alone."""
        self.command_recorder = None
        self.diagnostics = Diagnostics(replace(self.hardware.sources, open_bus=lambda: None))
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

        # Clear nodes to release resources
        self.nodes.clear()

        logger.info("All nodes stopped and resources released")

    async def _start_nodes(self) -> None:
        """Start all nodes as async tasks."""
        logger.info("Starting nodes...")
        if self.motor_watchdog is not None:
            self.motor_watchdog.start()  # Armed by the control loop's first tick

        for node in self.nodes:
            task = self._start_task(node.run())
            self.task_nodes[task] = node
            self.tasks.append(task)

        logger.info(f"Started {len(self.tasks)} node tasks")

    def _start_task(self, coroutine) -> asyncio.Task:
        """A task whose end reaches run() through `ended`."""
        task = asyncio.create_task(coroutine)
        task.add_done_callback(lambda done: self.ended.put_nowait(done) if self.ended is not None else None)
        return task

    def _part_running(self, component: str) -> bool:
        """Whether the part's newest node is running."""
        node = self.parts.get(component)
        task = next((task for task, each in zip(self.tasks, self.nodes) if each is node), None)
        return task is not None and not task.done()

    def _launch(self, node: Node, component: str) -> None:
        """Run a part's node made while the bot runs, in the place of the node it replaces."""
        old = self.parts.get(component)
        task = self._start_task(node.run())
        self.task_nodes[task] = node
        self.components_of[id(node)] = component
        self.parts[component] = node
        index = next((index for index, each in enumerate(self.nodes) if each is old), None)
        if index is None:
            self.nodes.append(node)
            self.tasks.append(task)
        else:
            self.nodes[index], self.tasks[index] = node, task

    async def _recover(self) -> None:
        """Every RECOVERY_INTERVAL, bring back each part whose node is not running, once it can start."""
        while True:
            await asyncio.sleep(RECOVERY_INTERVAL)
            for revival in self.revivals:
                if not self._part_running(revival.component):
                    await self._revive(revival)
                elif revival.failures and time.monotonic() - revival.launched_at > STEADY_SECONDS:
                    revival.failures, revival.next_try = 0, 0.0  # Well again: its next stop is tried at once

    async def _revive(self, revival: Revival) -> None:
        """Make the part's node again if its probe finds it and its wait is over, and run it."""
        if not revival.present():
            revival.failures, revival.next_try = 0, 0.0  # Tried at once when it appears
            return
        if time.monotonic() < revival.next_try:
            return
        label = COMPONENTS[revival.component][0]
        wait = min(RECOVERY_INTERVAL * 2 ** revival.failures, RETRY_LIMIT)
        revival.failures += 1  # Counted until the part has run for STEADY_SECONDS
        try:
            # In a worker thread, so the motors' control loop runs on meanwhile. A factory opens its
            # device and makes its subscriptions (a queue each, added to its topic's list); every
            # operation on those queues stays on this loop.
            node = await asyncio.to_thread(revival.open)
        except Exception as error:
            revival.next_try = time.monotonic() + wait
            finding = self.diagnoses[revival.component](error)
            self.health.report(revival.component, *finding, detail={"error": f"{type(error).__name__}: {error}"})
            logger.warning(f"The {label} did not start: {finding.message}; trying again in {wait:g} s")
            return
        revival.launched_at = time.monotonic()
        revival.next_try = revival.launched_at + wait  # Should it stop again soon, it waits its turn
        self._launch(node, revival.component)
        revival.started(node)
        if revival.working is not None:
            self.health.report(revival.component, Status.OK, revival.working)
        logger.info(f"Brought back the {label}")

    def _check_boards(self, i2c: dict) -> None:
        """The motor boards, from the I2C probe. The bot sets the boards up and centres the head only
        when it starts, so a board it started without that answers now needs a restart; one it drives
        that stops answering is reported before its next command fails, which stops the bot."""
        if self.config.simulate_motors or not i2c or not i2c.get("available"):
            return
        for board, address, drives in (("servo_board", SERVO_BOARD_ADDRESS, "the head's pan and tilt"),
                                       ("motor_board", MOTOR_BOARD_ADDRESS, "the waist and jaw")):
            answers, current = i2c["addresses"].get(f"{address:#04x}"), self.health.get(board)
            if answers is None or current is None or board not in self.driven:
                continue
            if self.driven[board]:
                self.board_misses[board] = 0 if answers else self.board_misses.get(board, 0) + 1
                if self.board_misses[board] >= BOARD_MISSES:
                    self.health.report(
                        board, Status.ERROR, f"Stopped answering at {address:#04x}",
                        "Press the HAT stack down onto the Pi's 40-pin header and check the board's power LED. "
                        "The bot stops at its next command to the board; its service then starts it again.",
                    )
                elif answers and current.status == Status.ERROR:
                    self.health.report(board, Status.OK, f"Answers again at {address:#04x}; {drives} are sent to it")
            elif answers:
                self.health.report(
                    board, Status.DEGRADED, f"Answers at {address:#04x} now, but the bot started without it",
                    restart=f"Restart the bot to drive {drives}: it sets up its motor boards only when it starts.",
                )
            elif current.status == Status.DEGRADED:  # Gone again
                self.health.report(board, *self.diagnoses[board](OSError(f"No I2C device at address: {address:#x}")))

    def _report_stopped(self, component: str, task: asyncio.Task) -> None:
        """A camera, detector or depth node ended: report why, against its part, and run on."""
        if task.cancelled():
            return
        error = task.exception() or RuntimeError("it stopped")
        diagnose = self.diagnoses.get(component, lambda error: Finding(Status.ERROR, str(error)))
        finding = diagnose(error)
        self.health.report(component, *finding, detail={"error": f"{type(error).__name__}: {error}"})
        logger.error(f"The {COMPONENTS[component][0]} stopped; running on without it: {finding.message}")

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
            except Exception as error:
                failures += 1
                self.health.diagnostics_failed(error)  # Sent, so the dashboard can tell "failing" from "not run yet"
                if failures == 1 or failures % 100 == 0:
                    logger.exception(f"A hardware check failed ({failures} so far); the others go on")
            await asyncio.sleep(DIAGNOSTICS_INTERVAL)

    async def _diagnose(self) -> None:
        driven = self.driven.get("servo_board", False) and self.health.ok("servo_board")  # The real board, in use
        params = self.config.orientation
        head = {role: channel for channel, role in params.servo_names.items()} if driven else {}
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
        errors = self.diagnostics.errors
        self.health.set_diagnostics(result, errors)
        if (logged := "; ".join(f"{name}: {error}" for name, error in errors.items())) != self.check_errors_logged:
            if logged:  # Once for each new set of errors
                logger.warning(f"Live hardware checks failed, the others go on: {logged}")
            self.check_errors_logged = logged
        self._check_boards(result["i2c"])
        if driven and self.health.ok("servo_board") and (judged := judge_servo_board(result["pca9685"])) is not None:
            self.health.report("servo_board", *judged)
        if (power := judge_power(result["power"])) is not None:
            self.health.report("power", *power)

    async def run(self) -> None:
        """Run all nodes until one that is not a part of the robot (a camera, detector or depth node)
        stops or crashes, then stop the rest. A part's node that ends is reported against its part,
        the rest run on, and the recovery brings the part back once it can start.

        A crashed node's exception is re-raised so failures surface instead of being hidden.
        """
        logger.info("=" * 60)
        logger.info("LAUNCHING TARGET TRACKER WITH SOPHISTICATED PUBSUB")
        logger.info("=" * 60)

        self.ended = asyncio.Queue()  # Each task as it ends, from its done callback, on this loop
        background: list[asyncio.Task] = []
        try:
            await self._start_nodes()
            jobs = [self._watch_health(), self._recover()]
            if self.diagnostics is not None:
                jobs.append(self._watch_diagnostics())
            background = [self._start_task(job) for job in jobs]
            while True:
                task = await self.ended.get()
                if task in background:
                    task.result()  # Re-raises; they never return
                    break
                node = self.task_nodes.get(task)
                component = self.components_of.get(id(node))
                if component is None:
                    task.result()  # Re-raises if the node crashed
                    break
                if self.parts.get(component) is node:  # Not a node already replaced
                    self._report_stopped(component, task)
            logger.info("A node stopped; stopping the others...")
        finally:
            for task in background:
                task.cancel()
            await asyncio.gather(*background, return_exceptions=True)
            await self._shutdown_nodes()
            if self.diagnostics is not None:
                self.diagnostics.close()
            self.pubsub.close()
            logger.info("All nodes stopped and pubsub closed")
