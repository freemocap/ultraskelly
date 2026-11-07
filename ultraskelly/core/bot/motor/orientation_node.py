import asyncio
import logging
import math
import time
from typing import Callable

from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.motor.bridge_motor import FAST_DECAY, SLOW_DECAY, BridgeMotor
from ultraskelly.core.bot.motor.head_aim import HeadAim, HeadAimParams
from ultraskelly.core.bot.motor.motor_watchdog import MotorWatchdog
from ultraskelly.core.bot.motor.roll_level import LEVEL, RollLevel, RollLevelParams, choose_face
from ultraskelly.core.bot.motor.waist_drive import MAX_STEP, WaistDrive, WaistDriveParams, WaistState
from ultraskelly.core.pubsub.bot_topics import (
    OakResultsMessage,
    OakResultsTopic,
    ServoStateMessage,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)

ROLL_LEVEL = LEVEL  # Roll servo degrees at which the head stands upright
PINNED_SLACK = 0.5  # Servo degrees from where the head stops short of an end stop that still count as at that end


class OrientationNodeParams(NodeParams):
    """Parameters for OrientationNode."""

    # Servo channels, found 2026-11-06 by wiggling one channel at a time with the user
    # watching: channel 3 turns the head left-right, 11 nods it, 7 tilts it ear to shoulder.
    pan_channel: int = Field(default=3, ge=0, le=15)
    tilt_channel: int = Field(default=11, ge=0, le=15)
    roll_channel: int = Field(default=7, ge=0, le=15, description="Levels the followed face; upright at ROLL_LEVEL")
    follow: bool = Field(
        default=True,
        description="Turn the head and waist toward the target from the start; off, they move only for motor tests "
                    "until following is switched on (OrientationNode.set_following)",
    )

    # Waist motor channel
    waist_motor_channel: int = Field(default=1, ge=1, le=4)
    
    # Target tracking
    target_x: int = Field(default=320, ge=0)
    target_y: int = Field(default=240, ge=0)
    offset_x_ratio: float = Field(default=0., ge=-1.0, le=1.0)
    offset_y_ratio: float = Field(default=0., ge=-1.0, le=1.0)
    
    # Head servo control
    aim: HeadAimParams = Field(default_factory=HeadAimParams, description="How the head turns toward its target")
    roll: RollLevelParams = Field(default_factory=RollLevelParams, description="How the head's roll levels the face it follows")
    head_deadzone: int = Field(default=100, ge=0, description="Jaw activation half-width in pixels")
    target_timeout: float = Field(default=0.35, gt=0)
    
    # Waist motor control
    waist: WaistDriveParams = Field(
        default_factory=WaistDriveParams, description="How the waist turns the torso toward the head's gaze"
    )
    
    @property
    def servo_names(self) -> dict[int, str]:
        """The head's servo board channels by role, as the command recorder names them."""
        return {self.pan_channel: "pan", self.tilt_channel: "tilt", self.roll_channel: "roll"}

    @property
    def x_offset(self) -> int:
        return int(self.offset_x_ratio * self.target_x * 2)
    
    @property
    def y_offset(self) -> int:
        return int(self.offset_y_ratio * self.target_y * 2)


class OrientationNode(Node):
    """Combined head and waist controller for maximum tracking range."""
    
    params: OrientationNodeParams
    
    clock: Callable[[], float] = Field(default_factory=lambda: time.monotonic, exclude=True)

    # Hardware
    servo_kit: SkipValidation[object] = Field(default=None, exclude=True)
    motor_kit: SkipValidation[object] = Field(default=None, exclude=True)
    waist_motor: SkipValidation[object] = Field(default=None, exclude=True)
    
    # Subscriptions
    target_subscription: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)
    faces_subscription: SkipValidation[TopicSubscriptionQueue | None] = Field(
        default=None, exclude=True, description="The OAK's face results; without them the roll holds upright",
    )
    
    # Head servo state: the angles last commanded
    pan_angle: float = Field(default=90.0)
    tilt_angle: float = Field(default=90.0)
    roll_angle: float = Field(default=ROLL_LEVEL)
    roll_level: SkipValidation[RollLevel] = Field(default=None, exclude=True)
    latest_faces: OakResultsMessage | None = Field(default=None, exclude=True)
    head_aim: SkipValidation[HeadAim] = Field(default=None, exclude=True)
    head_still_since: float | None = Field(default=None, exclude=True)
    body_turn: float = Field(default=0.0, exclude=True, description="How far the waist has turned, estimated from its drive")
    last_step: float | None = Field(default=None, exclude=True)
    
    # Waist motor state
    waist_drive: SkipValidation[WaistDrive] = Field(default=None, exclude=True)
    waist_mode: WaistState = Field(default=WaistState.IDLE)
    manual: bool = Field(default=False, description="True while a motor test drives the actuators; tracking commands nothing")
    following: bool | None = Field(
        default=None, description="Whether the head and waist follow the target now: params.follow at start, then switched",
    )
    motor_watchdog: SkipValidation[MotorWatchdog | None] = Field(
        default=None, exclude=True, description="Ticked every control step; brakes the DC motors if the steps stop",
    )
    
    # Target tracking state
    latest_target: TargetLocationMessage | None = Field(default=None, exclude=True)
    target_received_at: float | None = Field(default=None)
    last_target_x: int | None = Field(default=None)
    last_target_y: int | None = Field(default=None)
    pinned_since: dict[str, float | None] = Field(
        default_factory=lambda: {"pan": None, "tilt": None}, exclude=True,
        description="Per head axis, since when it has sat at an end of its range with the person followed still "
                    "off the aim point beyond that end, else None",
    )

    def pinned(self, now: float) -> dict[str, float]:
        """The head axes that cannot reach the person followed: at an end of their range with the person still
        off the aim point that way, and for how many seconds."""
        return {axis: now - since for axis, since in self.pinned_since.items() if since is not None}

    def _track_pinned(self, now: float, errors: tuple[float, float] | None) -> None:
        """`errors`: the person's pixel offset from the aim point, None while nobody is followed."""
        aim = self.params.aim
        for index, (axis, angle, (low, high), direction) in enumerate((
            ("pan", self.pan_angle, aim.pan_range, aim.pan_direction),
            ("tilt", self.tilt_angle, aim.tilt_range, aim.tilt_direction),
        )):
            error = None if errors is None else errors[index]
            wants = 0.0 if error is None or abs(error) <= self.params.head_deadzone else error * direction
            stuck = ((wants > 0 and angle >= high - aim.end_margin - PINNED_SLACK)
                     or (wants < 0 and angle <= low + aim.end_margin + PINNED_SLACK))
            if not stuck:
                self.pinned_since[axis] = None
            elif self.pinned_since[axis] is None:
                self.pinned_since[axis] = now

    def model_post_init(self, context: object) -> None:
        if self.waist_drive is None:
            self.waist_drive = WaistDrive(self.params.waist)
        if self.head_aim is None:
            self.head_aim = HeadAim(self.params.aim, self.pan_angle, self.tilt_angle)
        if self.roll_level is None:
            self.roll_level = RollLevel(self.params.roll, self.roll_angle)
        if self.following is None:
            self.following = self.params.follow

    def set_following(self, on: bool) -> None:
        """Follow the target, or hold still: the head stays where it was last sent and the waist brakes at once,
        unless a motor test is driving it (the test brakes it when it ends). Either way the waist's next move
        starts from rest. Runs on the event loop, between control steps."""
        if not on and not self.manual:
            self.waist_motor.throttle = 0.0
        self.waist_drive = WaistDrive(self.params.waist)  # No stale move resumes
        self.head_still_since = None
        self.following = on

    @classmethod
    def create(
        cls,
        *,
        pubsub: PubSubTopicManager,
        params: OrientationNodeParams,
        pwm_frequency: float = 1600.0,
    ) -> "OrientationNode":
        """Factory method to create and initialize OrientationNode.

        `pwm_frequency` is the motor board's; the jaw's factory must be given the same, as the
        board has one PWM clock for all its motors.
        """
        from adafruit_motorkit import MotorKit
        from adafruit_servokit import ServoKit

        node = cls(pubsub=pubsub, params=params)
        
        # Initialize servo kit
        node.servo_kit = ServoKit(channels=16)
        node.command_start_angles()

        # Initialize motor kit
        node.motor_kit = MotorKit(address=0x60, pwm_frequency=pwm_frequency)
        
        # The waist motor, both bridge inputs set in one I2C transfer (see bridge_motor.py)
        node.waist_motor = BridgeMotor.from_motorkit(node.motor_kit, params.waist_motor_channel)
        # Braking between PWM pulses: quieter and gentler at low drive (see waist_drive.py)
        node.waist_motor.decay_mode = SLOW_DECAY if params.waist.slow_decay else FAST_DECAY
        
        # Get subscription
        node.target_subscription = pubsub.get_subscription(TargetLocationTopic)
        node.faces_subscription = pubsub.get_subscription(OakResultsTopic)
        
        return node

    def command_start_angles(self) -> None:
        """Send the head's servos their starting angles: as tracking holds them, roll upright."""
        self.servo_kit.servo[self.params.pan_channel].angle = self.pan_angle
        self.servo_kit.servo[self.params.tilt_channel].angle = self.tilt_angle
        self.servo_kit.servo[self.params.roll_channel].angle = self.roll_angle

    def _target_errors(self, msg: TargetLocationMessage) -> tuple[int, int]:
        """The target's offset in pixels from where the head steers it."""
        return (msg.x - self.params.target_x - self.params.x_offset,
                msg.y - self.params.target_y - self.params.y_offset)

    def _upright(self, error_x: float, error_y: float, captured: float | None, now: float) -> tuple[float, float]:
        """A target's offset as an upright camera would have seen it: the head's roll turned the picture's
        content anticlockwise by the rotation commanded a servo lag before the frame was captured."""
        p = self.params.roll
        turn = math.radians(self.roll_level.rolled_at((captured if captured is not None else now - p.assumed_latency) - p.servo_lag))
        return (error_x * math.cos(turn) - error_y * math.sin(turn),
                error_x * math.sin(turn) + error_y * math.cos(turn))

    def _command_roll(self, roll: float) -> None:
        """Send the roll servo its new angle, when it changed."""
        if roll != self.roll_angle:
            self.roll_angle = roll
            self.servo_kit.servo[self.params.roll_channel].angle = roll

    def _command_head(self, pan: float, tilt: float) -> None:
        """Send the servos the head's new angles, when they changed."""
        if pan != self.pan_angle:
            self.pan_angle = pan
            self.servo_kit.servo[self.params.pan_channel].angle = pan
        if tilt != self.tilt_angle:
            self.tilt_angle = tilt
            self.servo_kit.servo[self.params.tilt_channel].angle = tilt

    async def step(self) -> None:
        """Center on new samples; hold lock between samples until a freshness timeout."""
        fresh = False
        while not self.target_subscription.empty():
            self.latest_target = await self.target_subscription.get()
            self.target_received_at = self.clock()
            fresh = True
        fresh_faces = False
        while self.faces_subscription is not None and not self.faces_subscription.empty():
            self.latest_faces = await self.faces_subscription.get()
            fresh_faces = True
        if self.manual or not self.following:  # A motor test drives, or nothing follows the target
            self._track_pinned(self.clock(), None)
            await self.pubsub.publish(ServoStateTopic, ServoStateMessage(  # Angles as commanded, not measured
                pan_angle=self.pan_angle, tilt_angle=self.tilt_angle, roll_angle=self.roll_angle,
                is_locked_x=False, is_locked_y=False, is_locked_roll=False,
            ))
            return
        now = self.clock()
        if (self.head_aim.pan, self.head_aim.tilt) != (self.pan_angle, self.tilt_angle):
            self.head_aim.place(self.pan_angle, self.tilt_angle)  # A motor test moved the head: carry on from there
        if self.roll_level.servo != self.roll_angle:
            self.roll_level.place(self.roll_angle)  # Or the roll
        msg = self.latest_target
        valid = (msg is not None and msg.x is not None and msg.y is not None
                 and self.target_received_at is not None
                 and now - self.target_received_at <= self.params.target_timeout)
        locked_x = locked_y = False
        errors = None
        if valid:
            error_x, error_y = errors = self._target_errors(msg)
            if fresh:
                upright_x, upright_y = self._upright(error_x, error_y, msg.captured, now)
                self.head_aim.observe(now=now, error_x=upright_x, error_y=upright_y, captured=msg.captured)
            if fresh_faces and self.latest_faces is not None:
                face = choose_face(self.latest_faces.faces, (msg.x, msg.y))  # The face of the person followed
                if face is not None:
                    self.roll_level.observe(now=now, face=face, captured=self.latest_faces.timestamp)
            locked_x, locked_y = abs(error_x) <= self.params.head_deadzone, abs(error_y) <= self.params.head_deadzone
            self.last_target_x, self.last_target_y = msg.x, msg.y
        # Estimate the body's turn from the drive sent since the last step (no sensor measures it), so the
        # head can counter-rotate at once. Negative drive turns the torso toward positive pan.
        dt = 0.0 if self.last_step is None else min(max(now - self.last_step, 0.0), MAX_STEP)
        self.last_step = now
        body_rate = -(self.waist_motor.throttle or 0.0) * self.params.waist.degrees_per_second
        self.body_turn += body_rate * dt
        self._command_head(*self.head_aim.update(now=now, body=self.body_turn, body_rate=body_rate))
        self._track_pinned(now, errors)
        self._command_roll(self.roll_level.update(now=now) if self.params.roll.enabled else ROLL_LEVEL)
        if not all(self.head_aim.settled):
            self.head_still_since = None
        elif self.head_still_since is None:
            self.head_still_since = now
        # Turn the torso toward the target, once the head has finished turning: by its bearing from
        # the body, which reaches past the geared head's few degrees of pan. The head counter-rotates
        # back toward the middle meanwhile; without a target the waist ramps to a stop.
        if self.params.waist.enabled:
            self.waist_motor.throttle = self.waist_drive.update(
                now=now, pan_offset=self.head_aim.demand() if valid else None,
                head_still_for=0.0 if self.head_still_since is None else now - self.head_still_since,
            )
        else:
            self.waist_motor.throttle = 0.0  # Off: braked, so only motor tests turn the waist
        self.waist_mode = self.waist_drive.state
        # Publish explicit loss as well as valid lock state so the jaw can release. Angles as commanded.
        await self.pubsub.publish(ServoStateTopic, ServoStateMessage(
            pan_angle=self.pan_angle, tilt_angle=self.tilt_angle, roll_angle=self.roll_angle,
            is_locked_x=locked_x, is_locked_y=locked_y, is_locked_roll=self.roll_level.level(),
        ))

    async def run(self) -> None:
        """Main control loop."""
        waist = self.params.waist
        logger.info(
            f"Starting OrientationNode [pan up to {self.params.aim.pan_speed:.0f} deg/s at {self.params.aim.pan_acceleration:.0f} deg/s^2, "
            f"waist {waist.min_throttle:.2f}-{waist.max_throttle:.2f} from {waist.start_throttle:.2f}, "
            f"ramps +{waist.ramp_up:.1f}/-{waist.ramp_down:.1f} per s]"
        )
        
        try:
            while not self.stop_event.is_set():
                await self.step()
                if self.motor_watchdog is not None:
                    self.motor_watchdog.tick()

                # Short sleep for responsive control
                await asyncio.sleep(0.01)  # 100Hz update rate
        
        except Exception as e:
            logger.error(f"OrientationNode error: {e}")
            raise
        
        finally:
            if self.motor_watchdog is not None:
                self.motor_watchdog.disarm()  # Stopping on purpose; the brake follows here
            # Always stop motors and center servos on exit
            self.waist_motor.throttle = 0.0
            self.servo_kit.servo[self.params.pan_channel].angle = 90.0
            self.servo_kit.servo[self.params.tilt_channel].angle = 90.0
            self.servo_kit.servo[self.params.roll_channel].angle = ROLL_LEVEL
            await asyncio.sleep(0.5)
            logger.info("OrientationNode stopped")
