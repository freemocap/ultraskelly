import asyncio
import logging
import time
from typing import Callable

import numpy as np
from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.motor.waist_drive import WaistDrive, WaistDriveParams, WaistState
from ultraskelly.core.pubsub.bot_topics import (
    ServoStateMessage,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)


class OrientationNodeParams(NodeParams):
    """Parameters for OrientationNode."""
    
    # Servo channels
    pan_channel: int = Field(default=11, ge=0, le=15)
    tilt_channel: int = Field(default=3, ge=0, le=15)
    
    # Waist motor channel
    waist_motor_channel: int = Field(default=1, ge=1, le=4)
    
    # Target tracking
    target_x: int = Field(default=320, ge=0)
    target_y: int = Field(default=240, ge=0)
    offset_x_ratio: float = Field(default=0., ge=-1.0, le=1.0)
    offset_y_ratio: float = Field(default=0., ge=-1.0, le=1.0)
    
    # Head servo control
    head_gain: float = Field(default=0.05, gt=0.0, le=1.0)
    head_deadzone: int = Field(default=100, ge=0, description="Jaw activation half-width in pixels")
    centering_tolerance: int = Field(default=1, ge=0)
    target_timeout: float = Field(default=0.35, gt=0)
    
    # Waist motor control
    waist: WaistDriveParams = Field(
        default_factory=WaistDriveParams, description="How the waist turns the torso toward the head's gaze"
    )
    
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
    
    # Head servo state
    pan_angle: float = Field(default=90.0)
    tilt_angle: float = Field(default=90.0)
    
    # Waist motor state
    waist_drive: SkipValidation[WaistDrive] = Field(default=None, exclude=True)
    waist_mode: WaistState = Field(default=WaistState.IDLE)
    manual: bool = Field(default=False, description="True while a motor test drives the actuators; tracking commands nothing")
    
    # Target tracking state
    latest_target: TargetLocationMessage | None = Field(default=None, exclude=True)
    target_received_at: float | None = Field(default=None)
    last_target_x: int | None = Field(default=None)
    last_target_y: int | None = Field(default=None)

    def model_post_init(self, context: object) -> None:
        if self.waist_drive is None:
            self.waist_drive = WaistDrive(self.params.waist)
    
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
        from adafruit_motor import motor as dc_motor
        from adafruit_motorkit import MotorKit
        from adafruit_servokit import ServoKit

        node = cls(pubsub=pubsub, params=params)
        
        # Initialize servo kit
        node.servo_kit = ServoKit(channels=16)
        node.servo_kit.servo[params.pan_channel].angle = node.pan_angle
        node.servo_kit.servo[params.tilt_channel].angle = node.tilt_angle
        
        # Initialize motor kit
        node.motor_kit = MotorKit(address=0x60, pwm_frequency=pwm_frequency)
        
        # Select waist motor channel
        match params.waist_motor_channel:
            case 1:
                node.waist_motor = node.motor_kit.motor1
            case 2:
                node.waist_motor = node.motor_kit.motor2
            case 3:
                node.waist_motor = node.motor_kit.motor3
            case 4:
                node.waist_motor = node.motor_kit.motor4
            case _:
                raise ValueError(f"Invalid waist motor channel: {params.waist_motor_channel}")
        
        if node.waist_motor is None:
            raise ValueError(f"Could not initialize waist motor on channel {params.waist_motor_channel}")
        # Braking between PWM pulses: quieter and gentler at low drive (see waist_drive.py)
        node.waist_motor.decay_mode = dc_motor.SLOW_DECAY if params.waist.slow_decay else dc_motor.FAST_DECAY
        
        # Get subscription
        node.target_subscription = pubsub.get_subscription(TargetLocationTopic)
        
        return node
    
    def _update_head_servos(
        self,
        *,
        target_x: int,
        target_y: int
    ) -> tuple[bool, bool]:
        """
        Update head servo positions to track target.
        
        Returns:
            Tuple of (is_locked_x, is_locked_y)
        """
        # Calculate errors
        error_x = target_x - self.params.target_x - self.params.x_offset
        error_y = target_y - self.params.target_y - self.params.y_offset
        
        # Check lock status
        is_locked_x = abs(error_x) <= self.params.head_deadzone
        is_locked_y = abs(error_y) <= self.params.head_deadzone
        
        # Update pan servo
        if abs(error_x) > self.params.centering_tolerance:
            self.pan_angle += error_x * self.params.head_gain
            self.pan_angle = float(np.clip(self.pan_angle, 0.0, 180.0))
            self.servo_kit.servo[self.params.pan_channel].angle = self.pan_angle
        
        # Update tilt servo
        if abs(error_y) > self.params.centering_tolerance:
            self.tilt_angle += error_y * self.params.head_gain
            self.tilt_angle = float(np.clip(self.tilt_angle, 0.0, 180.0))
            self.servo_kit.servo[self.params.tilt_channel].angle = self.tilt_angle
        
        return is_locked_x, is_locked_y
    
    async def step(self) -> None:
        """Center on new samples; hold lock between samples until a freshness timeout."""
        fresh = False
        while not self.target_subscription.empty():
            self.latest_target = await self.target_subscription.get()
            self.target_received_at = self.clock()
            fresh = True
        if self.manual:
            await self.pubsub.publish(ServoStateTopic, ServoStateMessage(
                pan_angle=self.pan_angle, tilt_angle=self.tilt_angle, roll_angle=90.0,
                is_locked_x=False, is_locked_y=False, is_locked_roll=False,
            ))
            return
        msg = self.latest_target
        valid = (msg is not None and msg.x is not None and msg.y is not None
                 and self.target_received_at is not None
                 and self.clock() - self.target_received_at <= self.params.target_timeout)
        locked_x = locked_y = False
        if valid:
            if fresh:
                locked_x, locked_y = self._update_head_servos(target_x=msg.x, target_y=msg.y)
            else:
                locked_x = abs(msg.x - self.params.target_x - self.params.x_offset) <= self.params.head_deadzone
                locked_y = abs(msg.y - self.params.target_y - self.params.y_offset) <= self.params.head_deadzone
            self.last_target_x, self.last_target_y = msg.x, msg.y
        # Turn the torso toward the head's gaze. The camera loop then counter-rotates the
        # head toward neutral as the waist turns; without a target the waist ramps to a stop.
        self.waist_motor.throttle = self.waist_drive.update(
            now=self.clock(), pan_offset=self.pan_angle - 90.0 if valid else None,
        )
        self.waist_mode = self.waist_drive.state
        # Publish explicit loss as well as valid lock state so the jaw can release.
        await self.pubsub.publish(ServoStateTopic, ServoStateMessage(
            pan_angle=self.pan_angle, tilt_angle=self.tilt_angle, roll_angle=90.0,
            is_locked_x=locked_x, is_locked_y=locked_y, is_locked_roll=False,
        ))

    async def run(self) -> None:
        """Main control loop."""
        waist = self.params.waist
        logger.info(
            f"Starting OrientationNode [head_gain={self.params.head_gain}, "
            f"waist {waist.min_throttle:.2f}-{waist.max_throttle:.2f} from {waist.start_throttle:.2f}, "
            f"ramps +{waist.ramp_up:.1f}/-{waist.ramp_down:.1f} per s]"
        )
        
        try:
            while not self.stop_event.is_set():
                await self.step()

                # Short sleep for responsive control
                await asyncio.sleep(0.01)  # 100Hz update rate
        
        except Exception as e:
            logger.error(f"OrientationNode error: {e}")
            raise
        
        finally:
            # Always stop motors and center servos on exit
            self.waist_motor.throttle = 0.0
            self.servo_kit.servo[self.params.pan_channel].angle = 90.0
            self.servo_kit.servo[self.params.tilt_channel].angle = 90.0
            await asyncio.sleep(0.5)
            logger.info("OrientationNode stopped")
