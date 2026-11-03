import asyncio
import time
import logging
from enum import Enum
from typing import Callable

from pydantic import Field, SkipValidation

from ultraskelly.core.bot.base_abcs import NodeParams, Node
from ultraskelly.core.bot.motor.bridge_motor import BridgeMotor
from ultraskelly.core.pubsub.bot_topics import ServoStateTopic, ServoStateMessage
from ultraskelly.core.pubsub.pubsub_abcs import TopicSubscriptionQueue
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

logger = logging.getLogger(__name__)


class MouthState(Enum):
    CLOSED = "closed"
    OPENING = "opening"
    OPEN = "open"
    CLOSING = "closing"


class MouthNodeParams(NodeParams):
    mouth_motor_channel: int = Field(default=2, ge=1, le=4)
    lock_timeout: float = Field(default=0.35, gt=0)
    opening_duration: float = Field(default=0.2, gt=0)
    spring_return_duration: float = Field(default=0.32, gt=0)
    startup_test: bool = Field(
        default=False,
        description="Drive the jaw each way once at startup (full power, within the actuator limits)",
    )


class MouthNode(Node):
    """Mouth controller that slowly closes after opening."""

    params: MouthNodeParams
    clock: Callable[[], float] = Field(default_factory=lambda: time.monotonic, exclude=True)

    motor_kit: SkipValidation[object] = Field(default=None, exclude=True)
    mouth_motor: SkipValidation[object] = Field(default=None, exclude=True)
    head_servo_subscription: SkipValidation[TopicSubscriptionQueue] = Field(default=None, exclude=True)

    # State tracking
    mouth_state: MouthState = Field(default=MouthState.CLOSED)
    state_start_time: float = Field(default=0.0)
    current_throttle: float = Field(default=0.0)
    last_lock: bool = False
    lock_received_at: float | None = None
    manual: bool = Field(default=False, description="True while a motor test drives the actuators; the jaw controller commands nothing")

    @classmethod
    def create(cls, *, pubsub: PubSubTopicManager, params: MouthNodeParams, pwm_frequency: float = 1600.0) -> "MouthNode":
        """Factory method to create and initialize MouthNode.

        `pwm_frequency` is the motor board's, shared with the waist: pass the waist's.
        """
        from adafruit_motorkit import MotorKit

        motor_kit = MotorKit(address=0x60, pwm_frequency=pwm_frequency)
        # Both bridge inputs set in one I2C transfer (see bridge_motor.py)
        mouth_motor = BridgeMotor.from_motorkit(motor_kit, params.mouth_motor_channel)

        head_servo_subscription = pubsub.get_subscription(ServoStateTopic)

        return cls(
            pubsub=pubsub,
            params=params,
            motor_kit=motor_kit,
            mouth_motor=mouth_motor,
            head_servo_subscription=head_servo_subscription
        )

    def run_startup_test(self) -> None:
        """Drive the jaw one way, then the other, then stop; blocks for one second.

        Opt-in through `MouthNodeParams.startup_test`. Run it after the motor is wrapped
        so the actuator limits apply.
        """
        logger.info("Testing mouth motor...")
        try:
            self.mouth_motor.throttle = 1.0
            time.sleep(0.5)
            self.mouth_motor.throttle = -1.0
            time.sleep(0.5)
        finally:
            self.mouth_motor.throttle = 0.0

    def _update_mouth_state(self, *, is_locked: bool) -> None:
        """Energize while locked; release immediately and let the spring close."""
        now = self.clock()
        if is_locked:
            if self.mouth_state not in (MouthState.OPENING, MouthState.OPEN):
                self.mouth_state = MouthState.OPENING
                self.state_start_time = now
            if now - self.state_start_time >= self.params.opening_duration:
                self.mouth_state = MouthState.OPEN
            self.current_throttle = -1.0
        else:
            if self.mouth_state not in (MouthState.CLOSING, MouthState.CLOSED):
                self.mouth_state = MouthState.CLOSING
                self.state_start_time = now
            if now - self.state_start_time >= self.params.spring_return_duration:
                self.mouth_state = MouthState.CLOSED
            self.current_throttle = 0.0

    async def step(self) -> None:
        while not self.head_servo_subscription.empty():
            msg: ServoStateMessage = await self.head_servo_subscription.get()
            self.last_lock = msg.is_locked_x and msg.is_locked_y
            self.lock_received_at = self.clock()
        if self.manual:
            return
        is_locked = (self.last_lock and self.lock_received_at is not None
                     and self.clock() - self.lock_received_at <= self.params.lock_timeout)
        self._update_mouth_state(is_locked=is_locked)
        self.mouth_motor.throttle = self.current_throttle

    async def run(self) -> None:
        """Main motor control loop."""
        logger.info("Starting spring-return MouthNode")

        try:
            while not self.stop_event.is_set():
                await asyncio.sleep(0.01)

                await self.step()

        finally:
            self.mouth_motor.throttle = 0.0
            await asyncio.sleep(0.5)
            logger.info('MouthNode stopped.')
