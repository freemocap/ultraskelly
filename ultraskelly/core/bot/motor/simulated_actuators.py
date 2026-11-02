"""Memory-only stand-ins for the servo and DC motor boards that record commanded values.

They replace the hardware factories' ServoKit and MotorKit objects, so the production
controllers run unchanged without touching I2C. Recorded values are commands, not
measured or simulated joint positions.
"""
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNode, OrientationNodeParams
from ultraskelly.core.pubsub.bot_topics import ServoStateTopic, TargetLocationTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

DEFAULT_HISTORY_SIZE = 10_000


@dataclass(frozen=True)
class ActuatorCommand:
    """One change in an actuator's commanded value."""
    actuator: str
    value: float | None
    at: float


class CommandRecorder:
    """Latest command per actuator plus a bounded history of changes, oldest first."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        history_size: int = DEFAULT_HISTORY_SIZE,
    ) -> None:
        self.clock = clock
        self.latest: dict[str, ActuatorCommand] = {}
        self.history: deque[ActuatorCommand] = deque(maxlen=history_size)
        self.change_count = 0

    def record(self, actuator: str, value: float | None) -> None:
        """Store a command unless it repeats that actuator's previous command."""
        previous = self.latest.get(actuator)
        if previous is not None and previous.value == value:
            return
        command = ActuatorCommand(actuator=actuator, value=value, at=self.clock())
        self.latest[actuator] = command
        self.history.append(command)
        self.change_count += 1


class SimulatedServo:
    """Stands in for one ServoKit channel; `angle` is the last commanded angle."""

    def __init__(self, *, name: str, recorder: CommandRecorder) -> None:
        self.name = name
        self.recorder = recorder
        self._angle: float | None = None

    @property
    def angle(self) -> float | None:
        return self._angle

    @angle.setter
    def angle(self, value: float | None) -> None:
        self._angle = value
        self.recorder.record(self.name, value)


class SimulatedServoKit:
    """Stands in for `ServoKit(channels=16)`. Channels are named by role where given, else servoN."""

    def __init__(
        self,
        *,
        recorder: CommandRecorder,
        names: dict[int, str] | None = None,
        channels: int = 16,
    ) -> None:
        names = names or {}
        self.servo = [
            SimulatedServo(name=names.get(channel, f"servo{channel}"), recorder=recorder)
            for channel in range(channels)
        ]


class SimulatedMotor:
    """Stands in for one MotorKit DC motor; `throttle` is the last commanded throttle."""

    def __init__(self, *, name: str, recorder: CommandRecorder) -> None:
        self.name = name
        self.recorder = recorder
        self._throttle: float | None = 0.0  # Idle until commanded; not itself a command

    @property
    def throttle(self) -> float | None:
        return self._throttle

    @throttle.setter
    def throttle(self, value: float | None) -> None:
        self._throttle = value
        self.recorder.record(self.name, value)


def create_simulated_motor_nodes(
    *,
    pubsub: PubSubTopicManager,
    orientation_params: OrientationNodeParams,
    mouth_params: MouthNodeParams,
    recorder: CommandRecorder,
) -> tuple[OrientationNode, MouthNode]:
    """Build both controllers around simulated actuators, mirroring their hardware factories.

    Like `OrientationNode.create`, the head is commanded to its starting angles. Unlike
    `MouthNode.create`, there is no blocking jaw drive test.
    """
    servo_kit = SimulatedServoKit(
        recorder=recorder,
        names={orientation_params.pan_channel: "pan", orientation_params.tilt_channel: "tilt"},
    )
    orientation = OrientationNode(
        pubsub=pubsub,
        params=orientation_params,
        servo_kit=servo_kit,
        waist_motor=SimulatedMotor(name="waist", recorder=recorder),
        target_subscription=pubsub.get_subscription(TargetLocationTopic),
    )
    servo_kit.servo[orientation_params.pan_channel].angle = orientation.pan_angle
    servo_kit.servo[orientation_params.tilt_channel].angle = orientation.tilt_angle

    mouth = MouthNode(
        pubsub=pubsub,
        params=mouth_params,
        mouth_motor=SimulatedMotor(name="mouth", recorder=recorder),
        head_servo_subscription=pubsub.get_subscription(ServoStateTopic),
    )
    return orientation, mouth
