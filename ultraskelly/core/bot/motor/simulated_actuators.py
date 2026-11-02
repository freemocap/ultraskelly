"""Record commanded servo angles and motor throttles, with or without the motor boards.

The simulated stand-ins replace the hardware factories' ServoKit and MotorKit objects,
so the production controllers run unchanged without touching I2C. The recording
wrappers sit in front of the real boards instead. Both clamp every command to the
actuator limits before it is forwarded or recorded. Recorded values are commands,
not measured or simulated joint positions.
"""
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits, clamp
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


def _limit_angle(name: str, value: float | None, angle_range: tuple[float, float] | None) -> float | None:
    return value if angle_range is None else clamp(name, value, *angle_range)


def _limit_throttle(name: str, value: float | None, max_throttle: float | None) -> float | None:
    return value if max_throttle is None else clamp(name, value, -max_throttle, max_throttle)


class SimulatedServo:
    """Stands in for one ServoKit channel; `angle` is the last commanded angle."""

    def __init__(
        self, *, name: str, recorder: CommandRecorder, angle_range: tuple[float, float] | None = None,
    ) -> None:
        self.name = name
        self.recorder = recorder
        self.angle_range = angle_range
        self._angle: float | None = None

    @property
    def angle(self) -> float | None:
        return self._angle

    @angle.setter
    def angle(self, value: float | None) -> None:
        value = _limit_angle(self.name, value, self.angle_range)
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
        limits: ActuatorLimits | None = None,
    ) -> None:
        names = names or {}
        self.servo = []
        for channel in range(channels):
            name = names.get(channel, f"servo{channel}")
            angle_range = limits.servo_range(name) if limits else None
            self.servo.append(SimulatedServo(name=name, recorder=recorder, angle_range=angle_range))


class SimulatedMotor:
    """Stands in for one MotorKit DC motor; `throttle` is the last commanded throttle."""

    def __init__(self, *, name: str, recorder: CommandRecorder, max_throttle: float | None = None) -> None:
        self.name = name
        self.recorder = recorder
        self.max_throttle = max_throttle
        self._throttle: float | None = 0.0  # Idle until commanded; not itself a command

    @property
    def throttle(self) -> float | None:
        return self._throttle

    @throttle.setter
    def throttle(self, value: float | None) -> None:
        value = _limit_throttle(self.name, value, self.max_throttle)
        self._throttle = value
        self.recorder.record(self.name, value)


class RecordingServo:
    """Wraps one hardware servo channel: clamps each commanded angle, forwards it, then records it.

    Reading `angle` returns the last command without querying the board over I2C.
    """

    def __init__(
        self,
        *,
        servo: object,
        name: str,
        recorder: CommandRecorder,
        angle_range: tuple[float, float] | None = None,
    ) -> None:
        self._servo = servo
        self.name = name
        self.recorder = recorder
        self.angle_range = angle_range
        self._commanded: float | None = None

    @property
    def angle(self) -> float | None:
        return self._commanded

    @angle.setter
    def angle(self, value: float | None) -> None:
        value = _limit_angle(self.name, value, self.angle_range)
        self._servo.angle = value
        self._commanded = value
        self.recorder.record(self.name, value)


class _RecordingServoChannels:
    """Indexable like `ServoKit.servo`; wraps each channel on first use."""

    def __init__(
        self, *, kit: object, recorder: CommandRecorder, names: dict[int, str], limits: ActuatorLimits | None,
    ) -> None:
        self._kit = kit
        self._recorder = recorder
        self._names = names
        self._limits = limits
        self._wrapped: dict[int, RecordingServo] = {}

    def __getitem__(self, channel: int) -> RecordingServo:
        if channel not in self._wrapped:
            name = self._names.get(channel, f"servo{channel}")
            self._wrapped[channel] = RecordingServo(
                servo=self._kit.servo[channel],
                name=name,
                recorder=self._recorder,
                angle_range=self._limits.servo_range(name) if self._limits else None,
            )
        return self._wrapped[channel]


class RecordingServoKit:
    """Wraps a hardware ServoKit so every commanded angle is clamped and recorded on its way to the board."""

    def __init__(
        self,
        *,
        kit: object,
        recorder: CommandRecorder,
        names: dict[int, str] | None = None,
        limits: ActuatorLimits | None = None,
    ) -> None:
        self.servo = _RecordingServoChannels(kit=kit, recorder=recorder, names=names or {}, limits=limits)


class RecordingMotor:
    """Wraps a hardware DC motor: clamps each commanded throttle, forwards it, then records it.

    Reading `throttle` returns the last command without querying the board over I2C.
    """

    def __init__(
        self, *, motor: object, name: str, recorder: CommandRecorder, max_throttle: float | None = None,
    ) -> None:
        self._motor = motor
        self.name = name
        self.recorder = recorder
        self.max_throttle = max_throttle
        self._commanded: float | None = None

    @property
    def throttle(self) -> float | None:
        return self._commanded

    @throttle.setter
    def throttle(self, value: float | None) -> None:
        value = _limit_throttle(self.name, value, self.max_throttle)
        self._motor.throttle = value
        self._commanded = value
        self.recorder.record(self.name, value)


def record_hardware_commands(
    *,
    orientation: OrientationNode,
    mouth: MouthNode,
    recorder: CommandRecorder,
    limits: ActuatorLimits | None = None,
) -> None:
    """Route both controllers' later hardware commands through the limits and the recorder.

    Call after their hardware factories ran; it records the head's starting angles,
    the only command those factories send.
    """
    limits = limits or ActuatorLimits()
    params = orientation.params
    orientation.servo_kit = RecordingServoKit(
        kit=orientation.servo_kit,
        recorder=recorder,
        names={params.pan_channel: "pan", params.tilt_channel: "tilt"},
        limits=limits,
    )
    orientation.waist_motor = RecordingMotor(
        motor=orientation.waist_motor, name="waist", recorder=recorder, max_throttle=limits.waist_max_throttle,
    )
    mouth.mouth_motor = RecordingMotor(
        motor=mouth.mouth_motor, name="mouth", recorder=recorder, max_throttle=limits.mouth_max_throttle,
    )
    recorder.record("pan", orientation.pan_angle)
    recorder.record("tilt", orientation.tilt_angle)
    recorder.record("mouth", 0.0)


def create_simulated_motor_nodes(
    *,
    pubsub: PubSubTopicManager,
    orientation_params: OrientationNodeParams,
    mouth_params: MouthNodeParams,
    recorder: CommandRecorder,
    limits: ActuatorLimits | None = None,
) -> tuple[OrientationNode, MouthNode]:
    """Build both controllers around simulated actuators, mirroring their hardware factories.

    Like `OrientationNode.create`, the head is commanded to its starting angles. The jaw
    test runs only when `mouth_params.startup_test` asks for it, as on hardware.
    """
    limits = limits or ActuatorLimits()
    servo_kit = SimulatedServoKit(
        recorder=recorder,
        names={orientation_params.pan_channel: "pan", orientation_params.tilt_channel: "tilt"},
        limits=limits,
    )
    orientation = OrientationNode(
        pubsub=pubsub,
        params=orientation_params,
        servo_kit=servo_kit,
        waist_motor=SimulatedMotor(name="waist", recorder=recorder, max_throttle=limits.waist_max_throttle),
        target_subscription=pubsub.get_subscription(TargetLocationTopic),
    )
    servo_kit.servo[orientation_params.pan_channel].angle = orientation.pan_angle
    servo_kit.servo[orientation_params.tilt_channel].angle = orientation.tilt_angle

    mouth = MouthNode(
        pubsub=pubsub,
        params=mouth_params,
        mouth_motor=SimulatedMotor(name="mouth", recorder=recorder, max_throttle=limits.mouth_max_throttle),
        head_servo_subscription=pubsub.get_subscription(ServoStateTopic),
    )
    if mouth_params.startup_test:
        mouth.run_startup_test()
    return orientation, mouth
