"""The head and jaw controllers on whichever motor boards answer.

The servo board (PCA9685, I2C 0x40) moves the head's pan and tilt servos and holds its roll
servo level; the Motor HAT
(0x60) drives the waist and the jaw. Each board is opened on its own, so a missing one
takes away only its actuators: with the Motor HAT alone the head stays still, with the
servo board alone the waist and jaw do. A missing board's actuators are stand-ins that
record what the controllers command and send nothing, so no command ever goes to a board
that is not there. Mirrors the controllers' hardware factories (OrientationNode.create and
MouthNode.create), which open both boards and fail if either is missing; here the jaw and
the waist share one Motor HAT object, as they share the board.
"""
from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
from ultraskelly.core.bot.motor.bridge_motor import FAST_DECAY, SLOW_DECAY, BridgeMotor
from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNode, OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import (
    CommandRecorder,
    RecordingMotor,
    RecordingServoKit,
    SimulatedMotor,
    SimulatedServoKit,
)
from ultraskelly.core.pubsub.bot_topics import OakResultsTopic, ServoStateTopic, TargetLocationTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

SERVO_BOARD_ADDRESS = 0x40
MOTOR_BOARD_ADDRESS = 0x60


def open_servo_board() -> object:
    """The PCA9685 servo board, as ServoKit; raises ValueError ("No I2C device at address") if it does not answer."""
    from adafruit_servokit import ServoKit

    return ServoKit(channels=16, address=SERVO_BOARD_ADDRESS)


def open_motor_board(pwm_frequency: float) -> object:
    """The Motor HAT, as MotorKit, at one PWM frequency for all its motors."""
    from adafruit_motorkit import MotorKit

    return MotorKit(address=MOTOR_BOARD_ADDRESS, pwm_frequency=pwm_frequency)


def build_motor_nodes(
    *,
    pubsub: PubSubTopicManager,
    orientation_params: OrientationNodeParams,
    mouth_params: MouthNodeParams,
    recorder: CommandRecorder,
    limits: ActuatorLimits,
    servo_kit: object | None,
    motor_kit: object | None,
) -> tuple[OrientationNode, MouthNode]:
    """Both controllers: a board's actuators go through the limit-and-record wrappers to it, or to
    recording stand-ins when it is None. The head is commanded to its starting angles, and the
    jaw's opt-in startup test runs only on a real Motor HAT, as in the hardware factories."""
    names = orientation_params.servo_names
    if servo_kit is not None:
        servos = RecordingServoKit(kit=servo_kit, recorder=recorder, names=names, limits=limits)
    else:
        servos = SimulatedServoKit(recorder=recorder, names=names, limits=limits)
    if motor_kit is not None:
        waist_bridge = BridgeMotor.from_motorkit(motor_kit, orientation_params.waist_motor_channel)
        # Braking between PWM pulses: quieter and gentler at low drive (see waist_drive.py)
        waist_bridge.decay_mode = SLOW_DECAY if orientation_params.waist.slow_decay else FAST_DECAY
        waist = RecordingMotor(motor=waist_bridge, name="waist", recorder=recorder,
                               max_throttle=limits.waist_max_throttle)
        jaw = RecordingMotor(motor=BridgeMotor.from_motorkit(motor_kit, mouth_params.mouth_motor_channel),
                             name="mouth", recorder=recorder, max_throttle=limits.mouth_max_throttle)
    else:
        waist = SimulatedMotor(name="waist", recorder=recorder, max_throttle=limits.waist_max_throttle)
        jaw = SimulatedMotor(name="mouth", recorder=recorder, max_throttle=limits.mouth_max_throttle)

    orientation = OrientationNode(
        pubsub=pubsub,
        params=orientation_params,
        servo_kit=servos,
        motor_kit=motor_kit,
        waist_motor=waist,
        target_subscription=pubsub.get_subscription(TargetLocationTopic),
        faces_subscription=pubsub.get_subscription(OakResultsTopic),
    )
    orientation.command_start_angles()
    mouth = MouthNode(
        pubsub=pubsub,
        params=mouth_params,
        motor_kit=motor_kit,
        mouth_motor=jaw,
        head_servo_subscription=pubsub.get_subscription(ServoStateTopic),
    )
    recorder.record("mouth", 0.0)
    if mouth_params.startup_test and motor_kit is not None:
        mouth.run_startup_test()
    return orientation, mouth
