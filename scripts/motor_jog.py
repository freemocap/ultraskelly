"""Move one actuator a small, known amount to check wiring and direction (MOVES REAL MOTORS).

Run on the Pi with the robot ready for motion and someone watching:

    .venv/bin/python -m scripts.motor_jog pan     # head pan: 90 -> 80 -> 100 -> 90 degrees
    .venv/bin/python -m scripts.motor_jog tilt    # head tilt: 90 -> 80 -> 100 -> 90 degrees
    .venv/bin/python -m scripts.motor_jog waist   # waist: 0.3 s forward, then 0.3 s reverse
    .venv/bin/python -m scripts.motor_jog mouth   # jaw: one normal open pulse, spring closes it
    .venv/bin/python -m scripts.motor_jog all

Channels and powers come from the bot's own defaults, so this checks the wiring the
bot uses. DC motors are always stopped on exit, including on Ctrl+C.
"""
import argparse
import time

from ultraskelly.core.bot.motor.mouth_node import MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import stop_board_motors

SERVO_STEPS = (90.0, 80.0, 100.0, 90.0)  # Small sweep around center
WAIST_SECONDS = 0.3
PAUSE_SECONDS = 0.8


def jog_servo(servo_kit, name: str, channel: int) -> None:
    for angle in SERVO_STEPS:
        print(f"{name} (servo channel {channel}) -> {angle:.0f} degrees", flush=True)
        servo_kit.servo[channel].angle = angle
        time.sleep(PAUSE_SECONDS)


def pulse(motor, name: str, throttle: float, seconds: float) -> None:
    print(f"{name} throttle {throttle:+.2f} for {seconds:.2f} s", flush=True)
    try:
        motor.throttle = throttle
        time.sleep(seconds)
    finally:
        motor.throttle = 0.0
    time.sleep(PAUSE_SECONDS)


def main() -> None:
    parser = argparse.ArgumentParser(description="Move one actuator a small amount (moves real motors).")
    parser.add_argument("actuator", choices=["pan", "tilt", "waist", "mouth", "all"])
    args = parser.parse_args()
    orientation, mouth = OrientationNodeParams(), MouthNodeParams()
    selected = ["pan", "tilt", "waist", "mouth"] if args.actuator == "all" else [args.actuator]

    from adafruit_motorkit import MotorKit
    from adafruit_servokit import ServoKit

    servo_kit = ServoKit(channels=16)
    motor_kit = MotorKit(address=0x60, pwm_frequency=1600)
    motors = {number: getattr(motor_kit, f"motor{number}") for number in (1, 2, 3, 4)}
    waist, jaw = motors[orientation.waist_motor_channel], motors[mouth.mouth_motor_channel]
    try:
        for actuator in selected:
            if actuator == "pan":
                jog_servo(servo_kit, "pan", orientation.pan_channel)
            elif actuator == "tilt":
                jog_servo(servo_kit, "tilt", orientation.tilt_channel)
            elif actuator == "waist":
                name = f"waist (motor M{orientation.waist_motor_channel})"
                pulse(waist, name, +orientation.waist.max_throttle, WAIST_SECONDS)
                pulse(waist, name, -orientation.waist.max_throttle, WAIST_SECONDS)
            elif actuator == "mouth":
                # The same pulse the bot uses to open the jaw; a spring closes it.
                pulse(jaw, f"jaw (motor M{mouth.mouth_motor_channel})", -1.0, mouth.opening_duration)
    finally:
        stop_board_motors({f"motor{number}": motor for number, motor in motors.items()})
        print("All DC motors stopped", flush=True)


if __name__ == "__main__":
    main()
