"""Brake every DC motor on the motor board. Moves nothing; servos keep their angles.

The motor board keeps driving its last command if the bot exits without stopping its
motors: killed, crashed in native code, or stopped by a reset of its own process. Run
this by hand after such an exit, or from a service's ExecStopPost so it runs after any:

    .venv/bin/python -m scripts.stop_motors

It does not take the robot lock: stopping must work while anything else holds it.
Exits non-zero if a motor could not be stopped.
"""
import logging
import sys
import time

from ultraskelly.core.bot.motor.bridge_motor import BridgeMotor
from ultraskelly.core.bot.motor.simulated_actuators import STOP_ATTEMPTS, stop_board_motors

BOARD_ADDRESS = 0x60
CONNECT_PAUSE = 0.05  # Seconds between attempts to reach the board


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    from adafruit_motorkit import MotorKit

    motors = None
    for attempt in range(1, STOP_ATTEMPTS + 1):
        try:
            kit = MotorKit(address=BOARD_ADDRESS)
            # Each brake sets both bridge inputs in one transfer, so a failing bus cannot leave one on alone.
            motors = {f"motor{number}": BridgeMotor.from_motorkit(kit, number) for number in (1, 2, 3, 4)}
            break
        except Exception as error:  # A disturbed I2C bus fails setup with OSError or, from blinka's reads, TypeError
            logging.warning(f"Motor board not reachable ({type(error).__name__}: {error}); attempt {attempt}")
            time.sleep(CONNECT_PAUSE)
    if motors is None:
        logging.error("Could not reach the motor board; its motors may still be driven")
        return 1
    try:
        stop_board_motors(motors)
    except OSError:
        return 1
    logging.info("All DC motors braked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
