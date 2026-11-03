"""Brake the DC motors from a separate thread when the 100 Hz control loop stops ticking.

The motor board keeps driving its last command until it is told otherwise, so a control
loop that stalls mid-move would leave the waist turning: the controllers and their
cleanup all run on the bot's event loop. This watchdog runs on its own thread instead.
Once the orientation controller's first tick arms it, it expects a tick every
`timeout` seconds; when none comes it brakes every DC motor once, through the same
limit-and-record wrappers the controllers use, so the brake shows in recordings. When
ticks resume it logs the recovery, and the waist drive starts its next move from rest.

It is one layer of three. The stall monitor stops the bot when camera frames or pose
results stop; systemd kills a bot whose event loop stops pinging it for 30 s, and the
service then brakes the motors again. A stall caused by code that holds Python's global
interpreter lock also stops this thread; those reach the systemd watchdog.
"""
import logging
import threading
import time
from typing import Callable

logger = logging.getLogger(__name__)


class MotorWatchdog:
    """Call `tick()` from every control loop step; `start()` before the loop and `stop()` after it."""

    def __init__(
        self,
        motors: dict[str, object],
        *,
        timeout: float = 0.5,
        period: float = 0.05,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.motors = motors  # Name -> object with a `throttle` (the controllers' motor wrappers)
        self.timeout = timeout
        self.period = period
        self.clock = clock
        self.last_tick: float | None = None  # Unarmed until the first tick
        self.tripped_at: float | None = None
        self.trips = 0
        self._stopping = threading.Event()
        self._thread: threading.Thread | None = None

    def tick(self) -> None:
        now = self.clock()
        if self.tripped_at is not None:
            stalled = now - self.last_tick if self.last_tick is not None else now - self.tripped_at
            logger.warning(f"Control loop running again after a {stalled:.1f} s stall; motors resume")
            self.tripped_at = None
        self.last_tick = now

    def disarm(self) -> None:
        """Stop expecting ticks, for a controller shutting down on purpose."""
        self.last_tick = None

    def check(self) -> bool:
        """Brake the motors if the loop has gone quiet; True if this call braked them."""
        last = self.last_tick
        if last is None or self.tripped_at is not None or self.clock() - last <= self.timeout:
            return False
        self.tripped_at, self.trips = self.clock(), self.trips + 1
        logger.error(f"Control loop stalled for over {self.timeout:.1f} s: braking the DC motors")
        for name, motor in self.motors.items():
            try:
                motor.throttle = 0.0
            except Exception:
                logger.exception(f"Motor watchdog could not brake {name}")
        return True

    def start(self) -> None:
        self._stopping.clear()
        self._thread = threading.Thread(target=self._watch, name="motor-watchdog", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stopping.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None

    def _watch(self) -> None:
        while not self._stopping.wait(self.period):
            self.check()
