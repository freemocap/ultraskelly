"""Warnings about a device that stays away, logged less and less often.

The dashboard's health panel shows each device's state live (core/bot/health.py), so while
a camera or the IMU is unplugged the log needs only the first failure and a reminder now
and then, not a line every 30 s all day.
"""
import logging
import time
from typing import Callable


class FadingWarning:
    """Logs the first warning, then one each time a wait that doubles from `first` to `longest`
    seconds has passed. Counts the failures since the device last worked."""

    def __init__(
        self, logger: logging.Logger, first: float = 30.0, longest: float = 600.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.logger, self.first, self.longest, self.clock = logger, first, longest, clock
        self.wait = first
        self.warned_at: float | None = None
        self.failures = 0  # Since the device last worked

    def warn(self, message: str) -> None:
        self.failures += 1
        now = self.clock()
        if self.warned_at is not None and now - self.warned_at < self.wait:
            return
        if self.warned_at is not None:
            self.wait = min(2 * self.wait, self.longest)
        self.warned_at = now
        self.logger.warning(message)

    def worked(self) -> int:
        """The device works again: how many failures came since it last did. The waits carry on, so
        a device that comes and goes keeps the log quiet too."""
        failures, self.failures = self.failures, 0
        return failures
