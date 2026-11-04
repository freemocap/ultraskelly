"""Warnings about a device that stays away come less and less often."""
import logging

from ultraskelly.core.bot.sensory.fading_warning import FadingWarning


def test_the_first_failure_is_logged_then_ever_more_rarely(caplog):
    now = [0.0]
    warning = FadingWarning(logging.getLogger("test"), first=30.0, longest=120.0, clock=lambda: now[0])
    logged_at = []
    with caplog.at_level(logging.WARNING, logger="test"):
        for second in range(0, 600):  # A try a second for ten minutes
            now[0] = float(second)
            before = len(caplog.records)
            warning.warn(f"no camera ({second} s)")
            if len(caplog.records) > before:
                logged_at.append(second)

    assert logged_at == [0, 30, 90, 210, 330, 450, 570]  # Waits of 30, 60, then 120 s at most
    assert warning.worked() == 600 and warning.worked() == 0
