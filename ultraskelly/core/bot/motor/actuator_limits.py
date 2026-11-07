"""Hard limits on every command sent to the motor boards.

The servo and motor wrappers clamp each command to these before it reaches a board,
so no controller, test script or future panel can exceed them. The defaults equal the
controllers' existing limits; tighten them once the assembled robot's physical travel
has been measured.
"""
import logging

from pydantic import Field, model_validator

from ultraskelly.core.bot.base_abcs import NodeParams

logger = logging.getLogger(__name__)


class ActuatorLimits(NodeParams):
    """Allowed servo angles (degrees) and largest DC motor throttle magnitudes."""

    pan_min_degrees: float = Field(default=0.0, ge=0.0, le=180.0)
    pan_max_degrees: float = Field(default=180.0, ge=0.0, le=180.0)
    tilt_min_degrees: float = Field(default=0.0, ge=0.0, le=180.0)
    tilt_max_degrees: float = Field(default=180.0, ge=0.0, le=180.0)
    # Levelling turns roll at most `span` (25) degrees from upright (roll_level.py); motor tests may send
    # it anywhere in 0-180, as pan and tilt (the owner, 2026-11-07).
    roll_min_degrees: float = Field(default=0.0, ge=0.0, le=180.0)
    roll_max_degrees: float = Field(default=180.0, ge=0.0, le=180.0)
    waist_max_throttle: float = Field(default=0.8, ge=0.0, le=1.0)
    mouth_max_throttle: float = Field(default=1.0, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def _ranges_are_ordered(self) -> "ActuatorLimits":
        if (self.pan_min_degrees > self.pan_max_degrees or self.tilt_min_degrees > self.tilt_max_degrees
                or self.roll_min_degrees > self.roll_max_degrees):
            raise ValueError("Each servo's minimum angle must not exceed its maximum")
        return self

    def servo_range(self, name: str) -> tuple[float, float] | None:
        """Allowed angles for the pan, tilt or roll servo; None for other channels."""
        return {
            "pan": (self.pan_min_degrees, self.pan_max_degrees),
            "tilt": (self.tilt_min_degrees, self.tilt_max_degrees),
            "roll": (self.roll_min_degrees, self.roll_max_degrees),
        }.get(name)

    def max_throttle(self, name: str) -> float | None:
        """Largest throttle magnitude for the waist or mouth motor; None for others."""
        return {"waist": self.waist_max_throttle, "mouth": self.mouth_max_throttle}.get(name)


_warned: set[str] = set()


def clamp(name: str, value: float | None, low: float, high: float) -> float | None:
    """Clamp a command into [low, high]. None (release a servo, coast a motor) passes through.

    The first clamp per actuator is logged so a misbehaving controller is visible.
    """
    if value is None:
        return None
    clamped = min(max(value, low), high)
    if clamped != value and name not in _warned:
        _warned.add(name)
        logger.warning(f"{name} command {value:.3f} clamped to {clamped:.3f}; later clamps are not logged")
    return clamped
