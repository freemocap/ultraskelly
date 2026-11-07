"""Plants that differ from the fit within its measured uncertainty, to check a tuning is robust.

The fitted plant (plant.py) comes from one recording; the robot differs from it, and from day to day.
`UNCERTAINTY` spans what the measurements leave open (docs/simulator.md, docs/audit-2026.md):

- head gearing: pan 0.26-0.36 between runs, tilt 0.07-0.12, and 0.050-0.083 roll;
- every servo's dead time, and the camera's delay (50-70 ms measured, 100 ms seen under load);
- slack, measured once per servo;
- pan's 1.9 Hz mode and its damping (0.41 one way and 0.28 the other, fitted);
- the waist: six pulses measured, so breakaway, stall, both speeds, how fast it reaches speed and
  how fast it stops are each known to a fifth or so;
- pixels per degree: the OAK's 69 degree field of view is its datasheet's, unmeasured;
- the detector's jitter, about a pixel.

`draw` samples one plant uniformly within those ranges; `plants(count, seed)` gives a repeatable set. With
59 draws all passing, a tuning fails on under 5 % of such plants with 95 % confidence (0.95 ** 59 < 0.05).
`CORNERS` are the extremes the head's aim must survive: gearing and pixels per degree 15 % either way.
"""
import math
import random

from ultraskelly.simulator import plant

UNCERTAINTY = {  # Simulator setting: (low, high)
    "pan_ratio": (plant.PAN.gain * 0.85, plant.PAN.gain * 1.15),
    "tilt_ratio": (plant.TILT.gain * 0.75, plant.TILT.gain * 1.25),
    "delay_scale": (0.75, 1.5),
    "camera_latency": (0.045, 0.10),
    "pan_backlash": (1.5, 3.5),
    "tilt_backlash": (1.0, 3.0),
    "pan_frequency": (plant.PAN.frequency * 0.85, plant.PAN.frequency * 1.15),
    "pan_damping": (0.30, 0.50),
    "pan_damping_down": (0.20, 0.36),
    "camera_hfov": (64.0, 74.0),
    "noise_px": (0.0, 1.0),
    "waist_breakaway": (0.58, 0.72),
    "waist_stall": (0.38, 0.52),
    "waist_speed": (plant.WAIST.speed_negative * 0.8, plant.WAIST.speed_negative * 1.2),
    "waist_speed_positive": (plant.WAIST.speed_positive * 0.8, plant.WAIST.speed_positive * 1.2),
    "waist_time_constant": (0.10, 0.25),
    "waist_brake_time": (0.02, 0.06),
    "waist_coast_time": (0.2, 0.45),
}


def hfov_for(px_scale: float, nominal: float = 69.0) -> float:
    """The field of view whose pixels per degree at the middle are `px_scale` times the nominal one's."""
    focal = 320.0 / math.tan(math.radians(nominal / 2))
    return 2.0 * math.degrees(math.atan(320.0 / (focal * px_scale)))


CORNERS = {  # Name: simulator settings
    f"pan gearing {gear:+.0%}, pixels per degree {pixels:+.0%}": {
        "pan_ratio": plant.PAN.gain * (1 + gear), "camera_hfov": hfov_for(1 + pixels)}
    for gear in (-0.15, 0.15) for pixels in (-0.15, 0.15)
}


def draw(rng: random.Random) -> dict:
    """One plant within the measured uncertainty, as simulator settings."""
    settings = {key: rng.uniform(low, high) for key, (low, high) in UNCERTAINTY.items()}
    settings["waist_stall"] = min(settings["waist_stall"], settings["waist_breakaway"] - 0.05)
    return settings


def plants(count: int, seed: int) -> list[dict]:
    """`count` plants drawn from `seed`: the same every time."""
    rng = random.Random(seed)
    return [draw(rng) for _ in range(count)]
