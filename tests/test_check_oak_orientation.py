"""The OAK orientation check tells a turned view from an unturned one, a few pixels off or not."""
import numpy as np

from scripts.check_oak_orientation import grey, verdict


def scene(seed=3):
    """A 640x480 view with detail at the scale the check compares."""
    return np.kron(np.random.default_rng(seed).uniform(0, 255, (30, 40, 3)), np.ones((16, 16, 1)))


def test_a_view_turned_180_degrees_passes_even_when_the_readout_starts_pixels_off():
    default = scene()
    flipped = np.roll(default[::-1, ::-1], (8, -12), axis=(0, 1))

    assert verdict(grey(default), grey(flipped))["result"] == "turned"


def test_the_default_view_again_fails():
    default = scene()

    assert verdict(grey(default), grey(default.copy()))["result"] == "not turned"


def test_a_blank_view_cannot_tell():
    blank = np.full((480, 640, 3), 128.0)

    assert verdict(grey(blank), grey(blank))["result"] == "inconclusive"
