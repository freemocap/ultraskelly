import numpy as np
import pytest

from ultraskelly.core.bot.sensory.hailo_stereonet import Band


def nearest(image, size):
    """A stand-in for cv2.resize: nearest-neighbour resampling to (width, height)."""
    width, height = size
    rows = (np.arange(height) * image.shape[0] / height).astype(int)
    columns = (np.arange(width) * image.shape[1] / width).astype(int)
    return image[rows][:, columns]


def test_the_network_band_is_centred_on_the_principal_row_and_kept_inside_the_image():
    vga = Band.centred((672, 376), (1232, 368), centre_row=193.4)
    assert (vga.top, vga.rows) == (93, 201) and vga.scale == pytest.approx(1232 / 672)
    hd = Band.centred((1280, 720), (1232, 368), centre_row=371.7)
    assert (hd.top, hd.rows) == (181, 382)
    assert Band.centred((672, 376), (1232, 368), centre_row=10).top == 0  # Clamped to the image
    assert Band.centred((672, 376), (1232, 368), centre_row=370).top == 376 - 201


def test_network_disparities_come_back_in_image_pixels_inside_the_band_only():
    band = Band.centred((672, 376), (1232, 368), centre_row=193.4)
    network = np.full((368, 1232), 55.0, np.float32)  # Network pixels

    full = band.place(network, (672, 376), nearest)

    assert full.shape == (376, 672)
    assert full[band.top, 0] == pytest.approx(55.0 / band.scale)  # 30 image pixels
    assert full[band.top - 1].max() == 0 and full[band.top + band.rows].max() == 0
