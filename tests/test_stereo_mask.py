import numpy as np
import pytest

from ultraskelly.core.bot.sensory.stereo_mask import ViewMask, view_mask_file


def test_the_matcher_window_covers_every_seen_pixel_and_its_matches_to_the_left():
    valid = np.zeros((376, 672), bool)
    valid[100:300, 200:450] = True
    rows, columns = ViewMask(valid=valid).window(search_range=64)

    assert (rows.start, rows.stop) == (100, 300)
    assert (columns.start, columns.stop) == (136, 450)  # 64 columns left of the first seen one
    assert ViewMask(valid=np.zeros((4, 4), bool)).window(64) == (slice(0, 0), slice(0, 0))
    valid[:, :] = True
    assert ViewMask(valid=valid).window(64) == (slice(0, 376), slice(0, 672))


def test_the_mask_file_sits_beside_the_cameras_calibration():
    assert view_mask_file("12395566").name == "SN12395566-view-mask.png"
    assert view_mask_file("12395566").parent.name == "zed"


def test_blocked_regions_reaching_a_side_are_filled_to_it_and_small_ones_ignored():
    pytest.importorskip("cv2")
    from ultraskelly.core.bot.sensory.stereo_mask import blocked_regions

    no_match = np.zeros((100, 200), bool)
    no_match[20:80, 150:160] = True  # The edge of an occluder that reaches the right side...
    no_match[20:80, 160:] = True
    no_match[40:44, 60:64] = True  # ...and a speck in the middle of the view
    no_match[10:90, 0:8] = True  # A strip the right eye cannot see, on the left side

    seen = blocked_regions(no_match)

    assert not seen[50, 150:].any() and not seen[50, :8].any()
    assert seen[50, 60] and seen[50, 100]  # The speck is noise; the middle stays seen
    assert seen[5, 180]  # Rows the occluder does not reach stay seen
