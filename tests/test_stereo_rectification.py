import numpy as np
import pytest

from ultraskelly.core.bot.sensory.stereo_rectification import (
    CALIBRATION_FOLDER,
    StereoRectification,
    calibration_file,
    find_zed_serial,
    read_zed_calibration,
)

CALIBRATION = (CALIBRATION_FOLDER / "SN12395566.conf").read_text()

SYNTHETIC = """
[LEFT_CAM_VGA]
fx=350
fy=350
cx=336
cy=188
k1=0
k2=0
p1=0
p2=0
k3=0

[RIGHT_CAM_VGA]
fx=350
fy=350
cx=336
cy=193
k1=0
k2=0
p1=0
p2=0
k3=0

[STEREO]
Baseline=63
TY=0
TZ=0
CV_VGA=0
RX_VGA=0
RZ_VGA=0
"""


def test_the_factory_calibration_is_read_per_resolution():
    vga = read_zed_calibration(CALIBRATION, (672, 376))
    hd = read_zed_calibration(CALIBRATION, (1280, 720))

    assert vga.left.camera_matrix[0, 0] == pytest.approx(349.95)
    assert vga.right.camera_matrix[1, 2] == pytest.approx(189.0225)
    assert hd.left.camera_matrix[0, 2] == pytest.approx(659.9)
    assert hd.left.distortion.tolist() == pytest.approx([-0.171428, 0.0258856, 8.29864e-05, 0.000145668, 3.81004e-10])
    assert vga.translation.tolist() == pytest.approx([63.0072, 0.00855499, -0.0438343])
    assert vga.rotation.tolist() == pytest.approx([0.000771589, 0.00874855, -0.000690209])  # RX, CV, RZ
    with pytest.raises(ValueError, match="640-pixel-wide"):
        read_zed_calibration(CALIBRATION, (640, 480))


def test_the_serial_number_comes_from_the_zeds_usb_device(tmp_path):
    def device(name: str, vendor: str, serial: str | None) -> None:
        folder = tmp_path / name
        folder.mkdir()
        (folder / "idVendor").write_text(vendor + "\n")
        if serial is not None:
            (folder / "serial").write_text(serial + "\n")

    device("1-2", "1915", "C0FFEE")
    device("2-1", "2b03", "")  # The video interface reports no serial
    assert find_zed_serial(tmp_path) is None
    device("1-1", "2b03", "12395566")  # Its HID interface does
    assert find_zed_serial(tmp_path) == "12395566"
    assert calibration_file("12395566").name == "SN12395566.conf"
    with pytest.raises(FileNotFoundError, match=r"calib\.stereolabs\.com/\?SN=1"):
        calibration_file("1", folder=tmp_path)


def test_depth_follows_from_disparity():
    shared = StereoRectification(width=672, height=376, focal_length=340.0, cx=331.0, cy=193.0, baseline=0.063,
                                 left_rotation=((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    assert shared.depth(np.array([21.42, 0.0, -3.0])).tolist() == pytest.approx([1.0, np.inf, np.inf], rel=1e-3)


def test_rectified_points_share_rows_and_give_their_depth():
    pytest.importorskip("cv2")
    from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectifier

    rectifier = StereoRectifier(read_zed_calibration(SYNTHETIC, (672, 376)))
    shared = rectifier.rectification
    # A point 2 m ahead: in the right eye, 63 mm further left and 5 px lower (its principal point).
    disparity = 350 * 0.063 / 2.0
    left = rectifier.rectify_points(0, [[400.0, 150.0]])[0]
    right = rectifier.rectify_points(1, [[400.0 - disparity, 155.0]])[0]

    assert left[1] == pytest.approx(right[1], abs=0.01)
    assert shared.depth(left[0] - right[0]) == pytest.approx(2.0, rel=0.01)
    assert shared.baseline == pytest.approx(0.063)
    image = np.zeros((376, 672, 3), np.uint8)
    image[150, 400] = 255
    rectified_left, rectified_right = rectifier.rectify(image, image)
    assert rectified_left.shape == rectified_right.shape == image.shape
