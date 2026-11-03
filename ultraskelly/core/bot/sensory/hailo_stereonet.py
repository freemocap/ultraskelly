"""StereoNet on the Hailo-8: dense disparity from a rectified colour pair.

The Hailo Model Zoo's StereoNet (trained on KITTI driving scenes) takes two 1232x368
RGB images. A band of rows around the image centre is scaled to that size, so only
those rows get depth: for the ZED's 672x376 eyes, rows 93-293 of 376. The network's
disparities, up to 136 of its pixels, are scaled back to the eye image's pixels.
Use the HEF compiled for the installed HailoRT: Model Zoo v2.17 for HailoRT 4.23.
"""
import contextlib
from dataclasses import dataclass

import numpy as np

DEFAULT_HEF = "/home/ultraskelly-rpi5-admin/ultraskelly-models/hailo8-v2.17/stereonet.hef"


@dataclass(frozen=True)
class Band:
    """The rows of an eye image the network sees, and the scale between them."""

    top: int  # First image row
    rows: int  # Image rows in the band
    scale: float  # Network pixels per image pixel, horizontally

    @classmethod
    def centred(cls, eye_size: tuple[int, int], network_size: tuple[int, int], centre_row: float) -> "Band":
        """The band of `network_size` (width, height) proportions centred on `centre_row`."""
        eye_width, eye_height = eye_size
        network_width, network_height = network_size
        scale = network_width / eye_width
        rows = min(eye_height, round(network_height / scale))
        top = int(np.clip(round(centre_row - rows / 2), 0, eye_height - rows))
        return cls(top=top, rows=rows, scale=scale)

    def place(self, band_disparity: np.ndarray, eye_size: tuple[int, int], resize) -> np.ndarray:
        """Network disparities (network pixels, network size) as a full eye-sized image in eye
        pixels; rows outside the band are zero (unknown)."""
        eye_width, eye_height = eye_size
        full = np.zeros((eye_height, eye_width), np.float32)
        full[self.top:self.top + self.rows] = resize(band_disparity, (eye_width, self.rows)) / self.scale
        return full


class HailoStereoNetMatcher:
    """StereoNet on the Hailo-8, behind the depth node's matcher interface."""

    name = "stereonet"
    color = True  # It matches RGB images

    def __init__(self, hef_path: str, eye_size: tuple[int, int], centre_row: float) -> None:
        import cv2
        import hailo_platform as hp

        self._cv2 = cv2
        self.eye_size = eye_size
        # Closing the stack releases the configured model before the device, the order HailoRT needs.
        self.resources = contextlib.ExitStack()
        try:
            device_params = hp.VDevice.create_params()
            device_params.scheduling_algorithm = hp.HailoSchedulingAlgorithm.ROUND_ROBIN
            device = self.resources.enter_context(hp.VDevice(device_params))
            model = device.create_infer_model(hef_path)
            model.set_batch_size(1)
            for stream in model.inputs:
                stream.set_format_type(hp.FormatType.UINT8)
            model.output().set_format_type(hp.FormatType.FLOAT32)
            self.configured = self.resources.enter_context(model.configure())
            height, width, _ = model.inputs[0].shape
            # input_layer1 takes the left image, input_layer2 the right.
            self.inputs = [np.empty(model.input(name).shape, np.uint8) for name in sorted(model.input_names)]
            self.output = np.empty(model.output().shape, np.float32)
            self.bindings = self.configured.create_bindings()
            for name, buffer in zip(sorted(model.input_names), self.inputs):
                self.bindings.input(name).set_buffer(buffer)
            self.bindings.output().set_buffer(self.output)
            quantization = hp.HEF(hef_path).get_output_vstream_infos()[0].quant_info
        except BaseException:
            self.close()
            raise
        self.network_size = (width, height)
        self.band = Band.centred(eye_size, self.network_size, centre_row)
        largest = quantization.qp_scale * (255 - quantization.qp_zp)  # Disparity of the top output code
        self.search_range = largest / self.band.scale

    def disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """Rectified RGB eye images in; disparity in their pixels out, zero outside the band."""
        band = slice(self.band.top, self.band.top + self.band.rows)
        for buffer, image in zip(self.inputs, (left, right)):
            buffer[:] = self._cv2.resize(image[band], self.network_size, interpolation=self._cv2.INTER_LINEAR)
        self.configured.run([self.bindings], 2000)
        resize = lambda image, size: self._cv2.resize(image, size, interpolation=self._cv2.INTER_AREA)
        return self.band.place(self.output[..., 0], self.eye_size, resize)

    def close(self) -> None:
        """Release the model, then the device."""
        if self.resources is not None:
            self.resources.close()
            self.resources = None
            self.bindings = self.configured = None
