"""Compare depth from OpenCV's matcher on the CPU with StereoNet on the Hailo-8, on the same pairs.

Uses the Hailo, so run it under the robot lock (the ZED itself needs none):

    flock -n /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.stereo_compare --save ~/stereo-compare

For each pair it times both matchers and reports how many pixels each gives a depth,
and where both do, how far apart their depths are. StereoNet runs a second time with
its two inputs swapped: the order that gives depth like the CPU's is the right one.
`--save` writes, for the last pair, the rectified left image beside both depth images.
"""
import argparse
import asyncio
import time
from pathlib import Path

import numpy as np

from scripts.zed_row_alignment import capture_pairs
from ultraskelly.core.bot.sensory.hailo_stereonet import DEFAULT_HEF, HailoStereoNetMatcher
from ultraskelly.core.bot.sensory.stereo_depth_node import SgbmMatcher, depth_millimetres, rgb_to_grey_with_opencv
from ultraskelly.core.bot.sensory.stereo_rectification import zed_rectifier
from ultraskelly.core.bot.sensory.zed_camera_node import ZedResolution

MIN_DEPTH, MAX_DEPTH = 0.3, 8.0


def compare(cpu: np.ndarray, hailo: np.ndarray) -> str:
    both = (cpu > 0) & (hailo > 0)
    if not both.any():
        return "no pixel has both depths"
    relative = np.abs(hailo[both].astype(float) - cpu[both]) / cpu[both]
    return (f"both on {100 * both.mean():.1f}% of pixels: StereoNet minus CPU median "
            f"{np.median(hailo[both].astype(float) - cpu[both]):+.0f} mm, |difference| median "
            f"{100 * np.median(relative):.1f}% of depth, within 10% on {100 * (relative < 0.1).mean():.0f}%")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--pairs", type=int, default=5)
    parser.add_argument("--resolution", type=ZedResolution, default=ZedResolution.VGA, choices=list(ZedResolution),
                        help="Eye size to capture; the CPU searches 64 disparities at VGA, 128 at larger sizes")
    parser.add_argument("--hef", default=DEFAULT_HEF)
    parser.add_argument("--save", type=Path, help="Folder for the last pair's images")
    args = parser.parse_args()

    pairs = asyncio.run(capture_pairs(args.resolution, args.pairs))
    height, width = pairs[0].left.shape[:2]
    rectifier = zed_rectifier((width, height))
    shared = rectifier.rectification
    cpu_matcher = SgbmMatcher(64 if args.resolution == ZedResolution.VGA else 128, 5)
    hailo_matcher = HailoStereoNetMatcher(args.hef, (width, height), centre_row=shared.cy)
    band = hailo_matcher.band
    print(f"StereoNet band: rows {band.top}-{band.top + band.rows - 1} of {height}, scale {band.scale:.3f}, "
          f"disparities up to {hailo_matcher.search_range:.0f} px")
    timings = {"cpu": [], "stereonet": []}
    try:
        for pair in pairs:
            left, right = rectifier.rectify(pair.left, pair.right)
            grey_left, grey_right = rgb_to_grey_with_opencv(left), rgb_to_grey_with_opencv(right)
            started = time.perf_counter()
            cpu = depth_millimetres(cpu_matcher.disparity(grey_left, grey_right), shared, MIN_DEPTH, MAX_DEPTH,
                                    cpu_matcher.search_range)
            timings["cpu"].append(time.perf_counter() - started)
            started = time.perf_counter()
            hailo = depth_millimetres(hailo_matcher.disparity(left, right), shared, MIN_DEPTH, MAX_DEPTH,
                                      hailo_matcher.search_range)
            timings["stereonet"].append(time.perf_counter() - started)
            swapped = depth_millimetres(hailo_matcher.disparity(right, left), shared, MIN_DEPTH, MAX_DEPTH,
                                        hailo_matcher.search_range)
            in_band = np.zeros(cpu.shape, bool)
            in_band[band.top:band.top + band.rows] = True
            print(f"frame {pair.frame_number}: depth on {100 * (cpu > 0).mean():.1f}% (CPU), "
                  f"{100 * (hailo > 0).mean():.1f}% (StereoNet); in StereoNet's band "
                  f"{100 * (cpu[in_band] > 0).mean():.1f}% / {100 * (hailo[in_band] > 0).mean():.1f}%")
            print(f"  as wired: {compare(cpu, hailo)}")
            print(f"  inputs swapped: {compare(cpu, swapped)}")
    finally:
        hailo_matcher.close()
    for name, values in timings.items():
        print(f"{name}: {1000 * np.median(values):.0f} ms per pair (median of {len(values)})")
    if args.save:
        import cv2

        from ultraskelly.ui.runtime_server import depth_to_rgb

        args.save.mkdir(parents=True, exist_ok=True)
        canvas = np.hstack([left, depth_to_rgb(cpu), depth_to_rgb(hailo)])
        path = args.save / f"rectified-cpu-stereonet-{args.resolution.lower()}.jpg"
        cv2.imwrite(str(path), cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
        print(f"Saved {path}")


if __name__ == "__main__":
    main()
