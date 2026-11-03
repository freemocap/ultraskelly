"""Judge depth from saved ZED side-by-side images, where no ground truth exists.

    .venv/bin/python -m scripts.stereo_quality frame1.jpg frame2.jpg ... [--stereonet] [--save DIR]

Each image holds both eyes side by side (left first), as the camera sends them, saved as
JPEG in OpenCV's BGR order. For each matcher the script reports, per image and pooled:
- coverage: the share of pixels given a depth;
- reprojection error: shift the right eye by each pixel's disparity and compare it with the
  left eye, in grey levels, on textured pixels only (a flat area matches at any disparity);
- steadiness: when several images show the same still scene, how much each pixel's depth
  varies between them;
- agreement with OpenCV's matcher where both give a depth.
`--stereonet` adds StereoNet on the Hailo-8, which needs the robot lock.
"""
import argparse
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.stereo_depth_node import EDGE_MARGIN, SgbmMatcher, depth_millimetres
from ultraskelly.core.bot.sensory.stereo_rectification import zed_rectifier

MIN_DEPTH, MAX_DEPTH = 0.3, 8.0
TEXTURE = 12.0  # Sobel magnitude (grey levels per pixel) above which a pixel counts as textured


def load_pair(path: Path) -> tuple[np.ndarray, np.ndarray]:
    import cv2

    image = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
    half = image.shape[1] // 2
    return np.ascontiguousarray(image[:, :half]), np.ascontiguousarray(image[:, half:])


def reprojection_error(left_grey: np.ndarray, right_grey: np.ndarray, disparity: np.ndarray, where: np.ndarray) -> float:
    """Median |left - right shifted by the disparity|, in grey levels, over `where`."""
    import cv2

    height, width = left_grey.shape
    columns, rows = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    warped = cv2.remap(right_grey, columns - disparity.astype(np.float32), rows, cv2.INTER_LINEAR)
    if not where.any():
        return float("nan")
    return float(np.median(np.abs(left_grey.astype(np.float32) - warped)[where]))


def main() -> None:
    import cv2

    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("images", type=Path, nargs="+")
    parser.add_argument("--stereonet", action="store_true", help="Also run StereoNet on the Hailo-8")
    parser.add_argument("--save", type=Path, help="Folder for each image's rectified left eye and depth images")
    parser.add_argument("--scale", type=float, nargs="*", help="Also OpenCV's matcher on images scaled by these")
    parser.add_argument("--mask", type=Path, help="View mask (scripts.zed_view_mask): judge only where both eyes see")
    args = parser.parse_args()

    pairs = [load_pair(path) for path in args.images]
    height, width = pairs[0][0].shape[:2]
    rectifier = zed_rectifier((width, height))
    shared = rectifier.rectification
    if args.mask:
        from ultraskelly.core.bot.sensory.stereo_mask import ViewMask

        seen = ViewMask.load(args.mask, (width, height)).valid
    else:
        seen = np.ones((height, width), bool)
    textured_kernel = lambda grey: cv2.magnitude(cv2.Sobel(grey, cv2.CV_32F, 1, 0), cv2.Sobel(grey, cv2.CV_32F, 0, 1)) > TEXTURE
    disparities = 64 if width <= 672 else 128
    matchers = {"sgbm": SgbmMatcher(disparities, 5)}
    for scale in args.scale or ():
        matchers[f"sgbm x{scale:g}"] = SgbmMatcher(disparities, 5, scale)
    if args.stereonet:
        from ultraskelly.core.bot.sensory.hailo_stereonet import DEFAULT_HEF, HailoStereoNetMatcher

        matchers["stereonet"] = HailoStereoNetMatcher(DEFAULT_HEF, (width, height), centre_row=shared.cy)
    results = {name: [] for name in matchers}
    try:
        for path, (left, right) in zip(args.images, pairs):
            color, right_color = rectifier.rectify(left, right)
            left_grey, right_grey = cv2.cvtColor(color, cv2.COLOR_RGB2GRAY), cv2.cvtColor(right_color, cv2.COLOR_RGB2GRAY)
            textured = textured_kernel(left_grey)
            images = [color]
            for name, matcher in matchers.items():
                inputs = (color, right_color) if matcher.color else (left_grey, right_grey)
                disparity = np.where(seen, matcher.disparity(*inputs), 0)
                depth = depth_millimetres(disparity, shared, MIN_DEPTH, MAX_DEPTH, matcher.search_range)
                known = depth > 0
                results[name].append((depth, disparity, known))
                error = reprojection_error(left_grey, right_grey, disparity, known & textured)
                print(f"{path.name} {name}: depth on {100 * known.mean():.1f}%, textured {100 * (known & textured).mean():.1f}%, "
                      f"reprojection error median {error:.1f} grey levels")
                from ultraskelly.ui.runtime_server import depth_to_rgb

                images.append(depth_to_rgb(depth))
            if args.save:
                args.save.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(args.save / f"quality-{path.stem}.jpg"), cv2.cvtColor(np.hstack(images), cv2.COLOR_RGB2BGR))
    finally:
        for matcher in matchers.values():
            matcher.close()
    for name, frames in results.items():
        depths = np.stack([depth for depth, _, _ in frames]).astype(float)
        known = depths > 0
        line = f"{name}: depth on {100 * known.mean():.1f}% of pixels ({100 * known[:, seen].mean():.1f}% of those both eyes see)"
        steady = known.all(axis=0)
        if len(frames) > 1 and steady.any():
            spread = depths[:, steady].std(axis=0) / depths[:, steady].mean(axis=0)
            line += f"; on the {100 * steady.mean():.1f}% known in every image, depth varies {100 * np.median(spread):.2f}% (median)"
        if name != "sgbm":
            both = known & (np.stack([depth for depth, _, _ in results["sgbm"]]) > 0)
            reference = np.stack([depth for depth, _, _ in results["sgbm"]]).astype(float)
            relative = np.abs(depths[both] - reference[both]) / reference[both]
            line += (f"; where OpenCV also has depth ({100 * both.mean():.1f}%), they differ by "
                     f"{100 * np.median(relative):.1f}% (median), within 10% on {100 * (relative < 0.1).mean():.0f}%")
        print(line)
    print(f"Search limits: " + ", ".join(f"{name} {matcher.search_range:.0f} px" for name, matcher in matchers.items())
          + f"; matches within {EDGE_MARGIN} px of the limit are dropped")


if __name__ == "__main__":
    main()
