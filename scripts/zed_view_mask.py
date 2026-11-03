"""Propose the ZED's view mask: where both eyes see past the skull. Redo it whenever the ZED is re-seated.

    .venv/bin/python -m scripts.zed_view_mask --preview ~/zed-mask             # From the live ZED
    .venv/bin/python -m scripts.zed_view_mask --images a.jpg b.jpg --out m.png  # From saved side-by-side images

Live, it writes ultraskelly/calibration/zed/SN<serial>-view-mask.png, which depth then uses.
Point the head at a scene with detail at several distances while it runs. `--preview` saves
the rectified left eye with the masked part tinted red, to check before committing the mask.
See ultraskelly/core/bot/sensory/stereo_mask.py for how the mask is found.
"""
import argparse
import asyncio
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.stereo_mask import ViewMask, propose_view_mask, view_mask_file
from ultraskelly.core.bot.sensory.stereo_rectification import find_zed_serial, zed_rectifier
from ultraskelly.core.bot.sensory.zed_camera_node import ZedResolution


def main() -> None:
    import cv2

    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--images", type=Path, nargs="+", help="Saved side-by-side images instead of the live ZED")
    parser.add_argument("--pairs", type=int, default=5, help="Live pairs to use (default 5)")
    parser.add_argument("--out", type=Path, help="Mask file; live default ultraskelly/calibration/zed/SN<serial>-view-mask.png")
    parser.add_argument("--preview", type=Path, help="Folder for an image showing the mask")
    args = parser.parse_args()

    if args.images:
        from scripts.stereo_quality import load_pair

        pairs = [load_pair(path) for path in args.images]
        out = args.out
        if out is None:
            parser.error("--out is needed with --images")
    else:
        from scripts.zed_row_alignment import capture_pairs

        pairs = [(pair.left, pair.right) for pair in asyncio.run(capture_pairs(ZedResolution.VGA, args.pairs))]
        serial = find_zed_serial()
        out = args.out or view_mask_file(serial)
    height, width = pairs[0][0].shape[:2]
    rectifier = zed_rectifier((width, height))
    grey = lambda image: cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    rectified = [(rectifier.rectify_eye(0, grey(left)), rectifier.rectify_eye(1, grey(right))) for left, right in pairs]
    mask = ViewMask(valid=propose_view_mask(rectified))
    mask.save(out)
    rows, columns = mask.window(64)
    print(f"Wrote {out}: both eyes see {100 * mask.valid.mean():.0f}% of the {width}x{height} view; "
          f"the CPU matcher covers rows {rows.start}-{rows.stop - 1}, columns {columns.start}-{columns.stop - 1}")
    if args.preview:
        image = cv2.cvtColor(rectifier.rectify_eye(0, pairs[-1][0]), cv2.COLOR_RGB2BGR)
        blocked = ~mask.valid
        image[blocked] = (image[blocked] * 0.35 + np.array([0, 0, 170]) * 0.65).astype(np.uint8)
        args.preview.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.preview / f"{out.stem}-preview.jpg"), image)
        print(f"Preview: {args.preview / f'{out.stem}-preview.jpg'}")


if __name__ == "__main__":
    main()
