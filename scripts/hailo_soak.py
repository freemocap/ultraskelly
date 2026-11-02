"""Run the Hailo-8 pose model in a loop, with no camera or motors, to test the driver.

Mirrors HailoPoseDetectorNode's setup: round-robin scheduler, UINT8 input, FLOAT32
outputs and buffers bound once. On the Pi:

    .venv/bin/python scripts/hailo_soak.py --seconds 600

Driver problems show up as kernel warnings in `journalctl -k`; see docs/audit-2026.md.
"""
import argparse
import contextlib
import time

import hailo_platform as hp
import numpy as np

HEF = "/usr/share/hailo-models/yolov8s_pose_h8.hef"


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the Hailo-8 pose model in a loop to test the driver.")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--hz", type=float, default=30.0, help="Inferences per second, like the camera")
    parser.add_argument("--hef", default=HEF)
    args = parser.parse_args()

    # Closing the stack releases the configured model before the device, the order HailoRT needs.
    with contextlib.ExitStack() as hailo:
        device_params = hp.VDevice.create_params()
        device_params.scheduling_algorithm = hp.HailoSchedulingAlgorithm.ROUND_ROBIN
        device = hailo.enter_context(hp.VDevice(device_params))
        model = device.create_infer_model(args.hef)
        model.set_batch_size(1)
        model.input().set_format_type(hp.FormatType.UINT8)
        for output in model.outputs:
            output.set_format_type(hp.FormatType.FLOAT32)
        configured = hailo.enter_context(model.configure())

        image = np.random.default_rng(0).integers(0, 256, model.input().shape, dtype=np.uint8)
        outputs = {name: np.empty(model.output(name).shape, dtype=np.float32) for name in model.output_names}
        bindings = configured.create_bindings()
        bindings.input().set_buffer(image)
        for name, buffer in outputs.items():
            bindings.output(name).set_buffer(buffer)

        runs, start = 0, time.monotonic()
        while time.monotonic() - start < args.seconds:
            configured.run([bindings], 1000)
            runs += 1
            time.sleep(max(0.0, start + runs / args.hz - time.monotonic()))
        elapsed = time.monotonic() - start
        print(f"{runs} inferences in {elapsed:.1f} s ({runs / elapsed:.1f}/s)")


if __name__ == "__main__":
    main()
