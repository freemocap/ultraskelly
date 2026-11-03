"""Explicit entry point, safe by default on a development machine."""
import argparse
import asyncio
import logging
import signal

from ultraskelly.systemd_notify import run_with_heartbeat


def _stop_on_sigterm(_signum: int, _frame: object) -> None:
    signal.raise_signal(signal.SIGINT)  # Same clean shutdown as Ctrl+C


def main() -> None:
    parser = argparse.ArgumentParser(description="UltraSkelly robot controller")
    parser.add_argument(
        "--hardware", action="store_true",
        help="Start the Raspberry Pi camera and motors (physically moves the robot).",
    )
    parser.add_argument(
        "--simulate-motors", action="store_true",
        help="With --hardware: run the cameras but record motor commands instead of moving the motors.",
    )
    parser.add_argument(
        "--no-window", action="store_true",
        help="With --hardware: skip the OpenCV camera window (the dashboard still runs).",
    )
    parser.add_argument(
        "--record", action="store_true",
        help="With --hardware: save the camera video, detections and motor commands under ~/ultraskelly-recordings/.",
    )
    parser.add_argument(
        "--microphone", metavar="NAME",
        help="With --record: also record audio from the sound card whose name contains NAME (see arecord -l).",
    )
    parser.add_argument(
        "--zed", action="store_true",
        help="With --hardware: also capture the ZED Mini stereo camera and stream its left and right images.",
    )
    parser.add_argument(
        "--zed-only", action="store_true",
        help="Run only the ZED Mini and the dashboard: no OAK, Hailo or motors, so the robot can be in use.",
    )
    parser.add_argument(
        "--depth", nargs="?", const="sgbm", choices=["sgbm", "stereonet"],
        help="With --zed or --zed-only: also compute depth from the ZED's pair, at most 5 Hz: with OpenCV on "
             "the CPU (sgbm, the default) or StereoNet on the Hailo-8 (stereonet, needs the robot lock).",
    )
    parser.add_argument("--port", type=int, default=8080, help="Dashboard port (default 8080).")
    args = parser.parse_args()
    if args.depth and not (args.zed or args.zed_only):
        parser.error("--depth is used together with --zed or --zed-only")
    if (args.simulate_motors or args.no_window or args.record or args.zed) and not args.hardware:
        parser.error("--simulate-motors, --no-window, --record and --zed are used together with --hardware")
    if args.microphone and not args.record:
        parser.error("--microphone is used together with --record")
    if args.zed_only and args.hardware:
        parser.error("--zed-only runs without --hardware")
    if not args.hardware and not args.zed_only:
        parser.print_help()
        return

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    from ultraskelly.__main__bot import main as run_bot

    # A shell without job control starts background commands with SIGINT ignored, which
    # Python keeps; restore Ctrl+C explicitly. Service managers stop with SIGTERM.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _stop_on_sigterm)
    try:
        # The bot opens its hardware before its first await, so a boot service hears
        # READY only once the robot is up, then watchdog pings while the event loop runs.
        asyncio.run(run_with_heartbeat(run_bot(
            simulate_motors=args.simulate_motors, show_window=not args.no_window,
            record=args.record, microphone=args.microphone,
            zed=args.zed, zed_only=args.zed_only, depth=args.depth, port=args.port,
        )))
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("Stopped by Ctrl+C or SIGTERM")  # Nodes already shut down cleanly


if __name__ == "__main__":
    main()
