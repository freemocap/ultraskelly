"""Explicit entry point, safe by default on a development machine."""
import argparse
import asyncio
import logging
import os
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
        "--window", action="store_true",
        help="With --hardware: also show the camera in an OpenCV window (needs a desktop session); "
             "the dashboard runs either way.",
    )
    # The window used to be on unless this was given; still accepted, so older commands run unchanged.
    parser.add_argument("--no-window", action="store_true", help=argparse.SUPPRESS)
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
        help="With --hardware: also capture the ZED Mini stereo camera and stream its left and right images "
             "and its IMU.",
    )
    parser.add_argument(
        "--zed-only", action="store_true",
        help="Run only the ZED Mini and the dashboard: no OAK, Hailo or motors, so the robot can be in use.",
    )
    parser.add_argument(
        "--depth", nargs="?", const="sgbm", choices=["sgbm", "stereonet"],
        help="With --zed or --zed-only: also compute depth from the ZED's pair: with OpenCV on the CPU, at most "
             "10 Hz (sgbm, the default), or StereoNet on the Hailo-8, 1 Hz (stereonet, needs the robot lock).",
    )
    parser.add_argument(
        "--depth-previews", action="store_true",
        help="With --depth: also stream the depth as colour images (cameras zed_rectified and zed_depth); "
             "the depth_frame messages carry the same, so the dashboard needs none.",
    )
    parser.add_argument("--port", type=int, default=8080, help="Dashboard port (default 8080).")
    args = parser.parse_args()
    if args.depth and not (args.zed or args.zed_only):
        parser.error("--depth is used together with --zed or --zed-only")
    if args.depth_previews and not args.depth:
        parser.error("--depth-previews is used together with --depth")
    if (args.simulate_motors or args.window or args.record or args.zed) and not args.hardware:
        parser.error("--simulate-motors, --window, --record and --zed are used together with --hardware")
    if args.window and args.no_window:
        parser.error("--window and --no-window contradict each other")
    if args.microphone and not args.record:
        parser.error("--microphone is used together with --record")
    if args.zed_only and args.hardware:
        parser.error("--zed-only runs without --hardware")
    if not args.hardware and not args.zed_only:
        parser.print_help()
        return

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    # One OpenBLAS thread for NumPy, set before NumPy loads: its idle workers also spin after
    # each call; the ZED IMU node alone used 6.3% of a core with them and 2.6% without.
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    import cv2

    # One OpenCV thread for the whole bot: the pool's idle workers spin between calls, which on
    # the Pi cost the bot with ZED depth 174% of a core instead of 112%, and pose its 30 fps.
    cv2.setNumThreads(1)
    from ultraskelly.__main__bot import launch_config
    from ultraskelly.bot_launcher import BotLauncher

    async def run_bot() -> None:
        # Building the nodes opens the hardware before the first await, so a boot service
        # hears READY only once the robot is up. Watchdog pings then follow while the event
        # loop runs, until the bot starts shutting down.
        launcher = BotLauncher.from_config(launch_config(
            simulate_motors=args.simulate_motors, show_window=args.window,
            record=args.record, microphone=args.microphone,
            zed=args.zed, zed_only=args.zed_only, depth=args.depth, depth_previews=args.depth_previews,
            port=args.port,
        ))
        await run_with_heartbeat(launcher.run(), healthy=launcher.healthy)

    # A shell without job control starts background commands with SIGINT ignored, which
    # Python keeps; restore Ctrl+C explicitly. Service managers stop with SIGTERM.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, _stop_on_sigterm)
    try:
        asyncio.run(run_bot())
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("Stopped by Ctrl+C or SIGTERM")  # Nodes already shut down cleanly


if __name__ == "__main__":
    main()
