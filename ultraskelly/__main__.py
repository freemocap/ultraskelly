"""Explicit entry point, safe by default on a development machine."""
import argparse
import asyncio
import logging


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
    args = parser.parse_args()
    if args.simulate_motors and not args.hardware:
        parser.error("--simulate-motors is used together with --hardware")
    if not args.hardware:
        parser.print_help()
        return

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    from ultraskelly.__main__bot import main as run_bot

    try:
        asyncio.run(run_bot(simulate_motors=args.simulate_motors))
    except KeyboardInterrupt:
        logging.getLogger(__name__).info("Stopped by Ctrl+C")  # Nodes already shut down cleanly


if __name__ == "__main__":
    main()
