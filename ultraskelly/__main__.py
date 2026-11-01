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
    args = parser.parse_args()
    if not args.hardware:
        parser.print_help()
        return

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    from ultraskelly.__main__bot import main as run_bot

    asyncio.run(run_bot())


if __name__ == "__main__":
    main()
