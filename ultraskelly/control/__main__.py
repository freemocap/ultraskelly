"""Run the control panel.

    python -m ultraskelly.control                       on the Pi (ultraskelly-control.service): port 80,
                                                        managing ultraskelly-bot.service through systemd
    python -m ultraskelly.control --port 8000 --local-bot
                                                        development: runs `python -m ultraskelly.replay --serve
                                                        --people 3` as the bot, which moves nothing
    python -m ultraskelly.control --port 8000 --local-bot python my_bot.py
                                                        the same with another command as the bot; it must serve
                                                        the dashboard's /stream and /api/motor-test on --bot-port
"""
import argparse
import logging
import os
import sys
from pathlib import Path

from aiohttp import web

from ultraskelly.control.bots import LocalBot, SystemdBot
from ultraskelly.control.options import StartOptions
from ultraskelly.control.server import ControlServer

REPO = Path(__file__).resolve().parents[2]


def main() -> None:
    parser = argparse.ArgumentParser(description="UltraSkelly control panel: turn the bot on and off from a web page")
    parser.add_argument("--port", type=int, default=80, help="Port to serve on (default 80)")
    parser.add_argument("--host", default=None, help="Interface to serve on (default: every IPv4 and IPv6 interface)")
    parser.add_argument("--bot-port", type=int, default=8080, help="The bot's dashboard port (default 8080)")
    parser.add_argument("--unit", default="ultraskelly-bot.service", help="The bot's systemd unit")
    parser.add_argument(
        "--options-file", type=Path, default=None,
        help="Where the switches the bot starts with are kept (default: $STATE_DIRECTORY/bot-options, which "
             "ultraskelly-bot.service reads; with --local-bot, var/control/bot-options in this checkout)",
    )
    parser.add_argument(
        "--local-bot", nargs=argparse.REMAINDER, default=None, metavar="COMMAND",
        help="Development without systemd, given last: run COMMAND as the bot, by default "
             "`python -m ultraskelly.replay --serve --people 3`",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    if args.options_file is not None:
        options_file = args.options_file
    elif args.local_bot is not None:
        options_file = REPO / "var" / "control" / "bot-options"
    else:
        options_file = Path(os.environ.get("STATE_DIRECTORY", "/var/lib/ultraskelly-control")) / "bot-options"
    options = StartOptions(options_file)

    local = args.local_bot is not None
    if local:
        command = args.local_bot or [sys.executable, "-m", "ultraskelly.replay", "--serve", "--people", "3",
                                     "--port", str(args.bot_port), "--host", "127.0.0.1"]
        server = ControlServer(None, options=options, bot_port=args.bot_port, local=True)
        server.bot = LocalBot(command, probe=server.probe)
    else:
        server = ControlServer(
            SystemdBot(args.unit), options=options, bot_port=args.bot_port,
            stop_motors=[sys.executable, "-m", "scripts.stop_motors"], stop_motors_cwd=str(REPO),
        )
    logging.getLogger(__name__).info(
        f"Control panel on port {args.port} for {'a local stand-in bot' if local else args.unit}; "
        f"start options in {options_file}"
    )
    web.run_app(server.build_app(), host=args.host, port=args.port, access_log=None, print=None)


if __name__ == "__main__":
    main()
