"""The switches the bot starts with, kept by the control panel across restarts, reboots and power cuts.

The bot's service reads them from an environment file after its own options from
/etc/default/ultraskelly-bot (scripts/systemd/ultraskelly-bot.service), and puts them last on its
command line, so the panel's choice wins:

    ULTRASKELLY_PANEL_OPTIONS=--no-follow

The file is replaced whole: written beside it, flushed to the card, then renamed over it, so a
power cut leaves the old file or the new one and never a part of either.
"""
import os
from pathlib import Path

PANEL_VARIABLE = "ULTRASKELLY_PANEL_OPTIONS"
BOT_VARIABLE = "ULTRASKELLY_BOT_OPTIONS"
BOT_DEFAULTS = Path("/etc/default/ultraskelly-bot")  # The service's own options, written by hand


def read_variable(path: Path, name: str) -> list[str] | None:
    """The words of `name` in a systemd environment file, split as `$NAME` splits on a command line, or None
    when the file or the variable is missing."""
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (FileNotFoundError, NotADirectoryError, PermissionError):
        return None
    for line in reversed(lines):  # The last assignment wins, as in systemd
        key, equals, value = line.strip().partition("=")
        if not equals or key.strip() != name:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        return value.split()
    return None


def write_variable(path: Path, name: str, words: list[str], *, note: str) -> None:
    """Replace the file with `name`'s one line, so that a power cut leaves the old file or the new one."""
    text = f"# {note}\n{name}={' '.join(words)}\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    draft = path.with_name(f".{path.name}.new")
    with open(draft, "w", encoding="utf-8") as file:
        file.write(text)
        file.flush()
        os.fsync(file.fileno())
    os.replace(draft, path)
    if hasattr(os, "O_DIRECTORY"):  # The rename itself reaches the card only once the directory is flushed
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)


def following_in(options: list[str] | None) -> bool | None:
    """What the last of --follow and --no-follow in `options` says, or None when neither is there."""
    for word in reversed(options or []):
        if word in ("--follow", "--no-follow"):
            return word == "--follow"
    return None


class StartOptions:
    """The panel's options for the bot's next start, and the service's own, to say which way it will start."""

    def __init__(self, path: Path, *, bot_defaults: Path = BOT_DEFAULTS) -> None:
        self.path = path
        self.bot_defaults = bot_defaults

    def panel(self) -> list[str]:
        return read_variable(self.path, PANEL_VARIABLE) or []

    def service(self) -> list[str]:
        return read_variable(self.bot_defaults, BOT_VARIABLE) or []

    def follows(self) -> bool:
        """Whether the bot follows people when it next starts: the panel's switch, else the service's options."""
        panel, service = following_in(self.panel()), following_in(self.service())
        return panel if panel is not None else service if service is not None else True

    def set_following(self, on: bool) -> None:
        kept = [word for word in self.panel() if word not in ("--follow", "--no-follow")]
        write_variable(self.path, PANEL_VARIABLE, [*kept, "--follow" if on else "--no-follow"],
                       note="Written by the control panel (ultraskelly/control); read by ultraskelly-bot.service at each start")

    def describe(self) -> dict:
        return {"following": self.follows(), "panel": self.panel(), "service": self.service(),
                "file": str(self.path)}
