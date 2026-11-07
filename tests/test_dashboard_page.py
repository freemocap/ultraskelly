"""The dashboard page's words (ultraskelly/ui/dashboard): live values on the panels, no fixed explanations. At the owner's
request (2026-11-07, "No static text") each panel's explanation sits behind a small i that opens it in a popover, and
the control column holds the motor tests and the nodes so the time series has the strip. Commanded values keep their
"cmd" (AGENTS.md: label commanded positions apart from measured ones). Checked on the page's files; the layout itself
was checked in headless Chrome (docs/dashboard.md)."""
import json
import shutil
import subprocess
from html.parser import HTMLParser
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DASHBOARD = ROOT / "ultraskelly" / "ui" / "dashboard"
NODE = shutil.which("node")
# Explanations, headings and captions that were on the panels; each now sits behind an i, in a tooltip, or is gone.
GONE = ["From motor commands", "Head approximate", "Drag to orbit", "commands as sent", "no motor reports its position",
        "Tracking camera", "Eyes and depth", "Time series", "last 30 s", "Nodes and rates", "Left eye", "Right eye",
        "Heat map", "No face located", "Depth aligned to the left eye", "The head and waist turn toward people",
        "None in the bot's recent log", "Last error", "Status", "Robot"]


class Page(HTMLParser):
    """The page's visible words (outside popovers, scripts and attributes) and its i buttons and popovers."""

    VOID = {"meta", "link", "br", "img", "input", "hr"}

    def __init__(self):
        super().__init__()
        self.open: list[tuple[str, bool]] = []  # (tag, inside a popover or script)
        self.words: list[str] = []
        self.buttons: dict[str, str] = {}  # popovertarget -> class
        self.popovers: set[str] = set()

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        if "popovertarget" in attributes:
            self.buttons[attributes["popovertarget"]] = attributes.get("class", "")
        if "popover" in attributes:
            self.popovers.add(attributes["id"])
        hidden = bool(self.open and self.open[-1][1]) or "popover" in attributes or tag in ("script", "style", "title")
        if tag not in self.VOID:
            self.open.append((tag, hidden))

    def handle_endtag(self, tag):
        while self.open:
            if self.open.pop()[0] == tag:
                break

    def handle_data(self, data):
        if data.strip() and not (self.open and self.open[-1][1]):
            self.words.append(" ".join(data.split()))


def page() -> Page:
    parsed = Page()
    parsed.feed((DASHBOARD / "index.html").read_text(encoding="utf-8"))
    return parsed


def test_the_panels_show_no_fixed_explanations():
    shown = " | ".join(page().words)
    for phrase in GONE:
        assert phrase not in shown, phrase


def test_each_panels_explanation_opens_from_its_own_i():
    parsed = page()
    assert set(parsed.buttons) == parsed.popovers == {"info-stage", "info-oak", "info-zed", "info-series"}
    assert all(kind == "info" for kind in parsed.buttons.values())
    notes = (DASHBOARD / "index.html").read_text(encoding="utf-8")
    assert "not by measured positions" in notes and "Drag to orbit" in notes  # Moved, not lost
    assert 'id="depth-info"' in notes and 'id="face-alignment"' in notes and 'id="stage-head"' in notes  # Live, inside them


def test_the_scripts_write_no_fixed_explanations_either():
    panel = (DASHBOARD / "panel.js").read_text(encoding="utf-8")
    dashboard = (DASHBOARD / "dashboard.js").read_text(encoding="utf-8")
    assert "up since" not in panel and "None in the bot's recent log" not in panel and "control-follow-note" not in panel
    assert "No face located" not in dashboard and " drawn · " not in dashboard and "target-face" not in dashboard
    # The control column takes the motor tests and the nodes from the strip.
    assert ".after(document.querySelector('.card.tests'))" in panel and ".after(document.querySelector('.card.nodes'))" in panel


@pytest.mark.skipif(NODE is None, reason="needs node to run the dashboard's JavaScript")
def test_the_time_series_lanes_are_short_and_the_commanded_ones_say_so():
    script = ("import { pathToFileURL } from 'node:url';"
              "const {LANES} = await import(pathToFileURL(process.argv[1]).href);"
              "console.log(JSON.stringify(LANES.map(lane => [lane.id, lane.title, lane.waiting ?? null])));")
    done = subprocess.run([NODE, "--input-type=module", "-e", script, str(DASHBOARD / "timeseries.js")],
                          capture_output=True, encoding="utf-8", check=True, timeout=60)
    lanes = {lane: (title, waiting) for lane, title, waiting in json.loads(done.stdout)}
    assert lanes["head"][0] == "Head cmd" and lanes["drive"][0] == "Drive cmd"  # Commands, not measured positions
    assert all(len(title) <= 10 and waiting is None for title, waiting in lanes.values())
