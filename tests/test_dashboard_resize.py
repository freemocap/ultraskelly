"""The dashboard's resizable panels (ultraskelly/ui/dashboard/resize.js), its parts that need no browser run under node:
the limits a dragged size takes into the page's CSS, which way a drag moves each size, how the strip's cards share its
width, which stored sizes are read back and what a double-click resets; the time series' lanes for the shape of their
card; and that the page's handles, the script's sizes and the stylesheets' properties name one another. The drags
themselves were checked in headless Chrome (docs/dashboard.md, "Sizing")."""
import json
import re
import shutil
import subprocess
from html.parser import HTMLParser
from pathlib import Path

import pytest

from ultraskelly.ui.dashboard_assets import ASSETS

ROOT = Path(__file__).resolve().parents[1]
DASHBOARD = ROOT / "ultraskelly" / "ui" / "dashboard"
SCRIPT = ROOT / "tests" / "js" / "dashboard_calls.mjs"
NODE = shutil.which("node")
needs_node = pytest.mark.skipif(NODE is None, reason="needs node to run the dashboard's JavaScript")
USUAL_CARDS = {"series": 2.4, "tests": 1, "nodes": 1}


def call(*calls: tuple) -> list:
    """Each (file, export, args) called under node; an export that is no function comes back as it is."""
    done = subprocess.run([NODE, str(SCRIPT), str(DASHBOARD), json.dumps([list(c) for c in calls])],
                          capture_output=True, encoding="utf-8", check=True, timeout=60)
    return json.loads(done.stdout)


def resize(name: str, *args):
    [result] = call(("resize.js", name, list(args)))
    return result


@needs_node
def test_a_dragged_size_carries_its_limits_so_a_smaller_window_narrows_it_rather_than_overflow():
    assert resize("expression", "control", 320.4) == "clamp(220px, 320px, 35vw)"
    # The camera column leaves the 3D scene 480 px beside the controls, whatever width they have or fold to.
    assert resize("expression", "sensors", 700) == "clamp(260px, 700px, 100vw - var(--control-w, 0px) - 512px)"
    assert resize("expression", "strip", 300) == "clamp(90px, 300px, 100vh - var(--header-h) - 244px)"  # 220 px of scene
    assert resize("expression", "oak", 383.6) == "384px"  # dashboard.css keeps the tracking camera within its column


@needs_node
def test_each_size_grows_the_way_its_handle_is_dragged():
    assert resize("dragged", "control", 300, 50, 10) == 350   # Right: wider controls
    assert resize("dragged", "sensors", 538, 50, 10) == 488   # Right: a narrower camera column, a wider 3D scene
    assert resize("dragged", "strip", 238, 10, -100) == 338   # Up: a taller strip
    assert resize("dragged", "oak", 391, 10, 40) == 431       # Down: a taller tracking camera


@needs_node
def test_two_cards_share_out_their_own_width_and_the_third_keeps_its():
    shares = resize("splitCards", USUAL_CARDS, "series-tests", [571, 238], -100)  # The Pi's strip, time series narrower
    assert shares["series"] == pytest.approx(3.4 * 471 / 809) and shares["tests"] == pytest.approx(3.4 * 338 / 809)
    assert shares["nodes"] == 1
    # Each card keeps its minimum while the pair has room: 200 px for the time series, 236 for the motor tests (whole
    # button labels), 160 for the nodes.
    assert resize("splitCards", USUAL_CARDS, "series-tests", [571, 238], -1000)["series"] == pytest.approx(3.4 * 200 / 809)
    assert resize("splitCards", USUAL_CARDS, "series-tests", [571, 238], 1000)["tests"] == pytest.approx(3.4 * 236 / 809)
    assert resize("splitCards", USUAL_CARDS, "tests-nodes", [238, 238], 1000)["tests"] == pytest.approx(2 * 316 / 476)
    # A pair narrower than both minimums together stays halved.
    assert resize("splitCards", USUAL_CARDS, "series-tests", [150, 150], 80)["series"] == pytest.approx(1.7)


@needs_node
def test_only_the_sizes_this_page_knows_are_read_back_from_the_browser():
    kept = '{"control": 300, "sensors": "wide", "strip": -5, "oak": null, "cards": {"series": 2, "tests": 1}, "zoom": 3}'
    assert resize("parseSizes", kept) == {"control": 300}  # A card's share missing: the cards' usual widths
    assert resize("parseSizes", '{"cards": {"series": 2, "tests": 1.5, "nodes": 1}}') == {"cards": {"series": 2, "tests": 1.5, "nodes": 1}}
    assert resize("parseSizes", "not JSON") == {} and resize("parseSizes", None) == {} and resize("parseSizes", "[1, 2]") == {}


@needs_node
def test_a_double_click_puts_back_the_usual_size_of_its_own_handle():
    sizes = {"control": 300, "sensors": 700, "oak": 400, "cards": {"series": 2, "tests": 1.4, "nodes": 1}}
    assert resize("reset", sizes, "sensors") == {"control": 300, "oak": 400, "cards": sizes["cards"]}
    # Either handle between the cards gives all three their usual widths: under 1600 px the control panel hides the
    # nodes card, whose own share would otherwise keep the other two from theirs.
    for grip in ("series-tests", "tests-nodes"):
        assert resize("reset", sizes, grip) == {"control": 300, "sensors": 700, "oak": 400}


@needs_node
@pytest.mark.parametrize("width, height, columns", [
    (549, 197, 2),   # The Pi's strip beside the controls, as before handles
    (646, 197, 2),   # The bot's own dashboard on the Pi
    (382, 117, 2),   # A 1280x720 window
    (354, 255, 2),   # A phone
    (194, 308, 1),   # Dragged narrow and tall: one lane under the other
    (1400, 100, 3),  # Dragged wide and short: three abreast
])
def test_the_time_series_lays_its_lanes_out_for_the_shape_of_its_card(width, height, columns):
    [result] = call(("timeseries.js", "laneColumns", [width, height]))
    assert result == columns


class Handles(HTMLParser):
    """The page's handles: data-grip name -> its attributes."""

    def __init__(self):
        super().__init__()
        self.found: dict[str, dict] = {}

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        if "data-grip" in attributes:
            self.found[attributes["data-grip"]] = attributes


@needs_node
def test_every_handle_on_the_page_has_a_size_that_the_stylesheets_read():
    page = Handles()
    page.feed((DASHBOARD / "index.html").read_text(encoding="utf-8"))
    sizes, card_grips, cards = call(("resize.js", "SIZES"), ("resize.js", "CARD_GRIPS"), ("resize.js", "CARDS"))
    assert set(page.found) == set(sizes) | set(card_grips) == {"control", "sensors", "strip", "oak", "series-tests", "tests-nodes"}
    for name, handle in page.found.items():
        across = name in card_grips or sizes[name]["side"] == "width"  # Between panels side by side
        assert handle["role"] == "separator" and handle["aria-label"]
        assert handle["class"] == ("grip col" if across else "grip row")
        assert handle["aria-orientation"] == ("vertical" if across else "horizontal")
    styles = (DASHBOARD / "dashboard.css").read_text(encoding="utf-8") + (DASHBOARD / "panel.css").read_text(encoding="utf-8")
    for size in sizes.values():
        assert f"var({size['property']}," in styles  # Read with the usual size as the fallback
    for card in cards:
        assert f"var(--{card}-fr," in styles
    assert "/resize.js" in ASSETS and "import { resizable } from '/resize.js'" in (DASHBOARD / "dashboard.js").read_text(encoding="utf-8")


def test_the_folded_control_column_wins_over_a_dragged_width_and_the_phone_layout_has_no_handles():
    panel = (DASHBOARD / "panel.css").read_text(encoding="utf-8")
    dashboard = (DASHBOARD / "dashboard.css").read_text(encoding="utf-8")
    assert "--control-w:var(--control-size," in panel
    assert re.search(r"body\.dashboard\.control\.control-folded\{--control-w:44px\}", panel)  # More specific: it wins
    assert re.search(r"body\.dashboard\.control:not\(\.control-folded\) \[data-grip=control\]\{display:block\}", panel)
    # Handles show only from 1000 px wide, where the panels sit side by side.
    assert ".dashboard .grip{display:none;" in dashboard
    assert re.search(r"@media \(min-width:1000px\)\{\s*\.dashboard \.grip\{display:block\}", dashboard)
