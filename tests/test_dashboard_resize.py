"""The dashboard's resizable panels (ultraskelly/ui/dashboard/resize.js), its parts that need no browser run under node:
the limits a dragged size takes into the page's CSS, which way a drag moves each size, which stored sizes are read back
and what a double-click resets; the time series' lanes for the shape of their card; and that the page's handles, the
script's sizes and the stylesheets' properties name one another. The drags themselves were checked in headless Chrome
(docs/dashboard.md, "Sizing")."""
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
    assert resize("expression", "control", 320.4) == "clamp(260px, 320px, 35vw)"  # 260: the motor tests keep whole labels
    # The camera column leaves the 3D scene 480 px beside the controls, whatever width they have or fold to.
    assert resize("expression", "sensors", 700) == "clamp(260px, 700px, 100vw - var(--control-w, 0px) - 512px)"
    assert resize("expression", "strip", 300) == "clamp(90px, 300px, 100vh - var(--header-h) - 244px)"  # 220 px of scene
    assert resize("expression", "oak", 383.6) == "384px"  # dashboard.css keeps the tracking camera within its column
    # Beside the time series, each card at most half of what leaves it 200 px.
    assert resize("expression", "tests", 300) == "clamp(236px, 300px, (100% - 216px) / 2)"
    assert resize("expression", "nodes", 250) == "clamp(160px, 250px, (100% - 216px) / 2)"


@needs_node
def test_each_size_grows_the_way_its_handle_is_dragged():
    assert resize("dragged", "control", 300, 50, 10) == 350   # Right: wider controls
    assert resize("dragged", "sensors", 538, 50, 10) == 488   # Right: a narrower camera column, a wider 3D scene
    assert resize("dragged", "strip", 238, 10, -100) == 338   # Up: a taller strip
    assert resize("dragged", "oak", 391, 10, 40) == 431       # Down: a taller tracking camera
    assert resize("dragged", "tests", 278, -50, 10) == 328    # Left: wider motor tests, a narrower time series
    assert resize("dragged", "nodes", 278, 30, 10) == 248     # Right: narrower nodes


@needs_node
def test_only_the_sizes_this_page_knows_are_read_back_from_the_browser():
    kept = '{"control": 300, "sensors": "wide", "strip": -5, "oak": null, "tests": 260, "cards": {"series": 2}, "zoom": 3}'
    assert resize("parseSizes", kept) == {"control": 300, "tests": 260}
    assert resize("parseSizes", "not JSON") == {} and resize("parseSizes", None) == {} and resize("parseSizes", "[1, 2]") == {}


@needs_node
def test_a_double_click_puts_back_the_usual_size_of_its_own_handle():
    sizes = {"control": 300, "sensors": 700, "oak": 400, "tests": 260}
    assert resize("reset", sizes, "sensors") == {"control": 300, "oak": 400, "tests": 260}
    assert resize("reset", sizes, "tests") == {"control": 300, "sensors": 700, "oak": 400}


@needs_node
@pytest.mark.parametrize("width, height, columns", [
    (1220, 306, 2),  # The Pi's strip, under the controls and the 3D scene
    (646, 306, 2),   # The bot's own dashboard on the Pi, beside the motor tests and the nodes
    (579, 198, 2),   # A 1280x720 window
    (352, 282, 1),   # A phone: one lane under the other
    (194, 308, 1),   # Dragged narrow and tall
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
    [sizes] = call(("resize.js", "SIZES"))
    assert set(page.found) == set(sizes) == {"control", "sensors", "strip", "oak", "tests", "nodes"}
    for name, handle in page.found.items():
        across = sizes[name]["side"] == "width"  # Between panels side by side
        assert handle["role"] == "separator" and handle["aria-label"]
        assert handle["class"] == ("grip col" if across else "grip row")
        assert handle["aria-orientation"] == ("vertical" if across else "horizontal")
    styles = (DASHBOARD / "dashboard.css").read_text(encoding="utf-8") + (DASHBOARD / "panel.css").read_text(encoding="utf-8")
    for size in sizes.values():
        assert f"var({size['property']}," in styles  # Read with the usual size as the fallback
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
