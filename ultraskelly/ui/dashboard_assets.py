"""The dashboard's files by URL path, for both servers that show it: the bot's runtime server
(ultraskelly/ui/runtime_server.py, port 8080) and the always-on control panel (ultraskelly/control,
port 80). Paths only, so the control panel loads none of the bot's code and keeps working when the
bot's code does not. Each file is read on every request: a pull and a page reload deploy a change.
"""
from pathlib import Path

DASHBOARD = Path(__file__).with_name("dashboard")
SHARED = Path(__file__).parents[1] / "simulator" / "static"  # Skeleton, style and vendored Three.js
# Explicit content types: Windows can map .js to text/plain, which browsers refuse for modules.
ASSETS: dict[str, tuple[Path, str]] = {
    "/": (DASHBOARD / "index.html", "text/html"),
    "/dashboard.js": (DASHBOARD / "dashboard.js", "text/javascript"),
    "/dashboard.css": (DASHBOARD / "dashboard.css", "text/css"),
    "/panel.js": (DASHBOARD / "panel.js", "text/javascript"),
    "/panel.css": (DASHBOARD / "panel.css", "text/css"),
    "/scene.js": (DASHBOARD / "scene.js", "text/javascript"),
    "/tiles.js": (DASHBOARD / "tiles.js", "text/javascript"),
    "/depth.js": (DASHBOARD / "depth.js", "text/javascript"),
    "/depth3d.js": (DASHBOARD / "depth3d.js", "text/javascript"),
    "/gaze.js": (DASHBOARD / "gaze.js", "text/javascript"),
    "/person3d.js": (DASHBOARD / "person3d.js", "text/javascript"),
    "/timeseries.js": (DASHBOARD / "timeseries.js", "text/javascript"),
    "/depth-worker.js": (DASHBOARD / "depth-worker.js", "text/javascript"),
    "/colormap.js": (DASHBOARD / "colormap.js", "text/javascript"),
    "/heatmap.js": (DASHBOARD / "heatmap.js", "text/javascript"),
    "/dom.js": (DASHBOARD / "dom.js", "text/javascript"),
    "/frameclock.js": (DASHBOARD / "frameclock.js", "text/javascript"),
    "/faces.js": (DASHBOARD / "faces.js", "text/javascript"),
    "/health.js": (DASHBOARD / "health.js", "text/javascript"),
    "/headpose.js": (DASHBOARD / "headpose.js", "text/javascript"),
    "/motor-test.js": (DASHBOARD / "motor-test.js", "text/javascript"),
    "/resize.js": (DASHBOARD / "resize.js", "text/javascript"),
    "/tune.js": (DASHBOARD / "tune.js", "text/javascript"),
    "/style.css": (SHARED / "style.css", "text/css"),
    "/skeleton.js": (SHARED / "skeleton.js", "text/javascript"),
    **{
        f"/vendor/{name}": (SHARED / "vendor" / name, "text/javascript")
        for name in ("three.module.js", "three.core.js", "OrbitControls.js")
    },
    # @freemocap/client's browser bundle, vendored by scripts/update-freemocap-client.sh.
    "/vendor/freemocap-client.js": (DASHBOARD / "vendor" / "freemocap-client.js", "text/javascript"),
    "/vendor/freemocap-client.js.map": (DASHBOARD / "vendor" / "freemocap-client.js.map", "application/json"),
}
