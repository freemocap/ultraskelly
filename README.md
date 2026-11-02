# UltraSkelly

Raspberry Pi animatronic skeleton: camera target tracking, head servos, waist and
mouth motors. Start with the [2026 audit and revival plan](docs/audit-2026.md).
The historical Pi application is retained; it has not yet been revalidated on hardware.

## Local skeleton simulator

From this directory, run:

```powershell
uv run --locked --python 3.12 python -m ultraskelly.simulator --open
```

Open [the local viewer](http://127.0.0.1:8765), then press **Run**. It starts paused.
The first command installs the portable dependencies if needed. After setup, the
viewer runs locally without internet, accounts, Node, a camera, or motor drivers.
Three.js is bundled in the repository under its MIT license.

Move the target sliders to switch to manual input, adjust controller tuning, or
pause and step by 10 ms. **Reset** clears simulated state and retains your settings.
The skeleton shows simulated positions; the table also shows controller commands.
See [the simulator guide](docs/simulator.md) for the model, code entry points, and
development workflow. Windows is verified; Linux and macOS use the same portable
Python/browser implementation but have not been tested in this session.

## Windows development

Install Python 3.11+ and uv, then run from this directory:

```powershell
uv sync --locked
uv run --locked python -m pytest -q
uv run --locked python -m ultraskelly
uv run --locked python -m ultraskelly.replay --steps 80
```

The default application command prints help. Replay feeds scripted targets into
the real orientation and mouth coroutines with memory-only actuators and prints
JSON lines of commanded state. It needs no camera, motors, browser, or API key.
It uses wall-clock scheduling; it is a test harness, not yet a deterministic
physics simulator. To save a run in PowerShell:

```powershell
uv run --locked python -m ultraskelly.replay --steps 400 | Set-Content -Encoding utf8 replay.jsonl
```

The repository requests Python 3.11 through `.python-version`. If you already have
3.12, use `uv sync --locked --python 3.12` and the same `--python 3.12` option on
`uv run`. This audit was validated with Python 3.12 on Windows.

## Raspberry Pi entry point

`python -m ultraskelly --hardware` explicitly starts the robot. The older
`python -m ultraskelly.__main__bot` entry point also remains available.
Both center the head when they start; the full-power jaw test runs only with
`MouthNodeParams.startup_test`. Every servo angle and motor throttle is clamped to
`LaunchConfig.limits` before it reaches a board.
Add `--simulate-motors` to run the cameras and controllers while recording motor
commands in memory instead of driving the motor boards. The OpenCV display requires
a graphical session; `--no-window` skips it. While the bot runs, port 8080 serves the
live dashboard to the network. One read-only WebSocket, `/stream`, carries FreeMoCap's
self-describing CBOR messages: `frame` messages with each OAK image and the poses found
in it, and `robot_state` messages with motor commands, target, rates and node health.
The browser decodes them with FreeMoCap's `@freemocap/client` and draws the overlay
itself. `scripts/open-dashboard.sh` opens it full-screen on the Pi's monitor.

Add `--record` to save the run under `~/ultraskelly-recordings/run-<time>/`: the OAK video
as it reached the bot, the poses found in each frame, targets, head state and every motor
command, with times on one clock so they line up. `--microphone NAME` adds audio from the
sound card whose name contains NAME (see `arecord -l`). `run.json` holds the configuration
and counts any frames the disk could not keep up with.

By default the robot tracks people with a Luxonis OAK USB camera and YOLOv8 pose
on a Hailo-8 AI HAT+. The original IMX500 AI camera remains available as
`DetectorType.POSE` in `ultraskelly/__main__bot.py`.

The optional `hardware` dependency group contains the Python motor, OpenCV, and
OAK (`depthai`) packages. Picamera2, libcamera, the IMX500 model, and the Hailo
runtime with its pose model come from Raspberry Pi OS, so on the Pi install them,
allow non-root OAK access, and build the environment on the system Python:

```bash
sudo apt install hailo-all python3-munkres
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
uv venv --system-site-packages --python /usr/bin/python3
uv sync --locked --extra hardware --python /usr/bin/python3
```

Installing the extra on Windows does not provide a Pi runtime. See the audit for
the hardware inventory and bring-up gates before running it.
