# UltraSkelly

Raspberry Pi animatronic skeleton: camera target tracking, head servos, waist and
mouth motors. Start with the [2026 audit and revival plan](docs/audit-2026.md), which
records what has been checked on the robot's hardware.

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
The plant is the robot's servos and waist as fitted to a recorded run; replaying that run's
commands must give its recorded views (`tests/test_plant_fidelity.py`). The suite runs the
same plant in closed-loop scenarios (people stepping in, walking across, swapping, flickering,
false detections, tilting their heads) that fail on overshoot, swinging, runaway, chasing,
waist hunting or a face left tilted before any motor runs:
`uv run --locked python -m pytest -q tests/test_scenarios.py tests/test_scenarios_tuning.py`.
`python -m ultraskelly.simulator.tuning` tunes the head's aim on that plant against a stated cost.
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
`--serve` loops the targets and serves the live dashboard on http://127.0.0.1:8081
instead; `--serve --people 3` replaces them with a synthetic camera showing three people,
so the video, poses and target channel in [the stream guide](docs/stream.md) can be
developed without the robot. It uses wall-clock scheduling; it is a test harness, not
yet a deterministic physics simulator. To save a run in PowerShell:

```powershell
uv run --locked python -m ultraskelly.replay --steps 400 | Set-Content -Encoding utf8 replay.jsonl
```

The repository requests Python 3.11 through `.python-version`. If you already have
3.12, use `uv sync --locked --python 3.12` and the same `--python 3.12` option on
`uv run`. This audit was validated with Python 3.12 on Windows.

## Raspberry Pi

The robot runs on a Raspberry Pi 5 with Raspberry Pi OS Trixie (desktop). By default it
tracks people with a Luxonis OAK USB camera and YOLOv8 pose on a Hailo-8 AI HAT+, and
drives a servo board at I²C `0x40` and a DC motor HAT at `0x60`; see
[the hardware notes](docs/hardware.md). The OAK reads its frames out the right way up,
encodes them on its own chip and runs face and hand models from Luxonis' Model Zoo: YuNet
face detection on every frame, head pose, gaze and MediaPipe face landmarks on the faces it
finds, and MediaPipe hands five times a second (`--no-oak-models` turns them off; what runs
where, and why, is in the audit).
`scripts/setup-pi.sh` fetches the models once (`.venv/bin/python -m scripts.fetch_oak_models`
does it alone); the bot then loads them without the internet, and with one missing it runs
the OAK as a plain camera and says why. depthai, the OAK's library, uploads
usage events to Luxonis by default; the bot and the OAK scripts turn that off
(`DEPTHAI_TELEMETRY=0`) unless the variable is already set.

### Set up a Pi

Image the Pi with Raspberry Pi Imager, setting the user, Wi-Fi and SSH there. Then, on
the Pi:

```bash
git clone https://github.com/freemocap/ultraskelly ~/ultraskelly
~/ultraskelly/scripts/setup-pi.sh
```

It installs the packages, including the Hailo runtime; patches the Hailo driver
(`scripts/patch-hailo-driver.sh`) and holds the Hailo packages, so an apt upgrade cannot
replace them; turns on I²C and PCIe Gen 3; installs the system files kept in
`scripts/pi-system/` (the OAK udev rule, a persistent journal synced every 10 s, a report
of why the Pi last stopped, the hardware watchdog, Wi-Fi power save off); makes the
desktop log in by itself and keep the screen on; installs uv and builds `.venv`; fetches
the OAK's models into `~/ultraskelly-models/oak-zoo`; and installs the robot's services,
disabled. The first sudo may ask for your password.
Reboot when it says so; it is safe to re-run. `.venv` is built on the system Python
because the Hailo runtime and its pose model come from Raspberry Pi OS; the `hardware`
extra adds the motor, OpenCV and OAK (`depthai`) packages. Installing the extra on
Windows does not provide a Pi runtime.

`scripts/setup-pi.sh --check` changes nothing. It reports every setting, the OAK's models
among them, why the Pi last stopped, the power and USB state, the Hailo-8, the I²C boards,
Wi-Fi, and any git objects or network profiles a power loss emptied. Run it after a power
loss or whenever something seems off.

### Run the robot

```bash
scripts/run-bot.sh                  # add bot options, such as --record
```

runs `python -m ultraskelly --hardware` from `.venv`, holding the robot
lock (below). The older `python -m ultraskelly.__main__bot` entry point also remains
available. Both center the head when they start; the full-power jaw test runs only with
`MouthNodeParams.startup_test`. Every servo angle and motor throttle is clamped to
`LaunchConfig.limits` before it reaches a board.
It starts with whatever hardware works: a missing camera, Hailo-8 or motor board is
reported on the dashboard with how to fix it, and the rest runs. A missing board's
actuators record their commands and send nothing. Cameras and the Hailo-8 are picked up
within seconds of appearing; a motor board that answers after the start is reported with a
restart to use it, since the boards are set up only at startup. Add `--simulate-motors` to run the
cameras and controllers while recording motor commands in memory instead of driving the
motor boards. While the bot runs, port
8080 serves the live dashboard to the network. One read-only WebSocket, `/stream`,
carries FreeMoCap's self-describing CBOR messages: `frame` messages with each OAK image
and the poses, faces and hands found in it, and `robot_state` messages with motor commands,
target, rates, node health and the OAK chip's load. The browser decodes them with FreeMoCap's `@freemocap/client` and
draws the overlay itself; [the stream guide](docs/stream.md) describes every field.
On the Pi, the control panel on port 80 serves the same page with big buttons that turn
the bot on and off, restart it and switch following
([Control panel](docs/dashboard.md#control-panel)); `scripts/open-dashboard.sh` opens it
full-screen on the Pi's monitor.
[The dashboard layout](docs/dashboard.md) describes its panels and sizing.

The dashboard's **Motor test** panel moves one actuator at a time: pan or tilt by 10°
or to center, a short waist pulse either way, or a jaw opening pulse. Tests are separate
commands (`POST /api/motor-test`, `POST /api/motor-test/stop`; see
`ultraskelly/ui/motor_test_routes.py`): each is answered once it has finished or been
stopped, or refused at once with the reason, never queued. The bot checks every test
against `LaunchConfig.motor_tests` (servos within 30° of center, waist up to 0.7 for
0.5 s, jaw up to 0.3 s) as well as the hard limits, pauses tracking during it, and brakes
the other DC motor. Set `motor_tests.enabled` to False to turn the panel off. With
`--simulate-motors` the tests drive the simulated motors, and so does
`python -m ultraskelly.replay --serve`, which needs no hardware.

The waist turns toward where the head looks in continuous, ramped moves
(`motor/waist_drive.py`). A watchdog thread brakes both DC motors if the control loop
stalls for half a second (`motor/motor_watchdog.py`), and each motor command sets both of a
motor's bridge inputs in one I2C transfer (`motor/bridge_motor.py`). If the bot ever exits
without stopping its motors, for example when killed, the motor board keeps its last
command: `.venv/bin/python -m scripts.stop_motors` brakes them all.

Add `--zed` to also capture the ZED Mini stereo camera in the skull's eye sockets: its
left and right images (672×376 each at 30 fps, sent at the size the dashboard draws them)
travel together in one `frame` message, as cameras `zed_left` and `zed_right`, and appear
as two more video panels. Its IMU
(accelerometer and gyroscope, 794 samples a second) streams as `imu` messages for display
and `--record`, never for control; `scripts/setup-pi.sh` installs the udev rule that lets
the bot read it.
Add `--depth` as well for depth from the pair (OpenCV on the CPU, 336×188, up to 10 frames a
second, about 17 ms each), streamed as a `depth_frame` message ([docs/depth-stream.md](docs/depth-stream.md)).
`--depth-previews` also sends it as colour images for viewers that draw only frames.
`--depth stereonet` uses StereoNet on the Hailo-8 instead: it needs the robot lock and slows
pose, see the audit.
`python -m ultraskelly --zed-only --port 8081` runs only the ZED and the dashboard,
without the OAK, Hailo or motors, so it can run while someone else uses the robot.

Add `--record` to save the run under `~/ultraskelly-recordings/run-<time>/`: the OAK video
as it reached the bot, the poses found in each frame, targets, head state and every motor
command (with `--zed`, also every IMU sample in `imu.csv`), with times on one clock so they
line up. `--microphone NAME` adds audio from the
sound card whose name contains NAME (see `arecord -l`). `run.json` holds the configuration
and counts any frames the disk could not keep up with.
`python -m scripts.analyze_run <run folder>` checks that a recording kept everything and
summarizes how the waist moved and how loud it was while moving.

### Run it from boot

```bash
scripts/install-services.sh --enable    # --disable stops it and keeps it from starting at boot
```

installs three services for the checkout the script lives in: `ultraskelly-bot`, a system
service that runs `scripts/run-bot.sh`; `ultraskelly-control`, the control panel on port
80, a system service that always runs and turns the bot on and off; and
`ultraskelly-dashboard`, the desktop user's service that opens the control panel full
screen. After any exit or
crash, systemd starts the bot again after 2 s, then waits longer each time, up to a
minute. If the bot's event loop stalls for 30 s, systemd kills it with SIGUSR1, which
first logs every thread's Python traceback. After every exit, crashes and kills
included, the service brakes the DC motors with `scripts/stop_motors.py`, giving up
after 10 s; it leaves them alone when the bot never started because another program
holds the robot lock. Missing or failing hardware never stops the bot: it runs with what
works, and the dashboard's health panel says what is missing and how to fix it (`health`
messages, [docs/stream.md](docs/stream.md)). A motor board that stops answering mid-run
(a write that still fails after its retries) ends the bot, and the restart starts without
it. A camera or detector that stops with an error is reported and the rest runs on; one
that stops sending while it still counts as working, 5 s without camera frames or the
Hailo detector's pose results after they had arrived, is the old freeze and ends the bot
for a restart (`StallMonitorNode`). Once the bot starts shutting down it stops feeding the watchdog,
so a shutdown that hangs is ended too. Re-run the script after moving the checkout.
Extra bot options go in `/etc/default/ultraskelly-bot`, for example while the motor
boards are disconnected:

```bash
echo 'ULTRASKELLY_BOT_OPTIONS=--simulate-motors' | sudo tee /etc/default/ultraskelly-bot
sudo systemctl restart ultraskelly-bot
```

```bash
systemctl status ultraskelly-bot                # state and recent log
journalctl -u ultraskelly-bot -f                # follow the log
sudo systemctl stop ultraskelly-bot             # free the robot for a test; start to resume
systemctl --user status ultraskelly-dashboard
sudo systemctl restart ultraskelly-control      # after pulling new code; it moves nothing
```

### One program at a time

Only one program may use the OAK, the Hailo-8 and the motor boards. Each holds
`/run/lock/ultraskelly-robot.lock` while it runs: `scripts/run-bot.sh` does, and test
scripts take it the same way:

```bash
flock -n /run/lock/ultraskelly-robot.lock .venv/bin/python -m scripts.motor_jog pan
```

While another program holds the lock, `scripts/run-bot.sh` exits with status 75 and
names the holder, and the service tries again later.

### When something goes wrong

- `journalctl -u ultraskelly-boot-report -b` says why the Pi last stopped: a reboot, a
  hang the hardware watchdog reset, or power that was lost or cut.
- A power loss can empty files written in the seconds before it, such as git objects or
  a saved Wi-Fi network; `scripts/setup-pi.sh --check` finds them. If the Wi-Fi does not
  come back, add the network again from the desktop's network menu. Shut the Pi down
  before re-plugging its power or USB devices.
- Reinstalling the `hailort-pcie-driver` package restores the unpatched driver, which
  hangs the Pi within minutes of inference; run `scripts/patch-hailo-driver.sh` again.
- The ZED Mini's video appears only with its USB-C plug one way round. If only its IMU
  shows up (`2b03:f681` in `lsusb`), flip the plug at the camera end.
