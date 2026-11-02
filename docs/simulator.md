# Local simulator and skeleton viewer

## Run

Requires Python 3.11+ and a WebGL2-capable browser. The tested environment is
Windows with Python 3.12.14; no platform-specific simulator code is used.

```sh
uv run --locked --python 3.12 python -m ultraskelly.simulator --open
```

This starts a loopback-only server at `http://127.0.0.1:8765`. Press **Run** to
advance time. Stop the server with Ctrl+C. If that port is occupied, use
`--port 8766`. Omitting `--open` leaves browser selection to you.

After `uv sync --locked --python 3.12`, you can also use the virtual environment
directly: `.venv/Scripts/python.exe -m ultraskelly.simulator` on Windows, or
`.venv/bin/python -m ultraskelly.simulator` on Linux/macOS. This avoids any package
resolution at launch. Setup downloads dependencies; runtime uses only local files
and loopback HTTP. No hardware factory is invoked, including on a Raspberry Pi.

## Hardware companion

Select **Hardware map** in the viewer header for the static wiring reconstruction.
See [hardware.md](hardware.md) for connection tables, evidence and unresolved power wiring.

## Explore

- **Run / Pause:** advance or freeze simulation time. Pausing freezes the entire
  simulated world, including actuator outputs and controller timers; it is not an
  emergency stop for a physical robot.
- **Step 10 ms:** pause and execute exactly one 100 Hz controller tick.
- **Reset:** rebuild controllers, clear history, and center the simulated joints.
  Tuning, target settings, and scenario selection are retained.
- **Sweeping target:** world-space target follows a repeatable sinusoidal path.
- **Manual target:** set world yaw/pitch relative to the stationary base. Moving
  either slider switches to this mode.
- **Sweep + periodic loss:** hide the target for three seconds in each eight-second
  cycle. **Target enabled** can suppress it in any scenario.
- **Apply tuning:** change gains, jaw activation zone, sensor rate, and the plant speed
  assumptions without editing code. Values are validated before application.
- **Export JSON:** download the last 30 seconds of sampled output at 20 Hz, plus
  current settings. This is a telemetry snapshot, not a complete replay file:
  it does not record previous tuning changes or the full input event history.
- **3D view:** drag to orbit, scroll to zoom, and use **Reset view** to restore the
  camera. The camera orientation is independent of robot control.

The camera panel shows the most recent sensor sample, not a video image. Its
dashed square activates the jaw; head tracking continues to the crosshair with a
one-pixel tolerance. The face-mounted green frustum shows the camera field of view,
and its orange rectangle corresponds to the jaw zone. Six traces show pan and tilt
commands/positions, waist and jaw angles, and both motor drives. The jaw drive
includes a state-color strip (gray closed, orange opening, green open, purple
closing). The table distinguishes servo-angle commands from DC motor throttle. The simulated waist/jaw angles are not measured robot state.

## Shared code and data flow

```text
scenario -> simulated camera -> TargetLocationTopic -> OrientationNode.step()
                                                     |
                                              ServoStateTopic
                                                     |
                                                MouthNode.step()
                                                     |
                  next camera sample <- plant <- fake actuator commands
```

- `ultraskelly/core/bot/motor/orientation_node.py` and `mouth_node.py`: the real
  robot `run()` loops and the simulator both call the same `step()` methods.
  Both paths share centering, lock freshness, and spring-return jaw behavior.
- Both controllers accept a callable clock. Hardware/realtime runs default to
  `time.monotonic`; the simulator injects `tick * 0.01`. Their factories, channel
  assignments, signs, limits, and startup tests are otherwise retained.
- `ultraskelly/replay.py`: fake actuator objects and controller construction;
  the earlier realtime scripted-input harness remains usable.
- `ultraskelly/simulator/engine.py`: deterministic scheduling, target generation,
  camera projection, plant model, and bounded history. Call `await sim.step(n)`
  to exercise it without a browser or server.
- `ultraskelly/simulator/__main__.py`: standard-library HTTP server plus one
  asyncio owner of simulation state. Requests use futures to enter that event
  loop. UI polling does not advance the simulation. Multiple tabs share one world.
- `ultraskelly/simulator/static/`: plain HTML/CSS/JavaScript and vendored Three.js.
  There is no build system, remote telemetry, hosted service, or runtime CDN.

The sensor publishes at 10, 20, 50, or 100 Hz. Each tick runs orientation, then
mouth, then the plant. This ordering makes tests repeatable; it does not reproduce
every interleaving or timing fault of concurrent Pi coroutines. Wall-clock load can
make the live simulator slower than realtime; it never changes the fixed timestep
to catch up. Run and step faults stop the simulator, zero fake motor outputs, and
display the error. Reset is required to resume after a fault.

## Plant assumptions

The head-mounted synthetic camera has a 640×480 image, 120° horizontal and 90°
vertical field of view. A pinhole projection uses the same world target position,
head transforms, and face-mounted camera origin as the viewer frustum. It does not render or infer a human pose.

Servos follow commands at a configured maximum angular speed (default 180°/s).
Waist velocity is `-throttle * waist_speed` (default 45°/s at full throttle), with
illustrative ±100° stops; a released (coasting) waist motor counts as zero drive. Negative mouth throttle opens the jaw toward 28° and
zero allows spring return, limited to 90°/s. Geometry is a schematic armature with
fixed arms and legs. These assumptions are intentionally simple and uncalibrated:
no inertia, torque, backlash, stalls, collisions, power limits, or I2C delays.

Head commands update once per sensor sample, continuing to a one-pixel centering
tolerance even inside the jaw activation square. Controllers retain lock across
normal gaps between samples, but expire missing input after 350 ms. Explicit
loss or leaving the square immediately removes jaw power. Negative drive holds
the spring-loaded jaw open for as long as lock remains valid; zero drive allows
spring return. Opening/closing state durations are estimates, not feedback.
The waist has one yaw axis and participates in the shared tracking controller
through `WaistDrive` (`motor/waist_drive.py`). When the head's pan is more than
15 degrees from neutral, the waist turns toward its gaze in one continuous move;
camera feedback brings the head back toward neutral, and the move continues until
the head is within 5 degrees. A move starts at 0.45 drive, below where the waist
is assumed to break loose, ramps up at 1.5 per second to a drive that grows with the
head's angle (0.7 at 15 degrees, up to 0.8), ramps down at 2 per second, releases the
motor to coast at 0.4, and brakes it 0.3 s later. Reversing passes through that stop.
A move lasting 6 s ramps down and rests 2 s. Target loss ramps the waist to a stop.
These levels are starting values until the waist sound test measures the real waist.
The state display shows idle, turning, stopping or resting.
The Pi launcher's 120-second restart workaround was removed on 2026-11-02;
the simulator does not run that launcher.

## Develop and verify

1. Reproduce the behavior with a scenario and tuning values.
2. Pause and step to inspect inputs, commands, and plant response.
3. Add a regression test using `Simulation(Settings(...))` and fixed tick counts.
4. Modify the shared controller step or the plant as appropriate.
5. Run `uv run --locked --python 3.12 python -m pytest -q`.
6. Restart the server after Python changes; reload the browser after HTML/JS/CSS
   changes. There is no automatic reload.

Tests cover centering inside the jaw zone, sustained jaw hold at 10 Hz, immediate
release on target loss, stale-input expiry, existing controller behavior, clock-driven timers, repeatable closed
loop tracking, target loss, bounded queues/history, HTTP control/tuning validation,
locally served assets, and failures during both Run and Step. On 2026-10-28:
26 tests passed on Windows; browser checks covered rendering, run/pause, step,
manual input, tuning, target loss, reset, and JSON export. No browser console
warnings/errors were observed. Linux/macOS and physical hardware remain unverified.

## Third-party rendering code

Only Three.js r180 and its OrbitControls addon are vendored here, from the official
`mrdoob/three.js` repository tag `r180`: `build/three.module.js`,
`build/three.core.js`, and `examples/jsm/controls/OrbitControls.js`.
Their license is preserved at `static/vendor/THREE-LICENSE.txt`. Upstream source:
https://github.com/mrdoob/three.js/tree/r180 . Runtime imports resolve locally.
Foxglove, its SDK, and MCAP are not used or planned.
