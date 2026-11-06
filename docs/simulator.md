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
- **Apply tuning:** change the head's top speed, the waist gain, the jaw activation zone,
  the sensor rate and the plant speed assumptions without editing code. Values are
  validated before application.
- **Export JSON:** download the last 30 seconds of sampled output at 20 Hz, plus
  current settings. This is a telemetry snapshot, not a complete replay file:
  it does not record previous tuning changes or the full input event history.
- **3D view:** drag to orbit, scroll to zoom, and use **Reset view** to restore the
  camera. The camera orientation is independent of robot control.

The camera panel shows the most recent sensor sample, not a video image. Its
dashed square activates the jaw; the head turns until the person is within its
1.5-degree deadband of the crosshair. The skull is drawn at the head's actual turn
through its geared linkage, not at the servo angles. The face-mounted green frustum shows the camera field of view,
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

The sensor samples at 10, 20, 30 (the default, as the OAK runs), 50 or 100 Hz, and
each sample reaches the controller `camera_latency` later carrying its capture time,
as the Hailo pose node's targets do. Each tick runs orientation, then mouth, then the plant. This ordering makes tests repeatable; it does not reproduce
every interleaving or timing fault of concurrent Pi coroutines. Wall-clock load can
make the live simulator slower than realtime; it never changes the fixed timestep
to catch up. Run and step faults stop the simulator, zero fake motor outputs, and
display the error. Reset is required to resume after a fault.

## Plant assumptions

The head-mounted synthetic camera sees as the OAK-1 does: a 640×480 image with a
69° horizontal field of view (`camera_hfov`), a pinhole with square pixels (8.1 px per
degree near the middle). The projection uses the same world target position, head
transforms, and face-mounted camera origin as the viewer frustum. It does not render or
infer a human pose: each sample reports the person the pose node would follow
(`choose_person`, shared with it), optionally with pixel noise (`noise_px`) and dropped
detections (`dropout`, from a seeded random generator). Frames reach the controller
`camera_latency` after capture (default 55 ms, measured 50-70 ms on the Pi).

Servos close on their commands with a first-order lag (`servo_lag`, default 80 ms: on the
robot the picture moved about 0.15 s after a command) and no faster than `servo_speed`
(default 180°/s). The head's linkage gears them down and has play, as measured on the robot
on 2026-11-06: the camera turns `pan_ratio` (0.36) degrees per pan servo degree and nods
`tilt_ratio` (0.1) per tilt servo degree, once the servo has crossed `pan_backlash` (6) or
`tilt_backlash` (2) servo degrees of slack after a reversal. So the head reaches only about
±30° of pan and ±9° of tilt. The roll servo turns the whole picture's content anticlockwise by
`roll_ratio` (0.3, not measured) degrees per servo degree. `pan_sign`/`tilt_sign`/`roll_sign`
can turn any servo the wrong way, to test polarity. Each sample also reports every visible
face as the OAK's face detector would: eyes, nose and mouth sized for the person's distance
(eyes 63 mm apart), tilted in the picture by the person's own tilt (`Person.roll`) less the
head's roll, with their own jitter (`landmark_noise_px`). Waist velocity is
`-throttle * waist_speed` (default 30°/s at full throttle; round 1 measured 18-23°/s at
0.75), with illustrative ±100° stops; a released (coasting) waist motor counts as zero drive.
Optional friction (`waist_breakaway`, `waist_stall`; both 0 by default, and set only
through the configure request, not the tuning form) makes a still waist wait for the
breakaway drive and a turning one stop below the stall drive, with speed scaled
between the stall drive and full drive. Negative mouth throttle opens the jaw toward 28° and
zero allows spring return, limited to 90°/s. Geometry is a schematic armature with
fixed arms and legs. These assumptions are intentionally simple: no inertia, torque,
collisions, power limits, or I2C delays, and the head's tilt offset at rest is not modelled.

The head turns through `HeadAim` (`motor/head_aim.py`). It works out where the person
is from where the camera pointed when the frame was captured, aims 65% of the way per
frame (so a scale or gear-ratio error cannot overshoot), smooths that goal, turns at most
40°/s of view with S-curves, keeps 5 servo degrees clear of the end stops, and moves only
once its goal is more than 1.5° away. It counter-rotates against the waist's estimated
turn at once, and commands each servo half its linkage's slack past the view in the way it
is moving, so a reversal takes the play up at once instead of leaving the head still. The
roll (`motor/roll_level.py`) levels the followed face in the picture the same calm way, and
eases back upright a second after the face is gone. Controllers retain lock across normal gaps between samples, but expire
missing input after 350 ms. Explicit loss or leaving the square immediately removes jaw
power. Negative drive holds the spring-loaded jaw open for as long as lock remains valid;
zero drive allows spring return. Opening/closing state durations are estimates, not feedback.
The waist has one yaw axis and participates in the shared tracking controller
through `WaistDrive` (`motor/waist_drive.py`). When the person's bearing from the body is
more than 15 degrees, and the head has held still for 0.3 s (or the person is beyond
40 degrees), the waist turns toward them in one continuous move while the head
counter-rotates; the move continues until the bearing is within 5 degrees; a stopping
move never resumes, so the waist rests between the two angles instead of hunting. A move starts at 0.15 drive and follows
S-curves (rate up to 1.0 per second rising and 2 falling, its change limited to 10
per second squared) toward a running drive of 0.5 at 15 degrees, up to 0.6. While
the bearing shows no progress (2 degrees within 0.5 s at the running drive), the drive
rises 0.3 per second toward 0.7 to break the waist loose, and eases off as soon as
the bearing shrinks again. A stopping move ramps down, releases the motor to coast at 0.2,
and brakes it 0.3 s later. Reversing passes through that stop. A move lasting 6 s
ramps down and rests 2 s. Target loss ramps the waist to a stop. On the robot the
waist motor brakes between PWM pulses (slow decay). These levels follow published
guidance and round 1's measurements (see the audit), not yet a run on the robot.
The state display shows idle, turning, stopping or resting.
The Pi launcher's 120-second restart workaround was removed on 2026-11-02;
the simulator does not run that launcher.

## Closed-loop scenarios

`ultraskelly/simulator/scenarios.py` runs the production controllers through this plant
with people placed in front of the robot, and judges each run before any motor turns:
someone stepping in 20° left or right, 6° up or down, 27° off (beyond the head's reach, so
the waist must turn), walking across, walking off out of reach, two people whose detector
scores keep swapping, a noisy detector that drops a third of its detections, and a person
who leaves while another appears. A run fails on overshoot (the camera more than 2° past
the person), oscillation (the head's actual turn or nod reversing by 0.5° more often than
the scenario allows), runaway (a servo command at its end stop, or losing a person who
stays put), aim (more than 2.5° off at the end) or waist hunting (more moves than allowed,
or turning back on a person who stands still). Five more put a person's tilted face in front
of it (7° either way, tilting then straightening, leaving, and noisy and flickering), and fail
if the roll turns the face more than 1.5° past level, leaves it more than 3° off, or does not
stand the head upright once the person has gone; in every scenario the roll must not turn back
and forth.

```sh
uv run --locked python -m pytest -q tests/test_scenarios.py
```

The tests run every scenario on the plant above and at the edges of what was measured or
assumed: camera delay 40-100 ms with servo lag 40-100 ms and field of view 64-74° (their
corners), servo lag up to 120 ms, pan ratio 0.28-0.42, tilt ratio 0.07-0.14, pan slack 3-8
and tilt slack 0-4 servo degrees, waist 17-37°/s, and roll ratio 0.2-0.45. A reversed pan
servo must be caught as a runaway, and a reversed roll by the faces it leaves tilted. `scenarios.run(scenario, prepare)` returns
the trace and the verdict, so a tuning change can be checked the same way; `prepare`
receives the `Simulation` before the run to change controller parameters.

## Develop and verify

1. Reproduce the behavior with a scenario and tuning values.
2. Pause and step to inspect inputs, commands, and plant response.
3. Add a regression test using `Simulation(Settings(...))` and fixed tick counts, or a
   closed-loop scenario when the behavior concerns tracking.
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
