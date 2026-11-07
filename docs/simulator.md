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
`camera_latency` after capture (default 60 ms, measured 50-70 ms on the Pi).

The head's servos and the waist are the robot as measured (`simulator/plant.py`): each
servo is a chain fitted to a recorded run (tests/data/motor-response-20261106) of steps,
staircases and ramps watched by the camera,

    command --dead time--> servo (slew-limited) --slack--> linkage x gain --> head mode --> view

where the head mode is the head's own ringing: a second-order system with dry friction and
damping that differs by the way it turns. Fitted by least squares on every segment's view and
the big steps' overshoot, settling and dead time:

| | gain (view °/servo °) | dead time | slew | slack | mode | damping up / down | friction |
|---|---|---|---|---|---|---|---|
| pan | 0.261 | 44 ms | 576 °/s | 2.5° | 11.8 rad/s (1.9 Hz) | 0.41 / 0.28 | 19 °/s² |
| tilt | 0.081 | 40 ms | 830 °/s | 1.8° | 38 rad/s | 0.64 / 0.37 | 22 °/s² |
| roll | 0.063 | 43 ms | 375 °/s | 12° | 82 rad/s | 0.30 | 7 °/s² |

Replaying every recorded command through the plant gives the recorded views within 1.01° RMS
(pan), 0.58° (tilt) and 0.40° (roll), each big step's overshoot within 8 points and its settling
within 0.25 s (tests/test_plant_fidelity.py, the gate any tuning on the simulator relies on).
Inside the servo windows the robot uses (pan 20-160, tilt 20-130) the head reaches about ±17°
of pan and 5° up and 2.7° down of tilt; beyond them the real linkages jam or go over centre,
which the plant does not model. The roll turns the picture's content clockwise for a larger
angle (`roll_sign` -1) and nods the camera down `roll_nod` (0.18) degrees per servo degree as it
does. The settings vary the measured models for robustness tests: `pan_ratio`, `tilt_ratio`,
`roll_ratio` (gains), `*_backlash` (slacks), `pan_frequency`, `pan_damping`, `pan_damping_down`
(pan's mode), `delay_scale` (every dead time), and `pan_sign`/`tilt_sign`/`roll_sign` turn a
servo the wrong way, to test polarity. Each sample also reports every visible face as the OAK's
face detector would: eyes, nose and mouth sized for the person's distance (eyes 63 mm apart),
tilted in the picture by the person's own tilt (`Person.roll`) less the head's roll, with their
own jitter (`landmark_noise_px`). The waist (`plant.WAIST`) needs more than 0.6 drive to break
loose and turns 50.4°/s at full negative drive but 21.5°/s at full positive drive, reaching
speed in 0.16 s; its six recorded pulses replay within 0.5°. Negative mouth throttle opens the
jaw toward 28° and zero allows spring return, limited to 90°/s. Geometry is a schematic
armature with fixed arms and legs; the head's tilt offset at rest is not modelled.

The head turns through `HeadAim` (`motor/head_aim.py`), its aim tuned on this plant ("Tuning
the head" below). It works out where the person is from where the camera pointed when the
frame was captured (the commands of 60 ms before, about pan's dead time), aims 30% of the way
per frame (so the goal approaches without passing the person while the real gearing is under
3.3 times what it assumes), takes 69% of each new estimate into its goal, turns at most 15°/s
of view along S-curves that accelerate at most 34°/s² (a change of acceleration that large
starts ringing of about a/ωₙ² = 34/11.84² = 0.24° in pan's 1.9 Hz mode, under the deadband, so
the input shaper it has is left off), keeps 5 servo degrees inside its servo windows (pan
20-160, tilt 20-130), and moves
only once its goal is more than 0.39° away. A goal more than 8° from the last (someone new, or
a false detection), and any within 40 px of the picture's left or right edge, must show up five
times with no more than 0.2 s between sightings before the head turns to it. Not the bottom
edge: the head cannot look down to a standing person, whose face then sits low in the picture
(`scenarios.LOW_FACE`, 204-238 px below the middle, pans as one at eye level does). It counter-rotates against the waist's estimated
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
someone stepping in 20° left or right, 4° up or 2° down, 27° off (beyond the head's reach, so
the waist must turn), walking across, walking off out of reach, two people whose detector
scores keep swapping, a noisy detector that drops a third of its detections, and a person
who leaves while another appears. A run fails on overshoot (the camera more than 2° past
the person), oscillation (the head's actual turn or nod reversing by 0.5° more often than
the scenario allows), runaway (a servo command at its end stop, or losing a person who
stays put), aim (more than 2.5° off at the end) or waist hunting (more moves than allowed,
or turning back on a person who stands still). Seven more put a person's tilted face in front
of it (7° and 3°, either way, tilting then straightening, leaving, and noisy and flickering),
and fail if the roll turns the face more than 1.5° past level, ends with it neither within 1.5°
of level nor a degree nearer level than the person's own tilt (the roll reaches about 2°), or
does not stand the head upright once the person has gone; in every scenario the roll must not
turn back and forth.

```sh
uv run --locked python -m pytest -q tests/test_scenarios.py tests/test_scenarios_tuning.py
```

`tests/test_scenarios.py` runs the whole robot, head, waist and roll, on the measured
plant. The waist is not tuned on it yet (it breaks loose only above 0.65 drive and turns
50°/s one way, 21.5°/s the other), and a person 20° off is beyond the head's reach inside its
servo window, so "left", "right" and "far" are expected to fail, strictly: they must start
passing once the waist is tuned. So is "tilt-noisy": the roll, also untuned on this plant,
reaches about 1.3° of picture and stops within its 1° deadband of that, just short of the
degree nearer level the judge asks of a noisy face. With its ratio anywhere in the fit's spread
(0.050-0.083), 8-16 servo degrees of slack and a nod of 0-0.3 the roll must stay calm: never
turning back and forth or a face past level, and upright again once the face has gone. A
reversed pan servo must be caught as a runaway, and a reversed roll by the faces it leaves tilted.

The robot runs its head alone for now (waist braked, roll upright). `HEAD_BATTERY` is that
setup's seventeen scenarios: steps of 4-16° that the head can reach, 4° up and 2° down (as far as
tilt reaches below its rest), someone beyond its reach, walking across, two people with
swapping scores, a noisy detector, a person leaving while another appears, and false
detections of 1-3 frames at a time (the robot's: run 20261106-194104), at the picture's edge
on alternate sides, at one spot by the edge, in the middle, and a half-seen person at the
edge, where the camera must not move by more than 0.5°. A run also records how long the
camera took to settle within 1.5° of a still person, how far the servos travelled and how
far the camera wandered. `tests/test_scenarios_tuning.py` runs the launched head through the
battery on the measured plant and checks it stays calm (no overshoot past 2°, oscillation,
runaway or chasing) on the plants the tuner weighed and on others: camera delay 40 or
100 ms, field of view 64 or 74°, and a tilt that barely turns or has 20 servo degrees of
slack, as tilt measured on the evening of 2026-11-06. It also checks that the battery catches
a head that swings and one that adds up false detections. `scenarios.run(scenario, prepare)`
returns the trace and the verdict, so a tuning change can be checked the same way; `prepare`
receives the `Simulation` before the run to change controller parameters.

## Tuning the head

`python -m ultraskelly.simulator.tuning [samples] [checkpoint.json]` tunes the head's aim
(`HeadAimParams`: reach, smoothing, top speed, acceleration, deadband, servo lag, jump
samples, the input shaper on or off) on the head battery. Each candidate runs all seventeen
scenarios on the measured plant (weight 1) and on six stressed ones (weight ½ each): pan
gearing 15% either way, 1.5 and 2 times (a tracking run once looked like twice), a head that
rings at 45% instead of 22-32%, and 1.5 times every dead time. A run costs

    5 × overshoot (°) + 3 × reversals beyond the scenario's allowance
    + settle time (s; the whole run if it never settles) + 2 × final error (°)   [still people within reach]
    + 0.002 × servo travel (°) + 5 × wander (°)   [false detections only]
    + 30 × each hard failure (runaway, a person lost, chasing false detections, aim)

The search samples the space at random, then refines the best setting one parameter at a
time, the step halving from 15% of each range to under 1%. A candidate takes about 4 s on
one worker; 120 samples and the refinement take a few minutes on eight. After every round
the checkpoint holds the best setting and the step, and a run given that checkpoint carries
on from it. Each worker reuses one event loop for all its runs: on Windows a new loop opens a
loopback socket pair whose port is held for about two minutes after it closes, and eight
workers' thousands of runs a minute would use up the machine's 16,384 ports. The pool stalled
twice before that change (and before workers stopped being replaced after 40 tasks); a batch
that stalls now stops the tuner, its checkpoint kept.

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
