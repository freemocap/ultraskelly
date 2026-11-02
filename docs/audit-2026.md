# UltraSkelly revival audit — 2026-10-28

**Follow-up:** the first local simulator and skeleton viewer are now implemented;
see [the simulator guide](simulator.md) for the current architecture and checks.
The user explicitly rejected Foxglove, its SDK, and MCAP after this audit. Those
options below are historical research only and are not part of the plan.

**Follow-up, 2026-11-02:** The Pi was re-imaged (Raspberry Pi OS Trixie, Python
3.13) and its hardware observed: no IMX500 ribbon camera; a Luxonis OAK-1 and a
Stereolabs ZED Mini on USB 3; a Hailo-8 AI HAT+ on PCIe; servo and motor boards at
`0x40` and `0x60`. See [hardware.md](hardware.md). The default launch path is now
`DetectorType.HAILO_POSE` (OAK camera, YOLOv8 pose on the Hailo-8); the IMX500 path
remains selectable. At the owner's direction the 120-second restart was removed:
`BotLauncher.run` stops all nodes when any node exits and re-raises a crash. A
150-second camera and pose run held 30 fps without motors. The historical freeze is
not reproduced or certified fixed, and no motor soak has run. The cleanup deletions
described below were committed in `5a25f7b`.

## Scope and evidence

Inspected the tracked repository at `c32fbd6` on Windows without the robot.
The working tree was clean before this audit. There was no AGENTS.md, test suite,
or substantive README. This is an audit, cleanup, and first portable test harness;
the 3D simulator and Pi qualification are subsequent stages.

The likely show application is `ultraskelly.__main__bot`: the VS Code launch
configuration points to it, and commit `c30a0fd` (2025-11-17, “restart every 2 mins
to avoid dumb freeze”) changes its launcher. The ordinary package entry point
previously started a separate voice API. Repository evidence does not establish
which checkout, environment, or local modifications were actually on the show Pi.

## The retained robot

```text
IMX500 / Picamera2 -> PoseDetectorNode -> TargetLocationTopic -> OrientationNode
                         |                                      |
                         | FrameTopic + PoseDataTopic           | ServoStateTopic
                         v                                      v
                      OpenCV UI <---------------------------- MouthNode subscribes
```

The UI subscribes to all four topics. MouthNode consumes servo lock flags; it
does not publish servo state. The arrows above describe information flow, not
separate processes: these nodes share one asyncio event loop.

| Area | What it does | Status |
| --- | --- | --- |
| `__main__bot.py`, `bot_launcher.py` | Pydantic configuration; construct nodes; stop all when any node exits (120-second restart removed 2026-11-02) | Retain; lifecycle needs work |
| `sensory/pose_detection_node.py` | IMX500 HigherHRNet COCO inference; select highest-score person and nose; publish target, pose, frame | Active default; Pi-only |
| `motor/orientation_node.py` | Increment head angles from pixel error; assist with waist DC motor | Active; tests now use fake actuators |
| `motor/mouth_node.py` | Open/ramp-close state machine based on head lock | Active; tests now use fake actuators |
| `ui/cv2_ui.py` | Camera image, skeleton overlay, target, servo state | Active; retain despite old browser UI removal |
| `sensory/camera_node.py`, `bright_point_detection_node.py` | Alternative bright-object tracking pipeline | Reachable config option; not hardware-validated |
| `motor/head_node.py`, `waist_node.py` | Earlier separate controllers | Not constructed by current launcher; candidates after import cleanup |
| `scratch/` | Manual motor tests, IMX500/Hailo trackers, ESP32 experiment and firmware | Quarantine by convention; retain wiring evidence |
| `replay.py`, `tests/` | Inject fake actuators and test the real retained controllers and bus | Added during this audit |

### Hardware facts encoded in source

- Pan servo channel 11, tilt channel 3, software range 0–180 degrees, startup 90.
- Waist MotorKit at I2C address `0x60`, channel 1, PWM 1600 Hz, magnitude roughly
  0.7–0.8; mouth independently constructs another MotorKit at the same address,
  channel 2. Shared board ownership needs review.
- Mouth construction drives +1 for 0.5 seconds, then -1 for 0.5 seconds, then 0.
  This happens again on every recreation. Do not treat construction as passive.
- Pose model path: `/usr/share/imx500-models/imx500_network_higherhrnet_coco.rpk`.
  Defaults: 640×480, 10 FPS inference, camera flipped in both axes.
- Roll exists in the old head controller, but the active orientation controller
  publishes a constant 90-degree roll with `is_locked_roll=False`.
- These are software settings, not measured physical limits. The current runtime
  has no encoder-based waist angle, measured jaw angle, or joint calibration file.

## Cleanup decisions

Removed 115 tracked legacy files. The exact list is in
[cleanup-files.txt](cleanup-files.txt); originals remain available at `c32fbd6`.
No hardware scratch file or firmware image was removed. Empty directories can
remain locally because deletion targeted tracked files rather than directory trees.

| Removed area | Evidence and reason |
| --- | --- |
| `ultraskelly-ui/` | Next.js assistant chat app; chat API calls an AI model and has no motor connection |
| `ultraskelly-ui-og/` | Earlier Vite voice client; endpoints and audio workflow separate from bot launcher |
| `mic-waveform-visualizer-extract/` | Standalone copy of the old voice microphone component; no robot references |
| `ultraskelly/api/`, `app.py`, `ultraskelly_app/` | Voice key/audio endpoints, placeholder conversations, empty application model; not launched by bot |
| `ui/gradio_ui.py` | No caller in retained launcher; synchronous `run()` incompatible with async node launching; thread uses asyncio queues and catches the wrong queue-empty exception |
| `system/`, `utilities/` | Legacy API paths, custom logging/WebSocket queue, port-killing helper and trivial waits; all consumers removed or replaced |

Replaced import-time global logging with explicit standard logging in the hardware
entry point. Removed unused web/plotting dependencies; declared NumPy directly,
added the previously missing MotorKit dependency to an optional hardware extra,
and added pytest to the development group. Refreshed `uv.lock` while preserving
existing compatible core versions. The hardware extra is not a tested Pi image.

Further deletion candidates: the older head/waist controllers after moving message
imports out of them, Hailo/ESP32 experiments after checking the physical setup,
and tracked IDE metadata. They are not proven broken merely because inactive.

## Reliability findings

### Reproduced and addressed on Windows

1. **A slow subscriber stalls the publisher.** Original subscriptions have capacity
   100 and `publish()` awaits each subscriber sequentially. A test stops consuming
   one subscription and sees the 101st publication time out, also starving a healthy
   consumer. Current state streams now overwrite stale values without waiting.
2. **Publication history retains every frame.** `create_topic()` overrides the
   bounded base queue with an unbounded queue; nothing reads that queue. The test
   observes 1,000 retained messages after 1,000 publishes. It now retains at most
   one. At 640×480×3 uint8 and 10 FPS, unique frame arrays alone could retain about
   9.2 MB/s before overhead; this is an estimate, not a measured Pi memory trace.
3. **Close leaves queued references alive.** Tests now require both publication
   and subscription queues to be drained when the topic closes.

These are state streams, not reliable command queues. Dropping stale states is
intentional and can skip brief transitions. Any future discrete command channel
needs its own acknowledgment/overflow policy. The existing consumers already
favor the newest state. The optional publication factory remains supported, but
publication history is still reduced to the newest value regardless of capacity.

### Identified in source; historical cause not established

- **Blocking log sink:** package imports configured a bounded 1,000-entry
  multiprocessing queue. Handler `emit()` used blocking `put()`; only the voice
  WebSocket code drained it. Bot-only runs can therefore block the entire event
  loop once enough logs arrive. Removing the unused logging stack eliminates this
  path. No show-floor log was available to match its timing to the reported freeze.
- **Subscriptions survive restarts:** launcher reused the manager while old node
  queues remained registered. Recreated publishers could block on abandoned queues.
  Shutdown now closes the old bus and creates a fresh one; new launchers also own
  independent managers. Full launcher restart testing still needs portable node
  factories; the current tests cover bus retirement, not Pi resource recreation.
- **Unsupervised task failures:** launcher creates tasks and sleeps for 120 seconds,
  observing exceptions only at shutdown via `gather(return_exceptions=True)`.
  A failed node may silently leave siblings running. Superseded 2026-11-02: the
  owner removed the timer, and the launcher now stops all nodes on the first exit
  and re-raises crashes. A hardware soak run is still outstanding.
- **Blocking camera calls:** `capture_array()`, camera startup/stop, OpenCV display,
  and I2C writes run in or alongside the event-loop path. An asyncio timeout cannot
  interrupt a blocked synchronous call on that same thread. Instrument first;
  establish camera-thread ownership or process isolation for non-cancellable calls.
- **Non-atomic detection state:** camera callback separately updates keypoints,
  scores, and boxes while the coroutine reads them. Person-count changes can expose
  inconsistent snapshots. Missing inference outputs also leave older results live.
- **Missing timestamps:** targets have no acquisition time or sequence. The pose loop
  republishes latest detections; controllers cannot distinguish a fresh target from
  a stale one. Conversely, their 100 Hz loops treat an empty queue as immediate
  target/lock loss between roughly 10 Hz camera updates, producing pulses and mouth
  state churn. Separate freshness timeout from “no new message this tick.”
- **Time and safety:** controllers use wall-clock time. Waist centering is explicitly
  disabled; its state may still report CENTERING. Shutdown re-centers servos, which
  is movement, and partial factory failures have no complete rollback. Cancellation
  tests verify fake motors stop; they cannot guarantee a stuck Pi/I2C call stops.
- **Configuration mismatch:** `__main__bot` passes `head=HeadNodeParams(...)`, but
  LaunchConfig has no active `head` field and silently ignores it. Actual gains
  come from `orientation`. Validate config extras before retuning on hardware.
- **Display mismatch:** UI requires roll lock for “LOCKED”; active orientation never
  reports it. Displayed lock and mouth behavior therefore disagree.
- **Packaging:** Pi imports previously existed without explicit NumPy, OpenCV, or
  MotorKit requirements. Picamera2/libcamera are OS-coupled and remain outside the
  portable dependency set. Camera/display imports still prevent importing the full
  launcher on Windows; only the controllers and bus are portable so far.

## Lightweight tools considered

Primary documentation checked 2026-10-28. Recommendations below are engineering
judgments for this small robot and the two-week deadline, not benchmark results.

| Tool | Useful capability | Recommendation |
| --- | --- | --- |
| [Python 3.11 TaskGroup](https://docs.python.org/3.11/library/asyncio-task.html#task-groups) | Own child tasks; propagate failures; cancel siblings | Adopt next for lifecycle supervision; already in supported Python |
| [Three.js Object3D](https://threejs.org/docs/pages/Object3D.html) | Hierarchical transforms for a small jointed armature | Preferred custom viewer if needed; keep control logic in Python |
| [Foxglove SDK](https://docs.foxglove.dev/docs/sdk) | Python live visualization and MCAP logging; MIT-licensed SDK | Time-box a local replay/telemetry trial before writing a large dashboard; assess viewer terms separately |
| [MCAP](https://mcap.dev/guides/getting-started) | Record/play back timestamped robotics data | Add when camera/pose traces outgrow JSONL; unnecessary for first controller tests |
| [MuJoCo Python](https://mujoco.readthedocs.io/en/stable/python.html) | Native simulation bindings | Defer unless inertia, collisions, or actuator physics become essential; uncalibrated dynamics would mislead |
| [ROS 2 installation](https://docs.ros.org/en/jazzy/Installation.html) | Broader robotics ecosystem | Defer migration: this four-node in-process system has no demonstrated need for middleware replacement. Official installation page access was blocked during this audit, so no release/platform compatibility claim is made |
| [Raspberry Pi AI Camera guide](https://www.raspberrypi.com/documentation/accessories/ai-camera.html) | Supported IMX500/Picamera2 integration | Reconcile the camera adapter with upstream examples during Pi bring-up; keep the existing inference hardware |

No new robotics framework was installed. Best immediate return: bounded state
streams, explicit time, lifecycle supervision, and reproducible replay.

## Revival stages and acceptance gates

1. **Portable baseline — delivered here.** Small install, safe default CLI, actual
   controller imports without drivers, queue regressions, fake-actuator replay,
   architecture notes, cleanup manifest. Check: `python -m pytest -q` passes and
   replay emits bounded commands without hardware modules loading.
2. **Deterministic controller core and lifecycle.** Extract `step(target, now, dt)`
   using monotonic/injected time; retain existing behavior with characterization
   tests before changing it. Add immutable timestamped detection snapshots,
   explicit actuator ports, freshness deadlines, and one shared hardware owner.
   Test centering, extremes, dropped/stale/invalid input, mouth transitions, slow
   subscribers, task exceptions, partial startup, cancellation, and 100 restart
   cycles. Gate: fake motors zero after every failure; queues/subscriptions remain
   bounded; replay produces identical output with the same seed and clock.
3. **Minimal simulation and 3D view.** Model a base, waist yaw, neck pan/tilt, skull,
   and jaw; fixed arms are sufficient initially. Use a simple kinematic plant with
   configurable travel limits, rate limits, delay, and a throttle-to-speed estimate.
   Project a world-space target back into the simulated camera to close the loop.
   Feed the same Python controller; viewer only renders snapshots and chooses test
   scenarios. Start with JSONL replay, then add a local socket only if live control
   is useful. Show commands versus simulated positions and label unknown waist/jaw
   calibration. Gate: pause, step, replay, target loss, saturation, and delayed input
   work; disconnecting the viewer cannot stall control. This viewer is not built yet.
4. **Pi bench bring-up.** Capture Pi model, OS/Python versions, package inventory,
   deployed revision, model file, wiring, power setup, and physical travel limits.
   Preserve the old working environment before changes. Verify camera/inference
   alone, actuators individually at conservative limits, then combined control.
   Record loop latency, memory, message ages, task failures, and motor state. Gate:
   explicit stop and failure paths observed on hardware; no unexplained process exits.
5. **Show qualification.** First run at least 30 minutes, then a two-hour soak
   without scheduled resets; exercise occlusion, crowded targets, UI loss, camera
   failure, restart, and operator stop. Require stable post-warmup memory and bounded
   command age. Capture diagnostics before any automatic restart; only then use an
   OS service as recovery. Keep a known-good deployment and operator restart procedure.

The next implementation slice should be stage 2, especially task supervision and
freshness. A visually attractive armature cannot establish that the motors stop
when a camera or controller fails.

## Live dashboard transport (2026-11-02)

At the owner's direction the dashboard moved from JSON snapshots (`/ws`,
`/api/state`) plus annotated MJPEG (`/video/oak.mjpg`) to FreeMoCap's
self-describing CBOR stream on one WebSocket, `/stream`. The Pi encodes with
`freemocap-stream`; the browser decodes with the vendored `@freemocap/client`
(both from FreeMoCap's `sfn26-ultraskelly` branch). Each frame message carries
the OAK JPEG with the keypoints the Hailo detector found in that same image
(`PoseDataMessage.frame`), so the browser draws the pose, target, aim point and
jaw zone itself. `robot_state` messages carry the rest at 20 Hz.

Measured on the Pi with the bot at `--hardware --simulate-motors --no-window`,
nobody in view, viewer 1 the Pi's full-screen Chromium window and viewer 2 a
client on another computer counting what arrives over 20 s. CPU is from
`/proc` over the same 20 s; "of a core" sums all threads of the processes.

| | MJPEG + JSON | FreeMoCap stream |
| --- | --- | --- |
| Viewer 2 receives | 30.1 JPEG/s (32 KiB), 19.6 JSON/s | 30.1 frames/s (29 KiB images), 18.9 `robot_state`/s |
| Viewer 1 shows | not measurable (`<img>` MJPEG) | 30 fps drawn, camera at 30.1 fps |
| Bot process, 1 / 2 viewers | 68% / 70% of a core | 73% / 72% of a core |
| Chromium on the Pi, 1 / 2 viewers | 83% / 90% of a core | 109% / 106% of a core |
| Whole Pi, 1 / 2 viewers | 41% / 43% of 4 cores | 47% / 47% of 4 cores |

Delivery keeps camera rate, and extra viewers stay cheap because each frame is
encoded once. The cost moved toward the browser: the Pi's Chromium spends about
a fifth of a core more than with native MJPEG. Replaying 120 captured frames
there, the SDK's decoding and validation take 0.13 ms per frame, JPEG decoding
2.59 ms and drawing 0.82 ms, so the image path, not the message format, is the
cost. The bot spends about 4% of a core more. The stream figures were measured
again after the Hailo driver fix below; samples taken during the warning storm
had been within 3% of them. Single samples with the same scene and viewers. Restarting
the bot from a non-interactive shell also exposed that background jobs inherit
an ignored SIGINT; `python -m ultraskelly` now restores Ctrl+C and maps SIGTERM
to the same clean shutdown.

### Hailo-8 driver warning storm and reset (2026-11-02)

The Pi reset itself at 10:12 with the bot running. The previous boot's kernel
log holds 11,707 identical warnings between 10:05:22 and 10:11:52, about one per
inference, then stops with no shutdown sequence; systemd arms the hardware
watchdog with a one-minute timeout. Every warning is `find_vma` asserting the
memory-map lock, called from `hailo_vdma_buffer_map` in `hailo_pci` 4.23.0 (DKMS
module, kernel 6.18.50+rpt-rpi-2712) on the bot's inference thread: the driver
looks up the process's memory map without the lock while other threads allocate
and free frames. HailoRT re-maps the bound input and output buffers on every run.

Earlier that boot the bot ran 55 and 31 minutes without one warning; the storm
began about a minute after the 10:04 start. After the reset it began within 20
seconds of every start (234 warnings in a 10-second run), and stopping the bot
stops it. Power was not throttled. Until this is fixed, keep Hailo runs short
and supervised. Candidate fixes: pre-map the buffers once if HailoRT exposes
DMA mapping, or a `hailo_pci` that takes the lock. This may or may not be the
historical one-minute freeze, which predates the Hailo.

**Fixed (2026-11-02).** The call is unconditional for every user buffer that is
not a dma-buf, and HailoRT maps the bound buffers on every inference; the kernel
warns whenever no other thread happens to hold the lock, so a quiet process
warns on nearly every inference. `scripts/hailo_soak.py`, the detector's setup
without camera or motors, reproduced it on demand: 532 warnings in 15 s.
hailort-drivers 4.24.0 (branch `hailo8`) changes exactly this call to take the
memory-map read lock; its `memory.c` differs from 4.23.0 only there.
`scripts/patch-hailo-driver.sh` applies that change
(`scripts/hailo_pci-4.23.0-find-vma-lock.patch`) to the DKMS source, rebuilds,
installs and reloads the module, and is safe to re-run. DKMS rebuilds the
patched source after kernel updates; reinstalling the `hailort-pcie-driver`
package restores the unpatched source, so run the script again then. After the
fix: 18,000 soak inferences in 10 minutes and a bot run with the dashboard
watching produced no warnings.

## Motion: waist drive, motor writes and motor tests (2026-11-02)

**Why the waist pulsed.** The orientation controller drove the waist at 0.7-0.8 for at most
0.5 s, then braked it (`throttle = 0` shorts the motor; `None` would let it coast) for a
0.4 s cooldown, repeating while the head was turned, and idled it whenever the head lost
lock unless the head was over 45 degrees off. A first sound-and-camera round with an older
microphone (`scripts/waist_sound_test.py`) heard broadband clicks at every start and stop
and a PWM whine near 1.6 kHz with harmonics. The camera saw the torso swing back at
12-25 deg/s after each brake, a wobble of about 2 Hz. The motor board's PCA9685 already
runs at its highest PWM frequency, so the whine cannot be moved above hearing.

**Continuous ramped drive.** `motor/waist_drive.py` replaces the pulses. The waist starts
when the head's pan is over 15 degrees from neutral, regardless of lock, and keeps turning
toward the gaze until the head is within 5 degrees. Each move starts at 0.45, ramps up at
1.5 per second to 0.6 plus 0.008 per degree beyond 15 (at most 0.8), ramps down at 2 per
second, releases the motor at 0.4 to coast, and brakes it 0.3 s later. Reversing passes
through that stop. A move lasting 6 s ramps down and rests 2 s, the remaining limit on
continuous drive. Losing the target ramps the waist down; stopping the bot still brakes at
once. These levels are provisional: the simulator (a linear plant) shows 9-17 moves of 1-6 s
per minute of a sweeping target instead of half-second bursts, but no robot measurement
exists yet. Round 2 of the sound test with the Yeti X (ALSA card 3, mixer 0-21 dB, run at
12) waits for the robot's rebuild, as does a recorded tracking run comparing old and new.
The sound test now centers and holds the head, refuses to run while the OAK sees too
little detail (Laplacian variance of a 160x120 frame: over 1300 in the room, 9 with the
lens covered), bounds its return move by the measured angle, measures where the waist
breaks loose and stalls, and has a `drive` profile that replays the bot's own drive.
`scripts/analyze_run.py` summarizes a recorded run: what was kept, the waist's moves and
stops, and the sound while it moves.

**Motor board writes.** While the servo board's loose jumper disturbed the I2C bus, motor
board transfers failed with EAGAIN after lost arbitration. adafruit_motor writes a DC
motor's two bridge inputs in separate transfers, and a brake writes the first input fully
on before the second, so a brake cut short there drives the motor at full power. The
recording wrappers now retry a failed write (3 attempts, stops 10). They also send an
unchanged throttle again only every 0.5 s: the 100 Hz controllers had been rewriting both
motors' unchanged values, about 400 transfers a second on a 100 kHz bus, roughly a fifth of
the event loop's time. A bus that dies for good between a brake's two transfers remains
uncovered; writing both channels in one transfer (motor 1 uses channels 9 and 10, motor 2
uses 11 and 12) would close it. If the bot exits without its cleanup (killed, or crashed
in native code), the motor board keeps its last command: `scripts/stop_motors.py` brakes
all four motors and suits a service's `ExecStopPost`.

**Motor tests.** The dashboard can move one actuator at a time through its own command
contract (`motor/motor_test.py`, `ui/motor_test_routes.py`). Each test is answered once it
has finished or been stopped; one that is malformed, outside the test limits or arrives
while another runs is refused at once with the reason, never queued, merged or silently
clamped. Test limits: servos within 30 degrees of center, waist up to 0.7 for 0.5 s and at
most 1.0 throttle-seconds net one way (there is no waist angle sensor), jaw up to 1.0 for
0.3 s; the hard actuator limits still clamp every command. Tracking holds its commands
during a test, the other DC motor is braked, and tracking resumes from where the test left
the head. POSTs must carry JSON, so another site's page cannot send them without a CORS
preflight, which the server does not answer. Verified with unit and endpoint tests and in
the browser against `python -m ultraskelly.replay --serve`; not yet run on the robot's
motors.

## Validation and limits

Initial portable queue tests: 3 failed / 2 passed, exposing blocked publication,
unbounded history, and retained references. After changes, the expanded suite
checks those cases plus head limits/deadzone, waist timeout/cooldown, cancellation,
production-coroutine replay, and import isolation. Final locked-environment run:
**10 passed in 2.66 seconds** on Windows / Python 3.12.14. The CLI replay produced
80 JSON rows with bounded head commands and stopped waist output on the final
missing target. `compileall` and `git diff --check` also passed. Tests use the real
bus/controller code; actuator
objects are fakes. The replay is scheduled with real time, not a deterministic clock.

No Raspberry Pi, IMX500, I2C board, physical motor, Hailo device, or ESP32 was used.
The original intermittent freeze is not reproduced end-to-end or certified fixed.
The old hardware startup motion, periodic restart, and tuning remain for subsequent
controlled changes. Removed web applications were not independently rebuilt because
they are outside the retained runtime; surviving Python imports were checked.

The requested cleanup commit is pending: automatic approval review rejected
staging/committing the broad deletion set, citing insufficient verification of
the exact scope. No commit was created. Review `cleanup-files.txt` alongside the
working-tree diff before approving that commit. All original files remain in Git
history at the baseline revision.
