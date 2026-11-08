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

- Pan servo channel 3, tilt channel 11, roll channel 7 held at 90, software range 0–180
  degrees, startup 90. Until 2026-11-06 the code had pan 11 and tilt 3 and left roll
  unpowered. That day each channel was wiggled alone (others held at 90, ±10° then ±30°)
  with the user watching: channel 3 turned the head left-right, 11 nodded it, 7 tilted it
  ear to shoulder, and 0 had no servo. Signs (which way positive turns) are not yet checked.
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

**Removed 2026-11-07**, with the originals in git history at `d4fe10b`. The original
camera's path, not fitted since 2026-11-02: the IMX500 pose detector
(`sensory/pose_detection_node.py`), the Picamera2 frame source (`sensory/camera_node.py`),
the bright-point detector that read it (`sensory/bright_point_detection_node.py`), their
`DetectorType` choices and launch settings, and the shutdown step that closed Picamera2.
The launcher now always builds the OAK camera and the Hailo-8 pose detector. The older
head and waist controllers (`motor/head_node.py`, `motor/waist_node.py`), superseded by
OrientationNode; nothing imported from them any more, and their messages already live in
`core/pubsub/bot_topics.py`. The OpenCV desktop window (`ui/cv2_ui.py`, its skeleton
drawing `ui/overlay.py`, the `show_window` setting and the `--window` and `--no-window`
options): the browser dashboard replaced the OpenCV window. A command that still passes
either option now stops with a usage error; `scripts/run-bot.sh` no longer passes
`--no-window`. Also removed: the commented-out alternative configurations in
`__main__bot.py` and `bot_launcher.py`, and the `FAIL_ON_IMPORTS` flag only the removed
camera modules imported. `CocoKeypoint`, which `__main__bot.py` took from the IMX500
detector's re-export, comes from `sensory/pose_common.py`, where it is defined.

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

### Encoding only what viewers draw (2026-11-03)

The server now reads each viewer's display feedback (docs/stream.md, "What a viewer
draws"). It encodes a feed only while some viewer draws it, sends each viewer only that,
and encodes the ZED's eyes at the largest size drawn when that is at most three quarters
of the camera's. It waits on its topics instead of polling them every 5 ms. It also sends
without permessage-deflate, which browsers negotiated and which deflated every JPEG and
zlib payload again for nothing.

Measured on the Pi with the bot at `--hardware --simulate-motors --zed --depth`, serving
on localhost only, with the Pi's full-screen Chromium as the viewer: 30 s samples after
30 s, main `eb16215` against `w4-stream-views` `ee61990`.

| | Before | After |
| --- | --- | --- |
| Bot process | 1.60 cores | 1.22 cores |
| Chromium on the Pi | 1.85 cores | 1.53 cores |
| Whole Pi | 89.5% of 4 cores | 72.1% of 4 cores |
| Sent to the dashboard | 3.9 MB/s | 2.9 MB/s |
| Frames the dashboard drew, OAK and ZED | 22 and 22 a second of 30 | 30 and 30 |
| Bot with no viewer | 0.66 cores | 0.68 cores |

The dashboard asked for the OAK at 640×480, both eyes at 317×178 and `depth_frame`; the
eyes went out at 318×178. The OAK keeps its pixels because the dashboard draws the aim
point and jaw zone in them, from `robot_state.overlays`; it could shrink too once the
dashboard scales those. Single samples; earlier samples of main with the dashboard
differed by up to 0.06 cores. `robot_state`'s unchanging fields (overlays, the node list,
the motor-test limits) are 570 of its 1443 bytes, but encoding them costs 36 µs a message,
0.07% of a core at 20 Hz, so they still travel in every message.

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

## Lost Wi-Fi and sudden power losses (2026-11-02)

The Wi-Fi radio never switched off: NetworkManager found it enabled in every boot,
and rfkill never blocked it. The Pi lost its saved network instead. Until 13:15 it
joined `HouseHouse-5G` (5745 MHz) through the profile Raspberry Pi Imager set up
with cloud-init and netplan (`/etc/netplan/90-NM-eaa52c7e-….yaml`). At 13:32:18–19,
while the robot's USB devices were being rearranged by hand, NetworkManager rewrote
that file and a second profile during a boot whose journal holds one second. Power
was lost before the data reached the card, and both files are empty. Later boots had
no Wi-Fi network to join until `HouseHouse` (2.4 GHz, channel 11) was added from the
desktop at 14:56. `HouseHouse-5G` is still on the air (channel 44, −52 dBm at the
robot). The same mechanism emptied git objects after the 10:12 reset.

Of the ten boots since 10:11, only the 10:50 power-off ended with a shutdown (the
journal's last line is `Journal stopped`); the rest ended mid-run. The current boot's
bootloader reset status (`/proc/device-tree/chosen/bootloader/rsts`) is `0x1000`, a
power-on reset: power was lost or cut. A reboot, kernel panic or watchdog reset sets
bit 5 (`0x1020`). Raspberry Pi does not document these values; see the
[forum thread](https://forums.raspberrypi.com/viewtopic.php?p=2300447) and
[rpi-eeprom #758](https://github.com/raspberrypi/rpi-eeprom/pull/758). No boot logged
undervoltage or USB over-current. The losses came from hands-on work on the powered
robot. Bursts of I2C "lost arbitration" errors on the servo and motor bus came before
them, the last one second before the 15:46 loss: the jumper-wired PCA9685 servo board
at `0x40` had been knocked loose while USB devices were swapped, and probes of the bus
failed; no motor was commanded. Boot times in `journalctl --list-boots` are rough:
after a power loss the clock restarts from the last saved time until NTP corrects it.

USB power: the 5 A supply lets the four USB ports draw 1.6 A together (the firmware
sets `usb_max_current_enable=1`; `config.txt` does not). Declared currents: ZED Mini
512 + 100 mA, OAK-1 500 mA before it boots, the Nordic receiver 100 mA, a Genesys USB
2 hub 100 mA and the Yeti X behind it 100 mA, 1.41 A in all. Actual draw was not
measured; the OAK can exceed its declaration while booting and inferring. Wi-Fi
power save was on (the driver default).

The Pi setup script below now syncs the journal every 10 s instead of every 5 minutes,
so the last seconds before a loss survive (they did at 15:46); logs at each boot the
reset status and whether the previous boot shut down
(`journalctl -u ultraskelly-boot-report`); flags emptied network profiles and git
objects; and turns Wi-Fi power save off. Shut the Pi down before re-plugging its power
or USB devices.

## Boot services and the Pi setup script (2026-11-02)

`scripts/install-services.sh` installs `ultraskelly-bot` (a system service) and
`ultraskelly-dashboard` (the desktop user's service) for the checkout it runs from;
both stay disabled until the owners finish testing on the robot. The bot runs through
`scripts/run-bot.sh`, which takes the shared robot lock without waiting: while a test
holds it, the bot exits with status 75 naming the holder, and systemd tries again.
After any exit systemd waits 2 s, then longer each time up to a minute, with no start
limit. The bot reports READY once its nodes are built and pings a 30 s watchdog from
its event loop. When the pings stop, systemd sends SIGUSR1, which prints every
thread's Python traceback before the process dies; not SIGABRT, because depthai
installs its own handlers for SIGABRT and SIGSEGV that print only native frames.
Extra bot options come from `/etc/default/ultraskelly-bot`; while the motor boards are
disconnected it holds `ULTRASKELLY_BOT_OPTIONS=--simulate-motors`.

Measured on the Pi with `--simulate-motors` (the motor boards were disconnected),
single runs:

| Test | Result |
| --- | --- |
| SIGSEGV to the bot | restarted after 2 s; ready 7 s after the kill |
| SIGSTOP to the bot | watchdog fired after 18 s (pings every 15 s), tracebacks of all threads logged; ready 25 s after the freeze |
| SIGSEGV to Chromium | dashboard reopened after 5 s; it reconnected to each restarted bot on its own |
| Test holding the robot lock | bot refused 5 times (2, 4, 8, 15 s apart), naming the holder; started once it was free |
| Owner's `flock -n` while the bot runs | refused |
| `systemctl stop` | clean shutdown, exit 0 |
| Reboot | bot ready 16 s after boot; dashboard full screen with the live stream |

The motors were not driven in these runs. A motor board keeps its last output until
told otherwise, and a crash or kill skips the bot's own stop commands, so the service
now brakes the DC motors after every exit: `ExecStopPost` runs
`scripts/stop-motors-after-bot.sh`, which runs `scripts/stop_motors.py` with a 10 s
limit, unless the bot exited with status 75, never having started because another
program held the robot lock. Checked with transient services on the Pi: the step ran
after a SIGSEGV, a clean stop and a watchdog kill, was skipped after status 75, and a
hung stand-in was cut off after 10 s. With the motor board unplugged, `stop_motors`
gave up after ten attempts in 0.9 s, so a restart waits about a second longer. Not yet
seen braking a running motor on the robot.

`scripts/setup-pi.sh --check` reports 20 settings and 8 observations of the hardware,
power, network and repository, changing nothing; without `--check` it puts missing
settings in place. The system files it installs live under `scripts/pi-system/` at
their paths under `/`. On the robot's Pi it held the seven Hailo packages (an apt
upgrade would replace the patched driver), turned Wi-Fi power save off, added the
watchdog drop-in and the commented OAK rule, and removed the two empty netplan files.
With five settings removed by hand, the check reported them missing and the script put
all five back. After the reboot every setting was in place (27 OK); the one warning,
once the services had stopped, was the disconnected I2C boards. Not tested: a freshly
imaged Pi, where the script also installs packages, uv and the virtual environment.
It has since gained two settings: the ZED IMU's udev rule, and (2026-11-03) the OAK's
models, which it fetches into `~/ultraskelly-models/oak-zoo` when one is missing; `--check`
reads only that cache, offline. On the robot's Pi the check found all six there.

### Control panel (2026-11-07)

At the owner's request ("I need to be able to turn the thing on and off") the Pi's monitor
shows a control panel: `ultraskelly-control.service` on port 80 serves the dashboard with
buttons that start, stop and restart `ultraskelly-bot`, and a following switch kept across
restarts in `/var/lib/ultraskelly-control/bot-options`
([docs/dashboard.md](dashboard.md#control-panel)). It runs as the robot's user with
`NoNewPrivileges=yes`; one polkit rule allows it exactly `start`, `stop` and `restart`
of the bot's unit. `tests/test_control_panel.py` covers it against a stand-in bot and
stand-ins for systemd, with the rule evaluated under node.

Seen on the Pi the same day (715e6c8, polkitd 126): the panel bound port 80 through
`CAP_NET_BIND_SERVICE` and answered at `http://ultraskelly-rpi5.local/`. Restart, Turn off
and Turn on went through the polkit rule: off in 16 s, the DC motors braked by the
service's stop hook, and on in 12 s. The following switch reached the running bot at
once, and the next start carried the panel's `--follow`. A POST naming another host, or
not sent as JSON, was refused. The user's `adm` group gave the panel the bot's log.
Chromium's first start on the new address opened at its default half-screen size; the
second went full screen. Not yet seen: the window after a reboot.

That size is Chromium's own default for a window with no saved place: half a 1920 px
screen less 15 px, so 945 px
([window_sizer.cc](https://source.chromium.org/chromium/chromium/src/+/main:chrome/browser/ui/window_sizer/window_sizer.cc)).
With `--app`, `--start-fullscreen` only asks for full screen just after the window
appears
([web_app_startup_utils.cc](https://source.chromium.org/chromium/chromium/src/+/main:chrome/browser/ui/startup/web_app_startup_utils.cc)),
and that ask can be missed. `scripts/open-dashboard.sh` now also passes `--start-maximized`,
which Linux builds apply to app windows as they are made, even over a smaller saved place,
so a missed ask still leaves the screen filled and F11 returns to that. Not yet seen on
the Pi.

The first version had a red button that stopped every motor and kept following off. The
owner had it removed the same day, and the column made smaller and foldable. Turn off is
the stop: the bot's service brakes the DC motors as it stops
(`scripts/stop-motors-after-bot.sh`).

The owner then asked for every panel to be resizable. The 8 px between two panels is now a
handle (`ultraskelly/ui/dashboard/resize.js`); the browser keeps the sizes, and a
double-click puts the usual size back ([docs/dashboard.md](dashboard.md#sizing)).
Checked in headless Chrome against the stand-in bot at 1920×1080 and 1280×720: with
nothing dragged every panel sits where it did, to the pixel. A reload keeps the dragged
sizes. The folded column stays 44 px over a dragged width and gets that width back when
opened. Dragged to every limit, nothing overflows the window. Not yet seen on the Pi.

In the same pass the owner asked for no static text, bigger plots and no empty space on
the left. Explanations moved behind a small i per panel (a popover). Fixed details became
tooltips, and the titles were cut to a word (OAK, ZED, Face). The motor tests and the
nodes moved into the control column, which is packed top to bottom. While the column fits
beside the 3D scene, the time series runs under both; `panel.js` measures this. On the Pi's
1920×1080 the time series grew from 571×238 to 1240×324. The column's empty space below
its last block went from 533 px to none. The visible words while running went from 271
to 153. The camera column has the plain dashboard's width again, so the OAK draws 1:1.
"cmd" stays on the commanded lanes, per AGENTS.md. Checked in headless Chrome at
1920×1080 (on and off), 1280×720, the bot's own page and a phone: nothing overflows.
Not yet seen on the Pi.

Then the owner asked for "the controls that I would need to tune it myself", in a pop-over
window. Tune, under the follow switch, opens it ([docs/dashboard.md](dashboard.md#tune)).
It is drawn from the controllers' own pydantic params (head aim, roll, waist and jaw:
70 settings today), so the controllers being rebuilt to docs/control-design.md bring
their new settings with them. A change is checked against the controller's whole model
and set on the running params objects; Save keeps what differs from `launch_config` in
`~/.config/ultraskelly/tuning.json`, merged over it at every start. A gear ratio halved
in mid-turn moved the simulated head's pan command by under 2 degrees, because the head
carries on from the angles last sent; without that it jumped more than 5
(`tests/test_tuning.py`). Checked in headless Chrome against the stand-in bot with the
real routes: four edits marked and unsaved, an out-of-bounds 50 refused with its reason
and the box restored, Save writing only the four, and the bot restarted with them. Not
yet seen on the Pi; nothing tuned there yet.

## Motion: waist drive, motor writes and motor tests (2026-11-02)

**Why the waist pulsed.** The orientation controller drove the waist at 0.7-0.8 for at most
0.5 s, then braked it (`throttle = 0` shorts the motor; `None` would let it coast) for a
0.4 s cooldown, repeating while the head was turned, and idled it whenever the head lost
lock unless the head was over 45 degrees off. A first sound-and-camera round with an older
microphone (`scripts/waist_sound_test.py`) heard broadband clicks at every start and stop
and a PWM whine near 1.6 kHz with harmonics. The camera saw the torso swing back at
12-25 deg/s after each brake, a wobble of about 2 Hz. The motor board's PCA9685 already
runs at its highest PWM frequency, so the whine cannot be moved above hearing.

**Continuous ramped drive.** `motor/waist_drive.py` replaces the pulses with one continuous
move per recentering. With the microphone shelved (the Yeti X left the Pi; round 2 is
deferred, not cancelled), its shape follows published guidance and round 1's data:

- *Slow decay.* The waist motor brakes between PWM pulses (adafruit_motor `SLOW_DECAY`).
  That is the TB6612FNG's own PWM mode, alternating drive and short brake
  ([datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf));
  adafruit_motor defaults to fast decay, which leaves both inputs low (high impedance)
  between pulses ([API](https://docs.circuitpython.org/projects/motor/en/latest/api.html)).
  Slow decay recirculates the current through the low-side switches with less ripple
  ([TI SLVA321](https://www.ti.com/lit/pdf/slva321)). Adafruit's measurements on a geared
  hobby motor found it lowers the slowest speed the motor can turn at from 3000 to 1200 RPM
  and makes speed follow throttle almost linearly
  ([guide](https://learn.adafruit.com/improve-brushed-dc-motor-performance)). That
  matches the waist's behavior: nothing below about 0.7 in fast decay, then a lurch.
- *PWM frequency stays at the PCA9685's top* (prescale 3, about 1.5 kHz). The chip
  offers 24-1526 Hz, the same for every output
  ([datasheet](https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf)), so the usual
  advice to switch above 20 kHz to avoid audible ripple
  ([Portescap](https://www.portescap.com/-/media/project/automation-specialty/portescap/portescap/pdf/whitepapers/wp_controlling_brushed_dc_motors_using_pwm.pdf))
  would need the bridge's PWM pin driven by the Pi's own hardware PWM, a rewiring.
  Adafruit's guide recommends 50-100 Hz for low-speed torque but warns of chatter, and
  ripple grows as the frequency falls. A lower frequency is the first thing to try once
  the microphone is back: `LaunchConfig.motor_pwm_hz` sets it for both motor factories,
  which must agree because the board has one PWM clock (the jaw's factory used to reset
  it). A 2025 scratch test (`scratch/motor_pwm_test.py`) listed 5-25 kHz, beyond the
  chip, and kept only 1600 Hz; it ran the jaw at full drive, where nothing switches, so it
  says nothing about the whine.
- *S-curve ramps.* Jerk-limited profiles spread each change of acceleration over time and
  inject less vibration, and so less noise, than linear ramps with corners
  ([PMD](https://info.pmdcorp.com/hubfs/Resources/Articles%20and%20Papers/s-curve-motion-profiles-are-vital-PMDCORP.pdf),
  [Zaber](https://www.zaber.com/articles/jerk-control)). The drive's rate is limited to
  1.5 per second rising and 2 falling, and its change to 10 per second squared, so torque
  never steps and no ramp has a corner.
- *Gentle start, lower running drive, bounded breakaway.* Static friction exceeds running
  friction, and pushing harder until the error clears makes a mechanism with friction
  hunt in a limit cycle ([friction and backlash](http://wescottdesign.com/articles/Friction/friction.pdf)).
  A move starts at 0.15 and eases up to a running drive of 0.5 at 15 degrees, plus 0.008
  per degree, at most 0.7. Only while the head shows no progress (2 degrees within 0.5 s
  at the running drive) does extra drive build, 0.3 per second toward 0.8, the hard limit.
  It drains at the same rate once the head comes back, a leaky integrator, so a waist that
  breaks loose does not lurch.
- *Ramp down, coast, then brake.* Round 1 saw the torso swing back at 12-25 deg/s after
  each brake at speed. With slow decay a falling drive slows the motor with it; the
  motor is released at 0.2 to coast and braked 0.3 s later, once still.
- *Deadband with hysteresis.* The same paper's cure for hunting is a deadband at least as
  wide as the jump the motor makes when it restarts. The waist starts past 15 degrees and
  stops within 5, and a stopping move never resumes, so it rests between the two.

A move lasting 6 s still ramps down and rests 2 s, the remaining limit on continuous
drive. Losing the target ramps the waist down; stopping the bot still brakes at once. In
the simulator (now with optional friction), a minute of sweeping target gives 8-13
continuous moves of 1.3-6.4 s, none ending in a brake at speed. With friction slowing the
simulated waist it could not keep up with the sweep and hit the 6 s limit twice a minute.
With breakaway set above the running drive, the waist still starts and recenters the head
on a still target. Within a move the drive changes by at most 0.02 per 10 ms tick, and its
slope by at most 0.002. None of these
levels has run on the robot: they wait for the motors and, for loudness, the microphone.
Round 1's speed (18-23 deg/s at 0.75 in fast decay) is the only measured anchor. Round 2's
tools remain: the sound test centers and holds the head, refuses to run while the OAK sees
too little detail (Laplacian variance of a 160x120 frame: over 1300 in the room, 9 with
the lens covered), bounds its return move by the measured angle, measures where the waist
breaks loose and stalls, and its `drive` profile replays the bot's own drive for one
recentering. `scripts/analyze_run.py` summarizes a recorded run: what was kept, the
waist's moves and stops, and the sound while it moves.

**Motor board writes.** While the servo board's loose jumper disturbed the I2C bus, motor
board transfers failed with EAGAIN after lost arbitration. adafruit_motor writes a DC
motor's two bridge inputs in separate transfers, and a brake writes the first input fully
on before the second, so a brake cut short there drives the motor at full power. The
recording wrappers now retry a failed write (3 attempts, stops 10). They also send an
unchanged throttle again only every 0.5 s: the 100 Hz controllers had been rewriting both
motors' unchanged values, about 400 transfers a second on a 100 kHz bus, roughly a fifth of
the event loop's time. `motor/bridge_motor.py` now sets both of a motor's bridge inputs in
one transfer. The PCA9685 changes its outputs when a transfer ends, however many PWM
registers it carried (MODE2's OCH bit, 0 by default; NXP data sheet Rev. 4, Table 6), and
each motor's inputs are neighbouring channels (motor 1 uses 9 and 10, motor 2 uses 11 and
12), so the two inputs switch together and a failed transfer changes neither. On the Pi,
against adafruit_pca9685 3.4.20 and adafruit_motor 3.4.18 over a recording bus, it left
the same register values as the libraries' own DCMotor for every throttle in both decay
modes, in one transfer where theirs took two. The bot's motor factories, the stop script
and the bench scripts all drive the motors this way. If the bot exits without its
cleanup (killed, or crashed in native code), the motor board keeps its last command:
`scripts/stop_motors.py` brakes all four motors, and the bot service runs it after every
exit unless another program holds the robot lock.

**Motor watchdog.** The controllers and their cleanup run on the bot's event loop, so a
loop that stalls mid-move would leave the waist turning. `motor/motor_watchdog.py` runs on
its own thread. The orientation controller's first tick arms it, so the slow camera and
Hailo startup does not count. When no tick comes for 0.5 s it brakes both DC motors once,
through the same wrappers, so the brake is retried and recorded. When ticks resume, the
waist drive, having seen no update for over 0.5 s, starts its next move from rest rather
than jumping back to its old drive. It is the fastest of three layers. The stall monitor
stops the bot when camera frames or pose results stop for 5 s. systemd kills a bot whose
event loop stops pinging it for 30 s and then brakes the motors through the stop script.
A stall in code that holds Python's global interpreter lock also stops the watchdog's
thread; those reach the systemd watchdog. Tested on Windows with a blocked event loop,
braked from the thread 0.15-0.4 s after the last tick, with a 0.2 s timeout; not yet run
with the motor boards.

**Motor tests.** The dashboard can move one actuator at a time through its own command
contract (`motor/motor_test.py`, `ui/motor_test_routes.py`). Each test is answered once it
has finished or been stopped; one that is malformed, outside the test limits or arrives
while another runs is refused at once with the reason, never queued, merged or silently
clamped. Test limits: servos anywhere in their 0-180 (until 2026-11-07, 30 degrees of
center), waist up to 0.7 for 0.5 s and at
most 1.0 throttle-seconds net one way (there is no waist angle sensor), jaw up to 1.0 for
0.3 s; the hard actuator limits still clamp every command. Tracking holds its commands
during a test, the other DC motor is braked, and tracking resumes from where the test left
the head. POSTs must carry JSON, so another site's page cannot send them without a CORS
preflight, which the server does not answer. Verified with unit and endpoint tests and in
the browser against `python -m ultraskelly.replay --serve`; not yet run on the robot's
motors.

**Run recording check.** With the motor boards disconnected for the rebuild, a 67-second
run of `--hardware --simulate-motors --no-window --record --microphone "Yeti X"` (commit
e26b349) stopped cleanly on SIGTERM and kept 1953 frames at 30.0 fps. None were dropped,
no frame numbers were missing, and the longest gap was one frame. Frames reached the
recorder 35 ms after capture (median; 43 ms at most). Every frame decoded with OpenCV on the
Pi and again on the Windows PC. The Yeti X audio (48 kHz mono) starts 18 ms after the run
and lasts 65.1 s: the stop signal reaches `arecord` too, so the last 1.7 s of shutdown are
silent. `run.json`, poses and targets for every frame, servo state, and the command log
were complete. Four dashboard motor tests sent during the run all ended "done" and appear in
`commands.csv` at their times; a fifth, over the waist limit, was refused with HTTP 400.
The OAK lay facing a keyboard, so nobody was tracked: a tracking run with a person in view
remains to be recorded once the head is back on.

## Calm head tracking (2026-11-06)

**Why the robot swung.** The head added 0.05 servo degrees per pixel of error to pan and
tilt on every camera sample, and the waist turned the torso whenever the pan servo was
more than 15 degrees from its middle. Two things made that loop swing. Each sample
describes a frame captured 50-70 ms earlier (measured on the Pi), while the head was still
turning toward the samples before it, so the increments piled up: an integrator on a
measurement d samples late rings once its gain per sample passes about
2 sin(π / (2(2d + 1))), 0.29 at three samples late. And the head is geared down: on
2026-11-06 `scripts/check_motor_directions.py` found 12 servo degrees moving the upright
OAK picture 35 px in pan and 9-10 px in tilt, about 0.36 and 0.1 degrees of view per servo
degree at the pinhole's 8.1 px per degree near the middle (640 px over 69 degrees, 9.3, is
the average across the frame). Returning pan to 90 undid only 17.7 of the 34.7 px, about 6
servo degrees of slack, which also explains why the first 12 degrees the other way moved it
only 11 px; tilt came back within 2 px. Every sign was as the controller assumes. So the
waist's 15 servo degrees were about 5 degrees of head turn, its 5-degree stop under 2, the
pan servo wound toward its end stops, and the waist and the slow head chased each other.
Closing the loop on the robot with the waist off (`scripts/check_tracking_loop.py`), the old
gain overshot a patch of the scene by 84 px and lost it; 0.012 settled without overshoot.

**A synthetic closed-loop layer.** The simulator's plant now has the OAK's 69-degree view,
the measured delay with each target's capture time, servos that trail their commands by
80 ms (the robot's picture moved about 0.15 s after a command), the measured gearing and
slack, a sign flip per servo, detection noise and dropouts, and any number of people. `simulator/scenarios.py` runs the production `OrientationNode.step()` through it on
ten scenarios and fails a run on overshoot, oscillation of the head's actual turn, runaway,
final aim or waist hunting (docs/simulator.md, "Closed-loop scenarios"). Main at 0d1e919
fails 8 of the 10 on the measured plant: overshoot of 9.8-11.3 degrees when someone steps in
to the side, 2-4 reversals of the head, the waist hunting in three (3-4 moves where one or
two are allowed) and the pan command at its end stop in three. On a head without gearing it
had failed all nine scenarios then defined, with 7-28 degrees of overshoot and 6-20
reversals. The plant is milder than the robot, though: with the waist off, the old gain
overshoots a still target there by 18-35 px, not the robot's 84, even with 0.2 s of servo
lag, so the real linkage has more delay or play than the plant models.

**Calm head aim.** `motor/head_aim.py` replaces the per-sample gain. It works in degrees of
view through the measured gear ratios (assuming 0.33 for pan, a little under 0.36, so it
errs toward turning short) and converts to servo degrees only for the commands. It commands
each servo half its linkage's slack past the view in the way it moves (5 servo degrees for
pan, a little under the 6 measured, as too much makes the head jump; 2 for tilt), so a
reversal takes the play up at once instead of leaving the head still while the body turns.
Each target becomes an estimate of where the person is relative to the base: where the
camera pointed when the frame was captured (the body's estimated turn and the head's
commanded view 80 ms earlier, as the servos trail their commands) plus 65% of the person's
offset, converted through the pinhole. Aiming short means a scale or gear-ratio error of up to half cannot overshoot; the
next frames close the rest. Estimates are smoothed (half of each), and one more than
8 degrees from the goal must repeat four times in a row before the head turns to it. The
head turns at most 40 degrees of view per second (its servos at most 150 per second) with
S-curves, keeps 5 servo degrees clear of its end stops, and moves only once its goal is
more than 1.5 degrees away. While the waist turns, the head counter-rotates against the
waist's estimated turn (27 deg/s at full drive) at once. The waist now follows the person's
bearing from the body rather than the pan servo, waits until the head has held still for
0.3 s unless the person is beyond 40 degrees, and keeps its 15 and 5 degree start and stop.
It also drives more gently, as the interim tuning on main had it that afternoon: at most
0.6 running and 0.7 to break loose (the hard limit stays 0.8), rising at most 1.0 per
second. That tuning's 3 s limit on a move is not kept: it stopped the waist mid-chase
when someone walked off, so the 6 s limit stays.
The pose node follows the credible person (half the best score or more) nearest the image
centre, so two people with swapping scores no longer pull the head between them.

**Results and limits.** The new controller passes all ten scenarios on the measured plant
(overshoot at most 1.1 degrees, one waist move where one is needed), and the suite checks
every scenario at the edges of camera delay 40-100 ms with servo lag 40-100 ms and field of
view 64-74 degrees (their eight corners), servo lag up to 120 ms, pan ratio 0.28-0.42, tilt
ratio 0.07-0.14, pan slack 3-8 and tilt slack 0-4 servo degrees, and waist speed 17-37 deg/s.
Outside that it degrades without swinging: two errors that both shorten the counter-rotation
(a pan ratio under 0.3 or 8 degrees of slack with a waist much faster than assumed) overshoot
by 2-3 degrees while the waist turns, and servos lagging 0.16-0.2 s by 3-4. The 11-06
picture also shows the waist moving the picture 105 px at -0.7 drive for 0.5 s but 47 px at
+0.7, and the slack may vary: the first pan move implies 8 degrees, the return 6.
`scripts/check_head_gearing.py` measures them: it steps each
servo up, down past the middle and back, and the waist each way for two durations, within the
dashboard's motor test limits, and prints the ratio each way, the slack, and the waist's speed
each way next to what the controller assumes. None of this has run on the robot's motors yet;
`scripts/check_tracking_loop.py` now drives the new head aim for the next check.

**Levelling faces with the roll.** At the user's request (2026-11-06) the head's roll servo
(channel 7) now turns the camera so the face it follows looks level in the picture, instead of
holding at 90. `motor/roll_level.py` takes the followed person's face from the OAK's own face
detector (the face whose box holds the pose's nose), measures the line from its right eye to
its left, and works as the head aim does: the camera's roll when the frame was captured plus
80% of the eye line's tilt becomes the goal, smoothed (10% of each estimate, less for faces
whose eyes are under 30 px apart, as their angle is noisier), and the roll turns toward it only
once the goal is a degree away, its servo at most 60 degrees a second, never more than 25 servo
degrees from upright, pushing half the linkage's slack ahead of it as the head does. Faces the detector doubts (score under 0.6), faces under 16 px
between the eyes (beyond about 1.9 m), eye lines steeper than 40 degrees and faces turned more
than 40 degrees away are not used; after a second without a usable face it eases back upright.
Motor tests and `--no-follow` leave it alone, the servo state reports the commanded roll, and
`ActuatorLimits` now clamps roll to 60-120. The pan and tilt errors are turned back upright by
the roll before the head aims, as a rolled camera turns the whole picture. In the simulator a
face's eye line tilts by the person's tilt less the roll, eyes scale with distance and have
their own jitter, and seven scenarios check that a tilted face is levelled without being turned
past level, that the head straightens again, and that it stands upright once the person has
gone; every other scenario checks that the roll does not move.
`scripts/check_motor_directions.py` turns roll 12 degrees each way and measures the picture's
turn from matched features (`cv2.estimateAffinePartial2D`). On the robot (2026-11-06, 19:16) the
roll turned the picture's content clockwise for a larger angle, the opposite of the first
guess: 12 degrees barely turned it (0.1-0.6 degrees), 25 turned it 2.1-2.9 degrees and moved it
27-29 px up and down, and returning to 90 left 1.1-2.3 degrees of the turn. That fits 0.12
degrees of picture per servo degree through 16-20 servo degrees of slack, with the camera
nodding about 0.18 degrees per servo degree as it rolls (it sits off the roll's axis). The
controller and the simulator now use those numbers (0.11, 16 compensated, clockwise), and the
tests pass from 0.09 to 0.16 degrees per servo degree, 12 to 22 of slack and no nod to 0.3; the
other way round is caught. Within 25 servo degrees the roll turns the picture only about 2
degrees: it turns a tilted face toward level by that much, never past it, and so the scenarios
ask for a face within 1.5 degrees of level or a degree nearer level than the person's own tilt.
Levelling a fully tilted head would take a stiffer linkage or more travel than the 60-120 the
hard limits allow; neither is changed here. The same evening tilt moved the picture only 0-4.5
px for 25 servo degrees, against 9-10 px for 12 at 16:53: its travel is uncertain until the
linkage is checked, and the tests check that the head stays calm, only failing to reach someone
above or below, if tilt turns a fifth as far or has 20 degrees of slack.

## A simulator fitted to the robot, and the head tuned on it (2026-11-06)

At the user's request ("make it ACCURATE to the real robot, then built and tune the controller on
there"), the simulated plant is now fitted to a recording of the robot's motors, a test holds it to
that recording, and the head's aim is tuned on it against a stated cost.

**The recording.** `scripts/record_motor_response.py` ran, inside the servo windows, steps of several
sizes, a staircase each way, ramps at several speeds and big steps on pan, tilt and roll, then short
waist pulses each way, while the OAK kept every frame with its capture time
(`tests/data/motor-response-20261106`: the commands, the segments, and the picture's yaw, pitch and roll
from registering frame to frame). `scripts/analyze_motor_response.py` measures from it each servo's gain
each way, slack, dead time, overshoot and settling.

**The model and the fit.** Each servo is a chain (`simulator/plant.py`): command, dead time, a
slew-limited servo, slack (a play operator), gain, and the head's own mode, second order with dry
friction and damping that differs by the way it turns. It is fitted by Nelder-Mead to every segment's
recorded view, each segment aligned by the median of its first four frames (the chained views drift
over the run), plus the big steps' overshoot, settling and dead time as `analyze_motor_response.py`
measures them on both. A bias force could not give pan its different overshoot each way, since a
constant force moves only where the head comes to rest; damping by direction does. Dry friction
removes a fixed amount each swing, so small moves overshoot less than large ones, as on the robot.
The fitted values are in docs/simulator.md ("Plant assumptions"): pan 0.261 degrees of view per servo
degree, 44 ms dead time, 2.5 servo degrees of slack, a 1.9 Hz mode damped 0.41 turning one way and 0.28
the other; tilt 0.081, 40 ms, 1.8; roll 0.063, 43 ms, 12. The waist breaks loose only above 0.65 drive,
then turns 50.4 deg/s at full negative drive and 21.5 at full positive, reaching speed in 0.16 s.

**Fidelity, the gate for everything below** (`tests/test_plant_fidelity.py`). Replaying every recorded
command through the plant gives the recorded view within 1.01 degrees RMS on pan, 0.58 on tilt and
0.40 on roll (bounds 1.5, 1.0, 1.0). Each big step's overshoot, robot against model, stays within the
8 points asked: pan 90 to 140 21.9 % against 22.0, 140 to 40 32.4 against 32.4, 40 to 90 26.0 against
21.6; tilt 1.3 against 5.5, 14.0 against 13.7, 8.5 against 5.6; roll 4.1 against 2.0, 5.1 against
5.3, 5.2 against 6.1. Settling to within 5 % agrees within 0.2 s but on pan's 90 to 140, 0.52 s
against 0.76, which the test allows as 0.25 s: the model is slow there, the conservative side for
tuning. The gains agree within 15 % on pan and tilt and 35 % on roll, whose own ratio varied from
0.050 to 0.083 between moves. The waist's pulses replay within 0.02 degrees: 13.24 against 13.26 for
+0.7 for 0.5 s, 5.65 against 5.66 for -0.7, and none at 0.6. The model reaches 2 % of a step about
35 ms later than the robot.

**What the plant reproduces.** An aggressive head (aiming the whole offset each frame, no smoothing,
3000 deg/s^2) overshoots a person 12 degrees off by 3.4 degrees with 11 reversals, the robot's swing.
False detections, the robot's 1-3 frames at a time, exposed a flaw in the head: a jump (a goal more
than 8 degrees from the last) was believed after `jump_samples` sightings in a row, but nothing reset
that count between sightings, so bursts seconds apart added up. Flickering at one spot by the edge,
they took the head 21.9 degrees of view, 84 pan servo degrees, after nothing: the order of the
67-servo-degree swing that followed a burst on the robot (run 20261106-194104). Now the count starts
again after `candidate_gap`, 0.2 s (six frames at 30 Hz) without a sighting, and a jump must show up
five times: the same scenarios leave the camera still, while a real person with a third of their
detections dropped breaks a run of sightings only after six drops in a row (0.33^6, about 0.1 %).

**The battery, the cost and the tuner** are in docs/simulator.md ("Closed-loop scenarios", "Tuning
the head"): seventeen head-alone scenarios, the robot's present setup, on the measured plant and six
stressed ones (pan gearing 15 % either way, 1.5 and 2 times, 45 % ringing, 1.5 times the dead time),
with the cost per run stated there.

**The tuned head.** 120 random settings and refinement one parameter at a time, on the head as it
launches (inside its servo windows), found reach 0.30, smoothing 0.69, acceleration 34 deg/s^2,
deadband 0.39 degrees, the commands of 60 ms before for where the camera pointed (`servo_lag`
0.055), five sightings for a jump and the input shaper off; top speed hardly mattered (15 and 24
deg/s cost 64.39 and 64.46), so it stays at the guard's 15. These are now `HeadAimParams`' defaults, and
the robot launches them. On the measured plant the head overshoots no one in any of the seventeen
scenarios, settles within 1.5 degrees of someone 12 degrees off in 1.75 s and of someone 4 degrees
off in 0.87 s, ends 0.18-0.34 degrees off a still person, and does not move for any of the false
detections. On every stressed plant it fails nothing; its worst overshoot is 0.51 degrees, with pan
geared twice the fit. Its cost is 64.4. The stopgap it replaces (reach 0.35, smoothing 0.3, deadband
1.5, 300 deg/s^2, sightings counted however far apart) costs 2209 on the same battery: it never
settles within 1.5 degrees of a step, ends 2.0-3.4 degrees off (five aim failures), and follows
false detections in four scenarios, by 2.2 to 16.6 degrees.

Why these values. The goal moves smoothing x reach = 0.21 of the remaining offset per frame, and
the tuner's best lies along that product: reach 0.35 at smoothing 0.59 costs 66.7, 0.30 at 0.69
64.4, 0.25 at 0.83 70.6 and 0.21 at 1.0 80.6. Lower, the head is slower (a 12-degree step settles
in 1.91 s at 0.25 and 2.09 s at 0.21, against 1.75), and a still head moves only once reach x its
offset passes the deadband, so it can rest up to deadband / reach off, 1.3 degrees at 0.30. Higher,
0.425 at 0.50 swings with pan geared twice (overshoot 1.7 degrees, oscillation). Reach 0.30 keeps
each estimate on the near side of the person while 1 - reach x g >= 0, g being the real gearing
over the assumed, so up to g = 3.3, inside the guard's reach <= 0.35 (2.86). The acceleration limit
is what keeps pan's 1.9 Hz mode quiet: for this second-order head a change of acceleration a starts
ringing of about a / omega_n^2 (1.06 a / omega_n^2 at damping 0.34), 34 / 11.84^2 = 0.24 degrees,
under the deadband, against 2.1 at the old 300 deg/s^2. With 300 and the rest as tuned, 12-degree
steps settle in 1.2-1.3 s on the measured plant, but the head swings with pan geared twice
(overshoot 2.2 degrees, oscillation; cost 106.5). The zero-vibration shaper (A1 0.76 now and A2
0.24 half a ringing period, 0.28 s, later) cancels the rest but delays every move: with it a
12-degree step settles in 2.07 s instead of 1.75, at cost 77.6 against 64.4, for a worst overshoot
on any plant of 0.04 degrees instead of 0.51. It stays in the controller, off.

Two artefacts went with the tuning. The tuner stalled twice with every worker idle, the second time
just as the pool replaced a worker after its 40 tasks. Each scenario run also made a new asyncio
loop, which on Windows opens a loopback socket pair whose port is held about two minutes after
closing: one candidate left 105 such ports, eight workers about 200 a second, enough to use up
Windows' 16,384 within that time. Each thread now reuses one loop (one port per worker), workers
are no longer replaced, and a stalled batch stops the tuner with its checkpoint kept rather than
waiting forever; three full runs have finished since, though which change cured the stall is not
established. And the
simulator's head speed setting replaced the controller's top speed in every run, so the scenarios
had run the head at 40 deg/s whatever its default; it now does so only when set.

The edge gate (a target within 40 px of the picture's edge must be confirmed like a jump) now covers
the left and right edges only. On the robot (run 20261106-222520) tilt went to its end turning down
and a standing person's face still sat 116-217 px below the middle, 3.1 degrees of head pitch by the
ZED's accelerometer: the head cannot look down to them, so real faces sit by the bottom edge, where
the gate would have held pan to a jump every five sightings. A face kept 204-238 px below the middle
now pans as one at eye level (within 1.5 degrees after 1.77 s against 1.76, ending 0.26 off).
Checked against the coordinator's checkpoint from the third tuner run (reach 0.40, smoothing 0.51,
15 deg/s^2, the minimum of that search's range), on the same head: it costs 65.8 against 64.4,
settles 12-degree steps in 2.22 s against 1.75, and overshoots at most 0.22 degrees on any plant
against 0.51; neither fails anything.

**Not done.** The waist and the roll are not tuned on the measured plant: the whole robot fails
"left", "right" and "far" (a person beyond the head's reach and a waist that breaks loose only above
0.65 drive) and "tilt-noisy" (the roll levels a noisy face by a little under the degree asked), as
strict expected failures; both stay off on the robot. Tilt's travel on the robot is uncertain since
its linkage measured smaller in the evening, so the head must stay calm, and does, with tilt turning
a quarter as far or with 20 servo degrees of slack. The plant comes from one recording, does not
model the linkages beyond the windows (pan jammed, tilt over centre), and has not yet been checked
against the head running closed-loop on the robot: the coordinator records that run.

### Tilt reach (2026-11-06)

With the tuned head following on the robot (run 20261106-224643), the tilt command reached the end
of its window, 125, within 2 s of a person stepping into view, and stayed there. The servo board was
read live at 1791 µs on channel 11, the pulse for 125°. The person's face stayed 125 to 246 px below
the aim point (median 183 px, about 22°). The ZED's accelerometer measured the head nodding 2.9° for
those 35 servo degrees. The user's dashboard tilt test, 115 to 65, measured 4.8° for 50, the same
0.08° per servo degree as the recorded plant (0.07 to 0.12 by stair). So tilt control works; the
mechanism cannot point the camera at a standing person's face. The head rests about 20° above it at
90, and its whole 20 to 130 window gives about 9°. The fix is mechanical: rest level at 90, and a
linkage giving at least 0.25° per servo degree. The `head` health check reports an axis held at the
end of its range for 5 s while the person followed is still off the aim point beyond that end.

### The head servos' full range (2026-11-07)

The owner found the head's range from software "super limited (esp in tilt)" next to its range by
hand with the bot off, when the servo board sleeps and every servo channel is released. Three
limits stood in the way: motor tests held to 30 degrees of 90, hard limits of pan 20-160 and tilt
20-130 on every command, and following windows of the same. At the owner's direction ("if the motor
can make it there then do it") motor tests now take any angle in the servos' 0-180, the hard limits
are 0-180, and following uses the whole travel `scripts/check_servo_travel.py` measured on
2026-11-06: pan 10-170 and tilt 10-140. Beyond those, pan jammed after being held at 5, and tilt's
linkage goes over centre past about 140, where more servo angle turns the head back and following
would drive the wrong way. Motor tests can still go there. Later that day the owner saw the head
go lower at 180 than at 140 ("the face track doesnt go ALL the way down"), so following's tilt now
runs to the servo's own end, 180. No margin is kept at the servo's own ends (`SERVO_ENDS`). Tilt's measured 0.08 degrees of nod per
servo degree is unchanged: the whole window nods the head about 10 degrees. If the head goes much
further by hand, the linkage gives somewhere between servo and head.

### Slower pan, tighter centring (2026-11-07)

At the owner's request ("decrease max pan servo speed", "tighten that range") the robot launches with
pan's servo no faster than 30 degrees a second, about 8 degrees of view: tracking's pan servo topped
out near 58. Motor tests now sweep at the same speed instead of jumping, so the end buttons take
seconds. It also launches with a deadband of 0.12 instead of 0.39. Each new sighting moves the goal
`reach` (0.30) of the face's offset, so a still head used to move again only for a face more than 1.3
degrees off (about 10 px); now it takes 0.4 (about 3 px). On the measured plant the head battery
passes with both: no new reversals, the worst settle 2.45 s where it was 2.05, and step-left now ends
0.06 degrees off where it ended 0.32. The controller's defaults keep 0.39 and full pan speed. A
tighter deadband keeps the head making small corrections, and the waist starts a move only once the
head has held still for 0.3 s. With the waist on, the full scenarios fail at every deadband ("far"
swings 14-20 degrees past the person): the waist was never tuned with the refitted head.
Told this, the owner had the waist turned on as it is, the same day ("Turn on as is").

### The ZED IMU's clock wrap blanked the dashboard's charts (2026-11-06)

The ZED Mini's IMU tick count is 24 bits, 39.0625 µs a tick, so it wraps every 655.36 s. `ZedImuNode`
took the drop for a camera reset and restarted its clock map. The reads around the wrap were mapped
about 655 s late, and every later timestamp stuck at that value until the new map caught up. In
run 20261106-224643 the host times froze for 400,000 to 500,000 samples at a time and leapt about
660 s at each wrap. The node logged "The ZED's IMU clock started over" five times in 40 minutes.
The strip charts' window ended at the newest time of any lane, so one batch timed ahead pushed it
past every other lane's data: all six charts went blank.

The node now unwraps the count. A drop is a wrap when the count lands within 10 s past it; any
other drop is a reset, and the samples before it are mapped on the old clock. Replayed offline, the
wrap left the error at 0.4 ms against 629 s before. The charts' window ends at the newest robot
state time, and samples outside it are not drawn. The `zed_imu` health check reports timestamps
more than 2 s off the Pi's clock.

## Recovering from lost connections (2026-11-02)

A lost board, camera or detector now ends the bot the same way: it stops every node,
which brakes the motors it can still reach, and exits. The boot service brakes the
motors again (`ExecStopPost`) and starts the bot after 2 s, then longer each time up to
a minute; each start opens every board and the camera afresh, which is the reconnection.
Brief faults stay inside the bot: the motor wrappers retry failed writes (above), and
HailoRT's 1 s inference timeout already turns a hung Hailo-8 into a crash.

Two gaps are closed. `StallMonitorNode` stops the bot when camera frames stop for 5 s or,
with the Hailo detector, pose results for camera frames stop for 5 s (20 s allowed for the
first of each); a camera that stayed connected but silent used to leave the bot running
blind, holding its motors still. The watchdog pings now stop as soon as the bot starts
shutting down, and STOPPING=1 is no longer sent, since it switched the watchdog off: a
shutdown stuck on a thread, such as a hung inference, ends with every thread's traceback
within 45 s.

Opening the camera and the Hailo-8 before the motor boards was proposed, so a missing
camera would fail before the head moves; it was not done. With the boards disconnected,
the likelier failure today, the bot would boot the OAK and load the model before failing
and leave them half open, while a missing camera only re-centers an already centered head.

Evidence. `tests/test_bot_recovery.py` runs the real launcher, controllers and retrying
wrappers on fake boards whose writes fail with EAGAIN, on the Pi: with one write in five
failing for a second the bot kept running and later stopped with both motors braked; a
servo board going dead mid-move ended the bot with the waist and jaw braked; a dead motor
board ended it instead of hanging; a silent camera ended it with `StreamStalledError` and
the motors braked. Unit tests cover the monitor's limits and the pings stopping. On the Pi
with the real OAK and Hailo-8 and simulated motors, a 100-second run under a watchdog unit
like the service started all six nodes, tripped nothing and stopped cleanly. Still to do:
real I2C and USB losses on the rebuilt robot, by unbinding the OAK's USB port and the I2C
adapter in software with the waist at rest.

## Running with missing hardware (2026-11-04)

After the rebuild on 2026-11-04 the robot had its AI HAT+ out (no /dev/hailo0), both
cameras off USB (the ZED had earlier shown only its 2b03:f681 IMU), and I²C boards that
came and went with the HAT stack's seating. The bot refused to start on each missing part.
To reach the motor-test panel at all it ran from a temporary launcher that stubbed the
OAK, the Hailo detector and the stall monitor, which ended the bot when no frame came
within 20 s.

The launcher now tries every part and runs with whatever works (`core/bot/health.py`,
`core/bot/motor/motor_boards.py`). Each part is a component with a status, a
plain-language message, a hint and since when, sent to the dashboard as `health` messages
(docs/stream.md). Presence probes tell a missing part from a failing one: the USB device
list for the OAK (03e7) and the ZED (2b03:f682 video, f681 IMU), /dev/hailo0 for the
Hailo-8 and /dev/i2c-1 for I²C, so "no OAK on USB" and "the OAK is on USB but failed" get
different hints. The servo board and the Motor HAT open separately. A missing one's
actuators are stand-ins that record their commands and send nothing, so no command goes to
a board that is not there; the motor watchdog stays on, and dashboard motor tests on a
missing board are refused with the reason. A camera, detector or depth node that stops or
crashes is reported against its part and the bot runs on; any other node ending still ends
the bot, which brakes the motors, for the service to restart it. The ZED Mini and its IMU
start without their device and pick it up when it appears; the other parts come back as
"Recovery" below describes.

The stall monitor keeps its restart only for a stream that was running and went quiet
while its source still counted as working, the freeze it was made for. A stream that never
started is reported instead: restarting cannot plug a camera in, and a bot that restarts
every 20 s cannot be used to find out what is wrong. A stream whose source crashed or was
unplugged is not watched until it runs again. The cost: a camera that drops off USB
without its node noticing looks like a freeze, so the bot restarts once, then starts
without it and reports it missing.

Evidence. `tests/test_degraded_startup.py` runs the real launcher, controllers and server
on fake parts. With nothing plugged in the bot starts and reports every part missing with
its hint. A missing servo board leaves the waist and jaw driven (their writes reach the
fake Motor HAT) and refuses pan tests; a missing Motor HAT leaves the head driven. A ZED
showing only its IMU says to flip the plug; an OAK on USB that fails is an error, not
missing; StereoNet without the Hailo says to use the CPU matcher. An OAK unplugged while
running is reported missing and the bot runs on past five frame timeouts; a ZED plugged in
later is picked up; an OAK that freezes while connected still restarts the bot; health
reaches a viewer that asked for nothing. The launcher now imports without the Pi's camera
libraries, so these tests and the recovery tests above run on any computer.

Live checks. Every 5 s, in a worker thread, the bot repeats the checks that were done by
hand on 2026-11-04 and sends them in the health message: which I²C addresses answer, the
servo board's own registers (awake, its frequency, and each head channel's pulse against
the pulse its commanded angle needs), USB presence of the OAK, the ZED's video and its
IMU, the Hailo-8 on PCIe, the Pi's throttling flags and PMIC rails, lost-arbitration
errors in the kernel log, and the motor watchdog's brakes. The readback answers the
question that afternoon ended on: the chip was awake at 50 Hz with 1.5 ms pulses on every
channel, so commands reached it, and a servo that still did not move pointed past the
chip, to the OE wire, V+ power or the plugs; the servo board's hint now says so whenever
the pulses match. The bot already holds the robot lock, and the kernel runs each I²C
transfer whole, so probing beside the controllers needs no other coordination: reads and
zero-length writes change nothing on a PCA9685. `tests/test_diagnostics.py` checks the
arithmetic, the readings of the Pi's own command output, and the judgement, on a fake bus.

Recovery. Every 5 s the bot brings back each part whose node is not running once it can
start: the OAK once 03e7 shows on USB, the Hailo-8 once /dev/hailo0 is there (after
another program lets go of it; seating the AI HAT+ takes a power cycle anyway), depth once
the ZED streams, and a camera or IMU node that crashed. Depth finds its calibration by the
camera's serial number, so it now waits for the ZED instead of failing at startup with
"The ZED reports no serial number", which the robot showed on 2026-11-04 with the ZED
simply unplugged. The new node takes the old one's place; the dashboard's OAK results and
status follow it, and the stall monitor watches its stream only after it has delivered.
Each part is made in a worker thread, so the motors' control loop and its watchdog run on
while an OAK boots, which takes seconds. A part that is there but fails to start, or stops
again within a minute of starting, is tried again after 5 s, then after waits that double
up to 5 minutes. While the OAK is out, its model cache is read again every 30 s, so models
fetched meanwhile show as downloaded without a restart (on 2026-11-04 the face_landmarks
folder had gone from the cache although its depthai lock file remained, and fetching it
again fixed it). The OAK is no longer searched for when USB lists no 03e7 device: on the
robot that search added 11.6 s to every start. While a camera or the IMU stays away, its
warnings fade: the first failure is logged, then one after waits that double from 30 s to
10 minutes, and a line when it works again; the health messages carry its state the whole
time, and `diagnostics_time` says when the live checks last ran, so readings left by a
failing check show as stale.

The motor boards are not brought back live. The bot sets them up and centres the head only
at startup, and the controllers, the motor watchdog and the dashboard's motor tests all
hold the actuators they started with; a restart re-initialises the board's outputs and
starts from a known state. So a board that answers after the start is degraded, with
`restart` saying to restart the bot, and motor tests on it are refused with that reason. A
board the bot drives that misses two checks in a row is an error before its next command
fails, which stops the bot for its service to start it again. In `--zed-only` mode the bot
holds no robot lock, so its checks leave the I²C bus alone.

Evidence for recovery, in the same tests on fake parts: an OAK plugged in later is brought
back, made off the event loop, and the dashboard's results follow it; an OAK unplugged and
plugged back in replaces its old node instead of running beside it; a Hailo-8 held by
another program is picked up once free; an OAK that stops each time it opens is tried less
and less often; tries at a part that keeps failing wait 5, 10, 20 s and then the limit,
and start over once the part is gone; depth starts once the ZED streams; a servo board
that answers after the start asks for a restart and refuses pan tests until then; a driven
board that stops answering is an error until it answers again; the ZED-only bot never
opens the I²C bus.

## ZED Mini stereo capture (2026-11-02)

The ZED Mini (serial 12395566) is read as a plain UVC camera, because the ZED SDK
needs CUDA. `ZedCameraNode` finds its V4L2 capture node by the name the camera
reports, reads raw side-by-side YUYV frames with OpenCV in a worker thread and
publishes them on `StereoFrameTopic` as they came (until 2026-11-03 it converted each
eye to RGB first; see "One conversion per eye" below). Frame times are the kernel
buffer timestamps, which uvcvideo takes on CLOCK_MONOTONIC: the clock of
`time.monotonic()` and of the OAK's depthai timestamps. They mark the arrival of a
frame's first USB packet, one transfer time (19 ms at VGA, 30 ms at HD720) before the
frame can be read. Frame numbers count camera frame periods, so a dropped frame skips
a number. The runtime server sends both eyes in one FreeMoCap frame message, as
cameras `zed_left` and `zed_right`. `--zed` adds the camera to the bot; `--zed-only`
runs only the ZED and the dashboard, so it needs no robot lock.

Each mode measured for 4 s on the Pi with OpenCV 5.0: reading plus converting both eyes.

| Mode, both eyes | Rate | CPU, of a core | YUYV to RGB |
| --- | --- | --- | --- |
| VGA 1344×376 @ 30 | 30.0 fps | 12% | 0.45 ms |
| VGA @ 100 | 99.8 fps | 38% | 0.43 ms |
| HD720 2560×720 @ 30 | 30.0 fps | 31% | 1.96 ms |
| HD720 @ 60 | 60.0 fps | 66% | 1.93 ms |
| HD1080 3840×1080 @ 30 | 29.9 fps | 70% | 4.6 ms |
| HD2K 4416×1242 @ 15 | 15.1 fps | 45% | 6.2 ms |

Frame spacing stayed within 0.2 ms of the period in every mode. The node defaults to
VGA at 30 fps; depth quality may justify HD720.

- **Orientation and view.** The image is upright, unlike the OAK's. The skull's nose
  bridge is a fixed, out-of-focus occluder on the inner side of each eye: about the
  right 30% of the left image, wider at the bottom, and the left 20% of the right
  image. Only the centre of the view is seen by both eyes; depth must mask the rest.
- **Damaged frames.** uvcvideo runs with `nodrop=1`, so frames with transfer errors
  reach applications, and OpenCV does not report V4L2's error flag. In the first test
  one frame of 1,061 had a band of zeroed rows across both eyes, while the OAK was
  booting. None of 960 probe frames afterwards, nor any of 17,580 frames the node
  captured in later runs, had one. The node drops frames whose sampled rows are mostly
  zero bytes and counts them. If torn frames appear instead (stale data, not zeros),
  `options uvcvideo nodrop=0` in `/etc/modprobe.d/` makes the driver drop them.
  Damaged frames also appeared when the reader fell behind: rectifying HD720 pairs in
  the capture thread (136% of a core) lost 8 of 173 frames. Keep the capture loop light.

With the bot (`--hardware --simulate-motors --no-window --zed`, OAK and pose at
30 fps), a client on another computer over Wi-Fi received OAK frames and ZED pairs at
29.1/s each (22 and 42 KiB per message) while a browser on that computer also
watched. The bot process used 107% of a core, against about 70% without the ZED
above, and the whole Pi 28% of four cores with no Chromium on the Pi. No throttling;
52 °C. A ZED-only run streamed 7,141 frames in 4 minutes, none lost.

**Rectification.** Stereolabs' factory calibration for this camera
(`ultraskelly/calibration/zed/SN12395566.conf`, downloaded unmodified; see `SOURCE`)
gives per resolution each eye's intrinsics and distortion, and for the pair a 63.0 mm
baseline and a rotation (RX, CV, RZ) of 0.04°, 0.50° and −0.04°.
`stereo_rectification.py` passes the rotation and translation to `cv2.stereoRectify`
as written (zero-disparity, `alpha=0`), as Stereolabs' MIT-licensed zed-open-capture
sample does. With the rotation transposed instead, matched features sat further from
their rows (median |dy| 1.25 against 0.74 px at VGA, 3.0 against 1.15 px at HD720)
and disparities changed by 12 px at HD720, so the convention matters for depth too.
`python -m scripts.zed_row_alignment` checks a live pair:

| Eyes | Matches | dy median, raw → rectified | \|dy\| median | 90th percentile |
| --- | --- | --- | --- | --- |
| VGA 672×376 | 72 | −5.66 → −0.31 px | 5.66 → 0.87 px | 2.28 px |
| HD720 1280×720 | 142 | −11.48 → −0.84 px | 11.48 → 1.05 px | 3.78 px |

Rectified, both eyes share a 339.8 px (VGA) or 683.3 px (HD720) focal length. The
constant −0.84 px left at HD720 (−0.31 at VGA) is a tilt of about 0.07° that the
factory file does not hold; a measured correction can remove it if depth needs it. The
skull's eye sockets now hide most of each view (since the rebuild), so the matches lie
in the small region both eyes see; repeat the check once the ZED is re-seated.

Remapping is the expensive step. In OpenCV 5.0 on the Pi, one 672×376 colour image
takes 2.4 ms with float tables and 4.3 ms with fixed-point ones (OpenCV 4.10: 4.6 and
4.3 ms); a grey image 1.3 ms. Rectifying every pair at 30 fps would cost about 14% of
a core at VGA, and HD720 fell behind. So the node publishes raw pairs, and depth
rectifies only the pairs it uses, at its own rate. (The node's own `rectify` option, off
by default and used by nothing, was removed on 2026-11-03.)

**Depth, CPU baseline.** `StereoDepthNode` (`--depth` with `--zed` or `--zed-only`)
takes the newest pair at most 5 times a second, rectifies the left eye in colour and
the right in grey, and matches them with OpenCV's StereoSGBM in 3-way mode (64
disparities, 5-pixel window, uniqueness 10, speckle filter). Depth is
`focal_length × baseline / disparity`, published as uint16 millimetres beside the
rectified left image (0 = unknown, nearer than 0.3 m or further than 8 m). Matches
within 4 px of the 64-pixel search limit are dropped: where the true disparity lies
beyond it, as for the skull a few centimetres in front of the eyes, the matcher
settles at the limit. On a live pair that pile-up held 22% of the valid pixels, and it
moved when the search range grew. The nearest depth kept at VGA is therefore 0.36 m.
The dashboard showed each depth frame as `zed_rectified` and `zed_depth` (red near,
blue far, black unknown) until it drew depth from the depth message that follows; since
2026-11-03 those preview frames are sent only with `--depth-previews`.

Measured on the Pi at VGA, capture and depth without the server, 15 s each. The skull
hides most of the view, so 18% of pixels had a depth.

| OpenCV threads | Depth rate | Compute per frame | Age when published | CPU, of a core |
| --- | --- | --- | --- | --- |
| 4 (default) | 5 Hz | 47 ms | 79 ms | 91% |
| 1 | 5 Hz | 67 ms | 88 ms | 39% |
| 1 | as fast as possible: 15.1 Hz | 66 ms | 100 ms | 106% |
| 4 | as fast as possible: 21.1 Hz | 47 ms | 85 ms | 331% |

OpenCV's default thread pool doubles the CPU of the same work here (idle workers
spin). With the server streaming 30 raw pairs and 5 depth frames a second, a ZED-only
run used 122% of a core.

**One OpenCV thread for the bot (2026-11-02, approved).** `python -m ultraskelly` now
calls `cv2.setNumThreads(1)` before building the bot. Each row is one 20-second sample
of the bot under `--hardware --simulate-motors --no-window`, measured from the PC; pose
rate is the OAK frames that reached a viewer with their poses.

| Bot | OpenCV threads | Bot process, of a core | Whole Pi, of 4 cores | Pose |
| --- | --- | --- | --- | --- |
| OAK and pose only | 4 (default) | 61% | 15% | 29.6 fps |
| OAK and pose only | 1 | 52% | 14% | 30.1 fps |
| with `--zed`, no depth | 4 | 101% | 26% | 30.1 fps |
| with `--zed`, no depth | 1 | 76% | 20% | 29.6 fps |
| with `--zed`, depth at full size, 5 Hz | 4 | 174% | 48% | 27.2 fps |
| with `--zed`, depth at full size, 5 Hz | 1 | 112% | 31% | 30.1 fps |
| with `--zed --depth` today (half size, 10 Hz) | 1 | 95% | 25% | 29.6 fps |

**StereoNet beside pose.** HailoRT gives a process the Hailo-8 once: the depth node's
own VDevice failed with `HAILO_OUT_OF_PHYSICAL_DEVICES` while the pose node held one.
Both now share `hailo_device.shared_vdevice()`, whose round-robin scheduler switches the
chip between the two models. StereoNet (Model Zoo v2.17, six contexts) holds the chip
for about 207 ms a frame, and a running frame cannot be pre-empted, so pose waits:

| StereoNet rate | Pose | StereoNet frame |
| --- | --- | --- |
| none | 30.0 fps | |
| 1 Hz | 24.9 fps | 207 ms |
| 1 Hz, StereoNet at scheduler priority 8 | 25.4 fps | 206 ms |
| as fast as it runs (4.8 Hz) | 4.7 fps | 212 ms |

Alone, StereoNet took 198 ms a pair, against the Model Zoo's 100 ms on a faster PCIe
link. Its two inputs are wired left then right: swapped, its depth differed from the
CPU's by about 50% instead of 9-16%.

**Depth quality, judged offline.** The skull blocks most of today's live view, so the
two matchers were compared on three side-by-side frames saved at 15:10 that show the
room and a person about 1.2 m away (`python -m scripts.stereo_quality`). With no ground
truth: reprojection error (shift the right eye by each pixel's disparity and compare it
with the left, on textured pixels), frame-to-frame variation of the still scene, and
coverage, all only where both eyes see the room (the view mask, below).

| Matcher | Depth on pixels both eyes see | Reprojection error, median | Varies between frames |
| --- | --- | --- | --- |
| OpenCV SGBM, full size (672×376) | 64.7% | 3-4 grey levels | 2.3% |
| OpenCV SGBM, three-quarter size | 69.5% | 3 | 2.5% |
| OpenCV SGBM, half size (336×188) | 75.5% | 3-4 | 3.0% |
| StereoNet on the Hailo-8, full size | 53.0% (its band of rows only) | 5-8 | 5.2% |

Half-size depth differs from full size by 3% (median), and by less than 10% on 89% of
pixels. StereoNet, trained on driving scenes, blurs the person's outline and differs
from OpenCV by 13% (median). It fills every pixel of its band, with no confidence, so
it also reports depth where nothing matches. The default is therefore OpenCV at half
size; `--depth stereonet` remains for experiments, at most about 1 Hz beside pose.

**Fast and lean.** One VGA depth frame at full size took 64 ms of one core: SGBM 57 ms
of it. The node now rectifies straight into half-size images (one remap from the
full-size frames, 1.4 ms for both eyes) and matches 32 disparities there: 10 ms. That
reaches the same nearest depth, 0.36 m, as 64 disparities at full size. Remapping to
half size without averaging first was no worse than averaging (49.6% coverage, 2.7%
variation, 1.1% difference) and three times quicker. Colour images use float remap
tables and grey ones fixed-point tables, each the faster in OpenCV 5.0 (2.4 against
4.3 ms, and 1.3 against 1.8 ms). Drawing depth in colour fell from 7.1 to 1.5 ms (a
masked `bitwise_and` instead of numpy's boolean assignment, 0.2 against 5.2 ms), and
the rectified image is now encoded once for both depth messages. A depth frame takes
17 ms; at 10 a second the bot with OAK, pose, ZED and depth uses 95% of a core, pose
holds 30 fps, and each `depth_frame` message is about 22 KiB (zlib level 1: 1.3 ms;
level 6 saves 8% for three times the time).

**The view mask.** The nose bridge (before the rebuild) and the eye-socket rims (since)
block part of each eye, and matches there are nonsense. A left-eye pixel whose 9×9 patch
matches no right-eye patch on its row, at any disparity up to 128, by better than 12 grey
levels on average, is blocked: the other eye cannot see the same thing. Blocked regions
that reach a side of the image are filled out to it, which covers the smooth interior of
the bridge too; smaller ones are noise. On the room frames this masked the bridge and a
strip at the left edge, which the right eye cannot see past its own side of the bridge,
39% of the image. `python -m scripts.zed_view_mask` proposes a mask from live pairs and
writes `ultraskelly/calibration/zed/SN<serial>-view-mask.png`, which depth then uses: it
blanks depth outside the mask and matches only the mask's bounding box. It belongs to
one mounting; redo it whenever the ZED is re-seated. No mask is committed yet: today's
mounting is about to change.

A failed depth frame is logged and skipped rather than stopping the bot, and StereoNet
defaults to one frame a second. What streaming stereo and depth needed from FreeMoCap's
SDK is recorded on its `sfn26-ultraskelly` branch, in
[depth-for-other-applications.md](https://github.com/freemocap/freemocap/blob/a7518c96/current-work-plans/03-transport/depth-for-other-applications.md):
a lossless depth block in the frame message, a calibrated-camera helper, a named camera
optical frame, and per-source decoding in the client.

**The tracked face in 3D.** With `--zed --depth` and pose tracking, `FaceLocatorNode`
follows the target pixel the pose detector found in the OAK image (the nose by default)
into the ZED's depth. The OAK-ZED alignment gives that pixel's ray in the ZED's rectified
left camera frame. The ray is marched in 2 cm steps, then 2 mm steps, to the first point
where it passes behind the surface the depth image sees: the median of a 5×5 window,
with unknown depth never counted as a crossing. Each result uses the newest depth frame
taken within 0.25 s of the OAK frame. It goes into `robot_state.face` in the camera frame
and the head frame, with its distance from the point between the eye sockets. One ray
takes 1.2 ms on the Pi, and the bot with it all ran at 94% of a core with pose at 30 fps.
Until it is measured, the alignment is nominal: the OAK upright, centred between the eyes
and 3.5 cm below them, facing the same way, with intrinsics from the OAK-1's datasheet
field of view (69 by 55 degrees), since this OAK has no factory calibration.
`python -m scripts.oak_zed_alignment` (robot lock; motors untouched) measures it: it
matches features between OAK frames and ZED depth frames taken within 20 ms of each other
and fits the OAK's pose to the ZED depth under them with `solvePnPRansac`, writing
`ultraskelly/calibration/oak-zed-alignment.json`. On synthetic points it recovered a known
pose to 1 cm and 0.3° with a tenth of the matches wrong. Tonight it found no feature both
cameras see, because the skull blocks the ZED's view, so the alignment and a check against
a tape-measured face distance wait for the re-seated lenses.

**The ZED Mini's IMU (2026-11-03).** With `--zed`, `ZedImuNode` reads the accelerometer and
gyroscope on the camera's own USB HID interface (2b03:f681) through Linux hidraw: no ZED
SDK and no library, only reads and one ioctl. The report layout, the command that starts
the stream (feature report 0x32) and the scale factors are Stereolabs' zed-open-capture's
(`include/sensorcapture_def.hpp`). Each 64-byte report holds one sample; the interface has
no report IDs, and its stream-status report cannot be read back (the request stalls). The
stream ran 60 s without the keep-alive ping zed-open-capture sends once a second, and one
ping timed out after 5 s, so the node sends none and restarts the stream after a second of
quiet instead. The udev rule `80-zed-mini-imu.rules` (installed by `scripts/setup-pi.sh`)
gives the bot's user `/dev/zed-mini-imu`.

The IMU's own clock counts 800 samples a second, but against `time.monotonic()` it ran
0.78% slow: 793.8 samples arrive a second, none lost in a minute. The node maps that clock
onto the Pi's with a line through the reads that arrived soonest, refitted every 0.25 s over
30 s, so the samples share the frames' clock. The first second's samples wait for the
first fit. With the arrival delays measured on the Pi (median 0.6 ms, 99% within 2.8 ms
after the soonest), a simulation put every sample within 0.4 ms of when it was taken.
Values are raw and in the IMU's own axes. At rest the accelerometer read
(-6.78, 7.25, -1.20) m/s², 10.0 m/s² in all, 2% more than gravity; the gyroscope read
(-0.72, 0.60, 1.34) °/s. Placing those axes in the head frame needs a measured motion, a pan
and a tilt with the gyroscope recording, so they are labelled `sensor` until then. Nothing
steers by the IMU: it is not fed to `OrientationNode` or any controller.

A reader thread inside the busy bot lost 9% of the samples (about 4,200 a minute). The
kernel keeps only 64 reports, 80 ms, and with the Pi at 90% of its cores the bot's other
threads often held the reader off for longer. A separate process loses none, even under
the same load, so the node runs `zed_imu_reader.py`, standard library only, which pipes each
read's reports and time to the node, and starts it again if it stops. In the full bot with
`--record`, `imu.csv` held all 35,795 samples of a 45 s run, none missed. The dashboard gets
them as `imu` messages, about 15 batches a second (docs/stream.md).

NumPy's OpenBLAS thread pool spins after each call, like OpenCV's: the IMU node alone used
6.3% of a core with the default pool and 2.6% with one thread. `python -m ultraskelly` now
sets `OPENBLAS_NUM_THREADS=1` before NumPy loads. Its effect on the whole bot is not
measured yet: the robot was in use by the other workstreams.

**One conversion per eye (2026-11-03).** The ZED node used to convert both eyes from YUYV
to RGB 30 times a second, whether anything used them or not. The stream then converted
each to BGR for its JPEG encoder, and depth converted them again to grey. Now
`StereoFrameMessage` carries the camera's YUYV as it came, and each consumer converts
what it needs once. The stream goes from YUYV straight to BGR in its encoding thread.
Depth matches the two eyes' Y bytes as they are, with no colour conversion, and converts
only the left eye to colour, for the image the depth belongs to (StereoNet, which matches
colour, converts both). The face locator and the recorder are untouched: they read depth
and the OAK's frames, whose contracts are unchanged. Timed on the Pi, which was busy,
for both eyes at 30 frames a second with depth at 10 a second:

| Path | Old | New |
| --- | --- | --- |
| With a viewer | 102-110 ms a second | 94-96 ms |
| With no viewer | 71-78 ms | 39-40 ms |

Depth's own share rose from 2.5 to 3.9-4.0 ms a frame: it now converts the left eye
itself and remaps that eye's grey apart from its colour. Rectifying the left eye's
chroma and converting at the small size instead took 5.1 ms, in part because building
its chroma planes in NumPy is slow, so it was not kept. The larger cost on this path is
JPEG-encoding both full-size eyes for a dashboard that draws them at 317×178: 4.8 ms a
pair, 145 ms a second at 30 frames, against 2.2 ms (66 ms a second) for eyes halved
first. Acting on the size a viewer reports it draws ("What a viewer draws" in
`docs/stream.md`) would save that.

## FreeMoCap's reviewed SDK and its consumer checks (2026-11-02)

FreeMoCap reviewed the stream SDK UltraSkelly uses and asked for consumer checks on the
robot ([handoff](https://github.com/freemocap/freemocap/blob/f729e8aeaba495839d7372c2f20c18c96618f4f6/current-work-plans/03-transport/ultraskelly-consumer-handoff.md)).
UltraSkelly moved to that batch, `f729e8ae` on `sfn26-ultraskelly`: `uv.lock` and the
vendored bundle changed and no code did. Using what the batch added:

- Each pose message carries the target published from the same detections and the row of
  the person it belongs to. In each frame the followed person is declared as tracker
  `target`, the others as `person-<row>`, and `target` also carries a custom numeric
  channel, `ultraskelly.target`, so a viewer draws the target with the image it was
  found in. The 20 Hz `robot_state.target` drawn over video came from another moment.
  [The stream guide](stream.md) describes every field; the dashboard adopts the channel in
  its rebuild.
- `python -m ultraskelly.replay --serve --people N` feeds the real producer from a
  synthetic camera: N people at 30 fps whose rows are shuffled every frame.
- `scripts/stream_acceptance.py` runs FreeMoCap's consumer checks in headless Chromium.

Evidence, on the Pi in Chromium 156 (headless, the page served over plain HTTP from a LAN
address, so not a secure context): against the synthetic camera all eight checks passed,
including identities kept through row reordering (303 of 303 frames), the custom channel's
descriptor and values, retries stopping after a disconnect during a connection failure,
and a reconnect drawing its first frame without earlier messages. Against the bot, with
the OAK and the Hailo-8 and simulated motors, the five checks that do not need people in
view passed: 30.1 frames/s decoded and drawn and 19.1 `robot_state`/s. Per frame on the OAK's
29 KB frames:

| Protocol | ms | Image path | ms |
| --- | --- | --- | --- |
| CBOR decode and validation | 0.065 | Payload read | 0.005 |
| Frame resolution | 0.001 | JPEG decode (median) | 2.9 |
| Channel views | 0.001 | Draw (median) | 0.4 |

As before, message handling is negligible and JPEG decoding is the client's cost.
FreeMoCap's [consumer report](https://github.com/freemocap/freemocap/blob/be58d412155902a0e6542d33c519c9212e7d94e1/current-work-plans/03-transport/ultraskelly-consumer-report.md)
has both runs, the method and the wishes that came out of them. Not yet checked: people
in front of the robot's camera, and a GPU-drawn window (headless Chromium had no GPU
context; the dashboard's WebGL view needed `--use-angle=swiftshader
--enable-unsafe-swiftshader` there). Any Chromium started in the Pi's desktop session
needs `--password-store=basic` and a throwaway profile, or it asks for the keyring
password on the robot's screen.

## Dashboard layout rebuild (2026-11-02)

The dashboard now follows [the approved layout](dashboard.md): the 3D scene as the main
panel, the tracking camera and the ZED eye and depth tiles in a column beside it, and the
readouts and motor tests in a strip below. Each image source has its own newest-wins decode
slot, so the ZED pair cannot hold up the OAK. People are drawn by tracker ID, and the target
ring comes from that frame's `ultraskelly.target` channel. The tracking tile shows whichever
camera `robot_state.overlays` describes, so `replay --serve --people N` shows there too.

Checked on Windows in headless Chrome against a synthetic stream (OAK frames with keypoints and
ZED pairs in one message, both at 30 fps): at 1920×1080, 1920×950, 1366×657 and 390×844 every
panel had its specified size, nothing overflowed, both sources drew 30 of 30 frames a second,
and the console stayed clean. Against `replay --serve --people 3`, all 156 frames in 5 s
carried the target channel and a `target` person.

On the Pi, the bot from the branch ran with `--hardware --simulate-motors --no-window` (OAK and
Hailo-8; ZED not started), with the dashboard in the full-screen Chromium window of
`ultraskelly-dashboard` and nobody in view. The OAK tile drew 30 frames a second from a
30.2 fps camera. Over 20 s Chromium used 1.02 cores, the bot 0.74 and the whole Pi 45.5% of 4
cores, with `throttled=0x0`. The old layout's figures in the table above are 1.09, 0.73 and
47%. One sample; the eye tiles were not exercised on the Pi.

### Depth views (2026-11-02)

The dashboard draws the ZED's `depth_frame` as heat-map and raw tiles and as a point cloud
and mesh on the skeleton's head ([how](dashboard.md#depth)). Checked on Windows against
W2's synthetic sample, whose geometry is known: every depth pixel fell on the wall, the floor
or the ball within 0.5 mm, and the ball's centre landed at (-0.168, 1.880, 1.131) m in the
scene, as the camera-to-head pose and the skeleton's 1.88 m eye height predict. Drawing
only on change cut the scene from about 60 to 6.7 renders a second with depth flowing.

On the Pi's monitor, fed by a fake stream on the Pi that replays W2's samples (no robot
hardware; OAK-sized frames with a moving person, ZED-sized pairs, both at 30 fps, and depth
5 times a second, with the head following the person): the OAK and ZED tiles drew 30 frames a
second each, the depth 5, and the cloud turned with the commanded head. Over 20 s Chromium used
0.99 cores with the synthetic sample (every pixel known) and 1.05 with the live one (20%
known); the whole Pi was at 27% and 28% of 4 cores, with `throttled=0x0`. One sample each;
live depth with the real matcher running on the same Pi is not measured yet.

### Gaze ray (2026-11-02)

The 3D scene draws the OAK's view cone and a ray from it through the followed person's target
pixel, ending where it meets the ZED depth ([how](dashboard.md#gaze)). Checked on Windows in
headless Chrome with the synthetic depth sample and a target pixel aimed at its ball, (401, 221)
from the OAK's best-guess pose: the face point landed 0.115 m from the ball's centre, within
5 mm of its 0.12 m surface, and the target row read 0.90 m. With the live sample, whose sparse
depth has nothing on that ray, it stayed a dashed direction. The poses of both cameras in the
head are nominal, so the face point is an estimate until W2 measures it or the camera bracket
gives the poses.

### Dashboard work per frame (2026-11-03)

The 3D scene had drawn for each of three unsynchronised inputs (the gaze each tracking frame,
`robot_state`, the depth) from an animation loop that woke on every display frame; the readouts
rebuilt about 40 elements on every `robot_state`; and the depth worker coloured the heat map
pixel by pixel. Now the scene draws on demand at most once per tracking-camera frame, without
multisampling on the Pi's GPU; text is written only where it changes, and node health and rates
once a second; and the heat map is coloured on the GPU, its range taken from every 4th pixel
([how](dashboard.md#frame-rate-on-the-pi)). The tiles had also rewritten their `live` and
`stale` classes with every video frame: `classList.add` and `remove` set the attribute even when
nothing changes, 200 mutation records a second on their own.

Measured in headless Chrome on Windows at the Pi's layout, against a fake stream with a moving
person: OAK and ZED frames at 30 a second, `robot_state` at 20 and the live depth sample at 10, on
independent clocks as on the Pi. One 8 s run each, the commit before (8e98317) against this one:

| Per second | Before | After |
| --- | --- | --- |
| 3D renders | 32 | 28 |
| Animation-frame callbacks | 60 | 36, and 13 timers |
| DOM mutation records | 645 | 81 |
| Layout and style recalculation | 16.0 ms | 4.3 ms |
| Renderer process | 0.38 cores | 0.31 cores |
| Idle (head held, target fixed, no depth): renders, callbacks, mutations | 1, 60, 483 | 0, 0, 1 |

The depth worker's round trip fell from 3.1 to 1.9 ms a frame. The GPU heat map matches the
old colouring within 3 of 255 levels on both of W2's samples, unknown pixels stay black, and
the enlarged copy is identical. The 81 remaining mutation records are text that changes: the
target pixel, the face position and age, the commanded angles. An orbit drag ends where it
did before and settles as fast, with half the renders. On the Pi this commit alone barely
moved Chromium's total (below).

### Dashboard CPU on the Pi (2026-11-03)

W2 measured Chromium at about 186% of a core with main's dashboard open, against about 106%
before the time-series milestone, and the bot's depth slowing from 17 to 36 ms a frame. With
`ps`, the Pi's own dashboard window (main's dashboard, the bot from the service, nobody
touching either) used 1.75 to 1.81 cores, by thread: the renderer's main thread 0.60, its
image decoding 0.26, its compositor 0.09, the depth worker 0.06, the GPU process's main
thread 0.41 and compositor 0.06, the browser 0.20, the network service 0.06. The bot used
about 1.6 cores, so the Pi's four cores were about 90% busy.

What it spent them on, from the DevTools profiler and from switching parts off (Windows,
headless Chrome with its CPU slowed four times, which keeps the main thread as busy as the
Pi's): the 3D scene, 67 draw calls a render, 64 of them the skeleton's separate meshes; the
video tiles, decoded by the SDK through a `Blob`, which copies every JPEG through the browser
process; about 60 page frames a second, because the cameras, `robot_state`, the depth and
the IMU each drew on their own; the time series, redrawn 10 times a second; and a console
warning for each `imu` message, a kind the dashboard had not registered.

f60f561 draws the page once per tracking-camera frame, decodes with WebCodecs, merges the
skeleton (17 draw calls) and draws the time series 4 times a second ([how](dashboard.md#frame-rate-on-the-pi)).
Against a fake stream with the Pi's rates (OAK and ZED 30 a second on their own clocks,
`robot_state` 20, depth 10, IMU 15):

| | main (e15087d) | f60f561 |
| --- | --- | --- |
| Pi, headless Chromium on its GPU, `ps` over 60 s | 1.57 cores: renderer 0.77, GPU 0.62, browser 0.13 | 1.17 cores: renderer 0.45, GPU 0.63, browser 0.03 |
| Windows, headless Chrome | 0.69 cores, main thread 134 ms a second, 60 page frames a second | 0.50 cores, 80 ms, 30 frames |

One run each on the Pi, with the bot not running (its service was waiting for the robot lock
this measurement held, see below). The GPU process did not fall: what it spends 0.6 cores on
is the next question, after the Pi's rebuild. The largest remaining cost is likely the ZED's
two 672×376 eyes, decoded 30 times a second each to be drawn at 317×178: encoding them at the
drawn size, as the display feedback allows, would also save the bot's encoding.

Holding `/run/lock/ultraskelly-robot.lock` keeps the bot service from starting
(`run-bot.sh` exits 75 and systemd retries), even for a measurement that uses no hardware;
this one kept the dashboard without the bot for about four minutes.

## The OAK's own chip (2026-11-03)

Until now the OAK-1's Myriad X (RVC2) ran nothing: it sent 640×480 RGB frames to the Pi,
which turned them over with NumPy and encoded them as JPEG, and the Hailo-8 ran every model.
At the owner's direction the camera's chip now does its share. The owner's priorities: the
face and head pose first, the skeleton next, then everything else that fits, and nothing
computed on two chips.

**Measured first.** `scripts/oak_models_bench.py` runs Luxonis Model Zoo models for RVC2 on
the robot's OAK with the camera at 30 fps, alone and together; second-stage models get fixed
crops, as if one face and one hand were in every frame. 10 s per row; the OAK's two CPUs and
chip temperature from its SystemLogger (43-51 °C throughout).

| Alone | fps | | Together (camera 30 fps) | fps |
| --- | --- | --- | --- | --- |
| YuNet face detection 320×240 | 30.0 | | YOLOv8n pose 2×5 + YuNet 1×3 + head pose 1×1 | 24.0 / 23.8 / 24.1 |
| Head pose, gaze, face landmarks | 30.0 each | | Same with depthai's default resources | 25.8 / 0 / 0 |
| Palm detection 128, hand landmarks | 30.0 each | | YuNet, head pose, gaze, face landmarks, tuned | 30.0 / 29.9 / 30.1 / 30.0 |
| YOLOv8n pose 512×288, depthai defaults | 18.1 | | Same with hands at 5 fps | 27.5 / 25.4 / 21.2 / 26.0 |
| YOLOv8n pose, best (2 threads × 5 SHAVEs) | 26.6 | | Everything at 15 fps | 11.1-14.8 |
| YOLO26n pose 512×288 / 512×384 | 25.5 / 13.0 | | | |

Luxonis' own single-model figures add up to 1.7 times the chip for every model at 30 fps. The
pose models are nano ones, and even alone YOLOv8n reaches only 26.6 fps; beside the face models
it held 24.0, under the 25 fps the owner set for moving it. **The skeleton stays on the
Hailo-8** (YOLOv8s, 30 fps). The full table is in the W7 notes; a comparison of the two pose
models on the same frames waits for a person in front of the robot (`scripts/compare_pose_models.py`).

**What runs where.** Hailo-8: YOLOv8s pose, and StereoNet when asked. OAK, with
`OakCameraNodeParams.models` (on by default; `--no-oak-models` turns it off):

| On the OAK | Rate (one face, no hand in view) |
| --- | --- |
| The 180° turn in the sensor's readout, the JPEG in the hardware encoder, each frame paired with its JPEG on the chip | 30 fps |
| YuNet face detection, every frame | 30 fps |
| Head pose and gaze, up to 2 faces, every second frame | 15 a second |
| MediaPipe face landmarks (468), largest face, every fourth frame | 7.5 a second |
| MediaPipe palm detection on the whole frame, every sixth frame; hand landmarks for up to 2 palms | 5 a second |

With all of it the OAK's real-time CPU (LeonRT) runs at 75-81% and frames reach the Pi at
30 fps, 44 ms after capture. The Pi decodes YuNet's boxes and the palms (NumPy, `yunet.py`,
`mediapipe_hands.py`) and sends each frame's crops back; a Script node on the OAK cuts them
out of that same frame (`oak_script.py`) and chains head pose into gaze on the chip. What the
chip finds in a frame is joined by frame number (`oak_results.py`) and travels in that frame's
stream message ([stream guide](stream.md), "Faces" and "Hands"); the server holds an OAK frame
up to 0.1 s for it. Models come from the zoo once (`python -m scripts.fetch_oak_models`) into
`~/ultraskelly-models/oak-zoo/` and then load without the internet. With one missing and no
internet, the OAK runs none of them and `robot_state.oak.models_error` names that command; on
the robot, with an empty cache behind a dead proxy, frames still came at 30 fps and the
Hailo's pose at 30 a second, and the bot stopped cleanly.

**Pi CPU the OAK and the Hailo changes took off.** Per 30 fps frame on the Pi: the NumPy flip
3.26 ms (9.8% of a core) and the JPEG 2.97 ms (8.9%) are gone. The Hailo's outputs now come as
the chip's 8-bit values with only the cells the decoder reads made real, bit for bit the FLOAT32
values: HailoRT's host work per inference fell from 5.84 to 2.59 ms (9.7% of a core). Its
letterbox padding is filled once. The OAK node waits on its outputs instead of polling every
2 ms. In one batch, same scene, ZED and depth on, the Pi's dashboard watching, 20 s each:

| Bot | Bot process | Chromium | Whole Pi |
| --- | --- | --- | --- |
| main (`79f6d3c`) | 1.02 cores | 0.83 | 50% of 4 cores |
| this work, all the OAK's models | 0.79 | 0.85 | 45% |
| this work, `--no-oak-models` | 0.69 | 0.80 | 41% |

The OAK's image work and the Hailo's outputs saved a third of a core; decoding YuNet and the
palms and sending the faces cost a tenth back. A recorded run kept every frame with the OAK's
own JPEGs (506 at 30.0 fps, none dropped or missed, all decoding).

**Found on the way.**
- depthai's default threads and SHAVEs let one network starve the others (0 fps); each network
  now has measured resources (`RESOURCES`).
- Six camera outputs never delivered a frame; five did.
- Frames held in queues come from the camera's pool, and a camera out of pool frames stops every
  output: with nobody in view the Script stopped reading frames and the whole OAK froze. Each
  output now has a pool of 14 frames, more than the queues and the Script's own 4 can hold.
- The OAK's ImageManip cut normalized rotated rectangles elsewhere than documented. Cut in
  pixels, every crop now matches OpenCV's on the same frame (mean difference 2.6-2.8 grey levels,
  against 16 for a box 20 px off). A crop with a new configuration costs the OAK's real-time CPU
  far more than a fixed one, which is what sets the rates above.
- A frame published in the same moment as another can be lost on a latest-value topic, so the
  server takes each frame's results from the OAK node by frame number.
- "Device crashed" at bot shutdown: 9 of 9 restarts of this work and 4 of 4 of main stopped
  cleanly, the OAK in 1.2-1.5 s. The two crash dumps from the morning of 2026-11-03 hold the
  output of a Script node probe that had crashed the OAK minutes before; depthai reports a
  crash at the next session's close.
- depthai 3.x uploads usage events to Luxonis (device start and stop, model loads, chip load),
  queued in `~/.cache/depthai/telemetry`; the queue on the Pi was draining, so they were being
  sent. The bot and the OAK scripts now set `DEPTHAI_TELEMETRY=0` unless it is already set;
  depthai then logs "Telemetry disabled via DEPTHAI_TELEMETRY".
- From the merge (4af4a76) until the fix, the bot's OAK image was upside down, with the motors
  off the robot: `setImageOrientation(ROTATE_180_DEG)` changes nothing on an OAK-1, whose
  board already reads its sensor out turned 180° by default (`AUTO`; its IMX378 sits upside
  down on the board, [luxonis/depthai#200](https://github.com/luxonis/depthai/issues/200)).
  The coordinator found it from dashboard screenshots before and after the merge and two
  frames, with and without the call, alike at +1.000. The sensor's own readout, `NORMAL`, is
  the turn: on the robot its frame matched the default's turned 180° (+0.83, unshifted) and
  not the default (-0.17), as the NumPy flip did before. Neither depthai's camera features
  (`AUTO`) nor a frame's transformation tell the readouts apart, so
  `scripts/check_oak_orientation.py` builds the bot's OAK pipeline with and without
  `flip_camera` and compares the frames and the JPEG.

**The target.** The head still steers by the Hailo's nose keypoint. YuNet's nose arrives 7-15 ms
sooner (its output reaches the Pi 39-47 ms after capture; the Hailo's pose message 54 ms, the
median of 506 in a recorded run, 58 ms at the 90th percentile), but
YuNet finds only faces turned toward the camera and gives no body angle. It could take over when
a face fills the frame and no body is found; that and its accuracy wait for people in view.

**Not yet checked:** with a person in front of the robot, the pose models on the same frames,
the signs of head pose and gaze, handedness on this unmirrored camera, hand landmarks on a real
hand, the chip's load with two faces, and that people stand upright in the OAK's image once
it is back in the skull.

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
