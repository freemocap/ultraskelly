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
clamped. Test limits: servos within 30 degrees of center, waist up to 0.7 for 0.5 s and at
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

## ZED Mini stereo capture (2026-11-02)

The ZED Mini (serial 12395566) is read as a plain UVC camera, because the ZED SDK
needs CUDA. `ZedCameraNode` finds its V4L2 capture node by the name the camera
reports, reads raw side-by-side YUYV frames with OpenCV in a worker thread, converts
each eye to RGB and publishes both on `StereoFrameTopic`. Frame times are the kernel
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
a core at VGA, and HD720 fell behind. So the node publishes raw pairs (`rectify` is
off by default), and depth rectifies only the pairs it uses, at its own rate.

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
The dashboard shows each depth frame as `zed_rectified` and `zed_depth` (red near,
blue far, black unknown); the depth message for the 3D view follows.

Measured on the Pi at VGA, capture and depth without the server, 15 s each. The skull
hides most of the view, so 18% of pixels had a depth.

| OpenCV threads | Depth rate | Compute per frame | Age when published | CPU, of a core |
| --- | --- | --- | --- | --- |
| 4 (default) | 5 Hz | 47 ms | 79 ms | 91% |
| 1 | 5 Hz | 67 ms | 88 ms | 39% |
| 1 | as fast as possible: 15.1 Hz | 66 ms | 100 ms | 106% |
| 4 | as fast as possible: 21.1 Hz | 47 ms | 85 ms | 331% |

OpenCV's default thread pool doubles the CPU of the same work here (idle workers
spin). `cv2.setNumThreads(1)` applies to the whole process, so it is a bot-wide
choice, not made here. With the server streaming 30 raw pairs and 5 depth frames a
second, a ZED-only run used 122% of a core.

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
