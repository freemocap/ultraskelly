# Live dashboard layout

The owner approved this layout on 2026-11-02. The 3D scene is the main panel; the
camera views sit in a column beside it, grouped by sensor; readouts run along the
bottom. On 2026-11-03 the owner asked for time series, one depth view instead of two and
the Pi's face point instead of a second computation of it. The layout must fit the Pi's
15.6" 1920×1080 monitor (about 141 pixels per inch, a full-screen Chromium app window) and
browsers on other computers, and keep video at camera rate on the Pi 5.

```text
┌ UltraSkelly / live ───────────────────────────────── live · 20/s · simulated motors ┐
│ ┌────────────────────────────────────────────────┐ ┌──────────────────────────────┐ │
│ │ 3D scene, from motor commands                  │ │ Tracking camera (OAK)        │ │
│ │  skeleton, gaze ray to the face point,         │ │  pose, aim, jaw zone, target │ │
│ │  point cloud and mesh on the head              │ │                              │ │
│ │                                                │ ├──────────────────────────────┤ │
│ │                                                │ │ target, face, jaw zone, age  │ │
│ │ layers             Orbit Behind-head Side Top  │ ├──────────────┬───────────────┤ │
│ └────────────────────────────────────────────────┘ │ Left eye     │ Right eye     │ │
│ ┌ Time series ┬ Motor tests ┬ Nodes, rates ──────┐ │ Heat map     │ Face          │ │
│ └─────────────┴─────────────┴────────────────────┘ └──────────────┴───────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Control panel

Since 2026-11-07 the Pi's monitor shows this dashboard with a control column on its left.
That page comes from the control panel, `ultraskelly-control.service`, on port 80
(`http://ultraskelly-rpi5.local/` from other computers). The panel runs all the time and
loads none of the bot's code, so it answers while the bot is off, starting or broken. The
bot's own server on port 8080 serves the same page without the column.

```text
┌ UltraSkelly / control panel ─────────────────────────────────── live · 20/s ┐
│ ┌ Robot ───────────‹──┐ ┌───────────────────────────────┐ ┌───────────────┐ │
│ │ ● RUNNING  up 1:12  │ │ 3D scene                      │ │ OAK           │ │
│ │ following people    │ │                               │ │               │ │
│ │ [Turn off][Restart] │ │                               │ ├───────────────┤ │
│ │ Follow people   ON  │ │                               │ │ ZED Mini      │ │
│ │ answer line         │ └───────────────────────────────┘ │ depth, IMU    │ │
│ │ Status lamps        │ ┌ Time series ┬ Motor tests ────┐ │               │ │
│ │ Last error          │ │                               │ │               │ │
│ └─────────────────────┘ └───────────────────────────────┘ └───────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

- **Readout:** the bot's state as a lit word (RUNNING, STARTING, RESTARTING, WAITING for
  the robot lock, STOPPING, OFF, FAILED), since when and for how long, and one line of
  why: following or holding still, simulated motors, how often it restarted by itself,
  how its last run ended, or which program holds the robot lock.
- **Turn on / Turn off** and **Restart:** `systemctl start`, `stop` and `restart` of
  `ultraskelly-bot`. Turn off lasts until Turn on or the next boot; the bot stays enabled
  at boot.
- **Follow people:** on, the head and waist turn toward people; off, they hold still and
  move only for motor tests. The switch reaches the running bot at once
  (`POST /api/follow` on the bot) and is kept for every later start as `--follow` or
  `--no-follow` in `/var/lib/ultraskelly-control/bot-options`. The bot's service passes
  it after the options in `/etc/default/ultraskelly-bot`, so the switch wins. While the
  bot is off, the switch says which way its next start goes.
- **‹ (fold):** folds the column to a 44 px strip that keeps the bot's lamp and state
  word; a click on the strip opens it again. The browser keeps the choice
  (`localStorage`), so the Pi's window stays folded across reloads and reboots. Turn off
  is the way to stop every motor: the bot's service brakes the DC motors as it stops.
- **Answer line:** what the last command did, the same on every page showing the panel.
- **Status:** the bot's own health checks (see Health) as lamps in five rows: Cameras
  (OAK, ZED Mini), Tracking (OAK models, Hailo-8, Depth), Motors (Servo board, Motor HAT,
  Head aim), Sensors (ZED IMU) and Power. Not checked while the bot is off.
- **Last error:** the newest error in the bot's journal, with the exception that followed
  it, and how long ago; or `scripts/run-bot.sh`'s "Not starting: another program is
  using the robot", with the program that holds the lock.

While the bot is not running, the views keep what they last showed, nothing covers
them, the motor test buttons are greyed out, and the header says "Bot off" (or
"Bot starting") instead of listing every part as failed. The live views read the bot's `/stream` straight from port 8080, so
the panel adds no second copy of the video. Motor tests pass through the panel to the bot.

How it runs:

- On the Pi's desktop, the UltraSkelly icon (`scripts/desktop/ultraskelly.desktop`, put
  there by `scripts/install-services.sh`) runs `scripts/start-ultraskelly.sh`. It starts
  the bot through the panel unless the bot is running or starting, then opens the window
  afresh, full screen and in front. What it did: `journalctl -t ultraskelly-shortcut`.
- `scripts/open-dashboard.sh` opens the window. It first sets the window's saved size to the
  whole screen, so out of full screen it stays full width. Chromium's default is half the
  screen, 945 px, under which the dashboard folds into its one-column phone layout.
- `python -m ultraskelly.control` (aiohttp). `scripts/install-services.sh` installs it
  from `scripts/systemd/ultraskelly-control.service`, enables it and restarts it on every
  run. Restart it after pulling new code: `sudo systemctl restart ultraskelly-control`,
  which moves nothing.
- It runs as the robot's user with `NoNewPrivileges=yes`, so neither it nor any program
  it runs can use that user's passwordless sudo; port 80 comes from
  `CAP_NET_BIND_SERVICE` alone. Starting, stopping and restarting the bot rest on one
  polkit rule, `scripts/systemd/ultraskelly-control.rules`, installed as
  `/etc/polkit-1/rules.d/50-ultraskelly-control.rules`. It allows exactly `start`,
  `stop` and `restart` of `ultraskelly-bot.service`, for that user only.
  `systemctl --no-ask-password` turns a missing rule into an error on the answer line
  instead of a wait for a password nobody can type.
- Its one file, `/var/lib/ultraskelly-control/bot-options` (its `StateDirectory`), is
  replaced whole: written beside it, flushed to the card, then renamed over it, so a
  power cut leaves the old file or the new one.
- Every POST must send JSON and name the robot in its `Host` header: an address,
  `localhost`, the Pi's name or `<name>.local`. So another web page can neither send a
  command nor reach the panel through a name of its own (DNS rebinding).
- It reads the bot's log with `journalctl` as the robot's user, which needs the `adm`
  or `systemd-journal` group; `scripts/setup-pi.sh` checks for it.

Development without the robot:

```bash
python -m ultraskelly.control --port 8000 --local-bot
```

runs `python -m ultraskelly.replay --serve --people 3` as the bot, started and stopped
by the panel's buttons; it moves nothing. `--local-bot COMMAND...`, given last, runs
another command as the bot, which must serve the dashboard's `/stream` and
`/api/motor-test` on `--bot-port` (default 8080).

## Panels

- **Header:** connection state, `robot_state` messages per second, whether the motors
  are simulated or real, and the problems banner (see Health).
- **3D scene:** the schematic skeleton driven by the commanded head pan and tilt, the
  waist drive arrow and the jaw. These are commands, not measured positions, and the
  scene says so. View buttons: orbit, behind the head, side, top. The ZED's point cloud
  and mesh and the OAK's gaze ride on the head, each with a layer toggle (see Depth and
  Gaze).
- **Tracking camera (OAK):** the video with the poses found in each frame, the aim
  crosshair, jaw zone and target ring, and the faces and hands the OAK's own chip finds
  (see Faces and hands), drawn in the browser. The row under it shows the
  target state, pixel, face distance, whether the target is inside the jaw zone, its body
  angle and its age; a narrow tile drops the last of these. Without video, as in
  `replay --serve`, the tile shows the target on a grid.
- **Eyes and depth (ZED Mini):** the left and right eyes (`zed_left`, `zed_right`), the
  depth heat map with a metre scale, and the tracked face in 3D as the Pi's face locator
  reports it (`robot_state.face`): its distance from between the eyes, its position in the
  head frame, the OAK pixel it came from, the alignment that placed it and its age.
- **Readout strip:** the time series (below), motor tests, and node health with the
  message rates and the OAK chip's own (`robot_state.oak`): its frames, faces found, head
  poses, face meshes and hands measured a second, and in one row its two CPUs, LeonOS and
  LeonRT, and its temperature, with its memory and any frames sent before all their
  measurements came in the row's tooltip. A camera whose frames stop appears there too.
- **Motor tests:** the buttons are in the strip card itself, no panel to open
  (`motor-test.js`): Pan −10, 90 and +10 servo degrees; Tilt ▲10 (head up), 90 and ▼10
  (head down), by the head's `tilt_direction`, since a larger tilt angle turns this head
  down; each with the commanded servo angle beside it; Waist − and + and Jaw Open, each sending the drive × seconds in its row; then Stop
  and a status line. The line shows this page's answer for a few seconds, then what the bot
  reports: a test running from any screen, the last test's outcome, or why tests cannot
  run. The card title shows the net waist travel; the test limits are the buttons' tooltip.
  A short strip scrolls the card. Angles stay commands, not where a servo is. After a servo
  test, a line appears when the servo board's read-back shows a problem: the command did
  not reach the chip, the chip is asleep, or its readings are stale. While a board is down
  the controllers still run and record their commands, but nothing reaches it, so the
  commands read "not sent" there and in the time series, the bot refuses that board's
  tests with its message and fix, and the status line shows the board's message, not "Ready".

Click a video tile to enlarge it over the 3D scene; click it again or press Esc to return.

## Time series

Strip charts of the last 30 s (`timeseries.js`), on the Pi's clock (`robot_state.time`):
the head pan and tilt and the waist and jaw drive, as commanded; the target's offset from
the aim point, which is the controllers' input; the face distance from the Pi's face
locator; and the ZED's accelerometer (m/s²) and gyroscope (°/s), measured, from the `imu`
messages ([the stream](stream.md#imu-messages)). The IMU's values are in its own sensor
axes, which the lanes say: the ZED sits rolled, so gravity reads about (-6.8, 7.3, -1.2)
m/s², and their place in the head frame is not measured. No motor reports its position:
the servos have no feedback and the DC motors no encoders, so there is no measured motor
trace, and the panel says so. Each lane's header gives the newest values. The canvas
redraws at most 4 times a second, only when something new arrived, at one point per pixel
column (the first, lowest, highest and last value in it). The IMU's 794 samples a second
arrive in batches of about 53; each batch keeps its lowest and highest value per axis, and
a missed batch breaks the line.

## Sizing

At least 1000 px wide:

- Header 44 px (36 px when the window is under 700 px tall); 8 px padding and gaps.
- Sensor column width: `clamp(300px, min(34.17vw, (100vh - header - 142px) / 1.3125), 900px)`.
  The OAK video (4:3) and the two rows of 16:9 tiles then fit the window height. At 1920
  px wide the column is 656 px, so the OAK's 640×480 frame draws without scaling.
- Readout strip height: `clamp(140px, 22vh, 240px)`; the time series takes 2.4 parts of
  its width, the motor tests and the nodes 1 part each.
- Text 11 to 14 px.

| Window | 3D scene | OAK video | Eye and depth tiles | Strip |
| --- | --- | --- | --- | --- |
| Pi monitor, 1920×1080 | 1240×774 | 640×480 | 317×178 | 238 |
| Desktop browser, 1920×950 | 1314×673 | 566×425 | 280×158 | 209 |
| Laptop browser, 1366×657 | 977×452 | 349×262 | 171×96 | 145 |

Every window at least 562 px tall fits without scrolling. Narrower than 1000 px, the
page is one scrolling column: 3D scene, OAK, ZED tiles, then the cards.

## Frame rate on the Pi

The OAK video is drawn one to one. Each image source has its own newest-wins decode slot,
so the ZED pair never starves the OAK. One depth image feeds the heat map and the 3D point
cloud on the GPU. Covered or hidden tiles skip decoding. Where the page may use WebCodecs
(a secure context, such as `localhost` on the Pi) the tiles decode their JPEGs with
`ImageDecoder`, about twice as fast as the SDK's `createImageBitmap` on a `Blob`, which also
copies each image through the browser process; elsewhere the SDK decodes them, and so does
every frame after one that WebCodecs could not decode.

Everything the page shows changes in one frame per tracking-camera frame
(`frameclock.js`): a decoded OAK picture asks for the frame, and the ZED pictures, the heat
map, the 3D scene, the readouts and the time series that changed since the last one draw in
it too. The cameras, `robot_state`, the depth and the IMU arrive out of step, and each
drawing on its own would make about twice as many frames for the browser to lay out and
composite. A picture is never lost to this: when a newer one for the same tile comes before
the frame, the older draws at once. Without tracking frames, changes draw at most once per
the tracking camera's last frame period. Nothing runs while nothing changes.

The 3D scene draws only when something in it changed (a command, the gaze, the depth or
the view). The skeleton is merged into one mesh per moving part and material, so a render
makes 17 draw calls instead of 67, with the same triangles. On the Pi's GPU (V3D) and on software renderers it draws without
multisampling; `?antialias=1` or `?antialias=0` in the address decides instead. Readouts are
written only where their text changes, node health and message rates refresh once a
second, and the time series draws at most 4 times a second.

## Depth

The ZED's `depth_frame` messages ([the depth stream](depth-stream.md)) are decoded in a
worker (`depth-worker.js`): it inflates the millimetres, takes the heat map's range from
every 4th pixel and decodes the rectified colour image, so the page itself only copies
bytes out of the stream and draws. The heat map is coloured on the GPU from the 16-bit
millimetres (`heatmap.js`), with the legend's turbo colours. Newest wins: while the worker
is busy, only the newest frame waits.

- **Heat map:** each frame from its 2nd to its 98th percentile, red near and blue far, the
  range eased between frames. The legend under the tiles gives that range in metres, the
  share of pixels with depth and the matcher (`sgbm` or `stereonet`). Hover the tile to
  read the depth under the cursor. One view is enough: a second, grey one showed the same
  data.
- **In 3D:** the scene is in metres, with the schematic skeleton drawn at the robot's estimated
  eye height of 1.88 m (skellycad's UltraSkelly mock-up). The vertex shaders unproject the
  16-bit depth with the message's intrinsics and place it on the head through
  `camera_to_head`, whose head frame starts midway between the eye sockets. Depth runs out of
  the face, never up: the ZED's lens axis is the head frame's forward axis, and its image down
  is the head's down. The depth turns with the commanded pan, tilt and roll; the waist angle is
  not measured, so where it points in the room is an estimate. **Cloud** shows every depth
  pixel and **Mesh** joins every second one, dropping triangles at unknown pixels and at depth
  jumps over 6%. The ZED's view cone reaches 1.5 m. Depth that stops arriving disappears after 3 s.
- **Head angles** (`headpose.js`, tested in `tests/test_dashboard_head_pose.py`): the head is
  drawn from its commands at the ratios measured on the robot, so the scene says "Head
  approximate". On 2026-11-06 `scripts/check_motor_directions.py` found the real head turning
  about 0.2° per servo degree of pan and 0.1° per servo degree of tilt (12 servo degrees moved
  the upright OAK picture 11-35 px and 9-10 px, at about 9.3 px per degree). More pan turns the
  head to its own right, more tilt nods it down, and negative waist drive turns the body to its
  right, which the waist arrow shows. That pan and waist turn is the mirror of the simulator's,
  whose camera image is mirrored too. Roll is held at 90; its direction and ratio are not
  measured. Drawn one servo degree per head degree, as before, a tracking controller that wound
  up to 0 or 180 pitched the skull 90° into the floor with the depth lying flat in front of it,
  while the real head moved a few degrees. A head command at the end of its range still says so
  under the scene's title ("Tilt commanded to its 180° limit"). At 90/90 the real head looks up
  at the ceiling; how far is not measured, so `HEAD_AT_LEVEL` draws it level until it is. The
  ZED's IMU could give the head's measured pitch and roll, but its axes have no place in the
  head yet (docs/stream.md, "imu"): two still readings would place them, the ZED level and
  upright, then pointing straight down.

## Gaze

The OAK's view cone starts at a best-guess lens position in the skull's nose hole, 4 cm below
the eye sockets, with a 69° field of view across the image. The gaze ray runs from the lens to
the face point the Pi's face locator found (`robot_state.face.position_head`: the OAK's target
pixel looked up in the ZED's depth), drawn in orange; the dashboard does not compute it again.
Without a face point newer than 0.5 s, the ray is a dashed 3 m direction through the frame's
`ultraskelly.target` pixel, and the head following nobody hides it. Camera poses are nominal
until the camera bracket gives them, and the waist angle is not measured.

## Health

The dashboard says what works, what does not and what to do about it (`health.js`). The bot
checks each part this launch uses, the OAK camera and its models, the AI HAT+ (Hailo), the
ZED camera, its IMU, the depth, the servo board at 0x40 and the Motor HAT at 0x60, and sends
a `health` message (W4's format, docs/stream.md) on every change, every 2 s and to each
viewer as it connects: each part's status (ok; degraded, working in part; missing; error;
off, not used on purpose), a message, a hint in plain words, since when, and, for a part
that fails because another does, which one. The dashboard shows them as the bot writes them,
and adds what only it knows: whether the bot is reachable, whether its checks have
stopped coming (none for 5 s while `robot_state` still comes), and whether its live
readings are old (`diagnostics_time` more than 12 s behind, two missed checks: the
readings say how old, and the motor tests show no read-back).

- **Banner**, in the header: with every part well, a quiet "All hardware OK · 8 checks";
  otherwise a red strip (amber when only degraded) with the number of problems and each one.
  A part that fails because another does is not counted again: with both cameras unplugged,
  the OAK and the ZED are two problems, and the ZED's IMU and the depth follow from the ZED.
- **Health panel**, over the left of the 3D scene (at the top on a phone): one row per part
  with its light, message, status word and since when, the part it follows from, its hint,
  and when a part is back but the bot must restart to use it, why. Then the bot's live
  readings: which I2C addresses answer, the servo board's read-back next to the pulse its
  commanded angle needs, USB, the Hailo device, the Pi's rails and throttling, and the motor
  watchdog. A row's tooltip gives the bot's detail for it. It opens by itself
  when a new problem appears and folds away when all is well; the banner's button, the
  panel's × and the nodes card's lights open and close it, and that choice holds until a
  problem the page has not seen yet.
- **Lights** on the nodes card, one per part.
- **Every view that waits on a part says why**, from the same checks: the OAK tile, the ZED's
  eyes, the heat map, the face card, the target row (no poses without the AI HAT+), the IMU
  lanes and a note under the 3D scene's title each give the missing part's message and fix. A
  feed that stops mid-run keeps its last picture, dimmed, with the reason over it; the 3D
  scene, the time series and the motor tests keep working whatever is missing.

A bot that sends no checks still gets a panel, from what `robot_state` says anyway: crashed
or stopped nodes with their errors, and cameras without frames, with no fixes, since only the
bot could write them.

## Faces and hands

The OAK's own chip finds faces and hands ([the stream](stream.md#faces-found-by-the-oaks-own-chip)),
and `faces.js` draws them over the tracking tile in the pixels the stream gives them in,
scaled to the image as it arrives: each face's box and its five landmarks joined to the
nose, the head-pose axes from the nose (red, green, blue, as OpenVINO's head-pose demos
draw them), the gaze as a yellow arrow from between the eyes, and the 468-point face mesh
as light dots; each hand's palm box, its 21 landmarks and bones, and its handedness score.
The chip measures less often than it detects: head pose and gaze every second frame, the
mesh every fourth, hands every sixth. A face ID names a face in one frame only, so each face
keeps the measurements of the previous frame's face its box overlaps most, for 0.3 s (the
mesh 0.5 s); hands stay until the next ones or for 0.25 s. The axes and handedness are the
models' published conventions, not yet checked against a person in front of the robot, and
the drawing takes the image as the stream sends it, upside down while the OAK's rotation
setting has no effect. Against W7's 8 s clip from the robot and `replay --serve --people 3`
on Windows, Chromium used 4% and 7% more than without them.
