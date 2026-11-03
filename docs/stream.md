# The live stream: what `/stream` carries

The bot serves its dashboard on port 8080 (`python -m ultraskelly.replay --serve` uses
127.0.0.1:8081). One read-only WebSocket, `/stream`, carries FreeMoCap's self-describing
CBOR messages, encoded on the Pi with `freemocap-stream` and decoded in the browser with
`@freemocap/client`. This page is the contract between the producer
(`ultraskelly/ui/runtime_server.py`) and any viewer, including the dashboard.

- **No handshake.** Every message decodes on its own. A viewer that connects or reconnects
  mid-stream draws the next frame without earlier messages.
- **Newest wins.** `robot_state`, each camera, the depth and the IMU have their own latest-value slot, so a slow
  viewer receives fewer, newer messages and the bot never waits for it. Nothing is
  encoded that no connected viewer shows (see "What a viewer draws").
- **Heartbeat.** A text `ping` gets a text `pong`; the client SDK sends it.

## `frame` messages

FreeMoCap's own kind: one message per camera frame. The camera's frame counter is the
envelope `sequence`, the message's `frame_number` and the image payload's frame number
(a count of encoded frames for a camera without a counter). `timestamp` is the capture
time the camera reports, in seconds on the Pi's `time.monotonic()` clock, not when the
message was encoded or sent; a camera that reports none gets the encoding time.

- `cameras`: one uncalibrated entry per image, with its ID (`oak`), index and `image_size`
  `[width, height]`. Images captured together travel in one message, one camera each (the
  ZED below).
- `image`: the SkellyCam image payload with each camera's JPEG.
- `trackers`: on the camera the pose detector reads (the OAK), one observation per person
  found in that very image. Frames travel without trackers while the detector is stalled
  (no pose message for 0.5 s).

### The ZED Mini (`--zed`)

The stereo camera in the eye sockets sends its two eyes together: one frame message with
cameras `zed_left` and `zed_right`, indices after the OAK's, at 30 per second, without
trackers. They are the raw eyes as the camera sees them, 672×376 each, shrunk when every
viewer draws them smaller (see "What a viewer draws"). Their frame
number counts camera frame periods, so a frame lost in USB transfer skips a number, and
the timestamp is when the frame's first USB packet arrived (the kernel's buffer time).

With `--depth`, the depth travels as the `depth_frame` message below, which also carries
the rectified left eye. Only with `--depth-previews` does each depth frame travel again as a
frame message, with cameras `zed_rectified`, the rectified left eye, and `zed_depth`, the
same view's depth drawn in colour (red at 0.3 m to blue at 5 m and beyond, black unknown),
up to 10 per second, for viewers that draw only frames; the dashboard needs neither.

### People

| Field | Value |
| --- | --- |
| `tracker_id` | `target` for the person the head follows in this frame; `person-<row>` for everyone else, where `<row>` is their place in this frame's detections (best score first) |
| `detector_type` | `yolov8_pose` |
| `model_id` | none; `connection_groups` holds `body`, the COCO-17 bones by keypoint name |

YOLOv8 pose does not track people over time, so an ID names a role in one frame, not a
person across frames: `person-1` in one frame and `person-1` in the next may be different
people. Find people by `tracker_id`, never by their position in the array.

Each person has an `OVERLAY_2D` channel: columns `x`, `y`, `visibility`; 17 rows named by
COCO keypoint (`nose` ... `right_ankle`); `camera_id` and `image_size` give the image its
pixels belong to: the camera's, also when the image goes out smaller (see "What a viewer
draws").

### The target: `ultraskelly.target`

The followed person (`tracker_id` `target`) also carries a custom numeric channel that
says where the head is steering in this image:

| Column | Meaning |
| --- | --- |
| `x`, `y` | the target keypoint (the nose by default) in whole pixels of `image_size`, as the controllers receive it |
| `body_angle_deg` | the shoulder line's angle from vertical in degrees, -90 to 90 (±90 for level shoulders, the sign set by which shoulder is on the left of the image); 0 when the shoulders are not visible |

It has one row, named `target`, with the same `camera_id` and `image_size` as the person's
keypoints. It is absent when the head follows nobody: no person, or the best person's
target keypoint is below the confidence threshold. Unlike `robot_state.target`, it belongs
to the image it travels with, so draw this one over video.

## `robot_state` messages

UltraSkelly's application kind, version 0, at 20 Hz while a viewer is connected. Motor
values are the commands sent, not measured positions.

| Field | Contents |
| --- | --- |
| `time` | when the snapshot was taken, in seconds on the Pi's `time.monotonic()` clock |
| `motors` | `simulated`, and `commands` with `value` and `age` (seconds) for `pan`, `tilt` (servo degrees), `waist`, `mouth` (throttle -1 to 1) |
| `servo_state` | the head controller's `pan`, `tilt`, `roll`, `locked_x`, `locked_y`, `age` |
| `target` | the controllers' latest target `x`, `y`, `angle`, `age`; not paired with any image |
| `face` | with `--zed --depth`: where the tracked face is in 3D. `position_head` [x forward, y left, z up] and `position_camera` [x right, y down, z forward] in metres, `distance` from the point between the eye sockets, the OAK `pixel` it came from, which `alignment` placed it (`nominal` until measured) and `age`. Positions and distance are null when no face was found |
| `cameras` | per camera ID (`oak`; with the ZED also `zed_left`, `zed_right`; with depth `zed_rectified`, the depth frames, and with `--depth-previews` also `zed_depth`): `fps`, `age` of the newest frame, `width`, `height` |
| `overlays` | per camera: the `aim` pixel the head steers the target to, and the `jaw_zone` half-width |
| `rates` | message rates in Hz: `servo_state`, `target`, `pose`, `face` with the face locator, and `imu` (batches) with the ZED |
| `nodes` | each node's `name` and `state` (starting, running, stopped, crashed), with `error` when crashed |
| `motor_test` | the dashboard motor tests' limits and what is running or ran last |

## `depth_frame` messages

UltraSkelly's application kind for ZED depth, version 0, with `--depth`: up to 10 per
second, about 22 KiB each. Each message holds one depth frame complete: uint16 millimetres
(little-endian, zlib-compressed, 0 = unknown), the rectified left image it belongs to
(JPEG), that image's pinhole intrinsics (`fx`, `fy`, `cx`, `cy`, `baseline_m`), the camera
axes, `frame_number` (the `zed_left`/`zed_right` frame it came from), `method`, and
`camera_to_head`, the camera's pose in the head frame (nominal until the camera bracket is
measured). [The depth message's own page](depth-stream.md) gives every field, how to turn
pixels into points, and a browser decoding example; `scripts/depth_frame_samples.py`
writes sample messages to build against.

## `imu` messages

UltraSkelly's application kind for the ZED Mini's accelerometer and gyroscope, version 0,
with `--zed`: about 15 batches a second, each with every sample since the last, 794 samples a
second in all. They are for display and recording (`imu.csv` with `--record`); nothing
steers by them.

| Field | Contents |
| --- | --- |
| `source` | `zed_mini` |
| `start_time` | the batch's first sample, in seconds on the Pi's `time.monotonic()` clock, the frames' clock |
| `count` | samples in the batch |
| `first_sample` | the first sample's number since the bot started: when it is not the previous batch's `first_sample` + `count`, batches were missed (newest wins) |
| `samples` | a channel block, kind `ultraskelly.imu`: a row per sample, little-endian float32 columns `time` (seconds after `start_time`), `ax`, `ay`, `az` (m/s², gravity included), `gx`, `gy`, `gz` (degrees per second), row-major in `data` |
| `axes` | `sensor: ...`: the IMU's own axes, not the head frame |
| `units` | `time` s, `acceleration` m/s^2, `angular_velocity` deg/s, `temperature` degC |
| `sample_rate` | samples per second by the Pi's clock, as measured; null for the bot's first second |
| `lost` | samples the IMU took that never reached the bot, so far |
| `temperature` | the IMU's own, in °C |

The envelope `timestamp` is the batch's last sample. The values are raw, with no bias or
scale calibration: at rest the gyroscope read about (-0.66, 0.63, 1.42) °/s, and the
accelerometer 10.0 m/s², 2% more than gravity. The axes are the IMU chip's, and their place
in the head frame is not measured yet; at rest gravity gives only the vertical, which read
(-6.78, 7.28, -1.19) m/s² with the ZED in its present seat. Times are the IMU's own,
mapped onto the Pi's clock from the reports that arrived soonest; with the arrival delays
measured on the Pi, a simulation put each sample within 0.4 ms of when it was taken. The
IMU counts 800 samples a second by its own clock, which ran 0.77% slow, so 793.9 a second
arrive by the Pi's.

## Reading it in a browser

The dashboard imports the vendored bundle (`/vendor/freemocap-client.js`):

- Register `robot_state` (and `depth_frame`, to draw depth, and `imu`, to plot the IMU) as
  application kinds when you construct `TransportService`; registrations are fixed at
  construction. Byte strings arrive as `Uint8Array`; inflate `depth_frame`'s depth with
  `DecompressionStream('deflate')`, and copy `imu`'s `samples.data` before reading it as a
  `Float32Array` (`new Float32Array(data.slice().buffer)`), since a view may start anywhere.
- Keep one newest-wins decode slot per image source, keyed by the camera IDs in the payload
  (`readImagePayload` first, decode after): with the OAK and the ZED both streaming, a single
  slot lets one camera's frames displace the other's.
- `subscribeToOverlay` delivers each person's keypoints just before the image of the same
  frame: pair them by frame number and camera ID, and tell people apart by `trackerId`.
- `subscribeToChannels` delivers every channel of each frame, including
  `ultraskelly.target`, with its owner (`trackerId`), descriptor, row count and float32
  values. Nothing is replayed on subscription; disconnects deliver `null`.
- `decodeImageBitmaps` returns `ImageBitmap`s the caller must `close()`.
- JPEG bytes are views into the received payload, and keypoint and channel values share
  the decoder's storage (an unchanged array can be the same object in the next frame).
  Copy them before modifying or transferring them, for example to a worker.
- A malformed image payload is rejected whole rather than drawn in part.

On the producer side, `freemocap-stream` rejects non-integer frame numbers, camera
indices, widths and heights (NumPy integers are fine, floats are not) and non-finite
timestamps.

## What a viewer draws

A viewer may tell the server what it draws, so the Pi encodes only the feeds someone shows.
It sends one JSON text message on `/stream`, FreeMoCap's inbound display feedback
(`TransportService.sendDisplayFeedback`) with one more field:

```json
{"frameNumber": 1234,
 "displayImageSizes": {"oak": {"width": 640, "height": 480},
                       "zed_left": {"width": 317, "height": 178}, "zed_right": {"width": 317, "height": 178}},
 "kinds": ["depth_frame"]}
```

- `displayImageSizes`: every camera ID the viewer draws, with the size in device pixels it
  draws it at. A camera that is not listed is not wanted. A stereo pair is wanted when
  either eye is listed.
- `kinds`: the application kinds the viewer wants besides `robot_state`, which always
  flows. Today those are `depth_frame` and, with the ZED, `imu`.
- `frameNumber`: the newest frame the viewer drew.

The dashboard sends it after every connect and whenever what it shows changes, at most
every 250 ms: a tile scrolled out of view, a tile enlarged, the 3D depth turned off, or the
page hidden, which asks for nothing. A viewer that never sends one gets every feed.
`robot_state.cameras` keeps reporting every camera.

The server encodes a feed only while some viewer draws one of its cameras, and sends each
viewer only the cameras it draws and the kinds it wants. A depth frame's `depth_frame` is
encoded only while some viewer wants it, and its preview (with `--depth-previews`) only
while one draws `zed_rectified` or `zed_depth`. A camera just asked for gets its newest
frame at once rather than at its next one. A text message without `displayImageSizes`
changes nothing.

The sizes set how large the images go out. When every viewer that draws a feed draws it at
most three quarters of its camera's size, its images are encoded at the largest drawn size
instead (each image's `image_size` says which): the Pi's dashboard draws the ZED eyes at
317×178, so they go out at 318×178, 4.5 times fewer pixels. An enlarged tile reports its
large size, and a viewer that gives no size gets the camera size. Keypoints, the target and
the guides in `robot_state.overlays` stay in the camera's pixels, which each channel's
`image_size` and `robot_state.cameras` give, so a viewer scales them to the image it got.
A frame that came as the camera's own JPEG goes out as it came. `depth_frame` and
`robot_state.cameras` always describe the camera's own size.

## Checking a stream

`scripts/stream_acceptance.py` runs FreeMoCap's consumer acceptance checks against any
`/stream` in headless Chromium, with a throwaway profile, no keyring and no proxy lookups,
and prints the results as JSON. It serves its page over plain HTTP from the machine's
network address, so the page is not a secure context. On the Pi, against the synthetic
camera (no robot lock needed):

```bash
.venv/bin/python -m ultraskelly.replay --serve --people 3 --host 0.0.0.0 --port 8085 &
.venv/bin/python -m scripts.stream_acceptance --stream ws://$(hostname -I | cut -d' ' -f1):8085/stream --people 3
```

Against the bot, start it under the robot lock and point `--stream` at port 8080 with
`--people 0`; the checks that need people in view then report `applicable: false`.

| Check | Passes when |
| --- | --- |
| `plain_http_connection` | the page connects without TLS |
| `custom_robot_state` | `robot_state` arrives and parses, at least 5 per second |
| `camera_decoding` | frames decode and draw, with every bitmap closed |
| `several_tracked_subjects` | at least two people (or `--people`) share a frame |
| `identities_survive_reordering` | in every frame, the overlay declared `target` is the person the target channel points at, wherever their row is |
| `custom_numeric_channel` | `ultraskelly.target` arrives with its descriptor, one row and finite x, y |
| `reconnect_mid_stream` | after `disconnect()`, and in a new client, the first frame draws without earlier messages |
| `retries_stop_after_disconnect` | a client that disconnects during a connection failure makes no further attempt |

It then times the protocol (CBOR decode and validation, frame resolution, channel views) and
the image path (payload read, JPEG decode, draw) per frame on the frames it captured.

## Moving the SDK

`pyproject.toml` takes `freemocap-stream` from FreeMoCap's `sfn26-ultraskelly` branch, and
`uv.lock` pins the commit. The browser bundle in `ultraskelly/ui/dashboard/vendor/` is
built from the same commit, which `freemocap-client.SOURCE` names. Move both together:

```bash
uv lock --upgrade-package freemocap-stream
scripts/update-freemocap-client.sh /path/to/freemocap   # checked out at the locked commit
```

The packages stay at version `0.1.0-dev.0`, so the commit is what identifies the code.
