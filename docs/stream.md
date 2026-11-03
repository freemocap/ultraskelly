# The live stream: what `/stream` carries

The bot serves its dashboard on port 8080 (`python -m ultraskelly.replay --serve` uses
127.0.0.1:8081). One read-only WebSocket, `/stream`, carries FreeMoCap's self-describing
CBOR messages, encoded on the Pi with `freemocap-stream` and decoded in the browser with
`@freemocap/client`. This page is the contract between the producer
(`ultraskelly/ui/runtime_server.py`) and any viewer, including the dashboard.

- **No handshake.** Every message decodes on its own. A viewer that connects or reconnects
  mid-stream draws the next frame without earlier messages.
- **Newest wins.** `robot_state` and each camera have their own latest-value slot, so a slow
  viewer receives fewer, newer messages and the bot never waits for it. Nothing is
  encoded while no viewer is connected.
- **Heartbeat.** A text `ping` gets a text `pong`; the client SDK sends it.

## `frame` messages

FreeMoCap's own kind: one message per camera frame. The camera's frame counter is the
envelope `sequence`, the message's `frame_number` and the image payload's frame number
(a count of encoded frames for a camera without a counter). `timestamp` is the capture
time the camera reports, in seconds on the Pi's `time.monotonic()` clock, not when the
message was encoded or sent; a camera that reports none gets the encoding time.

- `cameras`: one uncalibrated entry per image, with its ID (`oak`), index and `image_size`
  `[width, height]`.
- `image`: the SkellyCam image payload with each camera's JPEG.
- `trackers`: on the camera the pose detector reads (the OAK), one observation per person
  found in that very image. Frames travel without trackers while the detector is stalled
  (no pose message for 0.5 s).

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
pixels belong to.

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
| `cameras` | per camera ID: `fps`, `age` of the newest frame, `width`, `height` |
| `overlays` | per camera: the `aim` pixel the head steers the target to, and the `jaw_zone` half-width |
| `rates` | message rates in Hz: `servo_state`, `target`, `pose` |
| `nodes` | each node's `name` and `state` (starting, running, stopped, crashed), with `error` when crashed |
| `motor_test` | the dashboard motor tests' limits and what is running or ran last |

## Reading it in a browser

The dashboard imports the vendored bundle (`/vendor/freemocap-client.js`):

- Register `robot_state` as an application kind when you construct `TransportService`;
  registrations are fixed at construction.
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
