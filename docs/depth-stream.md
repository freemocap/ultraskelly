# The ZED depth message on `/stream`

Status: draft for the dashboard's depth views (2026-11-02). The format is meant to stay;
values marked nominal will be measured later.

With `--zed --depth` (or `--zed-only --depth`), `/stream` carries these ZED messages, all
self-describing like everything else on the stream:

| Message | Rate | What it holds |
| --- | --- | --- |
| `frame` with cameras `zed_left`, `zed_right` | 30 per second | The raw eyes, JPEG, as the cameras see them |
| `depth_frame` | at most 10 per second | The depth itself in millimetres and the rectified left eye, with everything needed to place it in 3D |
| `frame` with cameras `zed_rectified`, `zed_depth` | the same depth frames, only with `--depth-previews` | A preview: the rectified left eye, and the depth drawn in colour (red near, blue far, black unknown) |

`depth_frame` is for the depth views that need numbers: a heat map with a scale, a raw
view, a point cloud and a mesh. A frame's JPEG cannot carry 16-bit values, so depth travels
in its own kind. The preview frames repeat what `depth_frame` holds for viewers that draw
only frames, so they are off unless asked for; the dashboard draws from `depth_frame`.

## `depth_frame`, version 0

One CBOR map. The envelope is the stream's: `kind` `"depth_frame"`, `version` 0,
`timestamp` (the stereo pair's capture time, seconds on the Pi's monotonic clock, the
clock of `robot_state.time` and of frame timestamps) and `sequence` (counts depth frames).

| Field | Meaning |
| --- | --- |
| `frame_number` | The stereo pair's frame number; the `zed_left`/`zed_right` frame with this number is the pair it came from |
| `method` | How disparity was found: `"sgbm"` (OpenCV on the Pi's CPU) or `"stereonet"` (StereoNet on the Hailo-8) |
| `camera.id` | `"zed_rectified"`: the rectified left eye |
| `camera.width`, `camera.height` | Its image size in pixels: 336 × 188 by default, half the VGA eye, the size depth is computed at |
| `camera.fx`, `camera.fy`, `camera.cx`, `camera.cy` | Its pinhole intrinsics in pixels. Rectified, so no distortion; `fx` equals `fy` |
| `camera.baseline_m` | Distance between the eyes, metres (0.063) |
| `camera.axes` | `"x image right, y image down, z forward"`: OpenCV's camera frame, in metres |
| `depth.width`, `depth.height` | The depth grid's size |
| `depth.step` | Depth pixel (u, v) is camera pixel (u × step, v × step). 1 by default: every pixel. A larger step keeps every step-th pixel, exact, never averaged |
| `depth.units`, `depth.encoding`, `depth.invalid` | `"mm"`, `"uint16le"` (row-major, little-endian), `0` = no depth |
| `depth.compression` | `"zlib"` (RFC 1950); `"none"` would mean raw bytes |
| `depth.data` | The bytes |
| `color.encoding`, `color.width`, `color.height`, `color.data` | `"jpeg"`: the rectified left image the depth belongs to, camera-sized |
| `camera_to_head` | The camera's pose in the head frame, or absent when unknown (below) |

Depth is distance along the camera's forward axis (z), not along the ray. Unknown pixels
are 0: nothing matched, nearer than 0.36 m or further than 8 m, outside the view mask
where the skull blocks one eye, or (for StereoNet) outside the band of rows it sees, rows
93-293 of 376. Most of each eye's view is currently the inside of the skull, which gets
no depth.

### Points, colours and the head

For each depth pixel (u, v) with value d ≠ 0:

```
z = d / 1000                      metres
x = (u * step - cx) * z / fx
y = (v * step - cy) * z / fy
```

Its colour is the JPEG's pixel (u × step, v × step). A mesh joins neighbouring grid
points whose depths are close; a large jump is an occlusion edge, not a surface.

`camera_to_head` places the points on the skeleton. It has `rotation` (3 × 3, rows),
`translation_m` (3 numbers), `head_frame` and `status`. A point maps as
`p_head = rotation · p_camera + translation_m`, in the head frame H: origin midway
between the eye-socket centres, x forward out of the face, y to the skull's left, z up.
H turns with the head's pan and tilt; the camera bracket design defines it. `status` is
`"nominal"` today: the ZED assumed level, its left lens at the left eye socket. It becomes
`"designed"` or `"measured"` when the bracket gives the real pose.

### Decoding in the browser

```js
const DepthFrame = {parse(message) {
  if (message.depth?.encoding !== 'uint16le' || typeof message.camera?.fx !== 'number') throw new Error('bad depth_frame');
  return message;
}};
const stream = new TransportService({url, kinds: {robot_state: RobotState, depth_frame: DepthFrame}});
stream.subscribeToKind('depth_frame', async frame => {
  const {depth, camera} = frame;
  let bytes = depth.data;                                   // Uint8Array
  if (depth.compression === 'zlib') {
    const inflated = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate'));
    bytes = new Uint8Array(await new Response(inflated).arrayBuffer());
  }
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const millimetres = new Uint16Array(depth.width * depth.height);
  for (let i = 0; i < millimetres.length; i++) millimetres[i] = view.getUint16(2 * i, true);
  const colour = await createImageBitmap(new Blob([frame.color.data], {type: 'image/jpeg'}));
  // ... build points as above, then colour.close()
});
```

`DecompressionStream('deflate')` reads zlib data; on little-endian machines (all current
browsers) `new Uint16Array(bytes.buffer, bytes.byteOffset, n)` also works when the
offset is even.

## Sizes

A depth grid of 336 × 188 is 126 KB raw; zlib at level 1 keeps a scene like the
robot's view to about 15 KB, in 1.3 ms on the Pi. With the rectified JPEG a message is
about 22 KiB, so depth costs about 0.2 MB/s at 10 frames a second. zlib level 6 saved 8%
for three times the time.

## Samples

`scripts/depth_frame_samples.py` writes two samples: a synthetic scene (a wall 3 m away, a
ball 1 m ahead and right of centre, a floor), with known geometry to check a decoder against, and one
recorded from the live ZED. Each file holds one encoded message, exactly as sent.
