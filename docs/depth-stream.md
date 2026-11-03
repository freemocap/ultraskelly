# The ZED depth message on `/stream`

Status: draft for the dashboard's depth views (2026-11-02). The format is meant to stay;
values marked nominal will be measured later.

With `--zed --depth` (or `--zed-only --depth`), `/stream` carries three kinds of ZED
message, all self-describing like everything else on the stream:

| Message | Rate | What it holds |
| --- | --- | --- |
| `frame` with cameras `zed_left`, `zed_right` | 30 per second | The raw eyes, JPEG, as the cameras see them |
| `frame` with cameras `zed_rectified`, `zed_depth` | at most 5 per second | A preview: the rectified left eye, and the depth drawn in colour (red near, blue far, black unknown) |
| `depth_frame` | the same depth frames | The depth itself in millimetres, with everything needed to place it in 3D |

The `frame` messages show in today's video panels with no change. `depth_frame` is for
the depth views that need numbers: a heat map with a scale, a raw view, a point cloud and
a mesh. A frame's JPEG cannot carry 16-bit values, so depth travels in its own kind.

## `depth_frame`, version 0

One CBOR map. The envelope is the stream's: `kind` `"depth_frame"`, `version` 0,
`timestamp` (the stereo pair's capture time, seconds on the Pi's monotonic clock, the
clock of `robot_state.time` and of frame timestamps) and `sequence` (counts depth frames).

| Field | Meaning |
| --- | --- |
| `frame_number` | The stereo pair's frame number; the `zed_left`/`zed_right` frame with this number is the pair it came from |
| `method` | How disparity was found: `"sgbm"` (OpenCV on the Pi's CPU) or `"stereonet"` (StereoNet on the Hailo-8) |
| `camera.id` | `"zed_rectified"`: the rectified left eye |
| `camera.width`, `camera.height` | Its image size in pixels (672 × 376 at the default VGA) |
| `camera.fx`, `camera.fy`, `camera.cx`, `camera.cy` | Its pinhole intrinsics in pixels. Rectified, so no distortion; `fx` equals `fy` |
| `camera.baseline_m` | Distance between the eyes, metres (0.063) |
| `camera.axes` | `"x image right, y image down, z forward"`: OpenCV's camera frame, in metres |
| `depth.width`, `depth.height` | The depth grid's size |
| `depth.step` | Depth pixel (u, v) is camera pixel (u × step, v × step); every step-th pixel is kept, exact, never averaged |
| `depth.units`, `depth.encoding`, `depth.invalid` | `"mm"`, `"uint16le"` (row-major, little-endian), `0` = no depth |
| `depth.compression` | `"zlib"` (RFC 1950); `"none"` would mean raw bytes |
| `depth.data` | The bytes |
| `color.encoding`, `color.width`, `color.height`, `color.data` | `"jpeg"`: the rectified left image the depth belongs to, camera-sized |
| `camera_to_head` | The camera's pose in the head frame, or absent when unknown (below) |

Depth is distance along the camera's forward axis (z), not along the ray. Unknown pixels
are 0: nothing matched, nearer than 0.36 m or further than 8 m, or (for StereoNet) outside
the band of rows it sees, rows 93-293 of 376. Most of each eye's view is currently the
inside of the skull, which gets no depth.

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

At VGA with `step` 2 a frame's depth grid is 336 × 188 (126 KB raw); zlib at level 1
keeps a scene like the robot's view to a few tens of KB. The rectified JPEG adds about
20 KB, so depth costs about 0.3 MB/s at 5 frames a second.

## Samples

`scripts/depth_frame_samples.py` writes two samples: a synthetic scene (a wall 3 m away, a
ball 1 m ahead and right of centre, a floor), with known geometry to check a decoder against, and one
recorded from the live ZED. Each file holds one encoded message, exactly as sent.
