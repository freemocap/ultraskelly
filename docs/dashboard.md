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
│ │                     (motor test panel docks    │ ├──────────────────────────────┤ │
│ │                      here when opened)         │ │ target, face, jaw zone, age  │ │
│ │ layers             Orbit Behind-head Side Top  │ ├──────────────┬───────────────┤ │
│ └────────────────────────────────────────────────┘ │ Left eye     │ Right eye     │ │
│ ┌ Time series ┬ Motor tests ┬ Nodes, rates ──────┐ │ Heat map     │ Face          │ │
│ └─────────────┴─────────────┴────────────────────┘ └──────────────┴───────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Panels

- **Header:** connection state, `robot_state` messages per second, and whether the
  motors are simulated or real.
- **3D scene:** the schematic skeleton driven by the commanded head pan and tilt, the
  waist drive arrow and the jaw. These are commands, not measured positions, and the
  scene says so. View buttons: orbit, behind the head, side, top. The ZED's point cloud
  and mesh and the OAK's gaze ride on the head, each with a layer toggle (see Depth and
  Gaze).
- **Tracking camera (OAK):** the video with the poses found in each frame, the aim
  crosshair, jaw zone and target ring, drawn in the browser. The row under it shows the
  target state, pixel, face distance, whether the target is inside the jaw zone, its body
  angle and its age; a narrow tile drops the last of these. Without video, as in
  `replay --serve`, the tile shows the target on a grid.
- **Eyes and depth (ZED Mini):** the left and right eyes (`zed_left`, `zed_right`), the
  depth heat map with a metre scale, and the tracked face in 3D as the Pi's face locator
  reports it (`robot_state.face`): its distance from between the eyes, its position in the
  head frame, the OAK pixel it came from, the alignment that placed it and its age.
- **Readout strip:** the time series (below), motor tests, and node health with the
  message rates; a camera whose frames stop appears there too.
- **Motor tests:** the strip card shows the running or last test and the net waist travel,
  with Open and Stop; Stop works with the panel closed. Open docks the motor test panel
  (`motor-test.js`) over the right of the 3D scene, so the skeleton stays in view. On a
  phone the card opens in place.

Click a video tile to enlarge it over the 3D scene; click it again or press Esc to return.

## Time series

Strip charts of the last 30 s (`timeseries.js`), on the Pi's clock (`robot_state.time`):
the head pan and tilt and the waist and jaw drive, as commanded; the target's offset from
the aim point, which is the controllers' input; the face distance from the Pi's face
locator; and the ZED's accelerometer and gyroscope, measured, once the stream carries
them. No motor reports its position: the servos have no feedback and the DC motors no
encoders, so there is no measured motor trace, and the panel says so. Each lane's header
gives the newest values. The canvas redraws at most 10 times a second, only when something
new arrived, at one point per pixel column (the first, lowest, highest and last value in it).

## Sizing

At least 1000 px wide:

- Header 44 px (36 px when the window is under 700 px tall); 8 px padding and gaps.
- Sensor column width: `clamp(300px, min(34.17vw, (100vh - header - 142px) / 1.3125), 900px)`.
  The OAK video (4:3) and the two rows of 16:9 tiles then fit the window height. At 1920
  px wide the column is 656 px, so the OAK's 640×480 frame draws without scaling.
- Readout strip height: `clamp(140px, 22vh, 240px)`; the time series takes 2.4 parts of
  its width, the motor tests and the nodes 1 part each.
- Motor test panel: `clamp(280px, 27%, 340px)` of the scene's width, up to 380 px tall.
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
cloud on the GPU. Covered or hidden tiles skip decoding. The 3D scene draws only when a
command, the depth or the view changed, and the time series at most 10 times a second.

## Depth

The ZED's `depth_frame` messages ([the depth stream](depth-stream.md)) are decoded in a
worker (`depth-worker.js`): it inflates the millimetres, colours the heat map and decodes
the rectified colour image, so the page itself only copies bytes out of the stream and
draws. Newest wins: while the worker is busy, only the newest frame waits.

- **Heat map:** each frame from its 2nd to its 98th percentile, red near and blue far, the
  range eased between frames. The legend under the tiles gives that range in metres, the
  share of pixels with depth and the matcher (`sgbm` or `stereonet`). Hover the tile to
  read the depth under the cursor. One view is enough: a second, grey one showed the same
  data.
- **In 3D:** the scene is in metres, with the schematic skeleton drawn at the robot's estimated
  eye height of 1.88 m (skellycad's UltraSkelly mock-up). The vertex shaders unproject the
  16-bit depth with the message's intrinsics and place it on the head through
  `camera_to_head`, whose head frame starts midway between the eye sockets. The depth turns
  with the commanded pan and tilt; the waist angle is not measured, so where it points in the
  room is an estimate. **Cloud** shows every depth pixel and **Mesh** joins every second one,
  dropping triangles at unknown pixels and at depth jumps over 6%. The ZED's view cone reaches
  1.5 m. Depth that stops arriving disappears after 3 s.

## Gaze

The OAK's view cone starts at a best-guess lens position in the skull's nose hole, 4 cm below
the eye sockets, with a 69° field of view across the image. The gaze ray runs from the lens to
the face point the Pi's face locator found (`robot_state.face.position_head`: the OAK's target
pixel looked up in the ZED's depth), drawn in orange; the dashboard does not compute it again.
Without a face point newer than 0.5 s, the ray is a dashed 3 m direction through the frame's
`ultraskelly.target` pixel, and the head following nobody hides it. Camera poses are nominal
until the camera bracket gives them, and the waist angle is not measured.
