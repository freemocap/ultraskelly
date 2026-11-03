# Live dashboard layout

The owner approved this layout on 2026-11-02. The 3D scene is the main panel; the
camera views sit in a column beside it, grouped by sensor; readouts run along the
bottom. It must fit the Pi's 15.6" 1920×1080 monitor (about 141 pixels per inch, a
full-screen Chromium app window) and browsers on other computers, and keep video at
camera rate on the Pi 5.

```text
┌ UltraSkelly / live ───────────────────────────────── live · 20/s · simulated motors ┐
│ ┌────────────────────────────────────────────────┐ ┌──────────────────────────────┐ │
│ │ 3D scene, from motor commands                  │ │ Tracking camera (OAK)        │ │
│ │  skeleton, gaze ray to the face point,         │ │  pose, aim, jaw zone, target │ │
│ │  point cloud and mesh on the head              │ │                              │ │
│ │                     (motor test panel docks    │ ├──────────────────────────────┤ │
│ │                      here when opened)         │ │ target, lock, face distance  │ │
│ │ layers             Orbit Behind-head Side Top  │ ├──────────────┬───────────────┤ │
│ └────────────────────────────────────────────────┘ │ Left eye     │ Right eye     │ │
│ ┌ Motor commands ┬ Motor tests ┬ Nodes ┬ Rates ──┐ │ Heat map     │ Raw depth     │ │
│ └────────────────┴─────────────┴───────┴─────────┘ └──────────────┴───────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## Panels

- **Header:** connection state, `robot_state` messages per second, and whether the
  motors are simulated or real.
- **3D scene:** the schematic skeleton driven by the commanded head pan and tilt, the
  waist drive arrow and the jaw. These are commands, not measured positions, and the
  scene says so. View buttons: orbit, behind the head, side, top. The ZED's point cloud
  and mesh ride on the head, each with a layer toggle (see Depth); the gaze ray to the
  face point comes next.
- **Tracking camera (OAK):** the video with the poses found in each frame, the aim
  crosshair, jaw zone and target ring, drawn in the browser. The row under it shows the
  target state, pixel, body angle, head lock and target age, and the face distance once
  depth exists. Without video, as in `replay --serve`, the tile shows the target on a grid.
- **Eyes and depth (ZED Mini):** the left and right eyes (`zed_left`, `zed_right`), then
  the depth heat map with a metre scale and the raw depth, both aligned to the left eye.
- **Readout strip:** motor commands (pan and tilt in degrees, waist and jaw drive from
  −1 to 1, each with its age and a bar across its range), motor tests, node health, and
  message rates, camera rates, frame ages and the connection.
- **Motor tests:** the strip card shows the running or last test and the net waist travel,
  with Open and Stop; Stop works with the panel closed. Open docks the motor test panel
  (`motor-test.js`) over the right of the 3D scene, so the skeleton stays in view. On a
  phone the card opens in place.

Click a video tile to enlarge it over the 3D scene; click it again or press Esc to return.

## Sizing

At least 1000 px wide:

- Header 44 px (36 px when the window is under 700 px tall); 8 px padding and gaps.
- Sensor column width: `clamp(300px, min(34.17vw, (100vh - header - 142px) / 1.3125), 900px)`.
  The OAK video (4:3) and the two rows of 16:9 tiles then fit the window height. At 1920
  px wide the column is 656 px, so the OAK's 640×480 frame draws without scaling.
- Readout strip height: `clamp(120px, 18vh, 200px)`.
- Motor test panel: `clamp(280px, 27%, 340px)` of the scene's width, up to 380 px tall.
- Text 11 to 14 px.

| Window | 3D scene | OAK video | Eye and depth tiles | Strip |
| --- | --- | --- | --- | --- |
| Pi monitor, 1920×1080 | 1240×818 | 640×480 | 317×178 | 194 |
| Desktop browser, 1920×950 | 1314×711 | 566×425 | 280×158 | 171 |
| Laptop browser, 1366×657 | 977×477 | 349×262 | 171×96 | 120 |

Every window at least 562 px tall fits without scrolling. Narrower than 1000 px, the
page is one scrolling column: 3D scene, OAK, ZED tiles, then the cards.

## Frame rate on the Pi

The OAK video is drawn one to one. Each image source has its own newest-wins decode slot,
so the ZED pair never starves the OAK. One depth image feeds the heat map, the raw view
and the 3D point cloud on the GPU. Covered or hidden tiles skip decoding. The 3D scene
draws only when a command, the depth or the view changed.

## Depth

The ZED's `depth_frame` messages ([the depth stream](depth-stream.md)) are decoded in a
worker (`depth-worker.js`): it inflates the millimetres, colours the two depth tiles and
decodes the rectified colour image, so the page itself only copies bytes out of the stream
and draws. Newest wins: while the worker is busy, only the newest frame waits.

- **Heat map:** each frame from its 2nd to its 98th percentile, red near and blue far, the
  range eased between frames. The legend under the tiles gives that range in metres, the
  share of pixels with depth and the matcher (`sgbm` or `stereonet`).
- **Raw depth:** 0.36 to 8 m in grey, near bright, unknown black.
- Hover either tile to read the depth under the cursor.
- **In 3D:** the scene is in metres, with the schematic skeleton drawn at the robot's estimated
  eye height of 1.88 m (skellycad's UltraSkelly mock-up). The vertex shaders unproject the
  16-bit depth with the message's intrinsics and place it on the head through
  `camera_to_head`, whose head frame starts midway between the eye sockets. The depth turns
  with the commanded pan and tilt; the waist angle is not measured, so where it points in the
  room is an estimate. **Cloud** shows every depth pixel and **Mesh** joins every second one,
  dropping triangles at unknown pixels and at depth jumps over 6%. The ZED's view cone reaches
  1.5 m. Depth that stops arriving disappears after 3 s.
