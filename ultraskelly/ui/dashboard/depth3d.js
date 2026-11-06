// The ZED's depth in the 3D scene (docs/depth-stream.md): a point cloud and a mesh unprojected on the
// GPU from the 16-bit depth texture, coloured by the rectified left image, and placed on the skeleton's
// head through each message's camera_to_head. They ride on the head, so they turn with the commanded
// pan, tilt and roll (headpose.js); the head's angles and the waist's are not measured, so where the head
// points in the room is the commands' picture of it, not a measurement.
import * as THREE from 'three';
import { H_AXES, OPTICAL_TO_HEAD } from '/headpose.js';

const MESH_STEP = 2;       // The mesh joins every second depth pixel; the cloud shows them all
const JUMP = 0.06;         // A relative depth change above 6% between neighbours is an occlusion edge
const FRUSTUM_DEPTH = 1.5; // Metres: how far the drawn view cone reaches
const STALE_MS = 3000;     // Without new depth for this long, the cloud and mesh are hidden

const COMMON = /* glsl */`
precision highp usampler2D;
uniform usampler2D depthMap;
uniform vec4 intrinsics;      // fx, fy, cx, cy of the camera image, pixels
uniform float pixelStep;      // depth pixel (u, v) is camera pixel (u, v) * pixelStep
uniform vec2 colorSize;       // the camera image's width and height
uniform mat3 cameraRotation;  // camera_to_head: p_head = rotation * p_camera + translation
uniform vec3 cameraTranslation;
float metres(ivec2 p) { return float(texelFetch(depthMap, p, 0).r) * 0.001; }
vec3 headPoint(ivec2 p, float z) {
  vec2 pixel = vec2(p) * pixelStep;
  return cameraRotation * vec3((pixel - intrinsics.zw) * z / intrinsics.xy, z) + cameraTranslation;
}
vec2 colorUv(ivec2 p) { return (vec2(p) * pixelStep + 0.5) / colorSize; }
`;

const POINTS_VERTEX = COMMON + /* glsl */`
uniform float pointSize;  // pixels at one metre
varying vec2 vUv;
void main() {
  ivec2 p = ivec2(position.xy);
  float z = metres(p);
  vUv = colorUv(p);
  if (z <= 0.0) { gl_Position = vec4(2.0, 2.0, 2.0, 1.0); gl_PointSize = 0.0; return; }  // Unknown: clipped
  vec4 view = modelViewMatrix * vec4(headPoint(p, z), 1.0);
  gl_Position = projectionMatrix * view;
  gl_PointSize = clamp(pointSize / -view.z, 1.0, 6.0);
}`;

// Each vertex reports whether it, or its right, lower or lower-right neighbour on the mesh grid, is
// unknown or across a depth jump. Every edge of the grid's triangles (p, p+right, p+right+down) and
// (p, p+right+down, p+down) is checked by one of its own vertices, so a triangle touching a break
// interpolates a non-zero flag and is discarded.
const MESH_VERTEX = COMMON + /* glsl */`
uniform float meshStep;
uniform float jump;
varying vec2 vUv;
varying float vBreak;
float breaks(ivec2 q, float z, ivec2 size) {
  if (q.x >= size.x || q.y >= size.y) return 0.0;
  float n = metres(q);
  return n <= 0.0 || abs(n - z) > jump * max(n, z) ? 1.0 : 0.0;
}
void main() {
  ivec2 size = textureSize(depthMap, 0);
  int s = int(meshStep);
  ivec2 p = min(ivec2(position.xy) * s, size - 1);
  float z = metres(p);
  vBreak = max(z <= 0.0 ? 1.0 : 0.0, max(breaks(p + ivec2(s, 0), z, size), max(breaks(p + ivec2(0, s), z, size), breaks(p + ivec2(s, s), z, size))));
  vUv = colorUv(p);
  gl_Position = projectionMatrix * modelViewMatrix * vec4(headPoint(p, max(z, 0.3)), 1.0);
}`;

// GLSL 3 materials declare their own output.
const COLOR_FRAGMENT = /* glsl */`
uniform sampler2D colorMap;
varying vec2 vUv;
out highp vec4 fragColor;
void main() { fragColor = vec4(texture2D(colorMap, vUv).rgb, 1.0); }`;

const MESH_FRAGMENT = /* glsl */`
uniform sampler2D colorMap;
varying vec2 vUv;
varying float vBreak;
out highp vec4 fragColor;
void main() {
  if (vBreak > 0.001) discard;
  fragColor = vec4(texture2D(colorMap, vUv).rgb, 1.0);
}`;

// A grid of vertices whose positions are their (u, v) grid coordinates; with triangles, two per cell.
function gridGeometry(columns, rows, triangles) {
  const position = new Float32Array(columns * rows * 3);
  for (let v = 0, i = 0; v < rows; v++) for (let u = 0; u < columns; u++, i += 3) { position[i] = u; position[i + 1] = v; }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(position, 3));
  if (triangles) {
    const index = new (columns * rows > 65535 ? Uint32Array : Uint16Array)((columns - 1) * (rows - 1) * 6);
    for (let v = 0, k = 0; v < rows - 1; v++) {
      for (let u = 0; u < columns - 1; u++, k += 6) {
        const a = v * columns + u, b = a + 1, c = a + columns, d = c + 1;
        index.set([a, b, d, a, d, c], k);
      }
    }
    geometry.setIndex(new THREE.BufferAttribute(index, 1));
  }
  return geometry;
}

// head: the group that turns with the commanded head (the skeleton's tilt group), in skeleton units;
// origin: the head frame's origin in that group (midway between the eye sockets); scale: metres per
// skeleton unit; invalidate: asks the scene to draw again.
export function createDepthLayer({head, origin, scale, invalidate}) {
  // The head frame H (+x forward, +y the skull's left, +z up), in metres, on the skeleton's head group
  // (+x the skull's left, +y up, +z out of the face): H_AXES gives H's axes in the group's.
  const group = new THREE.Group();
  group.matrixAutoUpdate = false;
  group.matrix.makeBasis(...H_AXES.map(axis => new THREE.Vector3(...axis)))
    .scale(new THREE.Vector3(1 / scale, 1 / scale, 1 / scale)).setPosition(origin);
  head.add(group);

  const uniforms = {
    depthMap: {value: null}, colorMap: {value: null},
    intrinsics: {value: new THREE.Vector4(1, 1, 0, 0)}, pixelStep: {value: 1}, colorSize: {value: new THREE.Vector2(1, 1)},
    cameraRotation: {value: new THREE.Matrix3()}, cameraTranslation: {value: new THREE.Vector3()},
    pointSize: {value: 4}, meshStep: {value: MESH_STEP}, jump: {value: JUMP},
  };
  const pointsMaterial = new THREE.ShaderMaterial({glslVersion: THREE.GLSL3, uniforms, vertexShader: POINTS_VERTEX, fragmentShader: COLOR_FRAGMENT});
  const meshMaterial = new THREE.ShaderMaterial({glslVersion: THREE.GLSL3, uniforms, vertexShader: MESH_VERTEX, fragmentShader: MESH_FRAGMENT, side: THREE.DoubleSide});
  const frustum = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color: 0x8a7fd8, transparent: true, opacity: .55}));
  frustum.visible = false;
  group.add(frustum);
  let points = null, mesh = null, depthTexture = null, colorTexture = null, size = '', lastAt = 0, placed = '';
  const shown = {cloud: true, mesh: false, frustum: true};

  function rebuild(width, height) {
    for (const object of [points, mesh]) if (object) { group.remove(object); object.geometry.dispose(); }
    depthTexture?.dispose();
    depthTexture = new THREE.DataTexture(new Uint16Array(width * height), width, height, THREE.RedIntegerFormat, THREE.UnsignedShortType);
    depthTexture.internalFormat = 'R16UI';
    depthTexture.minFilter = depthTexture.magFilter = THREE.NearestFilter;
    depthTexture.generateMipmaps = false;
    depthTexture.unpackAlignment = 2;
    uniforms.depthMap.value = depthTexture;
    points = new THREE.Points(gridGeometry(width, height, false), pointsMaterial);
    mesh = new THREE.Mesh(gridGeometry(Math.ceil(width / MESH_STEP), Math.ceil(height / MESH_STEP), true), meshMaterial);
    for (const object of [points, mesh]) { object.frustumCulled = false; group.add(object); }
    size = `${width}x${height}`;
  }

  function placeFrustum(camera, rotation, translation) {
    const corner = (x, y) => new THREE.Vector3((x - camera.cx) / camera.fx, (y - camera.cy) / camera.fy, 1)
      .multiplyScalar(FRUSTUM_DEPTH).applyMatrix3(rotation).add(translation);
    const corners = [corner(0, 0), corner(camera.width, 0), corner(camera.width, camera.height), corner(0, camera.height)];
    const segments = corners.flatMap((c, i) => [translation, c, c, corners[(i + 1) % 4]]);
    frustum.geometry.setFromPoints(segments);
  }

  // The cloud and mesh show while depth is fresh, the frustum once depth has come; a draw only when one of
  // them appears or disappears.
  function applyVisibility() {
    const fresh = Boolean(lastAt) && performance.now() - lastAt < STALE_MS, was = [points?.visible, mesh?.visible, frustum.visible].join();
    if (points) points.visible = fresh && shown.cloud;
    if (mesh) mesh.visible = fresh && shown.mesh;
    frustum.visible = Boolean(lastAt) && shown.frustum;
    if ([points?.visible, mesh?.visible, frustum.visible].join() !== was) invalidate();
  }

  // One decoded depth frame (depth.js): result.mm and result.color are this layer's to keep and close.
  function update(result, meta) {
    if (`${result.width}x${result.height}` !== size) rebuild(result.width, result.height);
    depthTexture.image.data = result.mm;
    depthTexture.needsUpdate = true;
    if (result.color) {
      if (!colorTexture) {
        colorTexture = new THREE.Texture();
        colorTexture.flipY = false;                       // Row 0 of the image is v = 0, like the depth grid
        colorTexture.colorSpace = THREE.NoColorSpace;     // Shown as the JPEG's own values
        colorTexture.minFilter = THREE.LinearFilter;
        colorTexture.generateMipmaps = false;
        uniforms.colorMap.value = colorTexture;
      }
      colorTexture.image?.close?.();
      colorTexture.image = result.color;
      colorTexture.needsUpdate = true;
    }
    const camera = meta.camera, pose = meta.cameraToHead;
    uniforms.intrinsics.value.set(camera.fx, camera.fy, camera.cx, camera.cy);
    uniforms.pixelStep.value = meta.step;
    uniforms.colorSize.value.set(camera.width, camera.height);
    // Without a pose, assume the nominal one: the left lens at the left eye socket, looking forward.
    const rows = pose?.rotation ?? OPTICAL_TO_HEAD;
    const rotation = uniforms.cameraRotation.value.set(...rows[0], ...rows[1], ...rows[2]);
    const translation = uniforms.cameraTranslation.value.fromArray(pose?.translation_m ?? [0, 0.0315, 0]);
    const placement = JSON.stringify([camera.fx, camera.fy, camera.cx, camera.cy, camera.width, camera.height, rows, pose?.translation_m]);
    const moved = placement !== placed;
    if (moved) { placed = placement; placeFrustum(camera, rotation, translation); }
    lastAt = performance.now();
    applyVisibility();
    if (points.visible || mesh.visible || (moved && frustum.visible)) invalidate();  // Hidden depth draws nothing
  }

  return {
    group, uniforms, update,
    setVisible(changes) { Object.assign(shown, changes); applyVisibility(); },
    get shown() { return {...shown}; },
    tick: applyVisibility,  // Once a second, so stale depth disappears
  };
}
