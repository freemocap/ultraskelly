// Depth frames off the main thread (docs/depth-stream.md). For each depth_frame: inflate the zlib
// depth, read the little-endian millimetres, find the heat map's range and decode the rectified colour
// JPEG. The millimetres and the colour image go back to the dashboard as transferred objects, shared with
// nothing; the heat map itself is coloured on the GPU (heatmap.js).

const NEAREST = 360, FURTHEST = 8000;  // Millimetres: the depth's whole range

let range = null;  // The heat map's range, eased between frames so it does not flicker

const SAMPLE = 4;  // The range comes from every 4th pixel: as good a percentile for a quarter of the work

// The 2nd and 98th percentiles of the valid depths, from a 50 mm histogram of a subsample.
function percentiles(mm) {
  const bins = new Uint32Array(Math.ceil(FURTHEST / 50) + 1);
  let valid = 0;
  for (let i = 0; i < mm.length; i += SAMPLE) if (mm[i]) { bins[Math.min(bins.length - 1, mm[i] / 50 | 0)]++; valid++; }
  if (!valid) return null;
  const at = share => { let seen = 0; for (let b = 0; b < bins.length; b++) { seen += bins[b]; if (seen >= share * valid) return b * 50 + 25; } return FURTHEST; };
  return {near: at(0.02), far: Math.max(at(0.98), at(0.02) + 200), valid: valid * SAMPLE};
}

const LITTLE_ENDIAN = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;

// The depth as millimetres. The bytes are this worker's own (inflated or copied), so on a little-endian
// machine, which every current browser runs on, they are viewed in place rather than copied value by value.
function millimetres(bytes, count) {
  if (LITTLE_ENDIAN && bytes.byteOffset % 2 === 0) return new Uint16Array(bytes.buffer, bytes.byteOffset, count);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength), mm = new Uint16Array(count);
  for (let i = 0; i < count; i++) mm[i] = view.getUint16(2 * i, true);
  return mm;
}

async function inflate(bytes) {
  const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate'));
  return new Uint8Array(await new Response(stream).arrayBuffer());
}

self.onmessage = async ({data: job}) => {
  try {
    const {width, height} = job, t0 = performance.now();
    const bytes = job.compression === 'zlib' ? await inflate(job.bytes) : job.bytes;
    const t1 = performance.now();
    if (bytes.byteLength < width * height * 2) throw new Error(`depth has ${bytes.byteLength} bytes for ${width}x${height}`);
    const mm = millimetres(bytes, width * height);
    const found = percentiles(mm);
    if (found) {
      range = range ? {near: range.near + .3 * (found.near - range.near), far: range.far + .3 * (found.far - range.far)} : found;
    }
    const t2 = performance.now();
    const near = range?.near ?? NEAREST, far = Math.max(near + 1, range?.far ?? FURTHEST);
    const color = job.color ? await createImageBitmap(new Blob([job.color], {type: 'image/jpeg'})) : null, t3 = performance.now();
    // Milliseconds per stage, for the legend's tooltip and for measuring the dashboard's cost.
    const timing = {inflate: t1 - t0, percentiles: t2 - t1, colorJpeg: t3 - t2, total: t3 - t0};
    self.postMessage({id: job.id, mm, width, height, near, far, valid: found?.valid ?? 0, timing, color},
                     [mm.buffer, ...(color ? [color] : [])]);
  } catch (problem) {
    self.postMessage({id: job.id, error: String(problem?.message ?? problem)});
  }
};
