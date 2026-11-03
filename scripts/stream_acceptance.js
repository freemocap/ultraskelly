// FreeMoCap's consumer acceptance checks against UltraSkelly's /stream, with the image path
// (payload read, JPEG decode, draw) timed separately from the protocol (CBOR decode and
// validation, frame resolution, channel views). Results collect in window.acceptance, which
// scripts/stream_acceptance.py reads. Query: ?stream=ws://host:port/stream&seconds=10
// &frames=120&people=3 (people: how many the feed shows, 0 for a real camera).
import {
  TransportService, MessageDecoder, FrameResolver, resolveChannelViews, readImagePayload, decodeImageBitmaps,
} from '/vendor/freemocap-client.js';

const query = new URLSearchParams(location.search);
const STREAM = query.get('stream') ?? `ws://${location.hostname}:8080/stream`;
const SECONDS = Number(query.get('seconds') ?? 10);
const SAMPLES = Number(query.get('frames') ?? 120);
const PEOPLE = Number(query.get('people') ?? 0);
const TARGET_CHANNEL = 'ultraskelly.target';
const RETRY_WAIT_MS = 6500;  // The SDK retries a failed connection after 5 s

const result = window.acceptance = {
  done: false, stream: STREAM, page: location.href, userAgent: navigator.userAgent,
  context: {protocol: location.protocol, secure: window.isSecureContext, randomUUID: typeof crypto.randomUUID},
  checks: {}, live: {}, timings: {}, errors: [],
};
const report = () => { document.getElementById('report').textContent = JSON.stringify(result, null, 1); };
const check = (name, pass, details = {}) => { result.checks[name] = {pass, ...details}; report(); };
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const round = value => Math.round(value * 1000) / 1000;
function stats(values) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  const at = share => round(sorted[Math.min(sorted.length - 1, Math.floor(sorted.length * share))]);
  return {median_ms: at(0.5), p95_ms: at(0.95), mean_ms: round(values.reduce((a, b) => a + b, 0) / values.length),
          n: values.length};
}
const median = values => values.length ? [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)] : null;
async function until(condition, ms) {
  for (const end = performance.now() + ms; performance.now() < end; await sleep(20)) if (condition()) return true;
  return condition();
}

// People: keypoints as dots, the followed person (tracker `target`) in orange.
const canvas = document.getElementById('view'), ctx = canvas.getContext('2d');
function drawPeople(overlays) {
  for (const overlay of overlays) {
    const width = overlay.columns.length, [x, y] = [overlay.columns.indexOf('x'), overlay.columns.indexOf('y')];
    ctx.fillStyle = overlay.trackerId === 'target' ? '#ffb269' : '#3ddc84';
    for (let row = 0; row < overlay.names.length; row += 1) {
      ctx.beginPath(); ctx.arc(overlay.data[row * width + x], overlay.data[row * width + y], 4, 0, 2 * Math.PI); ctx.fill();
    }
  }
}

// The stream under test. Its socket factory counts connections and keeps copies of the first
// packets, which are timed offline at the end.
const live = result.live;
Object.assign(live, {robotStates: 0, imagesReceived: 0, imagesDrawn: 0, framesWithPeople: 0, mostPeople: 0,
                     protocolErrors: 0, channelFrames: 0, targetChannels: 0, channelResets: 0});
const packets = [];
const RobotState = {parse(message) {
  if (typeof message?.motors !== 'object' || !Array.isArray(message?.nodes)) throw new Error('robot_state without motors or nodes');
  return message;
}};
function capturingSocket(url) {
  const socket = new WebSocket(url);
  socket.addEventListener('message', event => {
    if (event.data instanceof ArrayBuffer && packets.length < SAMPLES * 4) packets.push(event.data.slice(0));
  });
  return socket;
}
const stream = new TransportService({url: STREAM, socketFactory: capturingSocket, kinds: {robot_state: RobotState}});
let latestState = null, lastImage = null, lastDrawnAt = 0;
stream.subscribeToKind('robot_state', state => { live.robotStates += 1; latestState = state; });
stream.subscribeToProtocolErrors(error => {
  live.protocolErrors += 1;
  if (result.errors.length < 10) result.errors.push(`protocol: ${error?.message ?? error}`);
});

// Channels come first for each frame, then its overlays, then its image.
const targetByFrame = new Map();
stream.subscribeToChannels(frame => {
  if (frame === null) { live.channelResets += 1; return; }
  live.channelFrames += 1;
  for (const view of frame.channels) {
    if (view.descriptor.kind !== TARGET_CHANNEL) continue;
    live.targetChannels += 1;
    const values = Array.from(view.data);  // A copy: views share the decoder's storage
    targetByFrame.set(frame.frameNumber, {owner: view.owner, values});
    result.targetChannel ??= {
      owner: view.owner, kind: view.descriptor.kind, columns: view.descriptor.columns,
      names: view.descriptor.names ?? null, camera_id: view.descriptor.camera_id ?? null,
      image_size: view.descriptor.image_size ?? null, rowCount: view.rowCount, values,
      frameNumber: frame.frameNumber, timestamp: frame.timestamp,
    };
  }
});
let overlaysOf = {frameNumber: -1, list: []};
stream.subscribeToOverlay(overlay => {
  if (overlay.frameNumber !== overlaysOf.frameNumber) overlaysOf = {frameNumber: overlay.frameNumber, list: []};
  overlaysOf.list.push(overlay);
});

// Identity: in every frame, the overlay declared `target` is the person the target channel
// points at, wherever that person's row is; with the synthetic camera it is also the middle one.
const identity = {frames: 0, consistent: 0, positions: new Set(), middle: 0, problems: []};
function checkIdentity(frameNumber, overlays) {
  const target = targetByFrame.get(frameNumber);
  for (const number of targetByFrame.keys()) if (number <= frameNumber) targetByFrame.delete(number);
  const position = overlays.findIndex(overlay => overlay.trackerId === 'target');
  if (position < 0 && !target) return;
  identity.frames += 1;
  const overlay = overlays[position];
  const nose = overlay && overlay.names.indexOf('nose') * overlay.columns.length;
  const x = overlay?.data[nose + overlay.columns.indexOf('x')], y = overlay?.data[nose + overlay.columns.indexOf('y')];
  const matches = overlay && target && target.owner.trackerId === 'target'
    && Math.trunc(x) === target.values[0] && Math.trunc(y) === target.values[1];
  if (matches) identity.consistent += 1;
  else if (identity.problems.length < 5) identity.problems.push({frameNumber, position, target});
  if (overlay && Math.abs(x - 320) <= 45) identity.middle += 1;
  identity.positions.add(position);
}

// Decode and draw newest-wins, closing every bitmap.
let decoding = false, waiting = null;
async function show(arrival) {
  if (decoding) { waiting = arrival; return; }
  decoding = true;
  try {
    for (let next = arrival; next; next = waiting, waiting = null) {
      const images = await decodeImageBitmaps(readImagePayload(next.buffer));
      try {
        for (const image of images) {
          if (canvas.width !== image.width || canvas.height !== image.height) {
            canvas.width = image.width; canvas.height = image.height;
          }
          ctx.drawImage(image.bitmap, 0, 0);
          drawPeople(next.overlays.filter(overlay => overlay.cameraId === image.cameraId));
          lastImage = {cameraId: image.cameraId, width: image.width, height: image.height, jpegBytes: image.jpeg.byteLength};
        }
        live.imagesDrawn += 1; lastDrawnAt = performance.now();
      } finally {
        for (const image of images) image.bitmap.close();
      }
    }
  } catch (problem) {
    if (result.errors.length < 10) result.errors.push(`image: ${problem?.message ?? problem}`);
  } finally {
    decoding = false;
  }
}
stream.subscribeToImages((buffer, frameNumber) => {
  live.imagesReceived += 1;
  const overlays = overlaysOf.frameNumber === frameNumber ? overlaysOf.list : [];
  live.mostPeople = Math.max(live.mostPeople, overlays.length);
  if (overlays.length) live.framesWithPeople += 1;
  checkIdentity(frameNumber, overlays);
  show({buffer, overlays});
});

async function run() {
  // Plain HTTP: this page is served without TLS, usually from a LAN address (not a secure context).
  stream.connect();
  const connected = await until(() => latestState && live.imagesDrawn > 0, 15000);
  check('plain_http_connection', Boolean(connected) && location.protocol === 'http:', {
    connected: Boolean(connected), protocol: location.protocol, secureContext: window.isSecureContext,
    randomUUID: typeof crypto.randomUUID, state: stream.getState(),
  });

  // A window of live streaming.
  const before = {...live};
  const started = performance.now();
  await sleep(SECONDS * 1000);
  const elapsed = (performance.now() - started) / 1000;
  const rate = name => round((live[name] - before[name]) / elapsed);
  Object.assign(result.rates = {}, {
    seconds: round(elapsed), robotStatesPerSecond: rate('robotStates'), imagesReceivedPerSecond: rate('imagesReceived'),
    imagesDrawnPerSecond: rate('imagesDrawn'), framesWithPeoplePerSecond: rate('framesWithPeople'),
  });
  check('custom_robot_state', live.robotStates > 0 && result.rates.robotStatesPerSecond >= 5, {
    perSecond: result.rates.robotStatesPerSecond, simulatedMotors: latestState?.motors?.simulated ?? null,
    cameras: latestState ? Object.keys(latestState.cameras ?? {}) : [], nodes: latestState?.nodes?.length ?? 0,
  });
  check('camera_decoding', live.imagesDrawn > 0 && result.errors.every(error => !error.startsWith('image')), {
    drawnPerSecond: result.rates.imagesDrawnPerSecond, receivedPerSecond: result.rates.imagesReceivedPerSecond,
    lastImage,
  });
  const needPeople = Math.max(PEOPLE, 2);
  check('several_tracked_subjects', live.mostPeople >= needPeople, {
    mostPeopleInAFrame: live.mostPeople, required: needPeople, framesWithPeople: live.framesWithPeople,
    applicable: PEOPLE > 0 || live.mostPeople >= 2,
  });
  check('identities_survive_reordering',
        identity.frames > 0 && identity.consistent === identity.frames && (PEOPLE < 2 || identity.positions.size > 1)
          && (PEOPLE < 3 || identity.middle === identity.frames), {
    framesWithTarget: identity.frames, consistent: identity.consistent, targetRowsSeen: [...identity.positions],
    middlePerson: identity.middle, problems: identity.problems, applicable: identity.frames > 0,
  });
  const channel = result.targetChannel;
  check('custom_numeric_channel', Boolean(channel) && channel.columns.join() === 'x,y,body_angle_deg'
        && channel.rowCount === 1 && channel.owner.trackerId === 'target' && channel.values.slice(0, 2).every(Number.isFinite), {
    descriptor: channel ?? null, framesWithChannel: live.targetChannels, applicable: Boolean(channel),
  });

  // Reconnect mid-stream: the same service after disconnect() (its decoder is reset), and a
  // brand-new service that never saw an earlier message. Each must draw its first frame alone.
  const resetsBefore = live.channelResets, drawnBefore = live.imagesDrawn;
  stream.disconnect();
  const resetSeen = live.channelResets > resetsBefore;
  const reconnectStarted = performance.now();
  stream.connect();
  const redrawn = await until(() => live.imagesDrawn > drawnBefore, 5000);
  const fresh = new TransportService({url: STREAM, kinds: {robot_state: RobotState}});
  let freshOverlays = [], freshImage = null;
  const freshStarted = performance.now();
  fresh.subscribeToOverlay(overlay => { freshOverlays.push(overlay); });
  fresh.subscribeToImages(async (buffer, frameNumber) => {
    if (freshImage) return;
    freshImage = {frameNumber, pending: true};
    try {
      const images = await decodeImageBitmaps(readImagePayload(buffer));
      freshImage = {frameNumber, cameras: images.map(image => `${image.cameraId} ${image.width}x${image.height}`),
                    people: freshOverlays.filter(overlay => overlay.frameNumber === frameNumber).length,
                    ms: round(performance.now() - freshStarted)};
      for (const image of images) image.bitmap.close();
    } catch (problem) {
      freshImage = {frameNumber, error: String(problem?.message ?? problem)};
    }
  });
  fresh.connect();
  await until(() => freshImage && !freshImage.pending, 5000);
  fresh.disconnect();
  check('reconnect_mid_stream', Boolean(redrawn) && resetSeen && Boolean(freshImage?.cameras), {
    sameServiceRedrawnMs: redrawn ? round(lastDrawnAt - reconnectStarted) : null, channelResetOnDisconnect: resetSeen,
    newServiceFirstFrame: freshImage,
  });

  // Retries stop when the caller disconnects during a connection failure (port 1 refuses).
  const refused = `ws://${new URL(STREAM).hostname}:1/stream`;
  const retries = ['error', 'state-change'].map(event => {
    let attempts = 0;
    const failing = new TransportService({url: refused, socketFactory: url => { attempts += 1; return new WebSocket(url); }});
    failing.on(event, value => { if (event === 'error' || value === 'RECONNECTING') failing.disconnect(); });
    failing.connect();
    return {event, failing, attempts: () => attempts};
  });
  await sleep(RETRY_WAIT_MS);
  const retryResults = retries.map(({event, failing, attempts}) => ({event, attempts: attempts(), state: failing.getState()}));
  check('retries_stop_after_disconnect', retryResults.every(item => item.attempts === 1 && item.state === 'DISCONNECTED'), {
    url: refused, waitedMs: RETRY_WAIT_MS, results: retryResults,
  });

  stream.disconnect();
  await timePackets();
  result.allPass = Object.values(result.checks).every(item => item.pass || item.applicable === false);
  result.done = true;
  report();
}

// Offline timings over the captured packets, in two groups: the protocol (CBOR decode with
// validation, frame resolution, channel views) and the image path (payload read, JPEG decode,
// draw). Without cross-origin isolation the browser's clock ticks in ~0.1 ms steps, so the
// sub-millisecond steps are timed over all frames at once and divided, five times over.
async function timePackets() {
  const framePackets = [], statePackets = [], frames = [];
  const decoder = new MessageDecoder();
  for (const packet of packets) {
    const bytes = new Uint8Array(packet), message = decoder.decode(bytes);
    if (message?.kind === 'frame' && message.image && frames.length < SAMPLES) { framePackets.push(bytes); frames.push(message); }
    else if (!message) statePackets.push(bytes);
  }
  const perFrame = (items, step) => {
    if (!items.length) return null;
    const runs = [];
    for (let run = 0; run < 5; run += 1) {
      const started = performance.now();
      step(items);
      runs.push((performance.now() - started) / items.length);
    }
    return {median_ms: round(median(runs)), runs_ms: runs.map(round), frames: items.length};
  };
  const protocol = {
    decode_validate: perFrame(framePackets, items => { const fresh = new MessageDecoder(); for (const bytes of items) fresh.decode(bytes); }),
    resolve_frame: perFrame(frames, items => { const resolver = new FrameResolver(); for (const frame of items) resolver.resolve(frame); }),
    channel_views: perFrame(frames, items => { for (const frame of items) resolveChannelViews(frame); }),
    robot_state_decode: perFrame(statePackets, items => { const fresh = new MessageDecoder(); for (const bytes of items) fresh.decode(bytes); }),
  };
  const payloads = frames.map(frame => readImagePayload(frame.image));
  const jpegDecode = [], draw = [];
  const resolver = new FrameResolver();
  for (const [index, payload] of payloads.entries()) {
    const overlays = resolver.resolve(frames[index]).overlays;
    const t0 = performance.now();
    const images = await decodeImageBitmaps(payload);
    const t1 = performance.now();
    for (const decoded of images) ctx.drawImage(decoded.bitmap, 0, 0);
    drawPeople(overlays);
    ctx.getImageData(0, 0, 1, 1);  // Make the draw finish before the clock stops
    const t2 = performance.now();
    for (const decoded of images) decoded.bitmap.close();
    jpegDecode.push(t1 - t0); draw.push(t2 - t1);
  }
  const image = {
    payload_read: perFrame(frames, items => { for (const frame of items) readImagePayload(frame.image); }),
    jpeg_decode: stats(jpegDecode), draw: stats(draw),
  };
  result.timings = {
    frames: frames.length, robotStates: statePackets.length, crossOriginIsolated: window.crossOriginIsolated,
    method: 'captured packets replayed after the live checks; protocol steps and payload_read are batch averages '
      + '(median of 5 runs over all frames), jpeg_decode and draw are per frame; times in ms per frame',
    protocol, image,
    medianFrameBytes: median(framePackets.map(bytes => bytes.byteLength)),
    medianJpegBytes: median(payloads.map(payload => payload.images.reduce((sum, item) => sum + item.jpeg.byteLength, 0))),
  };
}

run().catch(problem => {
  result.errors.push(`run: ${problem?.stack ?? problem}`);
  result.done = true;
  report();
});
