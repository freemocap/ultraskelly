// What works and what does not, and what to do about it (docs/dashboard.md, "Health"). The bot checks each part and
// sends a `health` message (docs/stream.md) on every change and every 2 s: each part's status, message, hint and
// since when, and its live diagnostics. The dashboard shows them as the bot writes them and adds only what it alone
// knows: whether the bot is reachable and whether its checks are fresh. A bot that sends no checks still gets a
// panel, from what robot_state says anyway: crashed or stopped nodes and cameras without frames.
import { placeKeyed, setProperty, setText } from '/dom.js';

// Short names for the nodes card's lights, by the bot's part ids.
const SHORT = {oak_camera: 'OAK', oak_models: 'Models', hailo: 'Hailo', zed_camera: 'ZED', zed_imu: 'IMU', depth: 'Depth',
               servo_board: 'Servo', motor_board: 'Motor', power: 'Power'};
// The bot's statuses and the light each gets: degraded works in part, missing and error not at all, off is not used.
const LIGHTS = {ok: 'ok', degraded: 'warn', missing: 'fail', error: 'fail', off: 'off'};
const PROBLEM = new Set(['warn', 'fail']);
export const STALE_S = 5;  // Checks this old, while robot_state still comes, are stale

const ago = seconds => seconds == null || !Number.isFinite(seconds) ? '' : seconds < 60 ? `${Math.max(0, Math.round(seconds))} s`
  : seconds < 3600 ? `${Math.round(seconds / 60)} min` : `${(seconds / 3600).toFixed(1)} h`;
const number = value => Number.isInteger(value) ? String(value) : value.toFixed(2);

// The bot's checks as the panel shows them: {checked, components, problems, diagnostics, time}. A part that fails
// because another does (`because`) shows in its own row, and only the part it follows from counts as a problem.
export function healthOf(state, report) {
  const components = report?.components ? report.components.map(part => ({
    id: part.id, name: part.label || part.id, short: SHORT[part.id] || part.label || part.id,
    status: LIGHTS[part.status] ?? 'warn', word: part.status, message: part.message || '', fix: part.hint || '',
    since: part.since ?? null, restart: part.restart || '', detail: part.detail ?? null, because: part.because || null,
  })) : fromNodesAndCameras(state);
  const byId = new Map(components.map(part => [part.id, part]));
  const failing = id => byId.has(id) && PROBLEM.has(byId.get(id).status);
  const rootOf = part => { let at = part; for (let hops = 0; at.because && failing(at.because) && hops < 8; hops++) at = byId.get(at.because); return at; };
  const problems = new Map();
  for (const part of components.filter(part => PROBLEM.has(part.status))) {
    const root = rootOf(part);
    part.root = root === part ? null : root;
    if (!problems.has(root.id)) problems.set(root.id, {id: root.id, status: root.status, text: root.message || `${root.name} ${root.word}`, parts: []});
    const problem = problems.get(root.id);
    problem.parts.push(part);
    if (part.status === 'fail') problem.status = 'fail';
  }
  return {checked: Boolean(report?.components), components, problems: [...problems.values()],
          diagnostics: report?.diagnostics ?? null, time: state?.time ?? report?.time ?? null};
}

// Why a part is not working, {message, fix}, or null while it is well, off, or not checked. A part that fails
// because of another takes that part's fix when it has none of its own.
export function reasonOf(model, id) {
  const part = model?.components.find(component => component.id === id);
  if (!part || !PROBLEM.has(part.status)) return null;
  return {message: part.message || `${part.name} ${part.word}`, fix: part.fix || part.root?.fix || ''};
}

// Without the bot's checks: what robot_state shows anyway, with no fixes, since only the bot could write them.
function fromNodesAndCameras(state) {
  const parts = [];
  for (const node of state?.nodes ?? []) {
    if (node.state !== 'crashed' && node.state !== 'stopped') continue;
    parts.push({id: `node:${node.name}`, name: node.name, short: node.name, status: node.state === 'crashed' ? 'fail' : 'warn',
                word: node.state, fix: '', since: null, restart: '', detail: null, because: null,
                message: node.state === 'crashed' ? `${node.name} crashed${node.error ? `: ${node.error}` : ''}` : `${node.name} stopped`});
  }
  for (const [camera, feed] of Object.entries(state?.cameras ?? {})) {
    if (feed.width && feed.age != null && feed.age <= 3) continue;
    parts.push({id: `camera:${camera}`, name: camera, short: camera, status: 'warn', word: 'no frames', fix: '', since: null,
                restart: '', detail: null, because: null,
                message: feed.width ? `No frames from ${camera} for ${ago(feed.age)}` : `No frames from ${camera} yet`});
  }
  return parts;
}

// One reading, compact: true is yes, numbers short, lists joined, small objects as "key value".
function describe(value) {
  if (value == null) return '—';
  if (typeof value === 'boolean') return value ? 'yes' : 'no';
  if (typeof value === 'number') return number(value);
  if (Array.isArray(value)) return value.length ? value.map(describe).join(', ') : 'none';
  if (typeof value === 'object') return Object.entries(value).map(([key, item]) => `${key.replaceAll('_', ' ')} ${describe(item)}`).join(' · ');
  return String(value);
}

// The bot's live diagnostics (docs/stream.md, "health"), a line each, in plain words where the reading is known.
const USB_NAMES = {oak: 'OAK', zed_video: 'ZED video', zed_hid: 'ZED HID (its IMU)'};
function diagnosticLines(diagnostics) {
  const lines = [];
  for (const [name, reading] of Object.entries(diagnostics ?? {})) {
    if (reading == null || (typeof reading === 'object' && !Object.keys(reading).length)) continue;
    if (name === 'i2c') {
      if (reading.available === false) { lines.push(['I2C', 'no I2C bus']); continue; }
      // 0x70 is the boards' all-call address: it goes quiet once the bot sets up the boards.
      const addresses = Object.entries(reading.addresses ?? {}).map(([address, answers]) =>
        `${address}${address === '0x70' ? ' (all-call)' : ''} ${answers ? 'answers' : 'silent'}`);
      lines.push(['I2C', [...addresses, reading.lost_arbitration ? `lost arbitration ${reading.lost_arbitration} times` : null].filter(Boolean).join(' · ') || describe(reading)]);
    } else if (name === 'pca9685') {
      const channels = Object.entries(reading.channels ?? {}).map(([channel, at]) => `${channel} ${at.pulse_us == null ? '—' : `${Math.round(at.pulse_us)} µs`}` +
        (at.commanded == null ? '' : ` for ${Math.round(at.commanded)}°`) +
        (at.expected_us == null ? '' : ` (expects ${Math.round(at.expected_us)} µs${at.matches === false ? ', does not match' : ''})`));
      lines.push(['Servo board', [reading.awake == null ? null : reading.awake ? 'awake' : 'asleep',
                                  reading.frequency_hz == null ? null : `${reading.frequency_hz.toFixed(1)} Hz`, ...channels].filter(Boolean).join(' · ')]);
    } else if (name === 'power') {
      if (reading.available === false) { lines.push(['Power', 'no readings on this machine']); continue; }
      const rails = Object.entries(reading.rails ?? {}).map(([rail, value]) => `${rail} ${value.toFixed(2)} V`);
      const flags = Array.isArray(reading.flags) ? reading.flags : [], throttled = reading.throttled && !/^0x0+$/i.test(String(reading.throttled));
      lines.push(['Power', [...rails, flags.length ? flags.join(', ') : throttled ? `throttled ${reading.throttled}` : 'not throttled'].join(' · ')]);
    } else if (name === 'hailo') {
      const device = typeof reading.device === 'string' ? `device ${reading.device}` : reading.device ? '/dev/hailo0 present' : 'no /dev/hailo0';
      lines.push(['Hailo', [reading.pci == null ? null : reading.pci ? 'on PCIe' : 'not on PCIe', device].filter(Boolean).join(' · ')]);
    } else if (name === 'usb') {
      // zed_imu_device is not a USB device: it is the /dev/zed-mini-imu link the udev rule makes for the bot.
      const {zed_imu_device: link, ...devices} = reading, there = Object.entries(devices), named = key => USB_NAMES[key] ?? key.replaceAll('_', ' ');
      const on = there.filter(([, present]) => present).map(([key]) => named(key)), off = there.filter(([, present]) => !present).map(([key]) => named(key));
      lines.push(['USB', [`on USB: ${on.join(', ') || 'none'}`, off.length ? `not on USB: ${off.join(', ')}` : null,
                          link == null ? null : link ? '/dev/zed-mini-imu present' : 'no /dev/zed-mini-imu'].filter(Boolean).join(' · ')]);
    } else if (name === 'motor_watchdog') {
      lines.push(['Motor watchdog', reading.trips ? `stopped the motors ${reading.trips} time${reading.trips === 1 ? '' : 's'}` : 'never tripped']);
    } else if (name === 'retried_writes') {
      const counted = Object.entries(reading).filter(([, count]) => count != null);  // None for a motor that is simulated
      if (counted.length) lines.push(['Retried writes', counted.map(([motor, count]) => `${motor} ${count}`).join(' · ')]);
    } else {
      lines.push([name.replaceAll('_', ' '), describe(reading)]);
    }
  }
  return lines;
}

// The banner, the panel and the nodes card's lights. The panel opens by itself when a problem appears and folds
// away when all is well; a click opens or closes it, and that choice holds until a problem the page has not seen yet.
export class HealthView {
  // elements: {banner, count, summary, toggle, panel, close, rows, lights, diagnostics, source}
  constructor(elements) {
    this.$ = elements;
    this.choice = null;          // 'open' or 'closed', the user's, until a new problem
    this.seen = new Set();       // Problem ids in the last update
    this.own = new Map();        // The dashboard's own problems: 'link' (the bot unreachable), 'checks' (stale)
    this.rows = new Map(); this.lights = new Map(); this.key = ''; this.readings = ''; this.model = null;
    elements.toggle.onclick = () => this.setOpen(this.$.panel.hidden);
    elements.lights.onclick = () => this.setOpen(this.$.panel.hidden);
    elements.close.onclick = () => this.setOpen(false);
  }

  setOpen(open) {
    this.choice = open ? 'open' : 'closed';
    this.apply();
  }

  problems() { return [...this.own.values(), ...(this.model?.problems ?? [])]; }

  apply() {
    const problems = this.problems().length;
    const open = this.choice ? this.choice === 'open' : problems > 0;
    setProperty(this.$.panel, 'hidden', !open);
    setText(this.$.toggle, open ? 'Hide' : problems ? 'Details' : 'Checks');
    if (this.$.toggle.getAttribute('aria-expanded') !== String(open)) this.$.toggle.setAttribute('aria-expanded', String(open));
    if (open) this.renderDiagnostics();  // The live readings are written only while someone can see them
  }

  // One of the dashboard's own problems ({text, status}), or null once it is gone. A new one opens the panel.
  setOwn(key, problem) {
    if ((problem?.text ?? null) === (this.own.get(key)?.text ?? null)) return;
    if (problem && !this.own.has(key)) this.choice = null;
    if (problem) this.own.set(key, {id: `own:${key}`, ...problem}); else this.own.delete(key);
    this.renderBanner(); this.apply();
  }

  // The checks with each robot_state (healthOf). Writes the page only where something changed.
  update(model) {
    this.model = model;
    const ids = new Set(model.problems.map(problem => problem.id));
    if ([...ids].some(id => !this.seen.has(id))) this.choice = null;
    this.seen = ids;
    const key = JSON.stringify([model.checked, model.components.map(p => [p.id, p.status, p.word, p.message, p.fix, p.since, p.restart, p.because]),
                                model.problems.map(p => p.id)]);
    if (key !== this.key) {
      this.key = key;
      this.renderBanner(); this.renderRows(); this.renderLights();
    }
    this.renderSince();
    this.apply();
  }

  renderBanner() {
    const model = this.model, problems = this.problems(), banner = this.$.banner;
    banner.classList.toggle('bad', problems.some(problem => problem.status === 'fail'));
    banner.classList.toggle('warn', problems.length > 0 && !problems.some(problem => problem.status === 'fail'));
    banner.classList.toggle('ok', problems.length === 0 && Boolean(model?.checked));
    if (problems.length) {
      setText(this.$.count, `${problems.length} problem${problems.length === 1 ? '' : 's'}:`);
      setText(this.$.summary, problems.map(problem => problem.text).join(' · '));
    } else if (model?.checked) {
      setText(this.$.count, 'All hardware OK');
      setText(this.$.summary, `${model.components.filter(part => part.status !== 'off').length} checks`);
    } else {
      setText(this.$.count, model ? 'No hardware checks from this bot' : 'Waiting for the bot');
      setText(this.$.summary, model ? 'nodes and cameras look fine' : '');
    }
    setProperty(banner, 'title', problems.map(problem => problem.parts?.map(part => `${part.name}: ${part.message}${part.fix ? ` (${part.fix})` : ''}`).join('\n') ?? problem.text).join('\n'));
  }

  renderRows() {
    const parts = this.model?.components ?? [];
    setText(this.$.source, this.model?.checked ? 'Checked by the bot' : 'From node and camera states: this bot sends no hardware checks');
    placeKeyed(this.$.rows, this.rows, parts.map(part => part.id), () => {
      const row = document.createElement('li'), made = ['i', 'b', 'time', 'span', 'span', 'span', 'span'].map(tag => document.createElement(tag));
      ['light', 'name', 'since', 'message', 'because', 'fix', 'restart'].forEach((name, i) => { made[i].className = name; });
      row.append(...made);
      return [row];
    });
    for (const part of parts) {
      const [row] = this.rows.get(part.id), [light, name, , message, because, fix, restart] = row.children;
      setProperty(row, 'className', `part ${part.status}`);
      setProperty(row, 'title', part.detail && Object.keys(part.detail).length ? describe(part.detail) : '');
      setProperty(light, 'title', part.word);
      setText(name, part.name);
      setText(message, part.message || (part.status === 'ok' ? 'OK' : part.status === 'off' ? 'Not used in this run' : part.word));
      setText(because, part.root ? `Because the ${part.root.name} is ${part.root.word}` : ''); setProperty(because, 'hidden', !part.root);
      setText(fix, part.fix); setProperty(fix, 'hidden', !part.fix);
      setText(restart, part.restart ? `Restart the bot to use it: ${part.restart}` : ''); setProperty(restart, 'hidden', !part.restart);
    }
  }

  // Each part's status word and how long it has been so, from the Pi's clock: once a second and with each update.
  renderSince() {
    const now = this.model?.time;
    for (const part of this.model?.components ?? []) {
      const since = this.rows.get(part.id)?.[0].children[2];
      if (!since) continue;
      const how = part.status === 'ok' ? '' : part.word, long = part.since == null || now == null ? '' : `since ${ago(now - part.since)}`;
      setText(since, [how, long].filter(Boolean).join(' · '));
    }
  }

  renderLights() {
    const parts = this.model?.checked ? this.model.components : [];
    setProperty(this.$.lights, 'hidden', !parts.length);
    placeKeyed(this.$.lights, this.lights, parts.map(part => part.id), () => {
      const light = document.createElement('span');
      light.append(document.createElement('i'), document.createTextNode(''));
      return [light];
    });
    for (const part of parts) {
      const [light] = this.lights.get(part.id);
      setProperty(light, 'className', `lamp ${part.status}`);
      setProperty(light, 'title', `${part.name}: ${part.message || part.word}`);
      setText(light.childNodes[1], part.short);
    }
  }

  // The bot's live diagnostics, compact, written only when what they say changed.
  renderDiagnostics() {
    const diagnostics = this.model?.diagnostics ?? null;
    const readings = JSON.stringify(diagnostics, (name, value) => name === 'time' ? undefined : value);
    if (readings === this.readings) return;
    this.readings = readings;
    const lines = diagnosticLines(diagnostics), list = this.$.diagnostics;
    setProperty(list.parentElement, 'hidden', !lines.length);
    if (list.childElementCount !== lines.length * 2) list.replaceChildren(...lines.flatMap(() => [document.createElement('dt'), document.createElement('dd')]));
    lines.forEach(([term, value], i) => { setText(list.children[2 * i], term); setText(list.children[2 * i + 1], value); });
  }
}
