// What works and what does not, and what to do about it (docs/dashboard.md, "Health"). The bot checks each part
// (robot_state.health, docs/stream.md) and writes the message and fix for each; the dashboard shows them as given
// and adds only what it alone knows: whether the bot is reachable. A bot that sends no checks still gets a panel,
// from what robot_state says anyway: crashed or stopped nodes and cameras without frames.
import { placeKeyed, setProperty, setText } from '/dom.js';

// The parts the bot checks, in the order shown, with names for a bot that does not send its own.
export const PARTS = [
  ['oak', 'OAK camera', 'OAK'], ['zed_video', 'ZED video', 'ZED'], ['zed_imu', 'ZED IMU', 'IMU'], ['hailo', 'AI HAT+ (Hailo)', 'Hailo'],
  ['servo_board', 'Servo board 0x40', 'Servo'], ['motor_hat', 'Motor HAT 0x60', 'Motor'], ['power', 'Pi power', 'Power'],
];
const NAMES = new Map(PARTS.map(([id, name]) => [id, name]));
const SHORT = new Map(PARTS.map(([id, , short]) => [id, short]));
const PROBLEM = new Set(['warn', 'fail']);
const STATUSES = new Set(['ok', 'warn', 'fail', 'off']);

const ago = seconds => seconds == null || !Number.isFinite(seconds) ? '' : seconds < 60 ? `${Math.max(0, Math.round(seconds))} s`
  : seconds < 3600 ? `${Math.round(seconds / 60)} min` : `${(seconds / 3600).toFixed(1)} h`;
const volts = value => value == null ? '—' : `${value.toFixed(2)} V`;
const percent = duty => duty == null ? '—' : `${(duty * 100).toFixed(2)}%`;

// The checks in one robot_state, as the panel shows them: {checked, components, problems, i2c, servo, power, time}.
// problems groups components that share a cause, so one unplugged hub is one problem, not three; the bot may name
// a cause in health.causes, and otherwise the group takes its first part's message.
export function healthOf(state) {
  const health = state?.health, time = state?.time;
  const components = health?.components?.length ? health.components.map(part => ({
    id: part.id, name: part.name || NAMES.get(part.id) || part.id, short: SHORT.get(part.id) || part.name || part.id,
    status: STATUSES.has(part.status) ? part.status : 'warn', message: part.message || '', fix: part.fix || '',
    since: part.since ?? null, cause: part.cause || null,
  })) : fromNodesAndCameras(state);
  const problems = [], byCause = new Map();
  for (const part of components.filter(part => PROBLEM.has(part.status))) {
    const group = part.cause && byCause.get(part.cause);
    if (group) { group.parts.push(part); if (part.status === 'fail') group.status = 'fail'; continue; }
    const made = {id: part.cause ? `cause:${part.cause}` : part.id, parts: [part], status: part.status,
                  text: (part.cause && health?.causes?.[part.cause]) || part.message || `${part.name}: ${part.status}`};
    if (part.cause) byCause.set(part.cause, made);
    problems.push(made);
  }
  return {checked: Boolean(health?.components?.length), components, problems, time,
          i2c: health?.i2c ?? null, servo: health?.servo_board ?? null, power: health?.power ?? null};
}

// Why a part is not working, {message, fix}, or null while it is well, off, or not checked.
export function reasonOf(model, id) {
  const part = model?.components.find(component => component.id === id);
  return part && PROBLEM.has(part.status) ? {message: part.message || `${part.name}: ${part.status}`, fix: part.fix} : null;
}

// Without the bot's checks: what robot_state shows anyway, with no fix hints, since only the bot could say.
function fromNodesAndCameras(state) {
  const parts = [];
  for (const node of state?.nodes ?? []) {
    if (node.state !== 'crashed' && node.state !== 'stopped') continue;
    parts.push({id: `node:${node.name}`, name: node.name, short: node.name, status: node.state === 'crashed' ? 'fail' : 'warn',
                message: node.state === 'crashed' ? `${node.name} crashed${node.error ? `: ${node.error}` : ''}` : `${node.name} stopped`,
                fix: '', since: null, cause: null});
  }
  for (const [camera, feed] of Object.entries(state?.cameras ?? {})) {
    if (feed.width && feed.age != null && feed.age <= 3) continue;
    parts.push({id: `camera:${camera}`, name: camera, short: camera, status: 'warn', fix: '', since: null, cause: null,
                message: feed.width ? `No frames from ${camera} for ${ago(feed.age)}` : `No frames from ${camera} yet`});
  }
  return parts;
}

// The banner, the panel and the nodes card's lights. The panel opens by itself when a problem appears and folds
// away when all is well; a click opens or closes it, and that choice holds until a problem the page has not seen yet.
export class HealthView {
  // elements: {banner, count, summary, toggle, panel, rows, lights, diagnostics, source}
  constructor(elements) {
    this.$ = elements;
    this.choice = null;          // 'open' or 'closed', the user's, until a new problem
    this.seen = new Set();       // Problem ids in the last update
    this.rows = new Map(); this.lights = new Map(); this.key = ''; this.readings = ''; this.model = null; this.link = null;
    elements.toggle.onclick = () => this.setOpen(this.$.panel.hidden);
    elements.lights.onclick = () => this.setOpen(this.$.panel.hidden);
    elements.close.onclick = () => this.setOpen(false);
  }

  setOpen(open) {
    this.choice = open ? 'open' : 'closed';
    this.apply();
  }

  apply() {
    const problems = (this.model?.problems.length ?? 0) + (this.link ? 1 : 0);
    const open = this.choice ? this.choice === 'open' : problems > 0;
    setProperty(this.$.panel, 'hidden', !open);
    setText(this.$.toggle, open ? 'Hide' : problems ? 'Details' : 'Checks');
    if (this.$.toggle.getAttribute('aria-expanded') !== String(open)) this.$.toggle.setAttribute('aria-expanded', String(open));
    if (open) this.renderDiagnostics();  // The live readings are written only while someone can see them
  }

  // The dashboard's own problem, shown with the bot's: the link to the bot, or null when it is fine.
  setLink(problem) {
    if ((problem?.text ?? null) === (this.link?.text ?? null)) return;
    if (problem && !this.link) this.choice = null;  // Losing the bot is a new problem
    this.link = problem;
    this.renderBanner(); this.apply();
  }

  // One robot_state's checks (healthOf). Writes the page only where something changed.
  update(model) {
    this.model = model;
    const ids = new Set(model.problems.map(problem => problem.id));
    if ([...ids].some(id => !this.seen.has(id))) this.choice = null;
    this.seen = ids;
    const key = JSON.stringify([model.checked, model.components.map(p => [p.id, p.status, p.message, p.fix, p.since]),
                                model.problems.map(p => p.id)]);
    if (key !== this.key) {
      this.key = key;
      this.renderBanner(); this.renderRows(); this.renderLights();
    }
    this.renderSince();
    this.apply();
  }

  renderBanner() {
    const model = this.model, problems = [...(this.link ? [this.link] : []), ...(model?.problems ?? [])];
    const banner = this.$.banner;
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
      const row = document.createElement('li'), light = document.createElement('i'), name = document.createElement('b');
      const message = document.createElement('span'), fix = document.createElement('span'), since = document.createElement('time');
      light.className = 'light'; message.className = 'message'; fix.className = 'fix'; since.className = 'since';
      row.append(light, name, since, message, fix);
      return [row];
    });
    for (const part of parts) {
      const [row] = this.rows.get(part.id), [light, name, , message, fix] = row.children;
      setProperty(row, 'className', `part ${part.status}`);
      setProperty(light, 'title', part.status);
      setText(name, part.name); setText(message, part.message || (part.status === 'ok' ? 'OK' : part.status === 'off' ? 'Not used in this run' : ''));
      setText(fix, part.fix); setProperty(fix, 'hidden', !part.fix);
    }
  }

  // How long each part has been as it is, from the Pi's clock: once a second and with each update.
  renderSince() {
    const now = this.model?.time;
    for (const part of this.model?.components ?? []) {
      const since = this.rows.get(part.id)?.[0].children[2];
      if (since) setText(since, part.since == null || now == null ? '' : `since ${ago(now - part.since)}`);
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
      setProperty(light, 'title', `${part.name}: ${part.message || part.status}`);
      setText(light.childNodes[1], part.short);
    }
  }

  // The bot's live readings, compact: which I2C addresses answer, what the servo board reads back, the Pi's rails.
  renderDiagnostics() {
    const {i2c, servo, power} = this.model ?? {}, lines = [];
    // Each reading's snapshot time changes with every message; what is shown does not.
    const readings = JSON.stringify([i2c, servo, power], (name, value) => name === 'time' ? undefined : value);
    if (readings === this.readings) return;
    this.readings = readings;
    if (i2c) {
      const answering = new Set(i2c.answering ?? []), expected = i2c.expected ?? [...answering];
      const errors = Object.entries(i2c.errors ?? {}).filter(([, count]) => count).map(([address, count]) => `${address} ${count}`);
      lines.push(['I2C', `bus ${i2c.bus ?? '?'}: ${expected.map(address => `${address} ${answering.has(address) ? 'answers' : 'silent'}`).join(' · ') || 'nothing answers'}` +
                  (errors.length ? ` · errors ${errors.join(', ')}` : '')]);
    }
    if (servo) {
      const channels = Object.entries(servo.channels ?? {}).map(([name, channel]) =>
        `${name} ${percent(channel.duty)}${channel.commanded_deg == null ? '' : ` for ${Math.round(channel.commanded_deg)}° (expects ${percent(channel.expected_duty)})`}`);
      lines.push(['Servo board', [servo.awake == null ? null : servo.awake ? 'awake' : 'asleep',
                                  servo.frequency_hz == null ? null : `${servo.frequency_hz.toFixed(1)} Hz`, ...channels].filter(Boolean).join(' · ')]);
    }
    if (power) {
      // Rails by name, 3v3 or 5v: "3.3 V rail 3.31 V"
      const rails = Object.entries(power.rails ?? {}).map(([rail, value]) =>
        `${rail.replace(/^(\d+)v(\d*)$/i, (_, whole, tenths) => tenths ? `${whole}.${tenths} V` : `${whole} V`)} rail ${volts(value)}`);
      const now = power.throttled_now ?? [], before = power.throttled_since_boot ?? [];
      lines.push(['Power', [...rails, now.length ? `now: ${now.join(', ')}` : 'not throttled', before.length ? `since boot: ${before.join(', ')}` : null].filter(Boolean).join(' · ')]);
    }
    const list = this.$.diagnostics;
    setProperty(list.parentElement, 'hidden', !lines.length);
    if (list.childElementCount !== lines.length * 2) list.replaceChildren(...lines.flatMap(() => [document.createElement('dt'), document.createElement('dd')]));
    lines.forEach(([term, value], i) => { setText(list.children[2 * i], term); setText(list.children[2 * i + 1], value); });
  }
}
