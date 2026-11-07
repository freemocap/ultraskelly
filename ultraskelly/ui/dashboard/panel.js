// The control column (docs/dashboard.md, "Control panel"). Served by the always-on control service
// (ultraskelly/control) instead of the bot, the page turns the bot on and off, restarts it, switches following
// and says how the bot is: running or not, for how long, what works and its last error. The column takes the motor
// tests and the nodes cards from the strip, so the controls sit together and the time series has the strip. It folds
// to a strip that keeps the bot's lamp, and stays folded across reloads. The live views beside it come straight from
// the bot's own port while it runs. Served by the bot itself, the page finds no /api/panel and stays the plain dashboard.
import { setProperty, setText } from '/dom.js';
import { Tune } from '/tune.js';

const FOLDED = 'ultraskelly.control.folded';  // localStorage: '1' while the column is folded to its strip
// localStorage, which a locked-down browser may refuse; the column then opens at every load.
function stored(key) { try { return localStorage.getItem(key); } catch { return null; } }
function store(key, value) { try { localStorage.setItem(key, value); } catch { /* Not kept */ } }

// The control service's settings ({bot_port, local, host, unit}), or null when the bot serves this page.
export async function panelSettings() {
  try {
    const reply = await fetch('/api/panel', {cache: 'no-store'});
    return reply.ok ? await reply.json() : null;
  } catch { return null; }
}

// Where the live stream is: the bot's own port when the control service serves the page, else here.
export function streamUrl(settings) {
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${scheme}://${settings ? `${location.hostname}:${settings.bot_port}` : location.host}/stream`;
}

// Each state of the bot (ultraskelly/control/bots.py STATES): its word, its lamp, and the readout's line when the word
// alone does not say what is wrong.
const STATES = {
  running: {word: 'Running', lamp: 'on'},
  starting: {word: 'Starting', lamp: 'busy'},
  stopping: {word: 'Stopping', lamp: 'busy'},
  restarting: {word: 'Restarting', lamp: 'busy'},
  waiting: {word: 'Waiting', lamp: 'warn', note: 'Another program is using the robot'},
  off: {word: 'Off', lamp: 'off'},
  failed: {word: 'Failed', lamp: 'fail'},
  missing: {word: 'Not installed', lamp: 'fail', note: 'No ultraskelly-bot service: run scripts/install-services.sh'},
  unknown: {word: 'Unknown', lamp: 'warn', note: 'The service manager did not answer'},
};
// The status lamps' order: the bot's check groups as core/bot/health.py COMPONENTS has them.
const GROUPS = ['cameras', 'ai', 'motors', 'sensors', 'power'];
// systemd's Result words: how the bot's last run ended.
const ENDED = {
  'exit-code': 'it exited with an error', signal: 'it was killed', 'core-dump': 'it crashed', watchdog:
  'the watchdog stopped it: it did not answer for 30 s', timeout: 'it took too long to start or stop',
  'start-limit-hit': 'it failed to start too often', resources: 'it could not be set up', 'oom-kill': 'it ran out of memory',
};
// The bot's hardware checks' short names in the status rows, which group them as core/bot/health.py COMPONENTS does.
const SHORT = {oak_camera: 'OAK', oak_models: 'OAK models', hailo: 'Hailo-8', zed_camera: 'ZED Mini', zed_imu: 'ZED IMU', depth: 'Depth',
               servo_board: 'Servo board', motor_board: 'Motor HAT', head: 'Head aim', power: 'Pi power'};

const clockTime = unix => new Date(unix * 1000).toLocaleTimeString([], {hour: '2-digit', minute: '2-digit'});
export function duration(seconds) {
  if (seconds == null || !Number.isFinite(seconds)) return '';
  const s = Math.max(0, Math.round(seconds));
  if (s < 60) return `${s} s`;
  if (s < 3600) return `${Math.floor(s / 60)} min`;
  if (s < 86400) return `${Math.floor(s / 3600)} h ${String(Math.floor(s / 60) % 60).padStart(2, '0')} min`;
  return `${Math.floor(s / 86400)} d ${Math.floor(s / 3600) % 24} h`;
}
const plural = (n, word) => `${n} ${word}${n === 1 ? '' : 's'}`;

// The readout's line under the state word, only for what the word, the buttons and the follow switch do not already
// say: that the bot keeps restarting, how a run ended, or which program holds the robot. Empty otherwise.
export function detailOf(status) {
  const bot = status.bot;
  const ended = bot.result && bot.result !== 'success' ? ENDED[bot.result] ?? `it ended (${bot.result})` : null;
  const exit = bot.exit?.code === 'exited' && bot.exit.status ? ` with status ${bot.exit.status}` : bot.exit?.code === 'killed' ? ` by signal ${bot.exit.status}` : '';
  if (bot.state === 'running') return bot.restarts ? `Restarted itself ${plural(bot.restarts, 'time')}` : '';
  if (bot.state === 'failed') return `Stopped: ${ended ?? 'it stopped'}${exit}`;
  if (bot.state === 'off') {
    const asked = status.action?.action === 'stop' && status.action.ok;
    return !asked && ended ? `Its last run ended: ${ended}${exit}` : '';
  }
  if (bot.state === 'waiting' && status.last_error?.waiting) return status.last_error.text.split('\n').slice(1).join(' · ') || STATES.waiting.note;
  if (bot.state === 'restarting') {
    return [ended && `${ended[0].toUpperCase()}${ended.slice(1)}${exit}`, bot.restarts && `${plural(bot.restarts, 'restart')} so far`].filter(Boolean).join(' · ');
  }
  return STATES[bot.state]?.note ?? '';
}

export class ControlPanel {
  // views: {stream, health, motorTest}: the dashboard's stream (connected only while the bot may answer), its health
  // view (whose banner says the bot is off rather than failing) and the motor test panel (built once the bot runs).
  constructor(settings, views) {
    this.settings = settings;
    this.views = views;
    this.$ = id => document.getElementById(id);
    this.status = null;
    this.offset = 0;  // The control service's clock minus this one
    this.live = null;  // From the stream's robot_state: {following, simulated}
    this.healthModel = null;
    this.pending = null;  // The command this page is waiting on
    this.answer = null;  // {text, kind, until}: the reply to this page's last command
    this.wasRunning = false;
    this.$('control').hidden = false;
    document.body.classList.add('control');
    document.title = 'UltraSkelly · Control panel';
    // The motor tests under the follow switch and the Tune key, the nodes under the status lamps: the controls together,
    // and the strip left to the time series.
    this.$('control-tune').after(document.querySelector('.card.tests'));
    this.$('control-parts').after(document.querySelector('.card.nodes'));
    this.$('control-power').onclick = () => {
      const on = this.isOn();
      this.send(on ? '/api/bot/stop' : '/api/bot/start', {}, on ? 'Turning the bot off…' : 'Turning the bot on…');
    };
    this.$('control-restart').onclick = () => this.send('/api/bot/restart', {}, 'Restarting the bot…');
    this.$('control-follow').onclick = () => {
      const on = this.$('control-follow').getAttribute('aria-checked') !== 'true';
      this.send('/api/follow', {following: on}, on ? 'Switching following on…' : 'Switching following off…');
    };
    this.tune = new Tune(this.$('tune'));
    this.$('control-tune').onclick = () => this.tune.open();
    this.$('control-collapse').onclick = () => this.fold(true);
    this.$('control-rail').onclick = () => this.fold(false);
    // Whether the time series can run under the column: checked as the 3D scene or any of the column's blocks changes size.
    let checking = 0;
    const watch = new ResizeObserver(() => { checking ||= requestAnimationFrame(() => { checking = 0; this.fitColumn(); }); });
    watch.observe(document.querySelector('.stage'));
    for (const block of this.$('control').children) watch.observe(block);
    this.fold(stored(FOLDED) === '1', false);
    this.poll();
    setInterval(() => this.poll(), 1000);
  }

  // Fold the column to its strip, or open it; `remember` keeps the choice for the next load (the Pi's window
  // reopens on the same browser profile at every boot).
  fold(folded, remember = true) {
    this.$('control').classList.toggle('folded', folded);
    document.body.classList.toggle('control-folded', folded);
    this.$('control-collapse').setAttribute('aria-expanded', String(!folded));
    this.$('control-rail').setAttribute('aria-expanded', String(!folded));
    if (remember) store(FOLDED, folded ? '1' : '0');
    this.fitColumn();
    dispatchEvent(new Event('resize'));  // The views beside the column take up the room it leaves or needs
  }

  // The time series runs under the column too, from the left edge (body.series-under, panel.css), when the column's
  // blocks fit beside the 3D scene with the nodes card at its smallest: on the Pi's 1080 px they do, unless the strip is
  // dragged tall. Folded, it always does. Neither height depends on the choice; it moves under only with 24 px to spare
  // and back only once the blocks no longer fit, so a status line coming and going cannot make it jump to and fro.
  fitColumn() {
    const column = this.$('control'), stage = document.querySelector('.stage'), was = document.body.classList.contains('series-under');
    let under = column.classList.contains('folded');
    if (!under) {
      const style = getComputedStyle(column), blocks = [...column.children].filter(block => block.getClientRects().length);
      const height = block => block.matches('.card.nodes') ? parseFloat(getComputedStyle(block).minHeight) || 0 : block.offsetHeight;
      const need = blocks.reduce((sum, block) => sum + height(block), 0) + (parseFloat(style.rowGap) || 0) * Math.max(0, blocks.length - 1)
        + parseFloat(style.paddingTop) + parseFloat(style.paddingBottom) + parseFloat(style.borderTopWidth) + parseFloat(style.borderBottomWidth);
      under = need <= stage.offsetHeight - (was ? 0 : 24);
    }
    if (under !== was) document.body.classList.toggle('series-under', under);
  }

  // The header's connection word while the bot is not running (the dashboard's own while it is), else null.
  get down() {
    if (this.lost) return '○ No panel';
    const state = this.status?.bot.state;
    return !state || state === 'running' ? null : `○ Bot ${(STATES[state] ?? STATES.unknown).word.toLowerCase()}`;
  }

  isOn() { return this.status && !['off', 'failed', 'missing', 'unknown'].includes(this.status.bot.state); }

  async poll() {
    try {
      const reply = await fetch('/api/bot', {cache: 'no-store'});
      if (!reply.ok) throw new Error(`HTTP ${reply.status}`);
      this.status = await reply.json();
      this.offset = this.status.time - Date.now() / 1000;
      this.lost = false;
    } catch (problem) {
      this.lost = true;  // The control service itself is unreachable: say so, keep the last status greyed out
    }
    this.render();
  }

  now() { return Date.now() / 1000 + this.offset; }

  async send(path, body, busyText) {
    if (this.pending) return;
    this.pending = path;
    this.say(busyText, 'busy', 30);
    this.render();
    try {
      const reply = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
      const answer = await reply.json().catch(() => ({}));
      const notes = answer.notes?.length ? ` (${answer.notes.join('; ')})` : '';
      if (reply.ok && answer.ok !== false) this.say(`${answer.message ?? 'Done'}${notes}`, 'done');
      else this.say(`${answer.error ?? answer.message ?? `Refused (HTTP ${reply.status})`}${notes}`, 'refused', 12);
    } catch (problem) {
      this.say(`The control panel did not answer: ${problem.message ?? problem}`, 'refused', 12);
    }
    this.pending = null;
    await this.poll();
  }

  say(text, kind, seconds = 8) { this.answer = {text, kind, until: performance.now() + seconds * 1000}; }

  // Each robot_state from the bot's stream, and the dashboard's health model of it.
  update(state, model) {
    const tests = state?.motor_test;
    this.live = tests ? {following: tests.following ?? null, simulated: Boolean(tests.simulated || state.motors?.simulated)}
                      : {following: null, simulated: Boolean(state?.motors?.simulated)};
    this.healthModel = model;
    this.renderFollow(); this.renderParts();
  }

  render() {
    if (!this.status) return;
    const bot = this.status.bot, info = STATES[bot.state] ?? STATES.unknown, running = bot.state === 'running';
    // Readout: the state word and for how long; since when, and on which robot, on hover
    const readout = this.$('control-readout');
    setProperty(readout, 'className', `readout ${info.lamp}${this.lost ? ' lost' : ''}`);
    setText(this.$('control-state'), this.lost ? 'No answer' : info.word);
    const since = running ? bot.started ?? bot.since : bot.since, host = this.settings.local ? 'the local stand-in' : this.settings.host;
    setText(this.$('control-uptime'), since ? duration(this.now() - since) : '');
    setProperty(readout, 'title', `${info.word}${since ? ` since ${clockTime(since)}` : ''}${host ? ` on ${host}` : ''}`);
    setText(this.$('control-detail'), this.lost ? 'The control panel is not answering' : detailOf(this.status));
    // The folded strip: the same lamp and word
    const word = this.lost ? 'No answer' : info.word;
    setProperty(this.$('control-rail'), 'className', `control-rail ${info.lamp}${this.lost ? ' lost' : ''}`);
    setText(this.$('control-rail-state'), word);
    setProperty(this.$('control-rail'), 'title', `Bot ${word.toLowerCase()}: show the robot controls`);
    // Buttons
    const on = this.isOn(), busy = Boolean(this.pending) || this.lost;
    const power = this.$('control-power');
    setProperty(power, 'className', `big power ${on ? 'turn-off' : 'turn-on'}`);
    setText(power.querySelector('b'), on ? 'Turn off' : 'Turn on');
    setProperty(power, 'title', on ? 'Stop the bot; its service brakes the DC motors as it stops' : 'Start the bot');
    setProperty(power, 'disabled', busy || bot.state === 'missing');
    setProperty(this.$('control-restart'), 'disabled', busy || !on);
    setProperty(this.$('control-follow'), 'disabled', busy);
    setProperty(this.$('control-tune'), 'disabled', this.lost || !running);
    setProperty(this.$('control-tune'), 'title', running ? 'Tune the controllers as the bot runs' : 'The controllers are tuned while the bot runs');
    // The reply to this page's last command, else the newest command from any screen
    const answer = this.answer && performance.now() < this.answer.until ? this.answer : null;
    const recent = this.status.action && this.now() - this.status.action.time < 8 ? this.status.action : null;
    const shown = answer ?? (recent ? {text: recent.message, kind: recent.ok ? 'done' : 'refused'} : null);
    setText(this.$('control-answer'), shown?.text ?? '');
    setProperty(this.$('control-answer'), 'className', `control-answer ${shown?.kind ?? ''}`);
    // The last error: one line with its time, the whole text and its age on hover; none, no line
    const error = this.status.last_error, line = this.$('control-error');
    setProperty(line, 'hidden', !error);
    setText(line, error ? `${clockTime(error.time)} ${error.text.split('\n')[0]}` : '');
    setProperty(line, 'title', error ? `${error.text}\n\n${clockTime(error.time)}, ${duration(this.now() - error.time)} ago` : '');
    setProperty(line, 'className', `control-error ${error?.waiting ? 'waiting' : 'error'}`);
    // The header, the motor test buttons and the stream follow the bot; the live views just keep what they last had
    this.views.health?.setOffline(running ? null : {title: `Bot ${info.word.toLowerCase()}`, text: ''});
    this.views.motorTest?.setOnline(running);
    this.follow(running || (bot.state === 'stopping' && this.views.stream?.getState() === 'CONNECTED'));
    if (running && !this.wasRunning) this.views.motorTest?.load();
    this.wasRunning = running;
    this.renderFollow(); this.renderParts();
  }

  // Read the bot's stream while it runs (its server starts once it is ready), and through its stop, so a robot that is
  // off or starting costs no failed connections.
  follow(wanted) {
    const stream = this.views.stream;
    if (!stream) return;
    const state = stream.getState();
    if (wanted && state === 'DISCONNECTED') stream.connect();
    else if (!wanted && state !== 'DISCONNECTED') stream.disconnect();
  }

  renderFollow() {
    if (!this.status) return;
    const running = this.status.bot.state === 'running';
    const liveFollowing = running ? this.live?.following : null;
    const following = liveFollowing ?? this.status.start?.following ?? true;
    const button = this.$('control-follow');
    if (button.getAttribute('aria-checked') !== String(following)) button.setAttribute('aria-checked', String(following));
    setText(button.querySelector('.switch-word'), following ? 'On' : 'Off');
    setProperty(button, 'title', liveFollowing != null
      ? following ? 'The head and waist turn toward people' : 'The head and waist hold still; motor tests still move them'
      : `${following ? 'On' : 'Off'} when the bot starts`);
  }

  // The status lamps: one per part the bot checks, in its groups' order, its message on hover. None while the bot is
  // not running or sends no checks; the readout says why.
  renderParts() {
    const running = this.status?.bot.state === 'running', model = running ? this.healthModel : null;
    const parts = model?.checked ? GROUPS.flatMap(group => model.components.filter(part => part.group === group)) : [];
    const row = this.$('control-parts'), key = JSON.stringify(parts.map(part => [part.id, part.status, part.message, part.fix]));
    setProperty(row, 'hidden', !parts.length);
    if (row.dataset.key === key) return;
    row.dataset.key = key;
    row.replaceChildren(...parts.map(part => {
      const lamp = document.createElement('span');
      lamp.className = `part-lamp ${part.status}`;
      lamp.title = `${part.name}: ${part.message || part.word}${part.fix ? `\nFix: ${part.fix}` : ''}`;
      lamp.append(document.createElement('i'), SHORT[part.id] ?? part.short ?? part.name);
      return lamp;
    }));
  }
}
