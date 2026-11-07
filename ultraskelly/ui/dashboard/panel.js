// The control column (docs/dashboard.md, "Control panel"). Served by the always-on control service
// (ultraskelly/control) instead of the bot, the page turns the bot on and off, restarts it, switches following,
// stops every motor and says how the bot is: running or not, since when, its last error and what works. The live
// views beside it come straight from the bot's own port while it runs. Served by the bot itself, the page finds
// no /api/panel and stays the plain dashboard.
import { setProperty, setText } from '/dom.js';

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

// Each state of the bot (ultraskelly/control/bots.py STATES): its word, its lamp, what the live views say meanwhile.
const STATES = {
  running: {word: 'Running', lamp: 'on'},
  starting: {word: 'Starting', lamp: 'busy', note: 'Booting the cameras and loading the models; it takes a few seconds'},
  stopping: {word: 'Stopping', lamp: 'busy', note: 'Stopping its parts; the head centres and the waist brakes'},
  restarting: {word: 'Restarting', lamp: 'busy', note: 'It stopped by itself and starts again in a moment'},
  waiting: {word: 'Waiting', lamp: 'warn', note: 'Another program is using the robot; the bot starts once it is done'},
  off: {word: 'Off', lamp: 'off', note: 'Turn it on to see its cameras, the 3D view and the motor tests'},
  failed: {word: 'Failed', lamp: 'fail', note: 'It stopped and will not start again by itself'},
  missing: {word: 'Not installed', lamp: 'fail', note: 'No ultraskelly-bot service here: run scripts/install-services.sh'},
  unknown: {word: 'Unknown', lamp: 'warn', note: 'The service manager did not answer'},
};
// systemd's Result words: how the bot's last run ended.
const ENDED = {
  'exit-code': 'it exited with an error', signal: 'it was killed', 'core-dump': 'it crashed', watchdog:
  'the watchdog stopped it: it did not answer for 30 s', timeout: 'it took too long to start or stop',
  'start-limit-hit': 'it failed to start too often', resources: 'it could not be set up', 'oom-kill': 'it ran out of memory',
};
// The bot's hardware checks' short names in the status rows, which group them as core/bot/health.py COMPONENTS does.
const SHORT = {oak_camera: 'OAK', oak_models: 'OAK models', hailo: 'Hailo-8', zed_camera: 'ZED Mini', zed_imu: 'ZED IMU', depth: 'Depth',
               servo_board: 'Servo board', motor_board: 'Motor HAT', power: 'Pi power'};

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

// What the readout says under the state word: why it is so, and what it is doing.
export function detailOf(status, live) {
  const bot = status.bot, start = status.start ?? {};
  const ended = bot.result && bot.result !== 'success' ? ENDED[bot.result] ?? `it ended (${bot.result})` : null;
  const exit = bot.exit?.code === 'exited' && bot.exit.status ? ` with status ${bot.exit.status}` : bot.exit?.code === 'killed' ? ` by signal ${bot.exit.status}` : '';
  const following = live?.following ?? start.following;
  if (bot.state === 'running') {
    return [following === false ? 'Holding still: following is off' : 'Following people',
            live?.simulated ? 'simulated motors' : null,
            bot.restarts ? `restarted itself ${plural(bot.restarts, 'time')}` : null].filter(Boolean).join(' · ');
  }
  if (bot.state === 'failed') return `Stopped: ${ended ?? 'it stopped'}${exit}`;
  if (bot.state === 'off') {
    const asked = status.action?.action === 'stop' || status.action?.action === 'stop motors';
    return asked && status.action.ok ? `Turned off from the panel at ${clockTime(status.action.time)}`
      : ended ? `Its last run ended: ${ended}${exit}` : 'Not running';
  }
  if (bot.state === 'waiting' && status.last_error?.waiting) return status.last_error.text.split('\n').slice(1).join(' · ') || STATES.waiting.note;
  if (bot.state === 'restarting') return `${ended ? `${ended[0].toUpperCase()}${ended.slice(1)}${exit}; ` : ''}starting again${bot.restarts ? ` (${plural(bot.restarts, 'restart')} so far)` : ''}`;
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
    const title = document.querySelector('header h1 span');
    if (title) title.textContent = '/ control panel';
    setText(this.$('control-host'), settings.local ? 'local stand-in' : settings.host ?? '');
    this.$('control-power').onclick = () => {
      const on = this.isOn();
      this.send(on ? '/api/bot/stop' : '/api/bot/start', {}, on ? 'Turning the bot off…' : 'Turning the bot on…');
    };
    this.$('control-restart').onclick = () => this.send('/api/bot/restart', {}, 'Restarting the bot…');
    this.$('control-follow').onclick = () => {
      const on = this.$('control-follow').getAttribute('aria-checked') !== 'true';
      this.send('/api/follow', {following: on}, on ? 'Switching following on…' : 'Switching following off…');
    };
    this.$('control-stop').onclick = () => this.send('/api/stop-motors', {}, 'Stopping every motor…', 'stop');
    this.poll();
    setInterval(() => this.poll(), 1000);
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

  async send(path, body, busyText, kind = '') {
    if (this.pending) return;
    this.pending = path;
    this.say(busyText, 'busy', 30);
    this.render();
    try {
      const reply = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
      const answer = await reply.json().catch(() => ({}));
      const notes = answer.notes?.length ? ` (${answer.notes.join('; ')})` : '';
      if (reply.ok && answer.ok !== false) this.say(`${answer.message ?? 'Done'}${notes}`, kind === 'stop' ? 'stopped' : 'done');
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
    const live = running ? this.live : null;
    // Readout
    const readout = this.$('control-readout');
    setProperty(readout, 'className', `readout ${info.lamp}${this.lost ? ' lost' : ''}`);
    setText(this.$('control-state'), this.lost ? 'No answer' : info.word);
    const since = running ? bot.started ?? bot.since : bot.since;
    setText(this.$('control-since'), since ? `${running ? 'up since' : 'since'} ${clockTime(since)}` : '');
    setText(this.$('control-uptime'), since ? duration(this.now() - since) : '');
    setText(this.$('control-detail'), this.lost ? 'The control panel is not answering; is the Pi on the network?' : detailOf(this.status, live));
    // Buttons
    const on = this.isOn(), busy = Boolean(this.pending) || this.lost;
    const power = this.$('control-power');
    setProperty(power, 'className', `big power ${on ? 'turn-off' : 'turn-on'}`);
    setText(power.querySelector('b'), on ? 'Turn off' : 'Turn on');
    setText(power.querySelector('span'), on ? 'Stop the bot' : 'Start the bot');
    setProperty(power, 'disabled', busy || bot.state === 'missing');
    setProperty(this.$('control-restart'), 'disabled', busy || !on);
    setProperty(this.$('control-stop'), 'disabled', this.pending === '/api/stop-motors' || this.lost);
    setProperty(this.$('control-follow'), 'disabled', busy);
    // The reply to this page's last command, else the newest command from any screen
    const answer = this.answer && performance.now() < this.answer.until ? this.answer : null;
    const recent = this.status.action && this.now() - this.status.action.time < 8 ? this.status.action : null;
    const shown = answer ?? (recent ? {text: recent.message, kind: recent.ok ? 'done' : 'refused'} : null);
    setText(this.$('control-answer'), shown?.text ?? '');
    setProperty(this.$('control-answer'), 'className', `control-answer ${shown?.kind ?? ''}`);
    // Last error
    const error = this.status.last_error;
    setText(this.$('control-error'), error ? error.text : 'None in the bot\'s recent log');
    setText(this.$('control-error-age'), error ? `${clockTime(error.time)} · ${duration(this.now() - error.time)} ago` : '');
    setProperty(this.$('control-error-box'), 'className', `control-error${error ? error.waiting ? ' waiting' : ' error' : ''}`);
    // The live views, the banner and the stream follow the bot
    const overlay = this.$('bot-off');
    setProperty(overlay, 'hidden', running);
    setProperty(overlay, 'className', `bot-off ${info.lamp}`);
    setText(this.$('bot-off-title'), {off: 'The robot is off', failed: 'The bot stopped', missing: 'No bot service'}[bot.state] ?? `${info.word}…`);
    setText(this.$('bot-off-note'), bot.state === 'waiting' ? detailOf(this.status, null) : info.note ?? '');
    this.views.health?.setOffline(running ? null : {title: `Robot ${info.word.toLowerCase()}`, text: bot.state === 'off' ? 'turn it on from the panel' : info.note ?? ''});
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
    setText(this.$('control-follow-note'), liveFollowing != null
      ? following ? 'The head and waist turn toward people' : 'The head and waist hold still; motor tests still move them'
      : `${following ? 'On' : 'Off'} when the bot starts`);
  }

  unchecked(running) {
    if (running) return 'no checks from this bot';
    return {off: 'not checked while it is off', starting: 'checked once it runs', restarting: 'checked once it runs again',
            waiting: 'checked once it runs'}[this.status?.bot.state] ?? 'not checked now';
  }

  renderParts() {
    const running = this.status?.bot.state === 'running', model = running ? this.healthModel : null;
    for (const row of this.$('control-parts').querySelectorAll('dd[data-group]')) {
      const parts = model?.checked ? model.components.filter(part => part.group === row.dataset.group) : [];
      const key = JSON.stringify([this.unchecked(running), parts.map(part => [part.id, part.status, part.message])]);
      if (row.dataset.key === key && row.childElementCount) continue;
      row.dataset.key = key;
      row.replaceChildren(...(parts.length ? parts.map(part => {
        const lamp = document.createElement('span');
        lamp.className = `part-lamp ${part.status}`;
        lamp.title = `${part.name}: ${part.message || part.word}${part.fix ? `\nFix: ${part.fix}` : ''}`;
        lamp.append(document.createElement('i'), SHORT[part.id] ?? part.short ?? part.name);
        return lamp;
      }) : [Object.assign(document.createElement('span'), {className: 'part-none', textContent: this.unchecked(running)})]));
    }
  }
}
