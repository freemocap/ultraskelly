// Motor tests, in the strip's card: buttons that move one actuator at a time through /api/motor-test, right in the
// card. The bot applies the test limits and pauses tracking during each test; every test is answered once it has
// finished, been stopped or been refused, and the answer shows on the card's status line for a few seconds. Then the
// line says what the bot reports: a test running from any screen, the last one's outcome, or why tests cannot run.
// Angles are commands, not measured positions, labelled "not sent" while their board is down. After a servo test, a
// line appears if the servo board's read-back says the command did not arrive (health.js's model).
import { setProperty, setText } from '/dom.js';
import { readingsStale } from '/health.js';

const CENTER = 90;      // Degrees; the head's neutral angle
const SERVO_STEP = 10;  // Degrees per nudge
const ANSWER_SECONDS = 6;  // How long this page's own answer stays on the status line
const RETRY_SECONDS = 3;  // While the bot does not answer yet

const element = (tag, properties = {}, ...children) => {
  const made = Object.assign(document.createElement(tag), properties);
  made.append(...children);
  return made;
};
const signed = value => `${value >= 0 ? '+' : '−'}${Math.abs(value).toFixed(2)}`;
const capital = text => `${text[0].toUpperCase()}${text.slice(1)}`;
// A read-back pulse this close to the one the commanded angle needs is the command: about 2.4 degrees. The bot's own
// tolerance (diagnostics.py, PULSE_TOLERANCE_US), so this card and the servo board's status never disagree.
const SAME_US = 20;
export const down = (model, id) => { const part = model?.components?.find(component => component.id === id); return part && (part.status === 'fail' || part.status === 'warn') ? part : null; };

// What the servo board says about the last servo test's channel, from the bot's checks (health.js's model): its
// diagnostics.pca9685, pulse widths in microseconds. [words, hint, kind], or null when there is nothing to say.
// A board that does not answer has nothing to read back: the bot refuses the test with its message and hint.
function readbackOf(model, name) {
  const part = model?.components?.find(component => component.id === 'servo_board');
  const chip = model?.diagnostics?.pca9685, channel = chip?.channels?.[name];
  if (part?.status === 'off') return [part.message || 'Simulated: commands are recorded, not sent', part.fix || '', ''];
  if (channel?.pulse_us == null) return null;
  if (readingsStale(model)) {  // An old register value is not this test's result
    return ['No fresh read-back: the bot\'s live checks stopped', 'See why in the bot\'s log: journalctl -u ultraskelly-bot', 'refused'];
  }
  const angle = channel.commanded == null ? 'the command' : `${Math.round(channel.commanded)}°`;
  const said = `${capital(name)} commanded ${angle}; the chip reads ${Math.round(channel.pulse_us)} µs` +
               (channel.expected_us == null ? '' : `, ${angle} needs ${Math.round(channel.expected_us)} µs`);
  if (chip.awake === false) return [`${said}, but the chip is asleep, so it sends no pulses`, 'The bot wakes it when it starts the servos', 'refused'];
  // The bot's own advice for a board that has the command (its hint), or the usual three places to look.
  const moved = (part?.status === 'ok' && part.fix) || 'If it did not move: the OE wire (it must be low), V+ servo power on the board\'s terminal, the plug\'s orientation';
  const same = channel.matches ?? (channel.expected_us == null ? null : Math.abs(channel.pulse_us - channel.expected_us) <= SAME_US);  // The bot's verdict first
  if (same === null) return [said, moved, ''];
  if (!same) {
    return [`${said}: the command did not reach the chip`, 'Check the I2C wires to the servo board (SDA, SCL, GND)', 'refused'];
  }
  const rate = chip.frequency_hz != null && Math.abs(chip.frequency_hz - 50) > 5 ? ` at ${chip.frequency_hz.toFixed(0)} Hz, where servos want about 50` : '';
  return [`${said}${rate}: the chip has the command`, moved, rate ? 'refused' : 'done'];
}

export class MotorTestPanel {
  // root: the card's body. travel: the card title's readout of the net waist test drive. `load`: ask the bot for its
  // tests now; the control panel loads them later, once the bot runs.
  constructor(root, {travel = null, load = true} = {}) {
    this.root = root;
    this.travel = travel;
    this.limits = null;
    this.running = false;
    this.answer = null;  // {text, kind, until}: this page's own answer, over the bot's status for a few seconds
    this.commands = {};
    this.shown = {};
    this.lastServo = null;  // The actuator of the last servo test, whose read-back is checked
    this.retry = null;
    if (load) this.load();
  }

  // Asks the bot for its tests and builds the buttons, once. While the bot does not answer (it is starting, or the
  // control panel cannot reach it) it asks again; a bot without tests (404) is answered for good.
  async load() {
    if (this.buttons) return;
    clearTimeout(this.retry);
    let reply = null;
    try { reply = await fetch('/api/motor-test', {cache: 'no-store'}); } catch { /* Not answering yet */ }
    if (this.buttons) return;  // Built meanwhile by another call
    if (!reply?.ok) {
      const none = reply?.status === 404;
      this.root.replaceChildren(element('p', {className: 'tests-status', textContent: none ? 'Not available in this run' : 'Waiting for the bot…'}));
      if (!none) this.retry = setTimeout(() => this.load(), RETRY_SECONDS * 1000);
      return;
    }
    const state = await reply.json();
    this.limits = state.limits;
    this.build(state.enabled);
  }

  build(enabled) {
    const limits = this.limits, windows = limits.servo_windows;  // {pan: [low, high], tilt: [low, high]}: the head's own
    this.buttons = [];
    const button = (label, title, action) => {
      const made = element('button', {textContent: label, title, onclick: action});
      this.buttons.push(made);
      return made;
    };
    const number = (value, max, label) => element('input', {type: 'number', value, min: .05, max, step: .05, title: label, ariaLabel: label});
    const name = text => element('span', {className: 'motor-test-name', textContent: text});
    // A servo's row: its two nudges, [text, title, change in servo degrees], either side of the middle.
    const servo = (actuator, label, first, last) => {
      const [low, high] = windows[actuator];
      // Whole degrees: a nudge starts from the head's commanded angle, which following leaves fractional.
      const move = angle => this.send({actuator, angle: Math.round(Math.min(high, Math.max(low, angle)))});
      const current = () => this.commands[actuator] ?? CENTER;
      this.shown[actuator] = element('span', {className: 'motor-test-value', textContent: '—', title: 'Commanded angle, not a measured position'});
      return [name(label), element('span', {className: 'motor-test-buttons'},
        button(first[0], first[1], () => move(current() + first[2])),
        button(`${CENTER}`, `${label}: back to ${CENTER}°`, () => move(CENTER)),
        button(last[0], last[1], () => move(current() + last[2]))), this.shown[actuator]];
    };
    // Tilt by where the head goes, not by the servo's number: on this robot a larger tilt angle turns it down.
    const headDown = limits.directions?.tilt ?? 1;  // +1: a larger tilt angle turns the head down
    // A DC motor's row: its buttons, then the drive and seconds each press sends.
    const dc = (actuator, label, drive, seconds, buttons) => [name(label), element('span', {className: 'motor-test-drive'},
      ...buttons.map(([text, title, sign]) => button(text, title, () => this.send({actuator, throttle: sign * Number(drive.value), seconds: Number(seconds.value)}))),
      drive, element('span', {textContent: '×'}), seconds, element('span', {textContent: 's'}))];
    const waistDrive = number(Math.min(.5, limits.waist_max_throttle), limits.waist_max_throttle, 'Waist drive (throttle)');
    const waistSeconds = number(Math.min(.3, limits.waist_max_seconds), limits.waist_max_seconds, 'Waist seconds');
    const jawDrive = number(limits.mouth_max_throttle, limits.mouth_max_throttle, 'Jaw drive (throttle)');
    const jawSeconds = number(Math.min(.2, limits.mouth_max_seconds), limits.mouth_max_seconds, 'Jaw seconds');
    const grid = element('div', {className: 'motor-test', title:
      `One at a time; tracking pauses during each test. Pan ${windows.pan[0]}–${windows.pan[1]}°, tilt ${windows.tilt[0]}–${windows.tilt[1]}°; waist up to ${limits.waist_max_throttle.toFixed(2)} ` +
      `for ${limits.waist_max_seconds.toFixed(2)} s, at most ${limits.waist_max_travel.toFixed(2)} throttle-seconds one way; jaw up to ` +
      `${limits.mouth_max_throttle.toFixed(2)} for ${limits.mouth_max_seconds.toFixed(2)} s.`},
      ...servo('pan', 'Pan', [`−${SERVO_STEP}`, `Pan: ${SERVO_STEP}° less`, -SERVO_STEP], [`+${SERVO_STEP}`, `Pan: ${SERVO_STEP}° more`, SERVO_STEP]),
      ...servo('tilt', 'Tilt', [`▲${SERVO_STEP}`, `Tilt: head up, ${SERVO_STEP}° of servo`, -SERVO_STEP * headDown],
                               [`▼${SERVO_STEP}`, `Tilt: head down, ${SERVO_STEP}° of servo`, SERVO_STEP * headDown]),
      ...dc('waist', 'Waist', waistDrive, waistSeconds, [['−', 'Drive the waist with negative throttle', -1], ['+', 'Drive the waist with positive throttle', 1]]),
      ...dc('mouth', 'Jaw', jawDrive, jawSeconds, [['Open', 'Drive the jaw open (negative throttle); its spring closes it', -1]]));
    this.stopButton = element('button', {textContent: '■ Stop', className: 'motor-test-stop', title: 'Stop the running test', onclick: () => this.stop()});
    this.result = element('span', {className: 'motor-test-result', role: 'status'});
    this.readback = element('p', {className: 'motor-test-readback', hidden: true});
    this.root.replaceChildren(grid, element('div', {className: 'motor-test-foot'}, this.stopButton, this.result), this.readback);
    this.enabled = enabled;
    this.show(enabled ? 'Ready' : 'Turned off in the launch configuration');
    this.setRunning(false);
  }

  setRunning(running) {
    this.running = running;
    const usable = this.enabled && this.online !== false;
    for (const made of this.buttons) made.disabled = running || !usable;
    this.stopButton.disabled = !usable;
  }

  // Whether the bot is up, from the control panel (the bot's own page never says, so its buttons stay usable): the
  // buttons are greyed out while it is not.
  setOnline(online) {
    if (this.online === online) return;
    this.online = online;
    if (!this.buttons) return;
    this.setRunning(this.running);
    if (!online) { this.answer = null; this.show('Bot not running'); }
  }

  show(text, kind = '') {
    setText(this.result, text);
    setProperty(this.result, 'className', `motor-test-result ${kind}`);
    setProperty(this.result, 'title', text);  // The whole text where two lines cut it
  }

  say(text, kind = '') {
    this.answer = {text, kind, until: performance.now() + ANSWER_SECONDS * 1000};
    this.show(text, kind);
  }

  async post(path, body) {
    const reply = await fetch(path, {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
    return [reply, await reply.json()];
  }

  async send(test) {
    if (this.running) return;
    if (test.angle != null) this.lastServo = test.actuator;
    this.setRunning(true);
    this.answer = null;
    this.show(test.angle != null ? `Moving ${test.actuator} to ${test.angle}°…` : `Driving ${test.actuator} ${signed(test.throttle)} for ${test.seconds} s…`);
    try {
      const [reply, body] = await this.post('/api/motor-test', test);
      if (!reply.ok) this.say(`Refused: ${body.error}`, 'refused');
      else this.say(`${capital(body.test.outcome)}: ${body.test.description}`, body.ok ? 'done' : 'refused');
    } catch (problem) {
      this.say(`No answer from the robot: ${problem.message}`, 'refused');
    } finally {
      this.setRunning(false);
    }
  }

  async stop() {
    try {
      const [reply, body] = await this.post('/api/motor-test/stop', {});
      if (!reply.ok) this.say(`Stop refused: ${body.error}`, 'refused');
      else if (!body.stopped && !this.running) this.say('Nothing was running');
    } catch (problem) {
      this.say(`No answer from the robot: ${problem.message}`, 'refused');
    }
  }

  // From each robot_state and the bot's checks (health.js's model): the commanded angles, the net waist travel, the
  // status line and the last servo test's read-back.
  update(state, model) {
    if (!this.limits) return;
    const servosDown = down(model, 'servo_board'), motorsDown = down(model, 'motor_board');
    for (const name of ['pan', 'tilt']) {
      const value = state.motors?.commands?.[name]?.value;
      this.commands[name] = value ?? undefined;
      setText(this.shown[name], value == null ? '—' : `${value.toFixed(0)}°${servosDown ? ' not sent' : ''}`);
    }
    const test = state.motor_test;
    if (this.travel) setText(this.travel, test ? `waist ${signed(test.waist_travel)}${motorsDown ? ' not sent' : ''}` : '');
    if (test?.last?.angle != null && !this.running) this.lastServo = test.last.actuator;  // Also a test from another screen
    this.showStatus(test, [servosDown, motorsDown].filter(Boolean));
    this.showReadback(model);
  }

  // The status line: this page's own answer for a few seconds, else what the bot says about its tests.
  showStatus(test, boards) {
    if (this.running) return;  // The test this page sent says what it is doing until its answer
    if (this.answer && performance.now() < this.answer.until) return;
    this.answer = null;
    if (!test) this.show('Not available in this run');
    else if (!test.enabled) this.show('Turned off in the launch configuration');
    else if (test.running) this.show(`Running: ${test.running.description}`);
    // Not "Ready" while a board does not answer: the bot refuses that board's tests, so say why in its words.
    else if (boards.length) this.show(boards.map(part => part.message || `${part.name}: ${part.word}`).join(' · '), 'refused');
    else if (test.last) this.show(`${capital(test.last.outcome)}: ${test.last.description}`, test.last.outcome === 'done' ? 'done' : 'refused');
    else this.show('Ready');
  }

  // After a servo test, a line only when the servo board's read-back shows a problem; the whole read-back on hover.
  showReadback(model) {
    const said = this.lastServo ? readbackOf(model, this.lastServo) : null;
    const problem = said?.[2] === 'refused';
    setProperty(this.readback, 'hidden', !problem);
    if (!problem) return;
    const [words, hint] = said;
    setText(this.readback, words);
    setProperty(this.readback, 'title', hint ? `${words}\n${hint}` : words);
  }
}
