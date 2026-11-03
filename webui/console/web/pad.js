// Manual drive: on-screen pad + keyboard.  While a direction is held the page
// POSTs /api/teleop every 100 ms; the gateway republishes at 20 Hz and stops
// by itself 0.5 s after the last keepalive (deadman), so a closed tab, lost
// Wi-Fi or a crashed browser always stops the robot.
import { post, store, esc, toast } from './core.js';

const DIRS = {
  fl: [1, 0, 1], f: [1, 0, 0], fr: [1, 0, -1],
  rl: [0, 0, 1], stop: null, rr: [0, 0, -1],
  bl: [-1, 0, -1], b: [-1, 0, 0], br: [-1, 0, 1],
  sl: [0, 1, 0], sr: [0, -1, 0],
};
const LABELS = {
  fl: ['↖', 'fwd-left'], f: ['↑', 'forward'], fr: ['↗', 'fwd-right'],
  rl: ['⟲', 'rotate'], stop: ['STOP', ''], rr: ['⟳', 'rotate'],
  bl: ['↙', 'back-left'], b: ['↓', 'back'], br: ['↘', 'back-right'],
  sl: ['⇠ strafe', ''], sr: ['strafe ⇢', ''],
};

export const speeds = {
  lin: store('lin') ?? 0.15,
  ang: store('ang') ?? 0.9,
};

const Tele = {
  vec: null,
  timer: 0,
  inflight: false,
  failed: 0,
  start(vec) {
    this.vec = vec;
    this.send();
    if (!this.timer) this.timer = setInterval(() => this.send(), 100);
  },
  stop() {
    if (this.timer) { clearInterval(this.timer); this.timer = 0; }
    if (this.vec) {
      this.vec = null;
      // explicit zero; the deadman would stop it anyway
      post('/api/teleop', { vx: 0, vy: 0, wz: 0 });
    }
    document.querySelectorAll('.pad button.held').forEach((b) => b.classList.remove('held'));
  },
  async send() {
    if (!this.vec || this.inflight) return;
    const [a, b, c] = this.vec;
    this.inflight = true;
    const r = await post('/api/teleop', { vx: a * speeds.lin, vy: b * speeds.lin, wz: c * speeds.ang });
    this.inflight = false;
    if (!r.ok && this.vec) {
      if (++this.failed === 1) toast('Manual drive failed: ' + (r.msg || 'no answer'), 'ERROR');
    } else {
      this.failed = 0;
    }
  },
};

export function stopDrive() { Tele.stop(); }

export function createPad(root) {
  const order = ['fl', 'f', 'fr', 'rl', 'stop', 'rr', 'bl', 'b', 'br'];
  root.innerHTML = `<div class="pad">${order.map((k) =>
    `<button data-dir="${k}" class="${k === 'stop' ? 'stop' : ''}">${esc(LABELS[k][0])}<small>${esc(LABELS[k][1])}</small></button>`).join('')}
    <div class="strafe"><button data-dir="sl">${LABELS.sl[0]}</button><button data-dir="sr">${LABELS.sr[0]}</button></div>
  </div>`;
  for (const btn of root.querySelectorAll('button[data-dir]')) {
    const dir = btn.dataset.dir;
    btn.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      if (dir === 'stop') { Tele.stop(); return; }
      btn.classList.add('held');
      Tele.start(DIRS[dir]);
    });
    for (const ev of ['pointerup', 'pointercancel', 'pointerleave']) {
      btn.addEventListener(ev, () => { if (dir !== 'stop') Tele.stop(); });
    }
    btn.addEventListener('contextmenu', (e) => e.preventDefault());
  }
}

export function createSpeeds(root) {
  root.innerHTML = `<div class="speeds">
    <label for="spLin">Linear</label><input id="spLin" type="range" min="0.10" max="0.35" step="0.01" value="${speeds.lin}"><span class="mono" id="spLinV"></span>
    <label for="spAng">Turn</label><input id="spAng" type="range" min="0.3" max="1.5" step="0.05" value="${speeds.ang}"><span class="mono" id="spAngV"></span>
  </div>
  <div class="muted small">This chassis needs ≥ 0.14 m/s to move and ≈ 0.9 rad/s to turn (open-loop PWM); strafing is unreliable.</div>`;
  const lin = root.querySelector('#spLin'), ang = root.querySelector('#spAng');
  const show = () => {
    root.querySelector('#spLinV').textContent = Number(lin.value).toFixed(2);
    root.querySelector('#spAngV').textContent = Number(ang.value).toFixed(2);
  };
  lin.oninput = () => { speeds.lin = Number(lin.value); store('lin', speeds.lin); show(); };
  ang.oninput = () => { speeds.ang = Number(ang.value); store('ang', speeds.ang); show(); };
  show();
}

// Keyboard: only while `enabled()` (Drive tab or map pad visible).
export function bindKeys(enabled) {
  const keys = new Set();
  const MAP = {
    ArrowUp: 'f', w: 'f', ArrowDown: 'b', s: 'b', ArrowLeft: 'rl', a: 'rl', ArrowRight: 'rr', d: 'rr',
    q: 'sl', e: 'sr',
  };
  const update = () => {
    if (!keys.size) { Tele.stop(); return; }
    const v = [0, 0, 0];
    for (const k of keys) DIRS[k].forEach((x, i) => { v[i] += x; });
    v.forEach((x, i) => { v[i] = Math.max(-1, Math.min(1, x)); });
    Tele.start(v);
  };
  window.addEventListener('keydown', (e) => {
    if (!enabled() || e.target.closest('input, select, textarea')) return;
    if (e.key === ' ' || e.key === 'Escape') { keys.clear(); Tele.stop(); e.preventDefault(); return; }
    const k = MAP[e.key] || MAP[e.key.toLowerCase()];
    if (!k || e.repeat) { if (k) e.preventDefault(); return; }
    e.preventDefault();
    keys.add(k);
    update();
  });
  window.addEventListener('keyup', (e) => {
    const k = MAP[e.key] || MAP[e.key.toLowerCase()];
    if (k && keys.delete(k)) update();
  });
  const release = () => { keys.clear(); Tele.stop(); };
  window.addEventListener('blur', release);
  document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
}
