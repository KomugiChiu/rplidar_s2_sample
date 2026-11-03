// Shared state, server connection and small helpers.

// Who fixes what.  team: nav = Nav2/SLAM team, ai = AI team, shared = platform.
export const OWNERS = {
  LIDAR: { label: 'Lidar (rplidar S2)', team: 'nav' },
  ODOM: { label: 'rf2o odometry', team: 'nav' },
  EKF: { label: 'EKF', team: 'nav' },
  SLAM: { label: 'SLAM (slam_toolbox)', team: 'nav' },
  TF: { label: 'TF tree', team: 'nav' },
  NAV2: { label: 'Nav2', team: 'nav' },
  CHASSIS: { label: 'Chassis driver', team: 'nav' },
  CAMERA: { label: 'RealSense D455', team: 'shared' },
  SYSTEM: { label: 'Board', team: 'shared' },
  CONSOLE: { label: 'Console', team: 'shared' },
  TOOLS: { label: 'Tools', team: 'shared' },
  AI: { label: 'AI detection', team: 'ai' },
  DEMO: { label: 'demo_manager', team: 'ai' },
};
export const TEAMS = { nav: 'Nav2 / SLAM team', ai: 'AI team', shared: 'Shared / platform' };
export const LEVEL_RANK = { ERROR: 3, WARN: 2, OK: 1, INFO: 1, OFF: 0, IDLE: 0 };

const TOPIC_OWNERS = [
  [/^\/scan/, 'LIDAR'], [/^\/odom_rf2o/, 'ODOM'], [/^\/odometry|^\/diagnostics/, 'EKF'],
  [/^\/map|^\/slam_toolbox/, 'SLAM'], [/^\/tf/, 'TF'], [/^\/camera/, 'CAMERA'],
  [/^\/person_infos|^\/human_detections|^\/debug_/, 'AI'], [/^\/demo|^\/clicked_point/, 'DEMO'],
  [/^\/ultrasonic|^\/line_sensor|^\/rgblight|^\/buzzer|^\/servo/, 'CHASSIS'],
  [/^\/cmd_vel$/, 'CHASSIS'],
  [/^\/cmd_vel|^\/plan|costmap|^\/navigate|^\/follow|^\/behavior|^\/collision|^\/bond|^\/goal_pose|^\/initialpose|^\/speed_limit|^\/route|^\/dock|^\/compute|^\/smooth|^\/spin|^\/backup|^\/wait|^\/drive_on|^\/assisted|^\/preempt|transition_event|_selector/, 'NAV2'],
  [/^\/rosout|^\/parameter_events/, ''],
];
export function topicOwner(t) {
  for (const [re, o] of TOPIC_OWNERS) if (re.test(t)) return o;
  return '';
}

export const S = {
  ch: {},            // latest value per state channel
  logs: [],          // log entries (with _seq)
  events: [],
  logSeq: 0,
  trail: [],
  sid: null,
  ip: '',
  connected: false,
};

const listeners = {};
export function on(ch, fn) { (listeners[ch] ||= []).push(fn); }
function fire(ch, d) {
  for (const fn of listeners[ch] || []) { try { fn(d); } catch (e) { console.error(ch, e); } }
  for (const fn of listeners['*'] || []) { try { fn(ch, d); } catch (e) { console.error(e); } }
}

// Channels every page needs; tabs add more with setChannels().
export const BASE_CHANNELS = ['sys', 'health', 'motion', 'events', 'pose', 'log', 'rates', 'lifecycle',
  'diag', 'tf', 'script', 'logstats'];
let wanted = new Set(BASE_CHANNELS);
let es = null;

export function connect() {
  es = new EventSource('/api/events?ch=' + encodeURIComponent(BASE_CHANNELS.join(',')));
  es.onmessage = (e) => {
    let o;
    try { o = JSON.parse(e.data); } catch { return; }
    if (o.ch === 'hello') {
      S.sid = o.d.sid; S.ip = o.d.ip;
      // the server resends its backlog to every new session
      S.logs = []; S.events = [];
      setConn(true);
      resubscribe();
      return;
    }
    handle(o.ch, o.d);
  };
  es.onerror = () => setConn(false);
}

function setConn(ok) {
  S.connected = ok;
  fire('conn', ok);
}

export function setChannels(extra) {
  wanted = new Set([...BASE_CHANNELS, ...extra]);
  resubscribe();
}
let subTimer = 0;
function resubscribe() {
  clearTimeout(subTimer);
  subTimer = setTimeout(() => {
    if (S.sid) post('/api/subscribe', { sid: S.sid, ch: [...wanted] });
  }, 50);
}

function handle(ch, d) {
  if (ch === 'log') {
    for (const it of d.items) { it._seq = ++S.logSeq; S.logs.push(it); }
    if (S.logs.length > 5000) S.logs.splice(0, S.logs.length - 4000);
  } else if (ch === 'events') {
    S.events.push(...d.items);
    if (S.events.length > 400) S.events.splice(0, S.events.length - 300);
  } else {
    S.ch[ch] = d;
    if (ch === 'pose' && d.ok && d.p) {
      const last = S.trail[S.trail.length - 1];
      if (!last || Math.hypot(last[0] - d.p[0], last[1] - d.p[1]) > 0.03) {
        S.trail.push([d.p[0], d.p[1]]);
        if (S.trail.length > 600) S.trail.shift();
      }
    }
  }
  fire(ch, d);
}

export async function post(url, body) {
  try {
    const r = await fetch(url, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok && j.ok === undefined) j.ok = false;
    return j;
  } catch (e) {
    return { ok: false, msg: String(e) };
  }
}

// Command to the ROS gateway with a toast for the result.
export async function cmd(body, okText) {
  const r = await post('/api/cmd', body);
  toast(r.ok ? (okText || r.msg || 'done') : (r.msg || 'failed'), r.ok ? 'OK' : 'ERROR');
  return r;
}

export function toast(text, lvl = 'INFO', ms = 3500) {
  const box = document.getElementById('toasts');
  const t = document.createElement('div');
  t.className = 'toast ' + lvl;
  t.textContent = text;
  box.appendChild(t);
  setTimeout(() => t.remove(), ms);
}

export function confirmDialog(title, body, okLabel = 'OK') {
  const dlg = document.getElementById('dlg');
  document.getElementById('dlgTitle').textContent = title;
  document.getElementById('dlgBody').innerHTML = body;
  document.getElementById('dlgOk').textContent = okLabel;
  return new Promise((res) => {
    dlg.onclose = () => res(dlg.returnValue === 'ok');
    dlg.showModal();
  });
}

// ---------------------------------------------------------------- formatting
const ESC = { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' };
export const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ESC[c]);
export const $ = (sel, root = document) => root.querySelector(sel);
export const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

export function f(n, d = 2) {
  if (n === null || n === undefined || Number.isNaN(n)) return '–';
  return Number(n).toFixed(d);
}
export function fAge(s) {
  if (s === null || s === undefined) return 'never';
  if (s < 1) return (s * 1000).toFixed(0) + ' ms';
  if (s < 90) return s.toFixed(1) + ' s';
  if (s < 5400) return (s / 60).toFixed(0) + ' min';
  return (s / 3600).toFixed(1) + ' h';
}
export function fBytes(b) {
  if (b === null || b === undefined) return '–';
  if (b < 1024) return b + ' B';
  if (b < 1048576) return (b / 1024).toFixed(1) + ' KB';
  return (b / 1048576).toFixed(1) + ' MB';
}
export function fTime(t) {
  const d = new Date(t * 1000);
  return d.toTimeString().slice(0, 8);
}
export function fTimeMs(t) {
  const d = new Date(t * 1000);
  return d.toTimeString().slice(0, 8) + '.' + String(d.getMilliseconds()).padStart(3, '0');
}
export function fDeg(rad) { return rad === null || rad === undefined ? '–' : (rad * 180 / Math.PI).toFixed(0) + '°'; }
export function fVec(v) {
  if (!v) return '–';
  const parts = [`vx ${f(v[0], 2)}`];
  if (Math.abs(v[1] || 0) > 1e-3) parts.push(`vy ${f(v[1], 2)}`);
  parts.push(`wz ${f(v[2], 2)}`);
  return parts.join(' ');
}
export const isZero = (v) => !v || v.every((x) => Math.abs(x || 0) < 1e-3);

export function ownerBadge(o) {
  if (!o) return '';
  const m = OWNERS[o] || { label: o, team: 'shared' };
  return `<span class="owner team-${m.team}" title="${esc(TEAMS[m.team])}">${esc(o)}</span>`;
}
export const pill = (lvl, text) => `<span class="pill lv-${esc(lvl)}">${esc(text ?? lvl)}</span>`;

export function worst(levels) {
  let w = 'OFF';
  for (const l of levels) if ((LEVEL_RANK[l] ?? 0) > (LEVEL_RANK[w] ?? 0)) w = l;
  return w;
}

// All checks: ROS-side health (gateway) + board/process checks (Go).
export function allChecks() {
  return [...(S.ch.health || []), ...((S.ch.sys && S.ch.sys.checks) || [])];
}

export function store(key, val) {
  try {
    if (val === undefined) return JSON.parse(localStorage.getItem('rc.' + key));
    localStorage.setItem('rc.' + key, JSON.stringify(val));
  } catch { return undefined; }
  return val;
}

// rAF-throttled render helper
export function throttled(fn) {
  let pending = false;
  return () => {
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => { pending = false; fn(); });
  };
}
