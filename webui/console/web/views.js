// Tab renderers.  Each render*() reads S.ch and rewrites its own DOM.
import {
  S, OWNERS, TEAMS, LEVEL_RANK, $, $$, esc, f, fAge, fBytes, fTime, fTimeMs, fDeg, fVec,
  ownerBadge, pill, worst, allChecks, topicOwner, cmd, post, toast, confirmDialog, setChannels, store, on,
} from './core.js';
import { MapView } from './map.js';
import { createPad, createSpeeds, speeds } from './pad.js';

const check = (id) => allChecks().find((c) => c.id === id);
const rate = (t) => (S.ch.rates || {})[t] || {};
const BAD_DEMO = ['TARGET_LOST', 'NAVIGATION_FAILED'];

// ============================================================== motion chain
export function motionVerdict(m) {
  if (!m) return { lvl: 'OFF', text: 'No data from the ROS gateway yet', owner: 'CONSOLE' };
  if (m.estop) return { lvl: 'ERROR', text: 'E-STOP active — zero velocity is being published', owner: 'CONSOLE' };
  if (m.teleop) return { lvl: 'INFO', text: `Manual override from ${m.teleop.src} (${fVec(m.teleop.v)})`, owner: 'CONSOLE' };
  const active = m.nav.active.length > 0;
  const moving = m.odom && m.odom.v && (Math.abs(m.odom.v[0]) > 0.02 || Math.abs(m.odom.v[2]) > 0.05);
  const recent = (st) => st && st.nz !== null && st.nz < 1.0;
  const bl = m.blockers || [];
  const demo = m.demo || {};
  if (!active) {
    if (demo.pubs && demo.state === 'FOLLOWING') {
      return { lvl: 'WARN', owner: 'DEMO',
        text: 'demo_manager is FOLLOWING but no Nav2 goal is active (person within follow distance, lost, or goal not sent)' };
    }
    if (demo.pubs && BAD_DEMO.includes(demo.state)) {
      return { lvl: 'WARN', owner: 'DEMO', text: `Idle — demo_manager is in ${demo.state} and stopped sending goals` };
    }
    if (moving) return { lvl: 'WARN', owner: 'CHASSIS', text: 'No goal and no command, but odometry reports motion (pushed? drift?)' };
    return { lvl: 'IDLE', text: 'Idle — no Nav2 goal active', owner: '' };
  }
  const nc = m.nav_cmd;
  if (!nc || nc.age === null || nc.age > 1.0) {
    return { lvl: 'ERROR', owner: 'NAV2', text: 'Goal active but controller_server is not publishing /cmd_vel_nav' };
  }
  if (!recent(nc)) {
    const b = bl[0];
    return { lvl: 'WARN', owner: b ? b.owner : 'NAV2',
      text: 'Controller outputs zero' + (b ? ` — ${b.label}` : ' (settle pause between pulses, or blocked — see logs)') };
  }
  if (recent(nc) && !recent(m.smoothed)) {
    return { lvl: 'WARN', owner: 'NAV2', text: 'velocity_smoother output is zero although the controller commands motion' };
  }
  const col = m.collision;
  if ((col && col.action !== 'DO_NOTHING') || (recent(m.smoothed) && !recent(m.out))) {
    const b = bl.find((x) => x.label.startsWith('collision')) || bl[0];
    return { lvl: 'WARN', owner: 'NAV2',
      text: `collision_monitor is holding the robot (${col ? col.action : 'no output'})` + (b ? ` — ${b.label}` : '') };
  }
  if (recent(m.out) && !moving) {
    return { lvl: 'ERROR', owner: 'CHASSIS',
      text: 'Commands reach the driver but odometry shows no motion → chassis / battery / below PWM deadband / rf2o' };
  }
  const fb = m.nav.fb;
  return { lvl: 'OK', owner: '', text: 'Moving to goal' + (fb && fb.dist !== undefined ? ` — ${f(fb.dist)} m left` : '') };
}

function stageLevel(st, active) {
  if (!active) return 'IDLE';
  if (!st || st.age === null || st.age > 1.0) return 'ERROR';
  return st.nz !== null && st.nz < 1.0 ? 'OK' : 'WARN';
}

function motionStages(m) {
  const out = [];
  const demo = m.demo || {};
  out.push({ t: 'demo_manager', o: 'DEMO',
    lvl: !demo.pubs ? 'OFF' : (BAD_DEMO.includes(demo.state) ? 'WARN' : 'OK'),
    v: demo.pubs ? (demo.state || '?') : 'not running', s: demo.pubs ? `selected ID ${demo.sel}` : 'AI stack not started' });
  const nav = m.nav;
  const active = nav.active.length > 0;
  const fb = nav.fb;
  let s = Object.entries(nav.goals || {}).map(([a, st]) => `${a.replace('navigate_', '').replace('_', ' ')}: ${st}`).join(' · ');
  if (fb) {
    s = [fb.dist !== undefined ? `${f(fb.dist)} m left` : '', fb.rec !== undefined ? `recoveries ${fb.rec}` : '',
      fb.wp !== undefined ? `waypoint ${fb.wp + 1}` : '', fb.elapsed !== undefined ? `${f(fb.elapsed, 0)} s` : '']
      .filter(Boolean).join(' · ');
  }
  out.push({ t: 'Nav2 goal', o: 'NAV2', lvl: active ? 'OK' : 'IDLE', v: active ? nav.active.join(', ') : 'none', s });
  const recov = (m.bt || []).some((n) => /BackUp|Wait|Spin|Clear/.test(n));
  out.push({ t: 'Behavior tree', o: 'NAV2', lvl: active ? (recov ? 'WARN' : 'OK') : 'IDLE',
    v: (m.bt || []).filter((n) => !/Recovery|Pipeline|Sequence|Fallback|RateController|Selector|RoundRobin/.test(n)).join(' › ') || '–',
    s: recov ? 'recovery running' : '' });
  const tw = (name, key, owner) => {
    const st = m[key];
    out.push({ t: name, o: owner, lvl: stageLevel(st, active), v: st ? fVec(st.v) : '–',
      s: st ? `${f(st.hz, 1)} Hz` + (st.nz !== null ? ` · ≠0 ${fAge(st.nz)} ago` : '') : '' });
  };
  tw('Controller /cmd_vel_nav', 'nav_cmd', 'NAV2');
  tw('Smoother /cmd_vel_smoothed', 'smoothed', 'NAV2');
  const col = m.collision;
  out.push({ t: 'collision_monitor', o: 'NAV2', lvl: !col ? 'OFF' : (col.action === 'DO_NOTHING' ? (active ? 'OK' : 'IDLE') : 'WARN'),
    v: col ? col.action : '–', s: col && col.poly ? col.poly : '' });
  const o = m.out;
  out.push({ t: 'Driver /cmd_vel', o: 'CHASSIS',
    lvl: m.teleop ? 'INFO' : (o && o.nz !== null && o.nz < 1 ? 'OK' : (active ? 'WARN' : 'IDLE')),
    v: o ? fVec(o.v) : '–', s: m.teleop ? 'manual override' : (o ? `${f(o.hz, 1)} Hz · ${o.pubs} publishers` : '') });
  const od = m.odom;
  const moving = od && od.v && (Math.abs(od.v[0]) > 0.02 || Math.abs(od.v[2]) > 0.05);
  out.push({ t: 'Actual (EKF)', o: 'EKF', lvl: !od ? 'ERROR' : (moving ? 'OK' : 'IDLE'), v: od ? fVec(od.v) : 'no odometry',
    s: od ? `age ${fAge(od.age)}` : '' });
  const so = m.sonar;
  out.push({ t: 'Ultrasonic', o: 'CHASSIS', lvl: !so ? 'OFF' : (so.d === 0 ? 'WARN' : (so.d < 0.2 ? 'WARN' : 'OK')),
    v: !so ? '–' : (so.d === 0 ? '0 (invalid)' : `${f(so.d, 2)} m`), s: so ? `age ${fAge(so.age)}` : '' });
  return out;
}

const stageHtml = (st) => `<div class="stage ${esc(st.lvl)}">
  <div class="t"><span>${esc(st.t)}</span>${ownerBadge(st.o)}</div>
  <div class="v">${esc(st.v)}</div>${st.s ? `<div class="s">${esc(st.s)}</div>` : ''}</div>`;

const blockersHtml = (bl) => (bl && bl.length ? `<div class="blockers">${bl.map((b) =>
  `<div class="blocker">${ownerBadge(b.owner)} <b>${esc(b.label)}</b> <span class="muted">${fAge(b.age)} ago · ${b.n}× total</span><code>${esc(b.text)}</code></div>`).join('')}</div>` : '');

const verdictHtml = (v) => `${pill(v.lvl === 'IDLE' ? 'OFF' : v.lvl, v.lvl)} <span>${esc(v.text)}</span> ${ownerBadge(v.owner)}`;

// ================================================================ overview
const TEAM_OWNERS = {
  nav: ['LIDAR', 'ODOM', 'EKF', 'SLAM', 'TF', 'NAV2', 'CHASSIS'],
  ai: ['AI', 'DEMO'],
  shared: ['CAMERA', 'SYSTEM', 'CONSOLE'],
};
const expanded = new Set(store('expanded') || []);

export function ownerLevels() {
  const out = {};
  for (const c of allChecks()) {
    const o = c.owner || 'SYSTEM';
    (out[o] ||= []).push(c);
  }
  return out;
}

export function renderOverview() {
  const m = S.ch.motion;
  $('#ovVerdict').innerHTML = verdictHtml(motionVerdict(m));
  $('#ovPipeline').innerHTML = m ? motionStages(m).map(stageHtml).join('') : '';
  $('#ovBlockers').innerHTML = m ? blockersHtml(m.blockers) : '';

  const by = ownerLevels();
  $('#ovOwners').innerHTML = Object.entries(TEAM_OWNERS).map(([team, owners]) => `
    <div class="team"><div class="team-title team-${team}">${esc(TEAMS[team])}</div><div class="owners">
    ${owners.map((o) => {
      const cs = (by[o] || []).slice().sort((a, b) => (LEVEL_RANK[b.level] ?? 0) - (LEVEL_RANK[a.level] ?? 0));
      const lvl = cs.length ? worst(cs.map((c) => c.level)) : 'OFF';
      const open = expanded.has(o);
      const bad = cs.filter((c) => c.level !== 'OK');
      const shown = open ? cs : bad.slice(0, 4);
      const okN = cs.filter((c) => c.level === 'OK').length;
      return `<div class="ocard ${lvl}" data-owner="${o}">
        <div class="h"><span>${esc(OWNERS[o].label)}</span>${pill(lvl)}</div>
        <ul>${shown.map((c) => `<li><span class="dot ${esc(c.level)}"></span><span class="nm">${esc(c.name)}
          ${c.detail && (open || c.level !== 'OK') ? `<span class="det">${esc(c.detail)}</span>` : ''}</span>
          <span class="val">${esc(c.value)}</span></li>`).join('')}</ul>
        ${!open && okN ? `<div class="muted small">${okN} check${okN > 1 ? 's' : ''} OK${bad.length > 4 ? ` · ${bad.length - 4} more issues` : ''} · tap for details</div>` : ''}
        ${!cs.length ? '<div class="muted small">no data</div>' : ''}
      </div>`;
    }).join('')}</div></div>`).join('');

  const p = S.ch.pose;
  const nav = m ? m.nav : null;
  const map = rate('/map');
  $('#ovRobot').innerHTML = `<div class="kv mono small">
    <span>Pose (map)</span><span>${p && p.ok ? `x ${f(p.p[0])}  y ${f(p.p[1])}  θ ${fDeg(p.p[2])}` : 'no map→base_link TF'}</span>
    <span>Velocity</span><span>${p && p.v ? fVec(p.v) : '–'}</span>
    <span>Ultrasonic</span><span>${m && m.sonar ? (m.sonar.d === 0 ? '0 (invalid reading)' : f(m.sonar.d) + ' m') : '–'}</span>
    <span>Navigation</span><span>${nav && nav.active.length ? esc(nav.active.join(', ')) + (nav.fb && nav.fb.dist !== undefined ? ` · ${f(nav.fb.dist)} m left` : '') : 'idle'}</span>
    <span>Console mission</span><span>${nav && nav.mission ? esc(nav.mission.label + ' · ' + nav.mission.status) : '–'}</span>
    <span>demo_manager</span><span>${m && m.demo.pubs ? esc(m.demo.state) + ' · ID ' + m.demo.sel : 'not running'}</span>
    <span>Map update</span><span>${map.age !== undefined ? fAge(map.age) + ' ago' : '–'}</span>
  </div>`;
  $('#ovBoard').innerHTML = boardHtml(true);
  const evs = S.events.slice(-40).reverse();
  $('#ovEvents').innerHTML = evs.length ? evs.map((e) => `<div class="ev"><time>${fTime(e.t)}</time>
    <span>${pill(e.level === 'INFO' ? 'INFO' : e.level, e.level)} ${ownerBadge(e.owner)}</span><span>${esc(e.text)}</span></div>`).join('')
    : '<div class="muted">no events yet</div>';
}

export function bindOverview(rerender) {
  $('#ovOwners').addEventListener('click', (e) => {
    const card = e.target.closest('.ocard');
    if (!card) return;
    const o = card.dataset.owner;
    expanded.has(o) ? expanded.delete(o) : expanded.add(o);
    store('expanded', [...expanded]);
    rerender();
  });
}

function bar(pct, warn = 75, err = 85) {
  const lvl = pct > err ? 'ERROR' : (pct > warn ? 'WARN' : '');
  return `<div class="bar ${lvl}"><i style="width:${Math.max(0, Math.min(100, pct || 0)).toFixed(0)}%"></i></div>`;
}

function boardHtml(compact) {
  const s = S.ch.sys;
  if (!s) return '<div class="muted">waiting…</div>';
  const cpu = s.cpu || {};
  const mem = s.mem || {};
  const tmax = Math.max(0, ...(s.temps || []).map((t) => t.c));
  const tmp = (s.disks || []).find((d) => d.path === '/tmp');
  return `<div class="small">CPU ${f(cpu.total, 0)} % <span class="muted">(4 cores; load avg ${esc(s.loadavg || '–')} is not meaningful on this SoC)</span></div>
    ${bar(cpu.total)}
    <div class="cores">${(cpu.cores || []).map((c, i) => `<div><span>core ${i} · ${f(c, 0)}%</span>${bar(c, 90, 95)}</div>`).join('')}</div>
    <div class="kv mono small">
      <span>Memory</span><span>${f(mem.MemAvailable / 1024, 0)} MB free of ${f(mem.MemTotal / 1024, 0)} MB</span>
      <span>Temperature</span><span>${tmax ? f(tmax, 0) + ' °C' : '–'}</span>
      <span>Wi-Fi</span><span>${s.wifi ? `${esc(s.wifi.iface)} ${f(s.wifi.level, 0)} dBm` : 'n/a'}</span>
      <span>/tmp free</span><span>${tmp ? f(tmp.free_mb / 1024, 1) + ' GB' : '–'}</span>
      ${compact ? '' : `<span>Uptime</span><span>${fAge(s.uptime)}</span><span>Clock sync</span><span>${s.clock_synced ? 'NTP synced' : 'not synced'}</span>`}
    </div>`;
}

// ===================================================================== map
let mapView = null;
let pendingConfirm = null;

export function initMap() {
  const canvas = $('#mapCanvas');
  mapView = new MapView(canvas, {
    onPick: onMapPick,
    onCursor: (w) => { mapView._cursor = w; renderHud(); },
  });
  const saved = store('layers');
  if (saved) Object.assign(mapView.layers, saved);
  const LAYERS = { map: 'Map', costmap_global: 'Global costmap', costmap_local: 'Local costmap', scan: 'Laser scan',
    plan: 'Global plan', people: 'People (AI + demo)', trail: 'Trail', grid: '1 m grid' };
  $('#mapLayers').innerHTML = Object.entries(LAYERS).map(([k, l]) =>
    `<label><input type="checkbox" data-layer="${k}" ${mapView.layers[k] ? 'checked' : ''}> ${esc(l)}</label>`).join('');
  $('#mapLayers').addEventListener('change', (e) => {
    const k = e.target.dataset.layer;
    mapView.layers[k] = e.target.checked;
    store('layers', mapView.layers);
    setChannels(mapChannels());
    mapView.dirty();
  });
  $('#mapLayersBtn').onclick = () => $('#mapLayers').classList.toggle('hidden');
  $('#mapTool').addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    $$('#mapTool button').forEach((x) => x.classList.toggle('on', x === b));
    mapView.setTool(b.dataset.tool);
    if (b.dataset.tool === 'wp') renderWaypoints();
  });
  const followBtn = $('#mapFollow');
  followBtn.onclick = () => { mapView.follow = !mapView.follow; followBtn.classList.toggle('on', mapView.follow); mapView.dirty(); };
  canvas.addEventListener('follow', (e) => followBtn.classList.toggle('on', e.detail));
  $('#mapFit').onclick = () => { mapView.fit(true); followBtn.classList.remove('on'); };
  $('#mapSide').addEventListener('click', (e) => {
    const b = e.target.closest('[data-cmd]');
    if (b) cmd({ cmd: b.dataset.cmd });
  });
  $('#sheetGrip').onclick = () => $('#mapSide').classList.toggle('collapsed');
  $('#mapDriveBtn').onclick = () => {
    const pad = $('#mapPad');
    pad.classList.toggle('hidden');
    $('#mapDriveBtn').classList.toggle('on', !pad.classList.contains('hidden'));
  };
  createPad($('#mapPad'));
  $('#mapLegend').innerHTML = `<b>Legend</b><br>
    <span style="color:var(--robot)">▲</span> robot · <span style="color:var(--scan)">■</span> scan ·
    <span style="color:var(--plan)">━</span> plan · <span style="color:var(--goal)">●</span> goal ·
    <span style="color:#13b5c9">○</span> AI person (TF) · ● demo_manager marker ·
    <span style="color:#ff006e">■</span> lethal · <span style="color:#a050ff">■</span> inscribed`;

  S._pending = store('pendingWps') || [];
  mapView.set('pending', S._pending);
  for (const ch of ['map', 'costmap_local', 'costmap_global']) on(ch, (g) => mapView.setGrid(ch, g));
}

export function mapChannels() {
  if (!mapView) return ['map', 'scan', 'plan', 'people', 'costmap_local'];
  const l = mapView.layers;
  return ['map', 'scan', 'plan', 'people', ...(l.costmap_local ? ['costmap_local'] : []),
    ...(l.costmap_global ? ['costmap_global'] : [])];
}

export function mapUpdate(ch, d) {
  if (!mapView) return;
  if (ch === 'scan') mapView.set('scan', d);
  else if (ch === 'plan') mapView.set('plan', d);
  else if (ch === 'people') mapView.set('people', d);
  else if (ch === 'pose') { mapView.d.trail = S.trail; mapView.set('pose', d); renderHud(); }
  else if (ch === 'motion') {
    mapView.d.mission = d.nav.mission;
    mapView.set('extGoal', d.nav.ext_goal);
  }
}
export function mapResize() { if (mapView) mapView.resize(); }
export function mapRetheme() { if (mapView) mapView.retheme(); }

function renderHud() {
  const p = S.ch.pose;
  const c = mapView._cursor;
  $('#mapHud').textContent = (p && p.ok ? `robot ${f(p.p[0])}, ${f(p.p[1])} ${fDeg(p.p[2])}` : 'robot: no TF') +
    (c ? `  ·  cursor ${f(c[0])}, ${f(c[1])}` : '');
}

function onMapPick(tool, x, y, yaw) {
  const box = $('#mapConfirm');
  const label = `(${f(x)}, ${f(y)}, ${fDeg(yaw)})`;
  if (tool === 'wp') {
    S._pending.push([x, y, yaw]);
    store('pendingWps', S._pending);
    mapView.d.preview = null;
    mapView.set('pending', S._pending);
    renderWaypoints();
    return;
  }
  pendingConfirm = { tool, x, y, yaw };
  box.innerHTML = tool === 'goal'
    ? `<span>Send Nav2 goal <b class="mono">${label}</b>?</span><button class="btn primary" data-act="ok">Send goal</button><button class="btn" data-act="no">Cancel</button>`
    : `<span>Publish /initialpose <b class="mono">${label}</b>? <span class="muted small">(AMCL mode only)</span></span><button class="btn primary" data-act="ok">Publish</button><button class="btn" data-act="no">Cancel</button>`;
  box.classList.remove('hidden');
  box.onclick = async (e) => {
    const act = e.target.closest('[data-act]');
    if (!act) return;
    box.classList.add('hidden');
    mapView.d.preview = null;
    mapView.dirty();
    if (act.dataset.act !== 'ok' || !pendingConfirm) return;
    const pc = pendingConfirm;
    pendingConfirm = null;
    await cmd({ cmd: pc.tool === 'goal' ? 'goal' : 'initialpose', x: pc.x, y: pc.y, yaw: pc.yaw });
  };
}

function renderWaypoints() {
  const wps = S._pending || [];
  $('#mapWps').innerHTML = `<h3>Waypoints <span class="muted small">(📍 tool: press, drag for heading)</span></h3>
    ${wps.length ? `<ol class="wp-list">${wps.map((w, i) => `<li><span>${i + 1}. ${f(w[0])}, ${f(w[1])}, ${fDeg(w[2])}</span><button data-rm="${i}" title="remove">✕</button></li>`).join('')}</ol>` : '<div class="muted small">none queued</div>'}
    <div class="row gap"><button class="btn primary" id="wpGo" ${wps.length ? '' : 'disabled'}>Go (${wps.length})</button>
    <button class="btn" id="wpClear" ${wps.length ? '' : 'disabled'}>Clear</button></div>`;
  $('#mapWps').onclick = async (e) => {
    const rm = e.target.closest('[data-rm]');
    if (rm) { wps.splice(Number(rm.dataset.rm), 1); }
    else if (e.target.id === 'wpClear') { wps.length = 0; }
    else if (e.target.id === 'wpGo') {
      const r = await cmd({ cmd: 'waypoints', poses: wps.map((w) => [w[0], w[1], w[2]]) }, `${wps.length} waypoints sent`);
      if (r.ok) wps.length = 0;
    } else return;
    store('pendingWps', wps);
    mapView.set('pending', wps);
    renderWaypoints();
  };
}

export function renderMapSide() {
  const m = S.ch.motion;
  if (!m) return;
  const v = motionVerdict(m);
  const nav = m.nav;
  const fb = nav.fb || {};
  $('#mapNav').innerHTML = `<h3>Navigation</h3>
    <div class="verdict small" style="font-size:13px">${verdictHtml(v)}</div>
    <div class="kv mono small">
      <span>Active</span><span>${nav.active.length ? esc(nav.active.join(', ')) : 'none'}</span>
      ${fb.dist !== undefined ? `<span>Remaining</span><span>${f(fb.dist)} m${fb.eta ? ` · ETA ${f(fb.eta, 0)} s` : ''}</span>` : ''}
      ${fb.rec !== undefined ? `<span>Recoveries</span><span>${fb.rec}</span>` : ''}
      ${fb.wp !== undefined ? `<span>Waypoint</span><span>${fb.wp + 1}${nav.mission && nav.mission.kind === 'waypoints' ? ' / ' + nav.mission.poses.length : ''}</span>` : ''}
      <span>Mission</span><span>${nav.mission ? esc(nav.mission.label + ' · ' + nav.mission.status) : '–'}</span>
      <span>Controller</span><span>${esc(fVec(m.nav_cmd && m.nav_cmd.v))}</span>
      <span>Collision</span><span>${esc(m.collision ? m.collision.action : '–')}</span>
    </div>${blockersHtml(m.blockers)}`;
  if (!$('#mapWps').innerHTML) renderWaypoints();
}

// =================================================================== camera
export class Cam {
  constructor(root, src, fps = 5) {
    this.root = root; this.wantSrc = src; this.fps = fps; this.src = null;
    root.innerHTML = '<img alt=""><canvas></canvas><div class="cam-msg">camera off</div>';
    this.img = root.querySelector('img');
    this.cv = root.querySelector('canvas');
    this.msg = root.querySelector('.cam-msg');
    this.img.onerror = () => { if (this.src) this.msg.textContent = 'no stream (camera or AI node not running?)'; };
    this.poll = 0;
  }
  start(src = this.wantSrc) {
    if (src === 'off') { this.stop(); this.msg.textContent = 'camera off'; return; }
    if (this.src === src) return;
    this.wantSrc = src; this.src = src;
    this.msg.textContent = 'connecting…';
    this.msg.classList.remove('hidden');
    this.img.src = `/api/camera?src=${src}&fps=${this.fps}&_=${Date.now()}`;
    clearInterval(this.poll);
    this.poll = setInterval(() => {
      if (this.img.naturalWidth > 1) { this.msg.classList.add('hidden'); clearInterval(this.poll); }
    }, 300);
  }
  stop() {
    if (!this.src) return;
    this.src = null;
    clearInterval(this.poll);
    // abort the MJPEG connection
    this.img.src = 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7';
    this.msg.classList.remove('hidden');
    this.msg.textContent = 'camera paused';
    this.clearBoxes();
  }
  clearBoxes() {
    const ctx = this.cv.getContext('2d');
    ctx.clearRect(0, 0, this.cv.width, this.cv.height);
  }
  boxes(det) {
    if (this.src !== 'color' || !det) { this.clearBoxes(); return; }
    const r = this.root.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.cv.width = r.width * dpr; this.cv.height = r.height * dpr;
    const ctx = this.cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, r.width, r.height);
    const iw = det.w || this.img.naturalWidth, ih = det.h || this.img.naturalHeight;
    if (!iw || !ih) return;
    const k = Math.min(r.width / iw, r.height / ih);
    const ox = (r.width - iw * k) / 2, oy = (r.height - ih * k) / 2;
    const sel = S.ch.motion && S.ch.motion.demo ? String(S.ch.motion.demo.sel) : '';
    ctx.font = '600 12px system-ui, sans-serif';
    for (const b of det.boxes || []) {
      const x = ox + (b.cx - b.w / 2) * k, y = oy + (b.cy - b.h / 2) * k;
      const isSel = b.id !== '' && b.id === sel;
      ctx.strokeStyle = isSel ? '#ffd60a' : '#13b5c9';
      ctx.lineWidth = isSel ? 3 : 2;
      ctx.strokeRect(x, y, b.w * k, b.h * k);
      const label = `${b.id ? '#' + b.id : ''} ${b.score !== null ? (b.score * 100).toFixed(0) + '%' : ''}`.trim();
      if (label) {
        const tw = ctx.measureText(label).width + 8;
        ctx.fillStyle = ctx.strokeStyle;
        ctx.fillRect(x, y - 18, tw, 18);
        ctx.fillStyle = '#000';
        ctx.fillText(label, x + 4, y - 5);
      }
    }
  }
}

// ==================================================================== drive
export let driveCam = null;
export function initDrive() {
  driveCam = new Cam($('#driveCam'), store('driveCam') || 'color');
  $$('#driveCamSrc button').forEach((b) => b.classList.toggle('on', b.dataset.src === driveCam.wantSrc));
  $('#driveCamSrc').addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    $$('#driveCamSrc button').forEach((x) => x.classList.toggle('on', x === b));
    store('driveCam', b.dataset.src);
    driveCam.wantSrc = b.dataset.src;
    driveCam.stop();
    driveCam.start(b.dataset.src);
  });
  createPad($('#drivePad'));
  createSpeeds($('#driveSpeeds'));
}

export function renderDrive() {
  const m = S.ch.motion;
  const st = $('#driveState');
  if (m && m.teleop) { st.className = 'pill lv-INFO'; st.textContent = 'MANUAL ' + (m.teleop.src || ''); }
  else if (m && m.estop) { st.className = 'pill lv-ERROR'; st.textContent = 'E-STOP'; }
  else if (m && m.nav.active.length) { st.className = 'pill lv-WARN'; st.textContent = 'Nav2 driving'; }
  else { st.className = 'pill lv-OFF'; st.textContent = 'idle'; }
  if (!m) return;
  $('#driveReadout').innerHTML = `
    <span>/cmd_vel</span><span>${fVec(m.out && m.out.v)}</span>
    <span>EKF velocity</span><span>${fVec(m.odom && m.odom.v)}</span>
    <span>Ultrasonic</span><span>${m.sonar ? (m.sonar.d === 0 ? '0 (invalid)' : f(m.sonar.d) + ' m') : '–'}</span>
    <span>Speed set</span><span>${f(speeds.lin)} m/s · ${f(speeds.ang)} rad/s</span>
    <span>Navigation</span><span>${m.nav.active.length ? esc(m.nav.active.join(', ')) + ' (will be cancelled)' : 'idle'}</span>`;
  driveCam.boxes(S.ch.detections);
}

// ======================================================================= AI
export let aiCams = null;
export function initAI() {
  aiCams = { color: new Cam($('#aiCamColor'), 'color', 5), debug: new Cam($('#aiCamDebug'), 'debug', 3) };
}

function aiStages() {
  const people = S.ch.people || {};
  const m = S.ch.motion || {};
  const lvlOf = (id, fallback = 'OFF') => (check(id) || {}).level || fallback;
  const r = (t) => rate(t);
  const st = [];
  const color = r('/camera/camera/color/camera_info'), depth = r('/camera/camera/aligned_depth_to_color/camera_info');
  st.push({ t: 'RealSense color', o: 'CAMERA', lvl: lvlOf('topic:/camera/camera/color/camera_info'), v: `${f(color.hz, 1)} Hz`, s: 'expected 30 Hz' });
  st.push({ t: 'Aligned depth', o: 'CAMERA', lvl: lvlOf('topic:/camera/camera/aligned_depth_to_color/camera_info'), v: `${f(depth.hz, 1)} Hz`, s: 'align_depth on CPU' });
  const pi = r('/person_infos');
  st.push({ t: 'AI /person_infos', o: 'AI', lvl: lvlOf('topic:/person_infos'), v: pi.pubs ? `${f(pi.hz, 1)} Hz` : 'not running',
    s: `${(people.ai || []).length} person(s) · age ${fAge(pi.age)}` });
  const hd = r('/human_detections');
  st.push({ t: 'AI /human_detections', o: 'AI', lvl: lvlOf('topic:/human_detections'), v: hd.pubs ? `${f(hd.hz, 1)} Hz` : 'not running',
    s: S.ch.detections ? `${(S.ch.detections.boxes || []).length} box(es)` : '' });
  const tfc = check('tf:base_link>ai_human_detection');
  st.push({ t: 'TF base_link→ai_human_detection', o: 'AI', lvl: tfc ? tfc.level : 'OFF', v: tfc ? tfc.value : 'missing',
    s: people.ai_frame ? `frame ${people.ai_frame}${people.ai_tf === false ? ' (map lookup failed)' : ''}` : '' });
  const mk = r('/demo/person_markers');
  st.push({ t: 'demo_manager markers', o: 'DEMO', lvl: !mk.pubs ? 'OFF' : (mk.hz > 0.5 ? 'OK' : 'WARN'),
    v: mk.pubs ? `${f(mk.hz, 1)} Hz` : 'not running', s: `${(people.demo || []).length} in map` });
  const demo = m.demo || {};
  st.push({ t: 'demo_manager state', o: 'DEMO', lvl: !demo.pubs ? 'OFF' : (BAD_DEMO.includes(demo.state) ? 'WARN' : 'OK'),
    v: demo.state || '–', s: `selected ID ${demo.sel ?? '–'}` });
  const nav = m.nav || { active: [] };
  st.push({ t: 'Nav2 follow goal', o: 'NAV2', lvl: nav.active.length ? 'OK' : 'IDLE', v: nav.active.length ? 'active' : 'none',
    s: nav.fb && nav.fb.dist !== undefined ? `${f(nav.fb.dist)} m left` : '' });
  return st;
}

function aiVerdict(st) {
  const people = S.ch.people || {};
  const demo = (S.ch.motion || {}).demo || {};
  const broken = st.slice(0, 7).find((s) => s.lvl === 'ERROR' || (s.lvl === 'OFF' && s.o !== 'NAV2'));
  if (broken) return { lvl: broken.lvl === 'OFF' ? 'WARN' : 'ERROR', owner: broken.o, text: `Breaks at “${broken.t}”: ${broken.v}` };
  const warn = st.slice(0, 7).find((s) => s.lvl === 'WARN');
  const ids = (people.ai || []).map((p) => p.id);
  if (demo.state === 'FOLLOWING' && demo.sel >= 0 && !ids.includes(demo.sel)) {
    return { lvl: 'WARN', owner: 'AI', text: `Selected ID ${demo.sel} is not in the current AI output (lost or tracking ID changed)` };
  }
  if (warn) return { lvl: 'WARN', owner: warn.o, text: `Degraded at “${warn.t}”: ${warn.v} ${warn.s}` };
  if (!ids.length) return { lvl: 'OK', owner: '', text: 'Chain healthy — nobody detected right now' };
  return { lvl: 'OK', owner: '', text: `Chain healthy — ${ids.length} person(s) detected` };
}

export function renderAI() {
  const st = aiStages();
  $('#aiVerdict').innerHTML = verdictHtml(aiVerdict(st));
  $('#aiPipeline').innerHTML = st.map(stageHtml).join('');
  aiCams.color.boxes(S.ch.detections);
  const people = S.ch.people || { ai: [], demo: [] };
  const demoById = Object.fromEntries((people.demo || []).map((d) => [d.id, d]));
  const ids = [...new Set([...(people.ai || []).map((p) => p.id), ...(people.demo || []).map((d) => d.id)])];
  $('#aiPeopleMeta').textContent = `AI frame ${people.ai_frame || '–'} · age ${fAge(people.ai_age)}`;
  $('#aiPeople').innerHTML = ids.length ? `<table class="tbl"><thead><tr><th>ID</th><th>conf</th><th>tracked</th>
    <th>camera xyz (m)</th><th>range</th><th>map xy via TF</th><th>demo_manager marker</th></tr></thead><tbody>
    ${ids.map((id) => {
      const a = (people.ai || []).find((p) => p.id === id);
      const d = demoById[id];
      const sel = id === people.sel;
      return `<tr class="${sel ? 'sel' : ''}"><td class="num">${sel ? '★ ' : ''}${id}</td>
        <td class="num">${a ? f(a.conf, 2) : '–'}</td><td>${a ? (a.tracked ? 'yes' : 'no') : '–'}</td>
        <td class="num">${a ? a.cam.map((x) => f(x, 2)).join(', ') : '<span class="muted">not in AI output</span>'}</td>
        <td class="num">${a ? f(a.range, 2) : '–'}</td>
        <td class="num">${a && a.map ? f(a.map[0]) + ', ' + f(a.map[1]) : '–'}</td>
        <td class="num">${d ? f(d.x) + ', ' + f(d.y) : '<span class="muted">none</span>'}</td></tr>`;
    }).join('')}</tbody></table>` : '<div class="muted">no people in /person_infos or /demo/person_markers</div>';
  const logs = S.logs.filter((l) => l.owner === 'AI' || l.owner === 'DEMO' || l.src === 'ai_debug').slice(-150).reverse();
  $('#aiLog').innerHTML = logs.length ? logs.map(logLine).join('') : '<div class="muted" style="padding:8px">no AI / demo_manager log lines yet (the AI node does not publish /debug_log yet)</div>';
}

// =================================================================== topics
const topicState = { sel: null, node: null, mode: 'all', filter: '', freeze: false };
export function topicExtraChannels() {
  return ['graph', ...(topicState.sel ? ['inspect:' + topicState.sel] : [])];
}
export function initTopics(onChange) {
  $('#topicFilter').oninput = (e) => { topicState.filter = e.target.value.toLowerCase(); onChange(); };
  $('#topicMode').addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    $$('#topicMode button').forEach((x) => x.classList.toggle('on', x === b));
    topicState.mode = b.dataset.mode;
    onChange();
  });
  $('#topicTable').addEventListener('click', (e) => {
    const tr = e.target.closest('tr[data-topic]');
    if (!tr) return;
    topicState.sel = tr.dataset.topic;
    topicState.node = null;
    topicState.freeze = false;
    $('#inspector').innerHTML = `<div class="muted">subscribing to ${esc(topicState.sel)}…</div>`;
    setChannels(topicExtraChannels());
    onChange();
  });
  $('#nodeList').addEventListener('click', async (e) => {
    const b = e.target.closest('button[data-node]');
    if (!b) return;
    topicState.sel = null;
    topicState.node = b.dataset.node;
    setChannels(topicExtraChannels());
    $('#inspector').innerHTML = `<div class="muted">querying ${esc(b.dataset.node)}…</div>`;
    const r = await post('/api/cmd', { cmd: 'node_info', node: b.dataset.node });
    renderNodeInfo(r);
  });
  $('#inspector').addEventListener('click', (e) => {
    if (e.target.id === 'inspClose') {
      topicState.sel = null; topicState.node = null;
      setChannels(topicExtraChannels());
      $('#inspector').innerHTML = '<div class="muted">Select a topic or node.</div>';
    } else if (e.target.id === 'inspFreeze') {
      topicState.freeze = !topicState.freeze;
      e.target.textContent = topicState.freeze ? 'Resume' : 'Freeze';
    } else if (e.target.dataset.topic) {
      topicState.sel = e.target.dataset.topic;
      topicState.node = null;
      setChannels(topicExtraChannels());
    }
  });
}

export function renderTopics() {
  const g = S.ch.graph;
  const rates = S.ch.rates || {};
  const bad = new Map();
  for (const c of allChecks()) if (c.id.startsWith('topic:') && c.level !== 'OK') bad.set(c.id.slice(6), c);
  let topics = g ? g.topics.slice() : [];
  for (const t of Object.keys(rates)) if (!topics.find((x) => x.name === t)) topics.push({ name: t, type: '', pubs: 0, subs: 0 });
  topics.sort((a, b) => a.name.localeCompare(b.name));
  const fl = topicState.filter;
  topics = topics.filter((t) => {
    if (topicState.mode === 'mon' && !rates[t.name]) return false;
    if (topicState.mode === 'bad' && !bad.has(t.name) && !(rates[t.name] && t.pubs === 0)) return false;
    if (!fl) return true;
    return (t.name + ' ' + t.type + ' ' + topicOwner(t.name)).toLowerCase().includes(fl);
  });
  $('#topicCount').textContent = g ? `${topics.length} / ${g.topics.length}` : '';
  const scroll = $('#topicTable').scrollTop;
  $('#topicTable').innerHTML = `<table class="tbl"><thead><tr><th>Topic</th><th>Pub/Sub</th><th>Rate</th><th>Age</th><th>Owner</th></tr></thead><tbody>
    ${topics.map((t) => {
      const r = rates[t.name];
      const b = bad.get(t.name);
      return `<tr class="click ${topicState.sel === t.name ? 'sel' : ''}" data-topic="${esc(t.name)}">
        <td><div class="name">${b ? `<span class="dot ${esc(b.level)}"></span> ` : ''}${esc(t.name)}</div><div class="type">${esc(t.type)}</div></td>
        <td class="num">${t.pubs}/${t.subs}</td>
        <td class="num">${r ? f(r.hz, 1) + ' Hz' : '<span class="muted">·</span>'}</td>
        <td class="num">${r ? fAge(r.age) : ''}</td>
        <td>${ownerBadge(topicOwner(t.name))}</td></tr>`;
    }).join('')}</tbody></table>`;
  $('#topicTable').scrollTop = scroll;
  const nodes = g ? g.nodes : [];
  $('#nodeCount').textContent = nodes.length ? `(${nodes.length})` : '';
  if ($('#nodeList').dataset.n !== String(nodes.length)) {
    $('#nodeList').dataset.n = String(nodes.length);
    $('#nodeList').innerHTML = nodes.map((n) => `<button class="btn" data-node="${esc(n)}">${esc(n)}</button>`).join('');
  }
  if (topicState.sel) renderInspector();
}

function renderInspector() {
  const d = S.ch['inspect:' + topicState.sel];
  if (!d) return;
  const heavy = /Image|PointCloud|OccupancyGrid|Costmap/.test(d.type || '');
  const qrows = (arr) => arr && arr.length ? `<table class="tbl"><tbody>${arr.map((q) =>
    `<tr><td class="name">${esc(q.node)}</td><td class="small">${esc(q.reliability)}</td><td class="small">${esc(q.durability)}</td><td class="num">${q.depth}</td></tr>`).join('')}</tbody></table>` : '<div class="muted small">none</div>';
  const keepMsg = topicState.freeze && $('#inspMsg') ? $('#inspMsg').textContent : null;
  $('#inspector').innerHTML = `<div class="card-h"><h2 class="mono">${esc(d.topic)}</h2>
      <div class="row gap"><button class="btn" id="inspFreeze">${topicState.freeze ? 'Resume' : 'Freeze'}</button><button class="btn" id="inspClose">✕</button></div></div>
    <div class="type mono small">${esc(d.type || 'type not known yet')} ${ownerBadge(topicOwner(d.topic))}</div>
    ${heavy ? '<div class="note" style="margin-top:6px">Large message type: inspecting it copies every message into the console gateway — close the inspector when done.</div>' : ''}
    <div class="kv mono small" style="margin-top:8px">
      <span>Rate</span><span>${f(d.hz, 2)} Hz</span><span>Bandwidth</span><span>${fBytes(d.bw)}/s</span>
      <span>Message size</span><span>${fBytes(d.size)}</span><span>Last message</span><span>${fAge(d.age)} ago</span>
      <span>Received</span><span>${d.n}</span></div>
    <h3 style="margin-top:12px">Publishers (${(d.pubs || []).length})</h3>${qrows(d.pubs)}
    <h3 style="margin-top:12px">Subscribers (${(d.subs || []).length})</h3>${qrows(d.subs)}
    <h3 style="margin-top:12px">Latest message</h3><pre id="inspMsg">${keepMsg !== null ? esc(keepMsg) : (d.msg === undefined ? 'waiting for a message…' : esc(typeof d.msg === 'string' ? d.msg : JSON.stringify(d.msg, null, 2)))}</pre>`;
}

function renderNodeInfo(r) {
  if (!r.ok) { $('#inspector').innerHTML = `<div class="muted">${esc(r.msg || 'failed')}</div>`; return; }
  const d = r.data;
  const list = (arr, link) => arr.length ? `<table class="tbl"><tbody>${arr.map(([n, t]) =>
    `<tr><td class="name">${link ? `<a href="javascript:void 0" data-topic="${esc(n)}">${esc(n)}</a>` : esc(n)}</td><td class="type">${esc(t)}</td></tr>`).join('')}</tbody></table>` : '<div class="muted small">none</div>';
  $('#inspector').innerHTML = `<div class="card-h"><h2 class="mono">${esc(d.node)}</h2><button class="btn" id="inspClose">✕</button></div>
    <h3>Publishes (${d.pubs.length})</h3>${list(d.pubs, true)}
    <h3 style="margin-top:12px">Subscribes (${d.subs.length})</h3>${list(d.subs, true)}
    <h3 style="margin-top:12px">Services (${d.srvs.length})</h3>${list(d.srvs, false)}`;
}

// ======================================================================= TF
export function renderTF() {
  const edges = S.ch.tf || [];
  const children = {};
  const childSet = new Set();
  for (const e of edges) { (children[e.p] ||= []).push(e); childSet.add(e.c); }
  const roots = Object.keys(children).filter((p) => !childSet.has(p)).sort();
  const lines = [];
  const walk = (frame, prefix, isLast, edge, depth) => {
    if (depth > 20) return;
    let tag = '';
    if (edge) {
      const c = check(`tf:${edge.p}>${edge.c}`);
      const lvl = c ? c.level : (edge.s ? 'OK' : (edge.age !== null && edge.age > 1 ? 'ERROR' : 'OK'));
      tag = ` <span class="pill lv-${lvl}">${edge.s ? 'static' : `${f(edge.hz, 0)} Hz · ${fAge(edge.age)}`}</span>`;
    }
    lines.push(`${esc(prefix)}${edge ? (isLast ? '└─ ' : '├─ ') : ''}<b>${esc(frame)}</b>${tag}`);
    const kids = (children[frame] || []).slice().sort((a, b) => a.c.localeCompare(b.c));
    kids.forEach((k, i) => walk(k.c, prefix + (edge ? (isLast ? '   ' : '│  ') : ''), i === kids.length - 1, k, depth + 1));
  };
  roots.forEach((r) => walk(r, '', true, null, 0));
  $('#tfTree').innerHTML = lines.join('\n') || '<span class="muted">no transforms received</span>';
  const rows = edges.slice().sort((a, b) => (a.s - b.s) || a.p.localeCompare(b.p));
  $('#tfTable').innerHTML = `<table class="tbl"><thead><tr><th>parent → child</th><th>kind</th><th>rate</th><th>age</th><th>lag</th><th>gaps</th><th>max gap</th><th>jumps</th><th>x, y, yaw</th></tr></thead><tbody>
    ${rows.map((e) => {
      const c = check(`tf:${e.p}>${e.c}`);
      return `<tr><td class="name">${c ? `<span class="dot ${esc(c.level)}"></span> ` : ''}${esc(e.p)} → ${esc(e.c)}</td>
        <td>${e.s ? 'static' : 'dynamic'}</td><td class="num">${e.s ? '' : f(e.hz, 1)}</td>
        <td class="num">${e.s ? '' : fAge(e.age)}</td><td class="num">${e.s ? '' : f(e.lag, 3)}</td>
        <td class="num">${e.gaps || ''}</td><td class="num">${e.max_gap ? f(e.max_gap, 1) + ' s' : ''}</td><td class="num">${e.jumps || ''}</td>
        <td class="num">${e.pose ? `${f(e.pose[0])}, ${f(e.pose[1])}, ${fDeg(e.pose[2])}` : ''}</td></tr>`;
    }).join('')}</tbody></table>`;
}

// =================================================================== system
export function initSystem() {
  $('#stackBtns').addEventListener('click', async (e) => {
    const b = e.target.closest('[data-script]');
    if (!b) return;
    const name = b.dataset.script;
    const what = {
      start_all: 'start lidar + rf2o + slam_toolbox, RealSense + EKF, Nav2 + chassis driver',
      stop_all: 'stop SLAM, EKF, RealSense, Nav2 and the chassis driver (the map is lost)',
      start_ai: 'start the AI node, AI TF and demo_manager',
      stop_ai: 'stop the AI node, AI TF and demo_manager',
    }[name];
    if (!await confirmDialog(`Run ${name}.sh?`, `This will ${esc(what)} on the robot.`, 'Run')) return;
    const r = await post('/api/stack/' + name);
    toast(r.ok ? `${name}.sh started` : (r.msg || 'failed'), r.ok ? 'OK' : 'ERROR');
  });
}

export function renderSystem() {
  const sc = S.ch.script;
  if (sc) {
    const st = sc.state || {};
    $('#stackDir').textContent = sc.dir || '';
    $$('#stackBtns [data-script]').forEach((b) => { b.disabled = st.running || !(sc.available || {})[b.dataset.script]; });
    const out = $('#scriptOut');
    const atBottom = out.scrollHeight - out.scrollTop - out.clientHeight < 30;
    const head = st.name ? `${st.running ? '▶ running' : `■ exit ${st.exit}`} ${st.name}.sh  (by ${st.by}, ${fTime(st.started)})\n` : 'no script run yet\n';
    out.textContent = head + (st.lines || []).join('\n');
    if (atBottom) out.scrollTop = out.scrollHeight;
  }
  const s = S.ch.sys;
  if (!s) return;
  const procs = s.procs || [];
  $('#procTable').innerHTML = `<table class="tbl"><thead><tr><th>process</th><th>owner</th><th>pid</th><th>CPU %</th><th>RSS</th><th>threads</th><th>up</th></tr></thead><tbody>
    ${procs.map((p) => `<tr title="${esc(p.cmd || '')}"><td>${p.missing ? pill(p.core && s.stack_up ? 'ERROR' : 'OFF', 'not running') + ' ' : ''}<span class="name">${esc(p.name)}</span></td>
      <td>${ownerBadge(p.owner)}</td><td class="num">${p.pid || ''}</td><td class="num">${p.missing ? '' : f(p.cpu, 1)}</td>
      <td class="num">${p.missing ? '' : f(p.rss_mb, 0) + ' MB'}</td><td class="num">${p.threads || ''}</td><td class="num">${p.missing ? '' : fAge(p.up_s)}</td></tr>`).join('')}</tbody></table>`;
  $('#sysBoard').innerHTML = boardHtml(false) + `<div class="small" style="margin-top:8px">${(s.temps || []).map((t) => `${esc(t.type || t.zone)} ${f(t.c, 0)}°C`).join(' · ')}</div>`;
  const lc = S.ch.lifecycle || {};
  $('#sysLifecycle').innerHTML = Object.keys(lc).length ? `<div class="kv small">${Object.entries(lc).sort().map(([n, v]) =>
    `<span class="mono">${esc(n)}</span><span>${pill(v === 'active' ? 'OK' : (v === 'unavailable' ? 'OFF' : 'ERROR'), v)}</span>`).join('')}</div>` : '<div class="muted">–</div>';
  const dg = S.ch.diag || [];
  $('#sysDiag').innerHTML = dg.length ? dg.map((d) => `<div class="small" style="margin-bottom:6px">${pill(['OK', 'WARN', 'ERROR', 'WARN'][d.level] || 'WARN', ['OK', 'WARN', 'ERROR', 'STALE'][d.level])} <b>${esc(d.name)}</b><div class="muted">${esc(d.msg)}</div></div>`).join('') : '<div class="muted">none</div>';
  const pf = s.pidfiles || [];
  $('#sysPids').innerHTML = pf.length ? `<table class="tbl"><tbody>${pf.map((p) => `<tr><td class="name small">${esc(p.file.replace('/tmp/', ''))}</td><td>${esc(p.name)}</td><td class="num">${p.pid}</td><td>${pill(p.alive ? 'OK' : 'WARN', p.alive ? 'alive' : 'dead')}</td></tr>`).join('')}</tbody></table>` : '<div class="muted">no PID files (stack not started by the scripts)</div>';
  $('#sysUsb').innerHTML = `<div>${pill(s.serial && s.serial.present ? 'OK' : 'ERROR', s.serial && s.serial.present ? 'present' : 'missing')} <span class="mono">${esc(s.serial ? s.serial.path : '')}</span></div>
    <pre class="term" style="max-height:160px">${esc((s.dmesg || []).join('\n') || 'no USB / ttyUSB kernel events')}</pre>`;
  const gw = s.gateway || {};
  const ls = S.ch.logstats || {};
  const hidden = Object.entries(ls.hidden || {});
  $('#sysConsole').innerHTML = `<div class="kv small">
      <span>ROS gateway</span><span>${pill(gw.up ? 'OK' : 'ERROR', gw.up ? 'up' : 'down')} ${gw.since ? 'since ' + fTime(gw.since) : ''} · restarts ${gw.restarts || 0}</span>
      <span>Viewers</span><span>${s.clients || 0} browser session(s)</span>
      <span>Logs stored</span><span>${ls.stored || 0} lines</span>
      ${hidden.length ? `<span>Hidden noise</span><span>${hidden.map(([k, v]) => `${esc(k)}: ${v}`).join('<br>')}</span>` : ''}
    </div>
    ${gw.last_err ? `<div class="note" style="margin-top:6px">last gateway error: ${esc(gw.last_err)}</div>` : ''}
    <details style="margin-top:6px"><summary class="small">gateway stderr</summary><pre class="term">${esc((gw.stderr || []).join('\n') || '(empty)')}</pre></details>
    <details><summary class="small">log files followed (${(ls.files || []).length})</summary><div class="small mono">${(ls.files || []).map((x) => `${esc(x.path)} · ${fBytes(x.size)}`).join('<br>')}</div></details>`;
}

// ===================================================================== logs
const LV = { D: 0, I: 1, W: 2, E: 3, F: 4 };
const logState = { lvl: 'I', text: '', re: null, owner: '', src: '', paused: false, last: 0, held: 0 };

export function logLine(l) {
  return `<div class="ln ${esc(l.lvl)}"><time>${fTimeMs(l.t)}</time><span class="lv">${esc(l.lvl)}</span>
    <span class="who">${ownerBadge(l.owner)} ${esc(l.node)}</span>
    <span class="msg">${esc(l.msg)}${l.hint ? `<span class="hint">↳ ${esc(l.hint)}</span>` : ''}</span></div>`;
}

function logMatch(l) {
  if ((LV[l.lvl] ?? 1) < LV[logState.lvl]) return false;
  if (logState.owner && l.owner !== logState.owner) return false;
  if (logState.src && !(l.src || '').startsWith(logState.src)) return false;
  if (logState.text) {
    const hay = l.node + ' ' + l.msg + ' ' + (l.hint || '');
    if (logState.re ? !logState.re.test(hay) : !hay.toLowerCase().includes(logState.text)) return false;
  }
  return true;
}

export function initLogs() {
  $('#logOwner').innerHTML = '<option value="">all owners</option>' +
    Object.entries(OWNERS).map(([k, v]) => `<option value="${k}">${esc(k)} — ${esc(v.label)}</option>`).join('');
  const refilter = () => { logState.last = 0; renderLogs(true); };
  $('#logFilter').oninput = (e) => {
    logState.text = e.target.value.toLowerCase();
    try { logState.re = e.target.value ? new RegExp(e.target.value, 'i') : null; } catch { logState.re = null; }
    refilter();
  };
  $('#logLevel').addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    $$('#logLevel button').forEach((x) => x.classList.toggle('on', x === b));
    logState.lvl = b.dataset.lvl;
    refilter();
  });
  $('#logOwner').onchange = (e) => { logState.owner = e.target.value; refilter(); };
  $('#logSrc').onchange = (e) => { logState.src = e.target.value; refilter(); };
  $('#logPause').onclick = (e) => {
    logState.paused = !logState.paused;
    e.target.textContent = logState.paused ? 'Resume' : 'Pause';
    if (!logState.paused) renderLogs();
  };
}

export function renderLogs(full) {
  const list = $('#logList');
  const ls = S.ch.logstats || {};
  const hiddenN = Object.values(ls.hidden || {}).reduce((a, b) => a + b, 0);
  if (logState.paused && !full) {
    const n = S.logs.filter((l) => l._seq > logState.last).length;
    $('#logStats').textContent = `paused · ${n} new line(s) waiting`;
    return;
  }
  const atBottom = list.scrollHeight - list.scrollTop - list.clientHeight < 40;
  if (full || !logState.last || (S.logs.length && S.logs[0]._seq > logState.last)) {
    const rows = S.logs.filter(logMatch).slice(-800);
    list.innerHTML = rows.map(logLine).join('');
  } else {
    const rows = S.logs.filter((l) => l._seq > logState.last && logMatch(l));
    if (rows.length) {
      list.insertAdjacentHTML('beforeend', rows.map(logLine).join(''));
      while (list.childElementCount > 1000) list.firstElementChild.remove();
    }
  }
  logState.last = S.logSeq;
  $('#logStats').textContent = `${S.logs.length} lines in browser · ${ls.stored || 0} on robot · ${hiddenN} noisy lines hidden (rf2o per-scan, AI per-person…) · newest at the bottom`;
  if (atBottom || full) list.scrollTop = list.scrollHeight;
}
