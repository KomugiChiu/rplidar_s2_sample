// Robot Console bootstrap: tabs, header, render scheduling.
import { S, on, connect, setChannels, $, $$, esc, f, fAge, cmd, confirmDialog, store, allChecks } from './core.js';
import {
  renderOverview, bindOverview, initMap, mapChannels, mapUpdate, mapResize, mapRetheme, renderMapSide,
  initDrive, renderDrive, driveCam, initAI, renderAI, aiCams, initTopics, renderTopics, topicExtraChannels,
  renderTF, initSystem, renderSystem, initLogs, renderLogs,
} from './views.js';
import { bindKeys, stopDrive } from './pad.js';

const TABS = ['overview', 'map', 'drive', 'ai', 'topics', 'tf', 'system', 'logs'];
let tab = 'overview';

const RENDER = {
  overview: renderOverview,
  map: renderMapSide,
  drive: renderDrive,
  ai: renderAI,
  topics: renderTopics,
  tf: renderTF,
  system: renderSystem,
  logs: () => renderLogs(false),
};
// channels that should trigger a re-render of each tab
const TRIGGERS = {
  overview: ['motion', 'health', 'sys', 'events', 'pose', 'rates'],
  map: ['motion'],
  drive: ['motion', 'detections'],
  ai: ['people', 'detections', 'rates', 'motion', 'log', 'health'],
  topics: ['graph', 'rates', 'health', 'inspect'],
  tf: ['tf', 'health'],
  system: ['sys', 'script', 'lifecycle', 'diag', 'logstats'],
  logs: ['log', 'logstats'],
};
const MIN_INTERVAL = { topics: 500, system: 400, ai: 300, tf: 500 };

function tabChannels(t) {
  switch (t) {
    case 'map': return mapChannels();
    case 'drive': return ['detections'];
    case 'ai': return ['people', 'detections'];
    case 'topics': return topicExtraChannels();
    default: return [];
  }
}

// ------------------------------------------------------------- scheduling
let pending = false;
let lastRender = 0;
function schedule() {
  if (pending) return;
  pending = true;
  const wait = Math.max(0, (MIN_INTERVAL[tab] || 0) - (performance.now() - lastRender));
  setTimeout(() => requestAnimationFrame(() => {
    pending = false;
    lastRender = performance.now();
    try { RENDER[tab](); } catch (e) { console.error('render', tab, e); }
  }), wait);
}

function showTab(t) {
  if (!TABS.includes(t)) t = 'overview';
  tab = t;
  $$('.tabs button').forEach((b) => b.classList.toggle('on', b.dataset.tab === t));
  $$('main > .tab').forEach((s) => s.classList.toggle('on', s.id === 'tab-' + t));
  if (location.hash !== '#' + t) history.replaceState(null, '', '#' + t);
  setChannels(tabChannels(t));
  updateCams();
  if (t === 'map') mapResize();
  if (t === 'logs') renderLogs(true);
  if (t !== 'drive' && $('#mapPad').classList.contains('hidden')) stopDrive();
  schedule();
}

function updateCams() {
  const vis = !document.hidden;
  if (vis && tab === 'drive') driveCam.start(); else driveCam.stop();
  if (vis && tab === 'ai') { aiCams.color.start(); aiCams.debug.start(); } else { aiCams.color.stop(); aiCams.debug.stop(); }
}

// ----------------------------------------------------------------- header
function renderHeader() {
  const s = S.ch.sys || {};
  const m = S.ch.motion;
  const gw = s.gateway || {};
  const tmax = Math.max(0, ...(s.temps || []).map((t) => t.c));
  const chip = (dot, label, val) => `<span class="chip"><span class="dot ${dot}"></span>${esc(label)} <b>${esc(val)}</b></span>`;
  $('#topChips').innerHTML = [
    chip(s.stack_up ? 'OK' : 'ERROR', 'stack', s.stack_up ? 'running' : 'stopped'),
    chip(gw.up ? 'OK' : 'ERROR', 'ROS', gw.up ? 'linked' : 'down'),
    chip(m && m.nav.active.length ? 'INFO' : '', 'nav', m && m.nav.active.length ? 'active' : 'idle'),
    chip(m && m.demo.pubs ? (['TARGET_LOST', 'NAVIGATION_FAILED'].includes(m.demo.state) ? 'WARN' : 'OK') : '', 'demo', m && m.demo.pubs ? m.demo.state : 'off'),
    chip(s.cpu && s.cpu.total > 85 ? 'ERROR' : (s.cpu && s.cpu.total > 75 ? 'WARN' : 'OK'), 'CPU', s.cpu ? f(s.cpu.total, 0) + '%' : '–'),
    chip(tmax > 85 ? 'WARN' : 'OK', 'temp', tmax ? f(tmax, 0) + '°C' : '–'),
  ].join('');
  const ban = $('#banner');
  let text = '', err = false;
  if (!S.connected) { text = 'Disconnected from the robot — reconnecting…'; err = true; }
  else if (m && m.estop) { text = 'E-STOP active'; err = true; }
  else if (m && m.teleop) { text = `MANUAL OVERRIDE from ${m.teleop.src} — Nav2 goals cancelled, no obstacle check`; }
  else if (S.ch.sys && gw.up === false) { text = 'ROS gateway is down — ROS data is stale (System tab → Console)'; err = true; }
  else if (S.ch.sys && !s.stack_up) { text = 'ROS stack is not running — start it from the System tab'; }
  ban.textContent = text;
  ban.classList.toggle('hidden', !text);
  ban.classList.toggle('err', err);

  // tab badges: owners in ERROR
  const by = {};
  for (const c of allChecks()) if (c.level === 'ERROR' || c.level === 'WARN') (by[c.owner] ||= new Set()).add(c.level);
  const errOwners = Object.entries(by).filter(([, l]) => l.has('ERROR')).map(([o]) => o);
  const badge = (name, owners) => {
    const el = $(`[data-badge="${name}"]`);
    const errs = owners.filter((o) => by[o] && by[o].has('ERROR')).length;
    const warns = owners.filter((o) => by[o] && !by[o].has('ERROR')).length;
    el.textContent = errs || warns || '';
    el.classList.toggle('warn', !errs && warns > 0);
  };
  $('[data-badge="overview"]').textContent = errOwners.length || '';
  badge('ai', ['AI', 'DEMO']);
  badge('system', ['SYSTEM', 'CONSOLE']);
}

// ------------------------------------------------------------------- boot
function applyTheme(t) {
  if (t) document.documentElement.dataset.theme = t; else delete document.documentElement.dataset.theme;
}

function boot() {
  $('#host').textContent = location.host;
  applyTheme(store('theme'));
  $('#themeBtn').onclick = () => {
    const dark = document.documentElement.dataset.theme
      ? document.documentElement.dataset.theme === 'dark'
      : matchMedia('(prefers-color-scheme: dark)').matches;
    const next = dark ? 'light' : 'dark';
    store('theme', next);
    applyTheme(next);
    mapRetheme();
  };
  $('#estop').onclick = () => { stopDrive(); cmd({ cmd: 'estop' }, 'E-STOP sent: goals cancelled, zero velocity'); };
  $('#tabs').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-tab]');
    if (b) showTab(b.dataset.tab);
  });
  window.addEventListener('hashchange', () => showTab(location.hash.slice(1)));
  document.addEventListener('visibilitychange', updateCams);

  bindOverview(schedule);
  initMap();
  initDrive();
  initAI();
  initTopics(schedule);
  initSystem();
  initLogs();
  bindKeys(() => tab === 'drive' || (tab === 'map' && !$('#mapPad').classList.contains('hidden')));

  on('*', (ch) => {
    const base = ch.startsWith('inspect:') ? 'inspect' : ch;
    if (['sys', 'motion', 'health'].includes(base)) renderHeader();
    if (tab === 'map') mapUpdate(base, S.ch[ch]);
    if ((TRIGGERS[tab] || []).includes(base)) schedule();
  });
  on('conn', () => renderHeader());
  connect();
  showTab(location.hash.slice(1) || 'overview');
  renderHeader();
}

boot();
