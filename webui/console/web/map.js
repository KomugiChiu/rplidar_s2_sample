// 2D map view (canvas): occupancy map, costmaps, scan, plan, goals, people, robot.
// World frame = map; x right, y up.  view.s = pixels per metre.

const FOOTPRINT = [[0.075, 0.05], [0.075, -0.05], [-0.075, -0.05], [-0.075, 0.05]];

function b64bytes(s) {
  const bin = atob(s);
  const a = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) a[i] = bin.charCodeAt(i);
  return a;
}
function cssColor(name) {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  const m = v.match(/^#([0-9a-f]{6})$/i);
  if (m) {
    const n = parseInt(m[1], 16);
    return [(n >> 16) & 255, (n >> 8) & 255, n & 255];
  }
  return [128, 128, 128];
}

export class MapView {
  constructor(canvas, { onPick, onCursor } = {}) {
    this.c = canvas;
    this.ctx = canvas.getContext('2d');
    this.view = { x: 0, y: 0, s: 80 };
    this.layers = { map: true, costmap_global: false, costmap_local: true, scan: true, plan: true,
      people: true, trail: true, grid: false };
    this.follow = true;
    this.tool = 'pan';
    this.grids = {};
    this.d = { pose: null, scan: null, plan: null, people: null, mission: null, extGoal: null, trail: [],
      pending: [], preview: null };
    this.onPick = onPick;
    this.onCursor = onCursor;
    this.pointers = new Map();
    this.fitted = false;
    this.w = 0; this.h = 0;

    canvas.addEventListener('pointerdown', (e) => this.down(e));
    canvas.addEventListener('pointermove', (e) => this.move(e));
    canvas.addEventListener('pointerup', (e) => this.up(e));
    canvas.addEventListener('pointercancel', (e) => this.up(e, true));
    canvas.addEventListener('wheel', (e) => this.wheel(e), { passive: false });
    new ResizeObserver(() => this.resize()).observe(canvas);
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => this.retheme());
  }

  retheme() {
    for (const [name, g] of Object.entries(this.grids)) this.setGrid(name, g.src);
  }

  resize() {
    const r = this.c.getBoundingClientRect();
    if (!r.width) return;
    const dpr = window.devicePixelRatio || 1;
    this.w = r.width; this.h = r.height; this.dpr = dpr;
    this.c.width = Math.round(r.width * dpr);
    this.c.height = Math.round(r.height * dpr);
    if (!this.fitted && this.grids.map) this.fit();
    this.dirty();
  }

  dirty() {
    if (this.raf) return;
    this.raf = requestAnimationFrame(() => { this.raf = 0; this.draw(); });
  }

  setTool(t) {
    this.tool = t;
    this.c.classList.toggle('pick', t !== 'pan');
  }

  // ------------------------------------------------------------------ data
  setGrid(name, g) {
    if (!g || !g.data) return;
    const bytes = b64bytes(g.data);
    const cv = document.createElement('canvas');
    cv.width = g.w; cv.height = g.h;
    const cx = cv.getContext('2d');
    const img = cx.createImageData(g.w, g.h);
    const px = img.data;
    if (name === 'map') {
      const unk = cssColor('--map-unknown'), free = cssColor('--map-free'), occ = cssColor('--map-occ');
      for (let j = 0; j < g.h; j++) {
        const row = (g.h - 1 - j) * g.w; // grid row 0 is the bottom
        for (let i = 0; i < g.w; i++) {
          const v = bytes[j * g.w + i];
          const o = (row + i) * 4;
          let c;
          if (v === 255) c = unk;
          else {
            const t = Math.min(100, v) / 100;
            c = [free[0] + (occ[0] - free[0]) * t, free[1] + (occ[1] - free[1]) * t, free[2] + (occ[2] - free[2]) * t];
          }
          px[o] = c[0]; px[o + 1] = c[1]; px[o + 2] = c[2]; px[o + 3] = 255;
        }
      }
    } else {
      for (let j = 0; j < g.h; j++) {
        const row = (g.h - 1 - j) * g.w;
        for (let i = 0; i < g.w; i++) {
          const v = bytes[j * g.w + i];
          const o = (row + i) * 4;
          if (v === 0 || v === 255) { px[o + 3] = 0; continue; }
          if (v >= 100) { px[o] = 255; px[o + 1] = 0; px[o + 2] = 110; px[o + 3] = 200; }       // lethal
          else if (v >= 99) { px[o] = 160; px[o + 1] = 80; px[o + 2] = 255; px[o + 3] = 170; }  // inscribed
          else { px[o] = 0; px[o + 1] = 150; px[o + 2] = 255; px[o + 3] = 40 + v * 1.3; }
        }
      }
    }
    cx.putImageData(img, 0, 0);
    this.grids[name] = { canvas: cv, w: g.w, h: g.h, res: g.res, o: g.o, src: g };
    if (name === 'map' && !this.fitted) this.fit();
    this.dirty();
  }

  set(key, v) { this.d[key] = v; this.dirty(); }

  // manual: user pressed Fit (stop following); automatic first fit keeps follow
  fit(manual = false) {
    const g = this.grids.map;
    if (!g || !this.w) return;
    const wm = g.w * g.res, hm = g.h * g.res;
    this.view.x = g.o[0] + wm / 2;
    this.view.y = g.o[1] + hm / 2;
    this.view.s = Math.max(5, Math.min(400, 0.9 * Math.min(this.w / wm, this.h / hm)));
    this.fitted = true;
    if (manual) this.follow = false;
    this.dirty();
  }

  // ------------------------------------------------------------- geometry
  sx(x) { return this.w / 2 + (x - this.view.x) * this.view.s; }
  sy(y) { return this.h / 2 - (y - this.view.y) * this.view.s; }
  world(px, py) {
    return [this.view.x + (px - this.w / 2) / this.view.s, this.view.y - (py - this.h / 2) / this.view.s];
  }
  local(e) {
    const r = this.c.getBoundingClientRect();
    return [e.clientX - r.left, e.clientY - r.top];
  }

  // ---------------------------------------------------------------- input
  down(e) {
    this.c.setPointerCapture(e.pointerId);
    const p = this.local(e);
    this.pointers.set(e.pointerId, p);
    if (this.pointers.size === 2) {
      const [a, b] = [...this.pointers.values()];
      this.pinch = { d: Math.hypot(a[0] - b[0], a[1] - b[1]), s: this.view.s,
        mid: this.world((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) };
      this.drag = null;
      this.d.preview = null;
      return;
    }
    if (this.tool === 'pan') {
      this.drag = { p, x: this.view.x, y: this.view.y };
    } else {
      const w = this.world(p[0], p[1]);
      this.drag = { p, pick: w };
      this.d.preview = { x: w[0], y: w[1], yaw: null, tool: this.tool };
      this.dirty();
    }
  }

  move(e) {
    const p = this.local(e);
    if (this.onCursor) this.onCursor(this.world(p[0], p[1]));
    if (!this.pointers.has(e.pointerId)) return;
    this.pointers.set(e.pointerId, p);
    if (this.pinch && this.pointers.size === 2) {
      const [a, b] = [...this.pointers.values()];
      const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
      this.view.s = Math.max(5, Math.min(600, this.pinch.s * d / this.pinch.d));
      const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
      this.view.x = this.pinch.mid[0] - (mid[0] - this.w / 2) / this.view.s;
      this.view.y = this.pinch.mid[1] + (mid[1] - this.h / 2) / this.view.s;
      this.follow = false;
      this.fireFollow();
      this.dirty();
      return;
    }
    if (!this.drag) return;
    if (this.drag.pick) {
      const dx = p[0] - this.drag.p[0], dy = p[1] - this.drag.p[1];
      if (Math.hypot(dx, dy) > 10) this.d.preview.yaw = Math.atan2(-dy, dx);
      this.dirty();
      return;
    }
    const dx = p[0] - this.drag.p[0], dy = p[1] - this.drag.p[1];
    if (Math.hypot(dx, dy) > 3 && this.follow) { this.follow = false; this.fireFollow(); }
    this.view.x = this.drag.x - dx / this.view.s;
    this.view.y = this.drag.y + dy / this.view.s;
    this.dirty();
  }

  up(e, cancelled) {
    this.pointers.delete(e.pointerId);
    if (this.pointers.size < 2) this.pinch = null;
    const drag = this.drag;
    this.drag = null;
    if (drag && drag.pick && !cancelled && this.pointers.size === 0) {
      const pv = this.d.preview;
      let yaw = pv.yaw;
      if (yaw === null) {
        // no drag: face away from the robot
        const r = this.d.pose && this.d.pose.p;
        yaw = r ? Math.atan2(pv.y - r[1], pv.x - r[0]) : 0;
      }
      this.d.preview = { ...pv, yaw };
      if (this.onPick) this.onPick(this.tool, pv.x, pv.y, yaw);
    } else if (cancelled) {
      this.d.preview = null;
    }
    this.dirty();
  }

  wheel(e) {
    e.preventDefault();
    const p = this.local(e);
    const before = this.world(p[0], p[1]);
    this.view.s = Math.max(5, Math.min(600, this.view.s * Math.exp(-e.deltaY * 0.0015)));
    const after = this.world(p[0], p[1]);
    this.view.x += before[0] - after[0];
    this.view.y += before[1] - after[1];
    if (this.follow) { this.follow = false; this.fireFollow(); }
    this.dirty();
  }

  fireFollow() { this.c.dispatchEvent(new CustomEvent('follow', { detail: this.follow })); }

  // ----------------------------------------------------------------- draw
  draw() {
    const ctx = this.ctx;
    if (!this.w) return;
    const pose = this.d.pose && this.d.pose.ok ? this.d.pose.p : null;
    if (this.follow && pose) { this.view.x = pose[0]; this.view.y = pose[1]; }
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.w, this.h);
    const s = this.view.s;

    for (const name of ['map', 'costmap_global', 'costmap_local']) {
      const g = this.grids[name];
      if (!g || !this.layers[name]) continue;
      ctx.save();
      ctx.translate(this.sx(g.o[0]), this.sy(g.o[1]));
      ctx.rotate(-g.o[2]);
      const k = g.res * s;
      ctx.scale(k, k);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(g.canvas, 0, -g.h);
      ctx.restore();
    }

    if (this.layers.grid) this.drawGridLines();

    const css = getComputedStyle(document.documentElement);
    const col = (n) => css.getPropertyValue(n).trim();

    // plan
    const plan = this.d.plan;
    if (this.layers.plan && plan && plan.pts && plan.pts.length > 1) {
      ctx.strokeStyle = col('--plan'); ctx.lineWidth = 2.5; ctx.lineJoin = 'round';
      ctx.beginPath();
      plan.pts.forEach(([x, y], i) => (i ? ctx.lineTo(this.sx(x), this.sy(y)) : ctx.moveTo(this.sx(x), this.sy(y))));
      ctx.stroke();
    }

    // trail
    if (this.layers.trail && this.d.trail.length > 1) {
      ctx.strokeStyle = col('--robot'); ctx.globalAlpha = 0.35; ctx.lineWidth = 2;
      ctx.beginPath();
      this.d.trail.forEach(([x, y], i) => (i ? ctx.lineTo(this.sx(x), this.sy(y)) : ctx.moveTo(this.sx(x), this.sy(y))));
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // scan
    const scan = this.d.scan;
    if (this.layers.scan && scan && scan.xy) {
      if (scan._xy !== scan.xy) {
        const b = b64bytes(scan.xy);
        scan._pts = new Float32Array(b.buffer);
        scan._xy = scan.xy;
      }
      const pts = scan._pts;
      ctx.fillStyle = col('--scan');
      const r = Math.max(1.5, Math.min(3, s * 0.02));
      for (let i = 0; i + 1 < pts.length; i += 2) ctx.fillRect(this.sx(pts[i]) - r / 2, this.sy(pts[i + 1]) - r / 2, r, r);
    }

    // goals: mission (console), external (RViz /goal_pose), pending waypoints
    const m = this.d.mission;
    if (m && m.poses) {
      m.poses.forEach((p, i) => this.drawPose(p[0], p[1], p[2], col('--goal'), m.kind === 'waypoints' ? String(i + 1) : '', true));
      if (m.poses.length > 1) this.polyline(m.poses, col('--goal'), [6, 5]);
    }
    if (this.d.extGoal && !(m && m.poses)) this.drawPose(...this.d.extGoal, col('--goal'), 'RViz', true);
    if (this.d.pending.length) {
      this.d.pending.forEach((p, i) => this.drawPose(p[0], p[1], p[2], col('--accent'), String(i + 1), false));
      if (this.d.pending.length > 1) this.polyline(this.d.pending, col('--accent'), [4, 4]);
    }
    const pv = this.d.preview;
    if (pv) this.drawPose(pv.x, pv.y, pv.yaw ?? 0, pv.tool === 'init' ? '#999' : col('--accent'), '', pv.yaw !== null);

    // people
    const ppl = this.d.people;
    if (this.layers.people && ppl) {
      ctx.font = '600 11px system-ui, sans-serif';
      for (const p of ppl.ai || []) {
        if (!p.map) continue;
        const x = this.sx(p.map[0]), y = this.sy(p.map[1]), r = Math.max(6, 0.22 * s);
        ctx.strokeStyle = '#13b5c9'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke();
        ctx.fillStyle = '#13b5c9';
        ctx.fillText('AI ' + p.id, x + r + 3, y - 2);
      }
      for (const p of ppl.demo || []) {
        const x = this.sx(p.x), y = this.sy(p.y), r = Math.max(5, 0.18 * s);
        const c = p.c || [0.5, 0.5, 0.5];
        ctx.fillStyle = `rgba(${c[0] * 255},${c[1] * 255},${c[2] * 255},0.75)`;
        ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = col('--text');
        ctx.fillText((p.text || '').split('\n').join(' '), x + r + 3, y + 12);
      }
    }

    // robot
    if (pose) {
      const [x, y, yaw] = pose;
      ctx.save();
      ctx.translate(this.sx(x), this.sy(y));
      ctx.rotate(-yaw);
      ctx.fillStyle = col('--robot'); ctx.globalAlpha = 0.35;
      ctx.beginPath();
      FOOTPRINT.forEach(([fx, fy], i) => (i ? ctx.lineTo(fx * s, -fy * s) : ctx.moveTo(fx * s, -fy * s)));
      ctx.closePath(); ctx.fill();
      ctx.globalAlpha = 1;
      const k = Math.max(9, 0.12 * s);
      ctx.beginPath();
      ctx.moveTo(k * 1.4, 0); ctx.lineTo(-k * 0.8, k * 0.75); ctx.lineTo(-k * 0.4, 0); ctx.lineTo(-k * 0.8, -k * 0.75);
      ctx.closePath();
      ctx.fillStyle = col('--robot'); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
      ctx.fill(); ctx.stroke();
      ctx.restore();
    }

    this.drawScale(col('--text'));
  }

  polyline(pts, color, dash) {
    const ctx = this.ctx;
    ctx.save();
    ctx.strokeStyle = color; ctx.lineWidth = 1.5; ctx.setLineDash(dash);
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(this.sx(p[0]), this.sy(p[1])) : ctx.moveTo(this.sx(p[0]), this.sy(p[1]))));
    ctx.stroke();
    ctx.restore();
  }

  drawPose(x, y, yaw, color, label, arrow) {
    const ctx = this.ctx;
    const px = this.sx(x), py = this.sy(y);
    ctx.save();
    ctx.fillStyle = color; ctx.strokeStyle = color; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(px, py, 6, 0, Math.PI * 2); ctx.fill();
    if (arrow && yaw !== null && yaw !== undefined) {
      const L = 26;
      const ex = px + Math.cos(yaw) * L, ey = py - Math.sin(yaw) * L;
      ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(ex, ey); ctx.stroke();
      const a = Math.atan2(ey - py, ex - px);
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex - 9 * Math.cos(a - 0.45), ey - 9 * Math.sin(a - 0.45));
      ctx.lineTo(ex - 9 * Math.cos(a + 0.45), ey - 9 * Math.sin(a + 0.45));
      ctx.closePath(); ctx.fill();
    }
    if (label) {
      ctx.font = '700 11px system-ui, sans-serif';
      ctx.fillStyle = '#fff';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      if (label.length <= 2) ctx.fillText(label, px, py + 0.5);
      else { ctx.fillStyle = color; ctx.textAlign = 'left'; ctx.fillText(label, px + 9, py - 9); }
    }
    ctx.restore();
  }

  drawGridLines() {
    const ctx = this.ctx;
    const [x0, y1] = this.world(0, 0), [x1, y0] = this.world(this.w, this.h);
    const step = this.view.s > 40 ? 1 : (this.view.s > 10 ? 5 : 10);
    ctx.save();
    ctx.strokeStyle = 'rgba(128,128,128,0.25)'; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) { ctx.moveTo(this.sx(x), 0); ctx.lineTo(this.sx(x), this.h); }
    for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) { ctx.moveTo(0, this.sy(y)); ctx.lineTo(this.w, this.sy(y)); }
    ctx.stroke();
    ctx.strokeStyle = 'rgba(224,56,75,0.6)';
    ctx.beginPath(); ctx.moveTo(this.sx(0), this.sy(0)); ctx.lineTo(this.sx(1), this.sy(0)); ctx.stroke();
    ctx.strokeStyle = 'rgba(30,158,90,0.6)';
    ctx.beginPath(); ctx.moveTo(this.sx(0), this.sy(0)); ctx.lineTo(this.sx(0), this.sy(1)); ctx.stroke();
    ctx.restore();
  }

  drawScale(color) {
    const ctx = this.ctx;
    const target = 100 / this.view.s;
    const opts = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20];
    const m = opts.find((o) => o >= target) || 20;
    const L = m * this.view.s;
    // bottom-left above the HUD; phones: below the toolbar (bottom sheet covers the bottom)
    const x = 16, y = this.w < 800 ? 120 : this.h - 46;
    ctx.save();
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x, y - 4); ctx.lineTo(x, y); ctx.lineTo(x + L, y); ctx.lineTo(x + L, y - 4); ctx.stroke();
    ctx.font = '11px system-ui, sans-serif'; ctx.textAlign = 'center';
    ctx.fillText(m < 1 ? `${m * 100} cm` : `${m} m`, x + L / 2, y - 6);
    ctx.restore();
  }
}
