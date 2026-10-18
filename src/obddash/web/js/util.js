/* obddash UI utilities: math, formatting, smoothing, HiDPI canvas, colour tokens. */
(function () {
  'use strict';
  const U = {};

  U.clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  U.lerp = (a, b, t) => a + (b - a) * t;
  U.deg = (r) => (r * 180) / Math.PI;
  U.rad = (d) => (d * Math.PI) / 180;
  U.isNum = (v) => typeof v === 'number' && Number.isFinite(v);

  // ---- formatting ---------------------------------------------------------
  U.fmt = (v, digits = 0, dash = '—') => (U.isNum(v) ? v.toFixed(digits) : dash);
  U.fmtInt = (v) => (U.isNum(v) ? Math.round(v).toString() : '—');
  U.fmtRpm = (v) => (U.isNum(v) ? Math.round(v).toLocaleString('en-US').replace(/,/g, ' ') : '—'); // thin space
  U.fmtTime = (s) => {
    if (!U.isNum(s)) return '--:--';
    s = Math.max(0, Math.floor(s));
    const m = Math.floor(s / 60), r = s % 60;
    return `${m.toString().padStart(2, '0')}:${r.toString().padStart(2, '0')}`;
  };
  U.gearLabel = (g) => (g === -1 ? 'R' : g === 0 || g == null ? 'N' : String(g));
  U.ordinal = (g) => (g === 1 ? '1st' : g === 2 ? '2nd' : g === 3 ? '3rd' : `${g}th`);
  U.signed = (v, d = 1) => {
    if (!U.isNum(v)) return '—';
    const r = Math.abs(v).toFixed(d);
    if (Number(r) === 0) return r; // never print a negative zero
    return (v > 0 ? '+' : '−') + r;
  };
  // clean axis ticks: 0..max in steps of 1/2/2.5/5 x 10^k, at most `count` intervals
  U.niceTicks = function (max, count = 4) {
    const raw = Math.max(max, 1e-6) / count, mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((st) => st >= raw) || raw;
    const top = Math.ceil(max / step - 1e-9) * step, out = [];
    for (let v = 0; v <= top + step * 1e-6; v += step) out.push(Math.round(v * 1000) / 1000);
    return { ticks: out, max: top };
  };

  // ---- smoothing for live readouts (critically damped approach) -----------
  U.Smooth = class {
    constructor(tau = 0.08, v = 0) { this.tau = tau; this.v = v; this.target = v; }
    to(target, dt) {
      if (!U.isNum(target)) return this.v;
      this.target = target;
      const k = 1 - Math.exp(-dt / this.tau);
      this.v += (target - this.v) * k;
      if (Math.abs(target - this.v) < 1e-4) this.v = target;
      return this.v;
    }
    set(v) { this.v = v; this.target = v; }
  };

  // ---- colour tokens from CSS ---------------------------------------------
  U.colors = function () {
    const cs = getComputedStyle(document.documentElement);
    const g = (n) => cs.getPropertyValue(n).trim();
    return {
      bg: g('--bg'), bg2: g('--bg-2'), grid: g('--grid'), gridMajor: g('--grid-major'),
      ink: g('--ink'), ink2: g('--ink-2'), inkDim: g('--ink-dim'), steel: g('--steel'), steelDim: g('--steel-dim'),
      text: g('--text'), muted: g('--muted'), faint: g('--faint'),
      amber: g('--amber'), red: g('--red'), green: g('--green'),
      font: g('--font-mono') || 'monospace',
    };
  };
  U.alpha = function (hex, a) {
    const h = hex.replace('#', '');
    const n = parseInt(h.length === 3 ? h.split('').map((c) => c + c).join('') : h, 16);
    return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${a})`;
  };

  // ---- HiDPI canvas --------------------------------------------------------
  U.fitCanvas = function (canvas) {
    const dpr = Math.min((window.devicePixelRatio || 1) * (window.OBD_PIXEL_SCALE || 1), 3);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    const pw = Math.round(w * dpr), ph = Math.round(h * dpr);
    if (canvas.width !== pw || canvas.height !== ph) { canvas.width = pw; canvas.height = ph; }
    return { w, h, dpr };
  };

  // A layer rendered once per size change (grids, tick rings, outlines).
  U.StaticLayer = class {
    constructor(drawFn) { this.drawFn = drawFn; this.canvas = document.createElement('canvas'); this.key = ''; }
    get(w, h, dpr, extraKey = '') {
      const key = `${w}x${h}@${dpr}|${extraKey}|${U.epoch}`;
      if (key !== this.key) {
        this.key = key;
        this.canvas.width = Math.round(w * dpr); this.canvas.height = Math.round(h * dpr);
        const ctx = this.canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, w, h);
        this.drawFn(ctx, w, h);
      }
      return this.canvas;
    }
    invalidate() { this.key = ''; }
  };

  // ---- misc -----------------------------------------------------------------
  U.epoch = 0; // bump to force static layers to redraw (e.g. after web fonts load)
  U.now = () => performance.now() / 1000;
  U.store = {
    get(k, d) { try { const v = localStorage.getItem(k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { /* storage unavailable */ } },
  };
  U.scale = (w, h) => Math.max(Math.min(w / 1280, h / 400), Math.min(w / 1024, h / 600));

  window.U = U;
})();
