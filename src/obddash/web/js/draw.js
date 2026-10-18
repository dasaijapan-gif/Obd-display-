/* obddash drawing primitives — the "schematic" vocabulary shared by every page.
   Conventions: hairline = 1 px solid (never dashed), data lines 2 px, text never takes a data colour. */
(function () {
  'use strict';
  const D = {};
  let C = null; // colour tokens, set by D.init
  D.init = (colors) => { C = colors; D.C = C; };

  const px = (v) => Math.round(v) + 0.5; // crisp 1 px lines
  D.px = px;

  D.font = (size, weight = 400) => `${weight} ${Math.max(7, size).toFixed(1)}px ${C.font}`;

  D.line = function (ctx, x1, y1, x2, y2, color, width = 1) {
    ctx.beginPath();
    if (width <= 1) { ctx.moveTo(px(x1), px(y1)); ctx.lineTo(px(x2), px(y2)); } else { ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); }
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'butt'; ctx.stroke();
  };
  D.rect = function (ctx, x, y, w, h, color, width = 1) {
    ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.strokeRect(px(x), px(y), Math.round(w), Math.round(h));
  };
  D.fill = function (ctx, x, y, w, h, color) { ctx.fillStyle = color; ctx.fillRect(x, y, w, h); };

  D.text = function (ctx, str, x, y, o = {}) {
    ctx.font = D.font(o.size || 12, o.weight || 400);
    ctx.fillStyle = o.color || C.text;
    ctx.textAlign = o.align || 'left';
    ctx.textBaseline = o.base || 'alphabetic';
    if (o.letter) { // manual letter spacing for uppercase labels
      const chars = String(str).split('');
      const widths = chars.map((c) => ctx.measureText(c).width);
      const total = widths.reduce((a, b) => a + b, 0) + o.letter * (chars.length - 1);
      let cx = x;
      if (ctx.textAlign === 'center') cx = x - total / 2; else if (ctx.textAlign === 'right') cx = x - total;
      ctx.textAlign = 'left';
      chars.forEach((c, i) => { ctx.fillText(c, cx, y); cx += widths[i] + o.letter; });
      return total;
    }
    ctx.fillText(String(str), x, y);
    return ctx.measureText(String(str)).width;
  };
  // small uppercase label with tracking
  D.label = function (ctx, str, x, y, o = {}) {
    return D.text(ctx, String(str).toUpperCase(), x, y, { size: o.size || 10, color: o.color || C.muted, align: o.align, base: o.base, letter: o.letter == null ? 1.2 : o.letter, weight: o.weight });
  };
  D.measure = function (ctx, str, size, weight = 400) { ctx.font = D.font(size, weight); return ctx.measureText(String(str)).width; };

  // ---- blueprint grid ---------------------------------------------------------
  D.grid = function (ctx, w, h, minor = 20, major = 100) {
    ctx.save();
    ctx.lineWidth = 1;
    for (let x = 0; x <= w; x += minor) { ctx.strokeStyle = x % major === 0 ? C.gridMajor : C.grid; ctx.beginPath(); ctx.moveTo(px(x), 0); ctx.lineTo(px(x), h); ctx.stroke(); }
    for (let y = 0; y <= h; y += minor) { ctx.strokeStyle = y % major === 0 ? C.gridMajor : C.grid; ctx.beginPath(); ctx.moveTo(0, px(y)); ctx.lineTo(w, px(y)); ctx.stroke(); }
    ctx.restore();
  };

  // ---- panel with corner brackets and a small-caps title -----------------------
  D.panel = function (ctx, x, y, w, h, title, o = {}) {
    const L = o.bracket || 10, col = o.color || C.steel;
    ctx.strokeStyle = col; ctx.lineWidth = 1;
    const X = px(x), Y = px(y), X2 = px(x + w), Y2 = px(y + h);
    ctx.beginPath();
    ctx.moveTo(X, Y + L); ctx.lineTo(X, Y); ctx.lineTo(X + L, Y);
    ctx.moveTo(X2 - L, Y); ctx.lineTo(X2, Y); ctx.lineTo(X2, Y + L);
    ctx.moveTo(X2, Y2 - L); ctx.lineTo(X2, Y2); ctx.lineTo(X2 - L, Y2);
    ctx.moveTo(X + L, Y2); ctx.lineTo(X, Y2); ctx.lineTo(X, Y2 - L);
    ctx.stroke();
    if (title) D.label(ctx, title, x + L + 4, y + 11, { size: o.titleSize || 10, color: o.titleColor || C.muted });
    if (o.note) D.label(ctx, o.note, x + w - L - 4, y + 11, { size: o.titleSize || 10, color: C.faint, align: 'right' });
  };

  // ---- arcs, ticks, hatch ----------------------------------------------------------
  D.arc = function (ctx, cx, cy, r, a0, a1, color, width = 1) {
    ctx.beginPath(); ctx.arc(cx, cy, r, a0, a1); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'butt'; ctx.stroke();
  };
  D.ticks = function (ctx, cx, cy, r, a0, a1, n, len, color, width = 1) {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
    for (let i = 0; i <= n; i++) {
      const a = a0 + ((a1 - a0) * i) / n;
      ctx.moveTo(cx + Math.cos(a) * r, cy + Math.sin(a) * r);
      ctx.lineTo(cx + Math.cos(a) * (r - len), cy + Math.sin(a) * (r - len));
    }
    ctx.stroke();
  };
  // 45° hatching clipped to an annular sector
  D.hatchSector = function (ctx, cx, cy, r0, r1, a0, a1, color, spacing = 6) {
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, cy, r1, a0, a1); ctx.arc(cx, cy, r0, a1, a0, true); ctx.closePath(); ctx.clip();
    ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.beginPath();
    const R = r1 + 2;
    for (let d = -2 * R; d <= 2 * R; d += spacing) { ctx.moveTo(cx + d - R, cy - R); ctx.lineTo(cx + d + R, cy + R); }
    ctx.stroke(); ctx.restore();
  };
  D.hatchRect = function (ctx, x, y, w, h, color, spacing = 6) {
    ctx.save(); ctx.beginPath(); ctx.rect(x, y, w, h); ctx.clip();
    ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.beginPath();
    for (let d = -h; d <= w + h; d += spacing) { ctx.moveTo(x + d, y); ctx.lineTo(x + d - h, y + h); }
    ctx.stroke(); ctx.restore();
  };

  // ---- leader line with elbow + label ------------------------------------------
  D.leader = function (ctx, x1, y1, x2, y2, label, o = {}) {
    const col = o.color || C.steel;
    ctx.strokeStyle = col; ctx.lineWidth = 1; ctx.beginPath();
    ctx.moveTo(px(x1), px(y1)); ctx.lineTo(px(x2), px(y2));
    const dir = o.dir || (x2 >= x1 ? 1 : -1), len = o.tail == null ? 14 : o.tail;
    ctx.lineTo(px(x2 + dir * len), px(y2)); ctx.stroke();
    ctx.beginPath(); ctx.arc(x1, y1, 2, 0, Math.PI * 2); ctx.fillStyle = col; ctx.fill();
    if (label != null) D.text(ctx, label, x2 + dir * (len + 4), y2 + 4, { size: o.size || 11, color: o.textColor || C.text, align: dir > 0 ? 'left' : 'right' });
  };

  // ---- vertical fill bar with scale ticks ---------------------------------------
  D.vbar = function (ctx, x, y, w, h, frac, color, o = {}) {
    D.rect(ctx, x, y, w, h, C.steel);
    const n = o.ticks || 10;
    for (let i = 1; i < n; i++) { const yy = y + (h * i) / n; D.line(ctx, x, yy, x + (i % 5 === 0 ? 8 : 4), yy, C.steelDim); }
    const f = U.clamp(frac || 0, 0, 1);
    if (f > 0) { ctx.fillStyle = color; ctx.fillRect(x + 2, y + h - f * (h - 4) - 2, w - 4, f * (h - 4)); }
    if (U.isNum(o.peakFrac) && o.peakFrac > 0) { const yy = y + h - U.clamp(o.peakFrac, 0, 1) * (h - 4) - 2; D.line(ctx, x - 4, yy, x + w + 4, yy, o.peakColor || C.text); }
    if (U.isNum(o.markFrac)) { const yy = y + h - U.clamp(o.markFrac, 0, 1) * (h - 4) - 2; D.line(ctx, x - 3, yy, x + w + 3, yy, o.markColor || C.amber); }
  };
  D.hbar = function (ctx, x, y, w, h, frac, color) {
    D.rect(ctx, x, y, w, h, C.steel);
    const f = U.clamp(frac || 0, 0, 1);
    if (f > 0) { ctx.fillStyle = color; ctx.fillRect(x + 2, y + 2, f * (w - 4), h - 4); }
  };

  // ---- marker dot with 2 px surface ring -----------------------------------------
  D.dot = function (ctx, x, y, r, color) {
    ctx.beginPath(); ctx.arc(x, y, r + 2, 0, Math.PI * 2); ctx.fillStyle = C.bg; ctx.fill();
    ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
  };
  D.roundTopBar = function (ctx, x, y, w, h, color, r = 4) {
    if (h <= 0) return;
    r = Math.min(r, w / 2, h);
    ctx.beginPath(); ctx.moveTo(x, y + h); ctx.lineTo(x, y + r); ctx.arcTo(x, y, x + r, y, r); ctx.lineTo(x + w - r, y);
    ctx.arcTo(x + w, y, x + w, y + r, r); ctx.lineTo(x + w, y + h); ctx.closePath(); ctx.fillStyle = color; ctx.fill();
  };

  // ---- generic time/xy chart: hairline grid, 2 px series, selective labels --------
  // spec: {x:[min,max], y:[min,max], xTicks:[...], yTicks:[...], xFmt, yFmt, series:[{pts:[[x,y]...], color, width, alpha}],
  //        refY:[{y,label,color}], refX:[{x,label,color}], yLabel, xLabel, legend:[{label,color}]}
  D.chart = function (ctx, r, spec) {
    const padL = spec.padL == null ? 44 : spec.padL, padR = spec.padR == null ? 10 : spec.padR, padT = spec.padT == null ? 8 : spec.padT, padB = spec.padB == null ? 18 : spec.padB;
    const x0 = r.x + padL, y0 = r.y + padT, pw = r.w - padL - padR, ph = r.h - padT - padB;
    const [xa, xb] = spec.x, [ya, yb] = spec.y;
    const X = (v) => x0 + ((v - xa) / (xb - xa || 1)) * pw;
    const Y = (v) => y0 + ph - ((v - ya) / (yb - ya || 1)) * ph;
    ctx.save();
    // grid
    (spec.yTicks || []).forEach((t) => { D.line(ctx, x0, Y(t), x0 + pw, Y(t), C.steelDim); D.text(ctx, spec.yFmt ? spec.yFmt(t) : t, x0 - 6, Y(t) + 3, { size: 9, color: C.faint, align: 'right' }); });
    (spec.xTicks || []).forEach((t) => { D.line(ctx, X(t), y0, X(t), y0 + ph, C.steelDim); if (!spec.hideX) D.text(ctx, spec.xFmt ? spec.xFmt(t) : t, X(t), y0 + ph + 12, { size: 9, color: C.faint, align: 'center' }); });
    // axes (hairline, one step up)
    D.line(ctx, x0, y0, x0, y0 + ph, C.steel); D.line(ctx, x0, y0 + ph, x0 + pw, y0 + ph, C.steel);
    if (spec.yLabel) D.label(ctx, spec.yLabel, x0 + 4, y0 + 9, { size: 9, color: C.faint });
    if (spec.xLabel) D.label(ctx, spec.xLabel, x0 - 14, y0 + ph + 12, { size: 9, color: C.faint, align: 'right' });
    // reference lines
    (spec.refY || []).forEach((rl) => { if (!U.isNum(rl.y)) return; const yy = U.clamp(Y(rl.y), y0, y0 + ph); D.line(ctx, x0, yy, x0 + pw, yy, rl.color || C.steel); if (rl.label) D.label(ctx, rl.label, x0 + pw - 4, yy - 3, { size: 9, color: rl.textColor || C.muted, align: 'right' }); });
    (spec.refX || []).forEach((rl) => { if (!U.isNum(rl.x)) return; const xx = U.clamp(X(rl.x), x0, x0 + pw); D.line(ctx, xx, y0, xx, y0 + ph, rl.color || C.steel); if (rl.label) D.label(ctx, rl.label, xx + 3, y0 + 9, { size: 9, color: rl.textColor || C.muted }); });
    // series, clipped to the plot
    ctx.beginPath(); ctx.rect(x0, y0 - 1, pw, ph + 2); ctx.clip();
    (spec.series || []).forEach((s) => {
      if (!s.pts || s.pts.length < 2) return;
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < s.pts.length; i++) {
        const p = s.pts[i]; if (!U.isNum(p[0]) || !U.isNum(p[1])) { started = false; continue; }
        const xx = X(p[0]), yy = Y(U.clamp(p[1], ya - (yb - ya), yb + (yb - ya)));
        if (!started) { ctx.moveTo(xx, yy); started = true; } else ctx.lineTo(xx, yy);
      }
      if (s.fill) { const last = s.pts[s.pts.length - 1], first = s.pts[0]; ctx.lineTo(X(last[0]), Y(ya)); ctx.lineTo(X(first[0]), Y(ya)); ctx.closePath(); ctx.fillStyle = U.alpha(s.color, 0.10); ctx.fill(); ctx.beginPath(); started = false; for (let i = 0; i < s.pts.length; i++) { const p = s.pts[i]; if (!U.isNum(p[0]) || !U.isNum(p[1])) continue; const xx = X(p[0]), yy = Y(p[1]); if (!started) { ctx.moveTo(xx, yy); started = true; } else ctx.lineTo(xx, yy); } }
      ctx.strokeStyle = s.alpha != null ? U.alpha(s.color, s.alpha) : s.color; ctx.lineWidth = s.width || 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round'; ctx.stroke();
    });
    ctx.restore();
    // legend (always for >= 2 series)
    if (spec.legend && spec.legend.length) {
      let lx = x0 + pw - 6;
      for (let i = spec.legend.length - 1; i >= 0; i--) {
        const it = spec.legend[i];
        const w = D.measure(ctx, it.label, 9) + 1.2 * it.label.length;
        lx -= w; D.label(ctx, it.label, lx, y0 + 9, { size: 9, color: C.muted });
        lx -= 16; D.line(ctx, lx, y0 + 6, lx + 12, y0 + 6, it.color, 2); lx -= 10;
      }
    }
    return { X, Y, x0, y0, pw, ph };
  };

  // ---- small arc gauge (0..1) ---------------------------------------------------
  D.dial = function (ctx, cx, cy, r, frac, o = {}) {
    const a0 = Math.PI * 0.75, a1 = Math.PI * 2.25;
    D.arc(ctx, cx, cy, r, a0, a1, C.steel, 1);
    D.ticks(ctx, cx, cy, r, a0, a1, o.ticks || 10, 5, C.steelDim);
    const f = U.clamp(frac || 0, 0, 1);
    if (f > 0) D.arc(ctx, cx, cy, r - 5, a0, a0 + (a1 - a0) * f, o.color || C.ink, 4);
    if (o.value != null) D.text(ctx, o.value, cx, cy + (o.valueSize || 16) * 0.35, { size: o.valueSize || 16, color: C.text, align: 'center', weight: 600 });
    if (o.label) D.label(ctx, o.label, cx, cy + r + 12, { size: 9, color: C.muted, align: 'center' });
  };

  // ---- car outline, top view, nose up; (cx, cy) centre, len x wid in px -------------
  D.carTopView = function (ctx, cx, cy, len, wid, color) {
    const L = len / 2, W = wid / 2;
    ctx.save(); ctx.translate(cx, cy); ctx.strokeStyle = color; ctx.lineWidth = 1.2; ctx.lineJoin = 'round';
    ctx.beginPath();
    ctx.moveTo(-W * 0.55, -L); ctx.quadraticCurveTo(-W, -L, -W, -L * 0.72);
    ctx.lineTo(-W, L * 0.72); ctx.quadraticCurveTo(-W, L, -W * 0.55, L);
    ctx.lineTo(W * 0.55, L); ctx.quadraticCurveTo(W, L, W, L * 0.72);
    ctx.lineTo(W, -L * 0.72); ctx.quadraticCurveTo(W, -L, W * 0.55, -L); ctx.closePath(); ctx.stroke();
    // cabin
    ctx.beginPath(); ctx.moveTo(-W * 0.8, -L * 0.25); ctx.lineTo(-W * 0.62, -L * 0.5); ctx.lineTo(W * 0.62, -L * 0.5); ctx.lineTo(W * 0.8, -L * 0.25);
    ctx.lineTo(W * 0.8, L * 0.35); ctx.lineTo(W * 0.62, L * 0.5); ctx.lineTo(-W * 0.62, L * 0.5); ctx.lineTo(-W * 0.8, L * 0.35); ctx.closePath(); ctx.stroke();
    // axles + centre line
    D.line(ctx, -W * 1.25, -L * 0.62, W * 1.25, -L * 0.62, color); D.line(ctx, -W * 1.25, L * 0.62, W * 1.25, L * 0.62, color);
    D.line(ctx, 0, -L * 1.08, 0, L * 1.08, U.alpha(color, 0.5));
    // wheels
    const wl = L * 0.22, ww = W * 0.22;
    [[-W * 1.12, -L * 0.62], [W * 1.12, -L * 0.62], [-W * 1.12, L * 0.62], [W * 1.12, L * 0.62]].forEach(([x, y]) => { ctx.strokeRect(px(x - ww / 2), px(y - wl / 2), Math.round(ww), Math.round(wl)); });
    ctx.restore();
    return { wheelW: W * 0.22, wheelL: L * 0.22, frontY: cy - L * 0.62, rearY: cy + L * 0.62, wheelX: W * 1.12 };
  };

  window.D = D;
})();
