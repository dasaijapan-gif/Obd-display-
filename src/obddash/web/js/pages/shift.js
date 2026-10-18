/* SHIFT page: last downshift rev-match analysis (score, traces vs target rpm), live next-gear target, history. */
(function () {
  'use strict';
  const bg = new U.StaticLayer(() => {});
  function layout(w, h) {
    return {
      left: { x: 10, y: 10, w: w * 0.255 - 10, h: h - 56 },
      center: { x: w * 0.27, y: 10, w: w * 0.455, h: h - 56 },
      right: { x: w * 0.74, y: 10, w: w * 0.26 - 10, h: h * 0.86 - 10 },
    };
  }
  const gradeColor = (C, g) => (g === 'S' || g === 'A' ? C.green : g === 'B' ? C.ink : C.amber);

  window.PAGES = window.PAGES || [];
  window.PAGES.push({
    id: 'shift', name: 'SHIFT',
    draw(ctx, L, S) {
      const { w, h, s, dpr } = L, C = D.C, T = S.tele, cfg = S.cfg, lay = layout(w, h);
      bg.drawFn = (c, W, H) => {
        D.grid(c, W, H);
        D.panel(c, lay.left.x, lay.left.y, lay.left.w, lay.left.h, 'rev match', { note: 'last downshift' });
        D.panel(c, lay.center.x, lay.center.y, lay.center.w, lay.center.h, 'trace', { note: 't rel. engagement (s)' });
        D.panel(c, lay.right.x, lay.right.y, lay.right.w, lay.right.h, 'history', { note: 'last 20' });
      };
      ctx.drawImage(bg.get(w, h, dpr), 0, 0, w, h);
      const sh = T.shift || {}, last = sh.last, ses = sh.session || {}, lf = lay.left;

      // ---- hero ------------------------------------------------------------------------
      if (last) {
        const col = gradeColor(C, last.grade);
        const big = Math.min(lf.h * 0.26, lf.w * 0.30, 130);
        D.text(ctx, Math.round(last.score), lf.x + 16, lf.y + 34 + big, { size: big, weight: 600, color: C.text });
        const gw = D.measure(ctx, Math.round(last.score), big, 600);
        D.text(ctx, last.grade, lf.x + 16 + gw + 14, lf.y + 34 + big, { size: big * 0.62, weight: 700, color: C.text });
        D.fill(ctx, lf.x + 16 + gw + 14, lf.y + 40 + big, big * 0.38, 3, col);
        D.label(ctx, `▼ ${U.gearLabel(last.from_gear)} → ${U.gearLabel(last.to_gear)}   ${last.heel_toe ? 'heel-toe' : last.blip ? 'blip' : 'no blip'}`, lf.x + 16, lf.y + 58 + big, { size: 10, color: C.muted });
        const chips = [
          ['error', `${U.signed(last.error_pct, 1)} %`, Math.abs(last.error_pct) <= 2 ? C.green : Math.abs(last.error_pct) >= 8 ? C.amber : null],
          ['duration', `${U.fmt(last.duration_s, 2)} s`, null],
          ['target', `${U.fmtRpm(last.target_rpm)} rpm`, null],
          ['actual', `${U.fmtRpm(last.actual_rpm)} rpm`, null],
          ['blip', last.blip ? 'yes' : 'no', last.blip ? null : C.amber],
          ['jolt', last.jolt_g == null ? 'n/a' : `${U.fmt(last.jolt_g, 2)} g`, null],
        ];
        const ncol = lf.w >= 280 ? 3 : 2, nrow = Math.ceil(chips.length / ncol);
        const cy0 = lf.y + 84 + big, cw = (lf.w - 32) / ncol, ch = Math.min(46, Math.max(30, (lf.y + lf.h - 92 - cy0) / nrow));
        chips.forEach(([k, v, flag], i) => {
          const x = lf.x + 16 + (i % ncol) * cw, y = cy0 + Math.floor(i / ncol) * ch;
          D.label(ctx, k, x, y, { size: 9 });
          D.text(ctx, v, x, y + 15 + 2 * s, { size: 11 + 2 * s, color: C.text });
          if (flag) D.fill(ctx, x + D.measure(ctx, k.toUpperCase(), 9) + k.length * 1.2 + 6, y - 7, 6, 6, flag);
        });
      } else {
        D.text(ctx, 'NO DOWNSHIFT YET', lf.x + 16, lf.y + 60, { size: 14, color: C.muted });
        ['brake', 'clutch in', 'blip the throttle', 'shift down, clutch out'].forEach((t, i) => D.text(ctx, `${i + 1}. ${t}`, lf.x + 16, lf.y + 90 + i * 20, { size: 11, color: C.faint }));
      }
      // live next target + session
      const ny = lf.y + lf.h - 58;
      D.line(ctx, lf.x + 12, ny - 22, lf.x + lf.w - 12, ny - 22, C.steelDim);
      D.label(ctx, 'next ▼', lf.x + 16, ny - 6, { size: 9 });
      if (sh.target_rpm != null) {
        const near = sh.target_rpm > cfg.redline_rpm * 0.92;
        D.text(ctx, `${U.ordinal(sh.target_gear)} @ ${U.fmtRpm(sh.target_rpm)}`, lf.x + 16, ny + 14, { size: 15, color: near ? C.amber : C.text });
      } else D.text(ctx, 'n/a', lf.x + 16, ny + 14, { size: 15, color: C.faint });
      D.label(ctx, sh.state && sh.state !== 'idle' ? sh.state : '', lf.x + lf.w - 16, ny - 6, { size: 9, color: C.amber, align: 'right' });
      D.label(ctx, `${ses.downshifts || 0} down · avg ${U.fmtInt(ses.avg_score)} · best ${U.fmtInt(ses.best_score)} · ${ses.heel_toe_count || 0} heel-toe`, lf.x + 16, ny + 34, { size: 9, color: C.muted, letter: 0.6 });

      // ---- traces ----------------------------------------------------------------------
      const cn = lay.center;
      if (last && last.rpm_trace && last.rpm_trace.length > 2) {
        const te = last.t, rel = (p) => [p[0] - te, p[1]];
        const rpmT = last.rpm_trace.map(rel), thrT = (last.throttle_trace || []).map(rel), brkT = (last.brake_trace || []).map(rel);
        const xa = Math.min(rpmT[0][0], -0.5), xb = Math.max(rpmT[rpmT.length - 1][0], 0.5);
        const rmin = Math.min(...rpmT.map((p) => p[1]), last.target_rpm), rmax = Math.max(...rpmT.map((p) => p[1]), last.target_rpm);
        const ya = Math.max(0, Math.floor((rmin - 300) / 500) * 500), yb = Math.ceil((rmax + 300) / 500) * 500;
        const xTicks = []; for (let x = Math.ceil(xa * 2) / 2; x <= xb; x += 0.5) xTicks.push(Math.round(x * 2) / 2);
        const ph = cn.h - 40, top = cn.y + 22;
        const r1 = { x: cn.x + 6, y: top, w: cn.w - 12, h: ph * 0.56 }, r2 = { x: cn.x + 6, y: top + ph * 0.56, w: cn.w - 12, h: ph * 0.22 }, r3 = { x: cn.x + 6, y: top + ph * 0.78, w: cn.w - 12, h: ph * 0.22 };
        const yT = []; for (let y = ya; y <= yb; y += 1000) yT.push(y);
        const refX = [{ x: 0, label: 'engage', color: C.steel }, { x: -last.duration_s, label: 'clutch in', color: C.steelDim }];
        const ch = D.chart(ctx, r1, { x: [xa, xb], y: [ya, yb], xTicks, hideX: true, yTicks: yT, yFmt: (v) => (v / 1000).toFixed(0) + 'k', yLabel: 'rpm', padB: 4,
          series: [{ pts: rpmT, color: C.ink, width: 2 }], refY: [{ y: last.target_rpm, label: `target ${U.fmtRpm(last.target_rpm)}`, color: C.ink2, textColor: C.muted }], refX });
        // actual rpm marker + leader
        const ax = ch.X(0), ay = ch.Y(last.actual_rpm);
        D.dot(ctx, ax, ay, 4, C.ink);
        const dir = ax > ch.x0 + ch.pw * 0.7 ? -1 : 1;
        D.leader(ctx, ax, ay, ax + dir * 26, ay - 18, `${U.fmtRpm(last.actual_rpm)} (${U.signed(last.error_pct, 1)} %)`, { dir, size: 10 });
        D.chart(ctx, r2, { x: [xa, xb], y: [0, 100], xTicks, hideX: true, yTicks: [0, 50, 100], yLabel: 'throttle %', padB: 4, series: [{ pts: thrT, color: C.ink, width: 2, fill: true }], refX: [{ x: 0, color: C.steel }, { x: -last.duration_s, color: C.steelDim }] });
        const bmax = Math.max(20, Math.ceil((Math.max(...brkT.map((p) => p[1]), 1) * 1.3) / 20) * 20);
        D.chart(ctx, r3, { x: [xa, xb], y: [0, bmax], xTicks, xFmt: (v) => (v > 0 ? '+' : '') + v.toFixed(1), yTicks: [0, bmax / 2, bmax], yLabel: 'brake bar', series: [{ pts: brkT, color: U.alpha(C.text, 0.85), width: 2, fill: true }], refX: [{ x: 0, color: C.steel }, { x: -last.duration_s, color: C.steelDim }] });
      } else {
        D.text(ctx, 'traces appear after the first downshift', cn.x + cn.w / 2, cn.y + cn.h / 2, { size: 12, color: C.faint, align: 'center' });
      }

      // ---- history ---------------------------------------------------------------------
      const rt = lay.right, hist = ses.history || [];
      const r0 = { x: rt.x + 10, y: rt.y + 26, w: rt.w - 20, h: rt.h - 36 };
      const padL = 6, padR = 24, padB = 14, pw = r0.w - padL - padR, ph2 = r0.h - padB - 8;
      const yOf = (v) => r0.y + 8 + ph2 - (U.clamp(v, 0, 100) / 100) * ph2;
      [[95, 'S'], [85, 'A'], [70, 'B'], [50, 'C']].forEach(([v, g]) => { D.line(ctx, r0.x + padL, yOf(v), r0.x + padL + pw, yOf(v), C.steelDim); D.text(ctx, g, r0.x + padL + pw + 6, yOf(v) + 3, { size: 9, color: C.faint }); });
      D.line(ctx, r0.x + padL, yOf(0), r0.x + padL + pw, yOf(0), C.steel);
      const n = 20, slot = pw / n, bw = Math.min(24, slot - 3);
      hist.forEach((v, i) => {
        const idx = n - hist.length + i, x = r0.x + padL + idx * slot + (slot - bw) / 2;
        const top = Math.min(yOf(v), yOf(0) - 2); // a zero score still shows as a 2 px stub
        D.roundTopBar(ctx, x, top, bw, yOf(0) - top, v < 50 ? C.amber : i === hist.length - 1 ? C.ink : U.alpha(C.ink, 0.55), Math.min(4, yOf(0) - top));
      });
      if (hist.length) {
        const avg = hist.reduce((a, b) => a + b, 0) / hist.length;
        D.line(ctx, r0.x + padL, yOf(avg), r0.x + padL + pw, yOf(avg), C.text);
        D.label(ctx, `avg ${avg.toFixed(0)}`, r0.x + padL + 2, yOf(avg) - 4, { size: 9, color: C.muted });
      } else D.text(ctx, 'no scores yet', r0.x + r0.w / 2, r0.y + r0.h / 2, { size: 11, color: C.faint, align: 'center' });
      D.label(ctx, 'oldest', r0.x + padL, r0.y + r0.h, { size: 8, color: C.faint });
      D.label(ctx, 'latest', r0.x + padL + pw, r0.y + r0.h, { size: 8, color: C.faint, align: 'right' });
    },
  });
})();
