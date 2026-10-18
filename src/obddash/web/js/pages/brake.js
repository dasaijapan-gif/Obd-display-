/* BRAKE page: top-view car with axle pressures, live pressure hero + 8 s trace, trail-braking and score dials, event table. */
(function () {
  'use strict';
  const bg = new U.StaticLayer(() => {});
  function layout(w, h) {
    return {
      car: { x: 10, y: 10, w: w * 0.27 - 10, h: h - 56 },
      center: { x: w * 0.285, y: 10, w: w * 0.425, h: h - 56 },
      right: { x: w * 0.725, y: 10, w: w * 0.275 - 10, h: h * 0.86 - 10 },
    };
  }
  window.PAGES = window.PAGES || [];
  window.PAGES.push({
    id: 'brake', name: 'BRAKE',
    draw(ctx, L, S) {
      const { w, h, s, dpr } = L, C = D.C, T = S.tele, sm = S.sm, cfg = S.cfg, lay = layout(w, h);
      const br = T.brake || {}, maxBar = cfg.max_brake_bar || 100;
      bg.drawFn = (c, W, H) => {
        D.grid(c, W, H);
        D.panel(c, lay.car.x, lay.car.y, lay.car.w, lay.car.h, 'hydraulic', { note: 'bar' });
        D.panel(c, lay.center.x, lay.center.y, lay.center.w, lay.center.h, 'pressure', { note: 'last 8 s' });
        D.panel(c, lay.right.x, lay.right.y, lay.right.w, lay.right.h, 'events', { note: 'latest first' });
      };
      ctx.drawImage(bg.get(w, h, dpr), 0, 0, w, h);

      // ---- car -------------------------------------------------------------------------
      const cp = lay.car, len = Math.min(cp.h * 0.72, (cp.w * 0.52) / 0.525), wid = len * 0.42;
      const cx = cp.x + cp.w * 0.62, cy = cp.y + cp.h * 0.52;
      const car = D.carTopView(ctx, cx, cy, len, wid, C.steel);
      const front = U.isNum(br.front_bar) ? br.front_bar : br.pressure_bar || 0, rear = U.isNum(br.rear_bar) ? br.rear_bar : br.pressure_bar || 0;
      const fillWheel = (x, y, frac) => { const ww = car.wheelW, wl = car.wheelL, f = U.clamp(frac, 0, 1); if (f > 0) D.fill(ctx, x - ww / 2 + 1, y + wl / 2 - 1 - f * (wl - 2), ww - 2, f * (wl - 2), C.ink); };
      fillWheel(cx - car.wheelX, car.frontY, front / maxBar); fillWheel(cx + car.wheelX, car.frontY, front / maxBar);
      fillWheel(cx - car.wheelX, car.rearY, rear / maxBar); fillWheel(cx + car.wheelX, car.rearY, rear / maxBar);
      const lx = cp.x + 14;
      D.leader(ctx, cx - car.wheelX - car.wheelW / 2, car.frontY, lx + 40, car.frontY - 10, null, { dir: -1, tail: 0 });
      D.label(ctx, 'front', lx, car.frontY - 14, { size: 9 });
      D.text(ctx, U.fmt(front, 1), lx, car.frontY + 6 + 4 * s, { size: 14 + 4 * s, color: C.text });
      D.leader(ctx, cx - car.wheelX - car.wheelW / 2, car.rearY, lx + 40, car.rearY - 10, null, { dir: -1, tail: 0 });
      D.label(ctx, 'rear', lx, car.rearY - 14, { size: 9 });
      D.text(ctx, U.fmt(rear, 1), lx, car.rearY + 6 + 4 * s, { size: 14 + 4 * s, color: C.text });
      if (!U.isNum(br.front_bar)) D.label(ctx, 'no split sensor · system pressure shown', cp.x + cp.w / 2, cp.y + cp.h - 8, { size: 8, color: C.faint, align: 'center' });
      D.label(ctx, '▲ front', cx, cy - len / 2 - 10, { size: 8, color: C.faint, align: 'center' });

      // ---- centre: hero + trace -------------------------------------------------------
      const cn = lay.center, big = Math.min(cn.h * 0.26, cn.w * 0.22);
      D.text(ctx, U.fmt(sm.press.v * maxBar / 100, 0), cn.x + 16, cn.y + 30 + big, { size: big, weight: 600, color: C.text });
      const bw = D.measure(ctx, U.fmt(sm.press.v * maxBar / 100, 0), big, 600);
      D.label(ctx, 'bar', cn.x + 16 + bw + 8, cn.y + 30 + big, { size: 11, color: C.muted });
      D.label(ctx, `${U.fmtInt(sm.press.v)} % of ${maxBar} bar`, cn.x + 16 + bw + 8, cn.y + 30 + big - 20, { size: 9, color: C.faint });
      const srcX = cn.x + cn.w - 16, src = br.source === 'estimate' ? 'ESTIMATED' : br.source === 'can' ? 'CAN' : 'NO SIGNAL', scol = br.source === 'estimate' ? C.amber : br.source === 'can' ? C.ink : C.faint;
      const sw = D.measure(ctx, src, 9) + 9 * 1.2 + 16;
      D.rect(ctx, srcX - sw, cn.y + 26, sw, 20, scol); D.label(ctx, src, srcX - sw / 2, cn.y + 40, { size: 9, color: C.text, align: 'center' });
      D.label(ctx, `peak ${U.fmt(br.peak_bar, 1)} bar`, srcX, cn.y + 62, { size: 9, color: C.muted, align: 'right' });
      D.label(ctx, `${br.count || 0} events`, srcX, cn.y + 76, { size: 9, color: C.muted, align: 'right' });
      if (br.active && U.isNum(br.trail_pct)) D.label(ctx, `trail ${br.trail_pct.toFixed(0)} %`, srcX, cn.y + 90, { size: 9, color: C.amber, align: 'right' });
      const trace = br.trace || [], tnow = T.t || 0;
      const tr = { x: cn.x + 6, y: cn.y + 40 + big, w: cn.w - 12, h: cn.h - 50 - big };
      const yN = U.niceTicks(Math.max(40, (br.peak_bar || 0) * 1.1), 5), ymax = yN.max, yT = yN.ticks;
      const xT = []; for (let x = -8; x <= 0; x += 2) xT.push(x);
      const ch = D.chart(ctx, tr, { x: [-8, 0], y: [0, ymax], xTicks: xT, xFmt: (v) => (v === 0 ? 'now' : `${v} s`), yTicks: yT, yLabel: 'bar', series: [{ pts: trace.map((p) => [p[0] - tnow, p[1]]), color: C.ink, width: 2, fill: true }], refY: [{ y: 2, color: C.steelDim }] });
      if (trace.length) { let pk = trace[0]; trace.forEach((p) => { if (p[1] > pk[1]) pk = p; }); if (pk[1] > 5) { const px = ch.X(pk[0] - tnow), py = ch.Y(pk[1]); D.dot(ctx, px, py, 3, C.ink); D.leader(ctx, px, py, px + (px > ch.x0 + ch.pw * 0.7 ? -24 : 24), Math.max(ch.y0 + 10, py - 14), `${pk[1].toFixed(0)} bar`, { dir: px > ch.x0 + ch.pw * 0.7 ? -1 : 1, size: 10 }); } }

      // ---- right: dials + table --------------------------------------------------------
      const R = lay.right, events = (br.events || []).slice().reverse(), lastEv = events[0];
      const dr = Math.min(R.w * 0.16, R.h * 0.16);
      D.dial(ctx, R.x + R.w * 0.27, R.y + 34 + dr, dr, lastEv ? lastEv.trail_pct / 100 : 0, { value: lastEv ? `${lastEv.trail_pct.toFixed(0)}%` : '—', valueSize: dr * 0.55, label: 'trail (last)' });
      D.dial(ctx, R.x + R.w * 0.73, R.y + 34 + dr, dr, (br.score || 0) / 100, { value: U.fmtInt(br.score), valueSize: dr * 0.55, label: 'session score', color: C.ink });
      const ty = R.y + 60 + dr * 2 + 10, cols = [0, 0.3, 0.48, 0.64, 0.82], hdr = ['entry→exit', 'peak', 'decel', 'trail', 'smooth'];
      D.line(ctx, R.x + 12, ty - 14, R.x + R.w - 12, ty - 14, C.steelDim);
      hdr.forEach((hh, i) => D.label(ctx, hh, R.x + 14 + cols[i] * (R.w - 28), ty, { size: 8, color: C.faint }));
      if (!events.length) D.text(ctx, 'no braking events yet', R.x + R.w / 2, ty + 30, { size: 11, color: C.faint, align: 'center' });
      events.slice(0, 6).forEach((e, j) => {
        const y = ty + 16 + j * 15;
        if (y > R.y + R.h - 6) return;
        [`${e.entry_kph.toFixed(0)}→${e.exit_kph.toFixed(0)}`, `${e.peak_bar.toFixed(0)}`, `${e.avg_decel_g.toFixed(2)}g`, `${e.trail_pct.toFixed(0)}%`, `${e.smoothness.toFixed(0)}`].forEach((v, i) => D.text(ctx, v, R.x + 14 + cols[i] * (R.w - 28), y, { size: 10, color: j === 0 ? C.text : C.muted }));
      });
    },
  });
})();
