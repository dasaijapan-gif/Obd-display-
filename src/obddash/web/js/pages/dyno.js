/* DYNO page: virtual dyno — wheel power and wheel-referred torque vs rpm as two stacked panels sharing one rpm axis. */
(function () {
  'use strict';
  const bg = new U.StaticLayer(() => {});
  function layout(w, h) {
    return {
      banner: { x: 10, y: 10, w: w - 20, h: Math.max(34, h * 0.10) },
      charts: { x: 10, y: Math.max(52, h * 0.13), w: w * 0.70, h: h - Math.max(52, h * 0.13) - 46 },
      right: { x: w * 0.725, y: Math.max(52, h * 0.13), w: w * 0.275 - 10, h: h * 0.86 - Math.max(52, h * 0.13) },
    };
  }
  const stateColor = (C, st) => ({ idle: C.muted, armed: C.amber, pulling: C.ink, done: C.green, rejected: C.red }[st] || C.muted);

  window.PAGES = window.PAGES || [];
  window.PAGES.push({
    id: 'dyno', name: 'DYNO', buttons: 'dynoButtons',
    draw(ctx, L, S) {
      const { w, h, s, dpr, now } = L, C = D.C, T = S.tele, cfg = S.cfg, lay = layout(w, h);
      bg.drawFn = (c, W, H) => {
        D.grid(c, W, H);
        D.panel(c, lay.charts.x, lay.charts.y, lay.charts.w, lay.charts.h, 'power / torque vs rpm', { note: 'wheel · corrected' });
        D.panel(c, lay.right.x, lay.right.y, lay.right.w, lay.right.h, 'peaks');
      };
      ctx.drawImage(bg.get(w, h, dpr), 0, 0, w, h);
      const dy = T.dyno || {}, st = dy.state || 'idle', sc = stateColor(C, st);

      // ---- banner -----------------------------------------------------------------------
      const B = lay.banner;
      D.fill(ctx, B.x, B.y, 6, B.h, sc);
      D.text(ctx, `DYNO · ${st.toUpperCase()}`, B.x + 16, B.y + B.h * 0.62, { size: 14 + 2 * s, weight: 700, color: C.text });
      const tw = D.measure(ctx, `DYNO · ${st.toUpperCase()}`, 14 + 2 * s, 700);
      D.text(ctx, dy.message || '', B.x + 16 + tw + 18, B.y + B.h * 0.62, { size: 11 + s, color: st === 'pulling' && Math.floor(now * 3) % 2 ? C.text : C.muted });
      if (st === 'pulling' || st === 'armed') { const pw = Math.min(260, w * 0.2); D.hbar(ctx, w - 190 - pw - 20, B.y + B.h / 2 - 5, pw, 10, dy.progress || 0, sc); D.label(ctx, 'span', w - 190 - pw - 20, B.y + B.h / 2 - 9, { size: 8, color: C.faint }); }

      // ---- charts ----------------------------------------------------------------------
      const ch = lay.charts, curve = dy.curve || [], runs = dy.runs || [];
      const xa = 1000, xb = Math.ceil((cfg.redline_rpm + 400) / 1000) * 1000;
      let pmax = 40, tmax = 100;
      const consider = (pts) => pts.forEach((p) => { if (U.isNum(p.power_kw)) pmax = Math.max(pmax, p.power_kw); if (U.isNum(p.torque_nm)) tmax = Math.max(tmax, p.torque_nm); if (U.isNum(p.ecu_torque_nm)) tmax = Math.max(tmax, p.ecu_torque_nm); });
      consider(curve); runs.forEach((r) => consider(r.curve || []));
      const pN = U.niceTicks(pmax * 1.08, 5), tN = U.niceTicks(tmax * 1.08, 5); pmax = pN.max; tmax = tN.max;
      const xTicks = []; for (let x = xa; x <= xb; x += 1000) xTicks.push(x);
      const inner = { x: ch.x + 6, y: ch.y + 22, w: ch.w - 12, h: ch.h - 30 };
      const rP = { x: inner.x, y: inner.y, w: inner.w, h: inner.h * 0.5 }, rT = { x: inner.x, y: inner.y + inner.h * 0.5, w: inner.w, h: inner.h * 0.5 };
      const prev = runs.filter((r) => !(dy.peak_power_rpm === r.peak_power_rpm && Math.abs(dy.peak_power_kw - r.peak_power_kw) < 1e-6));
      const pSeries = prev.map((r) => ({ pts: (r.curve || []).map((p) => [p.rpm, p.power_kw]), color: C.ink, width: 1.5, alpha: 0.25 }));
      const tSeries = prev.map((r) => ({ pts: (r.curve || []).map((p) => [p.rpm, p.torque_nm]), color: C.ink, width: 1.5, alpha: 0.25 }));
      pSeries.push({ pts: curve.map((p) => [p.rpm, p.power_kw]), color: C.ink, width: 2 });
      tSeries.push({ pts: curve.map((p) => [p.rpm, p.torque_nm]), color: C.ink, width: 2 });
      const hasEcu = curve.some((p) => U.isNum(p.ecu_torque_nm));
      if (hasEcu) tSeries.push({ pts: curve.filter((p) => U.isNum(p.ecu_torque_nm)).map((p) => [p.rpm, p.ecu_torque_nm]), color: C.ink2, width: 2 });
      const pT = pN.ticks, tT = tN.ticks;
      const cP = D.chart(ctx, rP, { x: [xa, xb], y: [0, pmax], xTicks, hideX: true, yTicks: pT, yLabel: 'power kW', padB: 4, series: pSeries, legend: prev.length ? [{ label: 'previous', color: U.alpha(C.ink, 0.3) }, { label: 'this pull', color: C.ink }] : null });
      const cT = D.chart(ctx, rT, { x: [xa, xb], y: [0, tmax], xTicks, xFmt: (v) => (v / 1000).toFixed(0) + 'k', xLabel: 'rpm', yTicks: tT, yLabel: 'torque Nm', series: tSeries, legend: hasEcu ? [{ label: 'wheel (calc)', color: C.ink }, { label: 'ecu', color: C.ink2 }] : null });
      if (curve.length > 2 && (st === 'done' || st === 'idle' || st === 'armed')) {
        const pk = curve.reduce((m, p) => (p.power_kw > m.power_kw ? p : m), curve[0]);
        const tk = curve.reduce((m, p) => (p.torque_nm > m.torque_nm ? p : m), curve[0]);
        const dirP = cP.X(pk.rpm) > cP.x0 + cP.pw * 0.65 ? -1 : 1, dirT = cT.X(tk.rpm) > cT.x0 + cT.pw * 0.65 ? -1 : 1;
        D.dot(ctx, cP.X(pk.rpm), cP.Y(pk.power_kw), 4, C.ink);
        D.leader(ctx, cP.X(pk.rpm), cP.Y(pk.power_kw), cP.X(pk.rpm) + dirP * 30, Math.max(cP.y0 + 12, cP.Y(pk.power_kw) - 16), `${pk.power_kw.toFixed(0)} kW · ${(pk.power_kw * 1.34102).toFixed(0)} whp @ ${U.fmtRpm(pk.rpm)}`, { dir: dirP, size: 10 });
        D.dot(ctx, cT.X(tk.rpm), cT.Y(tk.torque_nm), 4, C.ink);
        D.leader(ctx, cT.X(tk.rpm), cT.Y(tk.torque_nm), cT.X(tk.rpm) + dirT * 30, Math.max(cT.y0 + 12, cT.Y(tk.torque_nm) - 16), `${tk.torque_nm.toFixed(0)} Nm @ ${U.fmtRpm(tk.rpm)}`, { dir: dirT, size: 10 });
      }
      if (st === 'pulling' && U.isNum(dy.live_rpm) && dy.live_rpm > xa) {
        D.dot(ctx, cP.X(dy.live_rpm), cP.Y(U.clamp(dy.live_power_kw, 0, pmax)), 5, C.ink);
        D.dot(ctx, cT.X(dy.live_rpm), cT.Y(U.clamp(dy.live_torque_nm, 0, tmax)), 5, C.ink);
        D.line(ctx, cP.X(dy.live_rpm), cP.y0, cP.X(dy.live_rpm), cT.y0 + cT.ph, C.steel);
      }
      if (!curve.length) {
        D.text(ctx, 'NO PULL YET', inner.x + inner.w / 2, inner.y + inner.h * 0.42, { size: 14, color: C.muted, align: 'center' });
        D.text(ctx, 'ARM, then hold full throttle in 3rd or 4th from ~2 500 rpm to the redline', inner.x + inner.w / 2, inner.y + inner.h * 0.42 + 20, { size: 11, color: C.faint, align: 'center' });
      }

      // ---- right: peaks + runs table ----------------------------------------------------
      const R = lay.right, x0 = R.x + 14;
      const big = Math.min(R.h * 0.22, R.w * 0.32);
      D.text(ctx, dy.wheel_hp ? dy.wheel_hp.toFixed(0) : '—', x0, R.y + 30 + big, { size: big, weight: 600, color: C.text });
      const bw = D.measure(ctx, dy.wheel_hp ? dy.wheel_hp.toFixed(0) : '—', big, 600);
      D.label(ctx, 'whp', x0 + bw + 8, R.y + 30 + big, { size: 11, color: C.muted });
      const rows = [
        ['peak power', dy.peak_power_kw ? `${dy.peak_power_kw.toFixed(1)} kW @ ${U.fmtRpm(dy.peak_power_rpm)}` : '—'],
        ['peak torque', dy.peak_torque_nm ? `${dy.peak_torque_nm.toFixed(0)} Nm @ ${U.fmtRpm(dy.peak_torque_rpm)}` : '—'],
        ['est. crank', dy.peak_power_kw && cfg.drivetrain_efficiency ? `${(dy.peak_power_kw / cfg.drivetrain_efficiency).toFixed(0)} kW (η ${cfg.drivetrain_efficiency})` : '—'],
        ['gear', dy.gear ? U.ordinal(dy.gear) : runs.length ? U.ordinal(runs[runs.length - 1].gear) : '—'],
      ];
      rows.forEach(([k, v], i) => { const y = R.y + 48 + big + i * (16 + 4 * s); D.label(ctx, k, x0, y, { size: 9 }); D.text(ctx, v, x0 + R.w * 0.40, y, { size: 11 + s, color: C.text }); });
      const ty = R.y + 56 + big + rows.length * (16 + 4 * s);
      if (runs.length && ty < R.y + R.h - 40) {
        D.line(ctx, x0, ty, R.x + R.w - 14, ty, C.steelDim);
        const cols = [0, 0.1, 0.26, 0.44, 0.62, 0.8], hdr = ['#', 'gear', 'kW', '@rpm', 'Nm', 'cf'];
        hdr.forEach((hh, i) => D.label(ctx, hh, x0 + cols[i] * (R.w - 28), ty + 14, { size: 8, color: C.faint }));
        runs.slice(-6).reverse().forEach((r, j) => {
          const y = ty + 30 + j * 15;
          if (y > R.y + R.h - 6) return;
          [r.id, U.ordinal(r.gear), r.peak_power_kw.toFixed(0), U.fmtRpm(r.peak_power_rpm), r.peak_torque_nm.toFixed(0), r.correction.toFixed(3)].forEach((v, i) => D.text(ctx, v, x0 + cols[i] * (R.w - 28), y, { size: 10, color: j === 0 ? C.text : C.muted }));
        });
      }
    },
  });
})();
