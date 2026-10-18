/* DRIVE page: speed hero, rpm arc with redline hatch and shift light, gear, throttle/brake bars, engine readouts. */
(function () {
  'use strict';
  const A0 = Math.PI * 0.75, A1 = Math.PI * 2.25;
  const bg = new U.StaticLayer(() => {});

  function layout(w, h) {
    const r = Math.min(w * 0.165, h * 0.40);
    return {
      speed: { x: 10, y: 10, w: w * 0.29 - 20, h: h - 56 },
      arc: { cx: w * 0.475, cy: h * 0.53, r },
      bars: { x: w * 0.655, y: h * 0.14, bw: Math.max(14, Math.min(30, w * 0.026)), bh: h * 0.60, gap: Math.max(14, w * 0.028) },
      right: { x: w * 0.775, y: 10, w: w * 0.225 - 10, h: h - 56 },
    };
  }

  window.PAGES = window.PAGES || [];
  window.PAGES.push({
    id: 'drive', name: 'DRIVE',
    draw(ctx, L, S) {
      const { w, h, s, dpr, now } = L, C = D.C, T = S.tele, sm = S.sm, cfg = S.cfg;
      const lay = layout(w, h);
      const maxRpm = Math.ceil((cfg.redline_rpm + 400) / 1000) * 1000;
      const ang = (rpm) => A0 + (U.clamp(rpm, 0, maxRpm) / maxRpm) * (A1 - A0);
      const key = `${maxRpm}|${cfg.redline_rpm}|${cfg.max_brake_bar}`;
      bg.drawFn = (c, W, H) => {
        D.grid(c, W, H);
        D.panel(c, lay.speed.x, lay.speed.y, lay.speed.w, lay.speed.h, 'speed', { note: 'km/h' });
        const { cx, cy, r } = lay.arc;
        D.arc(c, cx, cy, r, A0, A1, C.steel, 1);
        D.arc(c, cx, cy, r - 18, A0, A1, C.steelDim, 1);
        D.ticks(c, cx, cy, r, A0, A1, maxRpm / 250, 6, C.steelDim, 1);
        D.ticks(c, cx, cy, r, A0, A1, maxRpm / 1000, 14, C.steel, 1);
        for (let k = 0; k <= maxRpm / 1000; k++) {
          const a = ang(k * 1000);
          D.text(c, k, cx + Math.cos(a) * (r - 30), cy + Math.sin(a) * (r - 30) + 4, { size: 10 + 2 * s, color: k * 1000 >= cfg.redline_rpm ? C.red : C.muted, align: 'center' });
        }
        D.hatchSector(c, cx, cy, r - 18, r, ang(cfg.redline_rpm), A1, U.alpha(C.red, 0.55), 5);
        D.label(c, 'rpm ×1000', cx, cy + r * 0.96, { size: 9, color: C.faint, align: 'center' });
        const b = lay.bars;
        D.label(c, 'thr', b.x + b.bw / 2, b.y - 8, { size: 9, align: 'center' });
        D.label(c, 'brk', b.x + b.bw + b.gap + b.bw / 2, b.y - 8, { size: 9, align: 'center' });
        D.text(c, '100', b.x - 4, b.y + 4, { size: 8, color: C.faint, align: 'right' });
        D.text(c, '0', b.x - 4, b.y + b.bh, { size: 8, color: C.faint, align: 'right' });
        D.text(c, `${cfg.max_brake_bar}`, b.x + 2 * b.bw + b.gap + 4, b.y + 4, { size: 8, color: C.faint });
        D.text(c, 'bar', b.x + 2 * b.bw + b.gap + 4, b.y + 14, { size: 8, color: C.faint });
        const R = lay.right;
        D.panel(c, R.x, R.y, R.w, R.h * 0.54, 'engine');
        D.panel(c, R.x, R.y + R.h * 0.58, R.w, R.h * 0.42, 'g');
      };
      ctx.drawImage(bg.get(w, h, dpr, key), 0, 0, w, h);

      // ---- speed hero --------------------------------------------------------------
      const sp = lay.speed, scx = sp.x + sp.w / 2;
      const size = Math.min(h * 0.38, sp.w * 0.5);
      D.text(ctx, U.fmtInt(sm.speed.v), scx, sp.y + sp.h * 0.50, { size, weight: 600, color: C.text, align: 'center' });
      const ly = sp.y + sp.h * 0.57;
      D.line(ctx, sp.x + 24, ly, sp.x + sp.w - 24, ly, C.steel);
      D.line(ctx, sp.x + 24, ly - 5, sp.x + 24, ly + 5, C.steel); D.line(ctx, sp.x + sp.w - 24, ly - 5, sp.x + sp.w - 24, ly + 5, C.steel);
      D.label(ctx, 'km/h', scx, ly + 14, { size: 10, color: C.muted, align: 'center' });
      // session strip
      const ss = T.session || {}, sy = sp.y + sp.h * 0.80, colw = sp.w / 3;
      [['dist', `${U.fmt(ss.distance_km, 1)} km`], ['max', `${U.fmtInt(ss.max_speed_kph)} km/h`], ['time', U.fmtTime(ss.duration_s)]].forEach(([k, v], i) => {
        const x = sp.x + colw * i + colw / 2;
        D.label(ctx, k, x, sy, { size: 9, align: 'center' });
        D.text(ctx, v, x, sy + 16 + 2 * s, { size: 11 + 3 * s, color: C.text, align: 'center' });
      });
      // last shift chip
      const last = T.shift && T.shift.last;
      if (last) {
        const fresh = now - S.lastShiftAt < 3;
        const col = last.grade === 'S' || last.grade === 'A' ? C.green : last.grade === 'B' ? C.ink : C.amber;
        const cw = Math.min(sp.w - 40, 200), chx = scx - cw / 2, chy = sp.y + 22;
        if (fresh) D.fill(ctx, chx, chy, cw, 26, U.alpha(col, 0.12 + 0.1 * Math.sin(now * 10)));
        D.rect(ctx, chx, chy, cw, 26, col);
        D.label(ctx, `last ▼ ${U.gearLabel(last.from_gear)}→${U.gearLabel(last.to_gear)}`, chx + 8, chy + 17, { size: 9, color: C.muted });
        D.text(ctx, `${last.grade} ${Math.round(last.score)}`, chx + cw - 8, chy + 18, { size: 13, weight: 700, color: C.text, align: 'right' });
      }

      // ---- rpm arc dynamic -----------------------------------------------------------
      const { cx, cy, r } = lay.arc, rpm = sm.rpm.v, veh = T.veh || {};
      const over = rpm >= cfg.redline_rpm;
      if (rpm > 50) D.arc(ctx, cx, cy, r - 9, A0, ang(rpm), over ? C.red : C.ink, 8);
      if (T.shift && T.shift.shift_light && Math.floor(now * 8) % 2 === 0) D.arc(ctx, cx, cy, r + 7, ang(cfg.shift_light_rpm), A1, C.amber, 4);
      const a = ang(rpm);
      ctx.beginPath(); ctx.moveTo(cx + Math.cos(a) * (r - 46), cy + Math.sin(a) * (r - 46)); ctx.lineTo(cx + Math.cos(a) * (r + 1), cy + Math.sin(a) * (r + 1)); ctx.strokeStyle = C.text; ctx.lineWidth = 2; ctx.stroke();
      // gear
      const gconf = veh.gear_conf == null ? 1 : veh.gear_conf;
      const gsize = r * 0.95;
      D.text(ctx, U.gearLabel(veh.gear), cx, cy + gsize * 0.30, { size: gsize, weight: 700, color: gconf < 0.5 ? C.faint : C.text, align: 'center' });
      D.label(ctx, veh.clutch_switch ? 'clutch' : 'gear', cx, cy - gsize * 0.50, { size: 9, color: veh.clutch_switch ? C.amber : C.faint, align: 'center' });
      D.text(ctx, U.fmtRpm(rpm), cx, cy + r * 0.62, { size: r * 0.22, color: C.text, align: 'center' });
      D.label(ctx, 'rpm', cx, cy + r * 0.62 + 12, { size: 9, color: C.faint, align: 'center' });

      // ---- throttle / brake bars ----------------------------------------------------
      const b = lay.bars, br = T.brake || {};
      D.vbar(ctx, b.x, b.y, b.bw, b.bh, sm.thr.v / 100, C.ink);
      D.vbar(ctx, b.x + b.bw + b.gap, b.y, b.bw, b.bh, sm.press.v / 100, U.alpha(C.text, 0.85), { peakFrac: (br.peak_bar || 0) / (cfg.max_brake_bar || 100), peakColor: C.amber });
      D.text(ctx, U.fmtInt(sm.thr.v), b.x + b.bw / 2, b.y + b.bh + 14, { size: 11, color: C.text, align: 'center' });
      D.text(ctx, U.fmtInt(sm.brk.v), b.x + b.bw + b.gap + b.bw / 2, b.y + b.bh + 14, { size: 11, color: C.text, align: 'center' });
      if (br.source === 'estimate') D.label(ctx, 'est', b.x + b.bw + b.gap + b.bw / 2, b.y + b.bh + 26, { size: 8, color: C.amber, align: 'center' });

      // ---- engine readouts ------------------------------------------------------------
      const R = lay.right, alerts = T.alerts || [];
      const items = [
        ['coolant', veh.coolant_c, '°C', alerts.includes('COOLANT HIGH')],
        ['oil', veh.oil_temp_c, '°C', alerts.includes('OIL HOT')],
        ['intake', veh.intake_c, '°C', false],
        ['batt', veh.batt_v, 'V', alerts.includes('BATT LOW'), 1],
        ['boost', veh.boost_kpa, 'kPa', false, 0, true],
        ['fuel', veh.fuel_pct, '%', false],
      ];
      const cols = 2, cw2 = (R.w - 24) / cols, rh = (R.h * 0.54 - 30) / 3;
      items.forEach(([k, v, unit, alert, dec, signed], i) => {
        const x = R.x + 12 + (i % cols) * cw2, y = R.y + 30 + Math.floor(i / cols) * rh;
        D.label(ctx, k, x, y, { size: 9 });
        const val = signed ? U.signed(v, dec || 0) : U.fmt(v, dec || 0);
        const vw = D.text(ctx, val, x, y + 16 + 4 * s, { size: 13 + 5 * s, color: alert ? C.red : C.text });
        D.text(ctx, unit, x + vw + 4, y + 16 + 4 * s, { size: 9, color: C.faint });
        if (alert) D.fill(ctx, x + cw2 - 16, y - 7, 6, 6, C.red);
      });
      // ---- mini friction circle -----------------------------------------------------
      const gp = { x: R.x, y: R.y + R.h * 0.58, w: R.w, h: R.h * 0.42 }, g = T.g || {};
      const gr = Math.min(gp.h * 0.40, gp.w * 0.22), gcx = gp.x + gp.w * 0.30, gcy = gp.y + gp.h * 0.52, k = gr / 1.2;
      D.arc(ctx, gcx, gcy, k * 1.0, 0, Math.PI * 2, C.steel); D.arc(ctx, gcx, gcy, k * 0.5, 0, Math.PI * 2, C.steelDim);
      D.line(ctx, gcx - gr, gcy, gcx + gr, gcy, C.steelDim); D.line(ctx, gcx, gcy - gr, gcx, gcy + gr, C.steelDim);
      const trail = g.trail || [];
      if (trail.length > 1) { ctx.beginPath(); trail.forEach((p, i) => { const x = gcx - p[0] * k, y = gcy - p[1] * k; if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); }); ctx.strokeStyle = U.alpha(C.ink, 0.35); ctx.lineWidth = 1.5; ctx.stroke(); }
      D.dot(ctx, gcx - U.clamp(sm.lat.v, -1.5, 1.5) * k, gcy - U.clamp(sm.lon.v, -1.5, 1.5) * k, 4, C.ink);
      const tx = gp.x + gp.w * 0.56;
      [['lat', sm.lat.v], ['lon', sm.lon.v], ['peak', g.peak_lat]].forEach(([kk, v], i) => {
        const y = gp.y + 30 + i * (gp.h - 44) / 3;
        D.label(ctx, kk, tx, y, { size: 9 }); const vw = D.text(ctx, U.signed(v, 2), tx, y + 14 + 2 * s, { size: 12 + 3 * s, color: C.text }); D.text(ctx, 'g', tx + vw + 4, y + 14 + 2 * s, { size: 9, color: C.faint });
      });
      // connection pips
      const conn = T.conn || {}, py = gp.y + gp.h - 10;
      [['bus', conn.bus], ['ecu', conn.ecu], ['imu', conn.imu]].forEach(([kk, ok], i) => {
        const x = gp.x + 14 + i * 46; D.fill(ctx, x, py - 7, 6, 6, ok ? C.green : C.steelDim); D.label(ctx, kk, x + 10, py, { size: 8, color: ok ? C.muted : C.faint });
      });
    },
  });
})();
