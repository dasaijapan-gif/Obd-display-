/* HANDLING page: friction circle with trail and adaptive envelope, balance bar, steering & yaw, scores. */
(function () {
  'use strict';
  const bg = new U.StaticLayer(() => {});
  function layout(w, h) {
    return {
      circle: { x: 10, y: 10, w: w * 0.42 - 10, h: h - 56 },
      right: { x: w * 0.435, y: 10, w: w * 0.565 - 10, h: h * 0.86 - 10 },
    };
  }
  window.PAGES = window.PAGES || [];
  window.PAGES.push({
    id: 'handling', name: 'HANDLING',
    draw(ctx, L, S) {
      const { w, h, s, dpr } = L, C = D.C, T = S.tele, sm = S.sm, lay = layout(w, h);
      const g = T.g || {}, hd = T.handling || {}, veh = T.veh || {};
      const cp = lay.circle, R = Math.min(cp.w, cp.h) * 0.40, cx = cp.x + cp.w / 2, cy = cp.y + cp.h / 2 + 8, k = R / 1.2;
      bg.drawFn = (c, W, H) => {
        D.grid(c, W, H);
        D.panel(c, cp.x, cp.y, cp.w, cp.h, 'friction circle', { note: 'g' });
        D.panel(c, lay.right.x, lay.right.y, lay.right.w, lay.right.h, 'chassis');
        D.line(c, cx - R - 8, cy, cx + R + 8, cy, C.steelDim); D.line(c, cx, cy - R - 8, cx, cy + R + 8, C.steelDim);
        D.arc(c, cx, cy, k * 0.5, 0, Math.PI * 2, C.steelDim); D.arc(c, cx, cy, k * 1.0, 0, Math.PI * 2, C.steel);
        D.text(c, '0.5', cx + k * 0.5 * 0.707 + 3, cy - k * 0.5 * 0.707 - 3, { size: 9, color: C.faint });
        D.text(c, '1.0', cx + k * 0.707 + 3, cy - k * 0.707 - 3, { size: 9, color: C.faint });
        D.label(c, 'accel', cx, cy - R - 12, { size: 9, align: 'center' }); D.label(c, 'brake', cx, cy + R + 20, { size: 9, align: 'center' });
        D.label(c, 'L', cx - R - 16, cy + 3, { size: 9, align: 'center' }); D.label(c, 'R', cx + R + 16, cy + 3, { size: 9, align: 'center' });
      };
      ctx.drawImage(bg.get(w, h, dpr), 0, 0, w, h);

      // envelope ring + peaks + trail + current
      const env = Math.max(0.8, g.total != null ? Math.max(0.8, 0.98 * Math.max(g.peak_lat || 0, g.peak_brake || 0, g.peak_accel || 0)) : 0.8);
      D.arc(ctx, cx, cy, Math.min(k * env, R + 4), 0, Math.PI * 2, C.inkDim);
      D.label(ctx, `env ${env.toFixed(2)} g`, cx + Math.min(k * env, R + 4) * 0.707 + 4, cy + Math.min(k * env, R + 4) * 0.707 + 10, { size: 8, color: C.inkDim });
      const tick = (x, y, horiz) => { if (horiz) D.line(ctx, x, y - 5, x, y + 5, C.text); else D.line(ctx, x - 5, y, x + 5, y, C.text); };
      if (g.peak_lat) { tick(cx - g.peak_lat * k, cy, true); tick(cx + g.peak_lat * k, cy, true); }
      if (g.peak_accel) tick(cx, cy - g.peak_accel * k, false);
      if (g.peak_brake) tick(cx, cy + g.peak_brake * k, false);
      const trail = g.trail || [];
      for (let i = 1; i < trail.length; i++) {
        const a = trail[i - 1], b = trail[i], al = 0.08 + 0.7 * (i / trail.length);
        ctx.beginPath(); ctx.moveTo(cx - a[0] * k, cy - a[1] * k); ctx.lineTo(cx - b[0] * k, cy - b[1] * k); ctx.strokeStyle = U.alpha(C.ink, al); ctx.lineWidth = 2; ctx.lineCap = 'round'; ctx.stroke();
      }
      D.dot(ctx, cx - U.clamp(sm.lat.v, -1.4, 1.4) * k, cy - U.clamp(sm.lon.v, -1.4, 1.4) * k, 6, C.ink);
      D.label(ctx, `friction used ${U.fmtInt(hd.friction_used_pct)} %`, cp.x + cp.w - 14, cp.y + cp.h - 10, { size: 9, color: C.muted, align: 'right' });
      D.label(ctx, `source ${g.source || 'none'}`, cp.x + 14, cp.y + cp.h - 10, { size: 9, color: g.source === 'derived' ? C.amber : C.faint });

      // ---- right readouts -------------------------------------------------------------
      const rt = lay.right, x0 = rt.x + 16, big = Math.min(rt.h * 0.18, rt.w * 0.09);
      const colw = (rt.w - 32) / 3;
      [['lateral', sm.lat.v], ['longitudinal', sm.lon.v], ['total', Math.hypot(sm.lat.v, sm.lon.v)]].forEach(([kk, v], i) => {
        const x = x0 + i * colw; D.label(ctx, kk, x, rt.y + 30, { size: 9 });
        const vw = D.text(ctx, i === 2 ? U.fmt(v, 2) : U.signed(v, 2), x, rt.y + 36 + big, { size: big, weight: 600, color: C.text }); D.text(ctx, 'g', x + vw + 6, rt.y + 36 + big, { size: 11, color: C.faint });
      });
      // balance bar
      const by = rt.y + 62 + big, bx = x0, bwid = rt.w - 32;
      D.label(ctx, 'balance', bx, by, { size: 9 });
      D.label(ctx, 'understeer', bx, by + 30, { size: 8, color: C.faint }); D.label(ctx, 'oversteer', bx + bwid, by + 30, { size: 8, color: C.faint, align: 'right' });
      D.rect(ctx, bx, by + 8, bwid, 12, C.steel); D.line(ctx, bx + bwid / 2, by + 4, bx + bwid / 2, by + 24, C.steel);
      if (U.isNum(hd.balance)) { const mx = bx + bwid / 2 + sm.bal.v * (bwid / 2 - 4); D.fill(ctx, Math.min(mx, bx + bwid / 2), by + 10, Math.abs(mx - bx - bwid / 2), 8, U.alpha(C.ink, 0.6)); D.line(ctx, mx, by + 4, mx, by + 24, C.text, 2); D.text(ctx, U.signed(hd.balance, 2), bx + bwid / 2, by + 44, { size: 10, color: C.muted, align: 'center' }); }
      else D.text(ctx, U.isNum(veh.yaw_dps) && U.isNum(veh.steering_deg) ? 'straight line · balance reads in corners' : 'n/a · needs yaw rate + steering angle', bx + bwid / 2, by + 44, { size: 10, color: C.faint, align: 'center' });
      // steering indicator + yaw
      const sy = by + 60, sr = Math.min(rt.h - (sy - rt.y) - 24, rt.w * 0.12) / 2;
      if (sr > 14) {
        const scx = x0 + sr + 6, scy = sy + sr + 8;
        D.arc(ctx, scx, scy, sr, 0, Math.PI * 2, C.steel); D.ticks(ctx, scx, scy, sr, 0, Math.PI * 2, 12, 4, C.steelDim);
        const ang = -Math.PI / 2 - U.rad(U.clamp(sm.steer.v, -540, 540)); // + = left = counter-clockwise
        D.line(ctx, scx, scy, scx + Math.cos(ang) * (sr - 6), scy + Math.sin(ang) * (sr - 6), C.ink, 2); D.dot(ctx, scx, scy, 3, C.ink);
        D.label(ctx, 'steering', scx + sr + 14, sy + 10, { size: 9 });
        D.text(ctx, U.isNum(veh.steering_deg) ? `${U.signed(veh.steering_deg, 0)}°` : 'n/a', scx + sr + 14, sy + 30 + 2 * s, { size: 16 + 2 * s, color: C.text });
        D.label(ctx, 'yaw rate', scx + sr + 14, sy + 54 + 2 * s, { size: 9 });
        D.text(ctx, U.isNum(veh.yaw_dps) ? `${U.signed(veh.yaw_dps, 1)} °/s` : 'n/a', scx + sr + 14, sy + 72 + 4 * s, { size: 14 + 2 * s, color: C.text });
        // scores column
        const sx = x0 + rt.w * 0.5;
        [['smoothness', U.fmtInt(hd.smoothness)], ['handling score', U.fmtInt(hd.score)], ['corners', hd.corner_count || 0], ['max lateral', `${U.fmt(hd.max_lat_g, 2)} g`]].forEach(([kk, v], i) => {
          const y = sy + 10 + i * ((rt.y + rt.h - sy - 20) / 4);
          D.label(ctx, kk, sx, y, { size: 9 }); D.text(ctx, v, sx + rt.w * 0.26, y, { size: 13 + 2 * s, color: C.text });
        });
      }
    },
  });
})();
