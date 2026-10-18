/* SYSTEM page: connection stats, vehicle, and a raw dump of every telemetry field (in-car diagnostics). */
(function () {
  'use strict';
  const bg = new U.StaticLayer(() => {});
  const clip = (s, n = 24) => (s.length > n ? '…' + s.slice(s.length - n + 1) : s);
  const fmtv = (v) => (v == null ? '—' : typeof v === 'number' ? (Number.isInteger(v) ? String(v) : Math.abs(v) < 0.005 ? '0.00' : v.toFixed(2)) : typeof v === 'boolean' ? (v ? 'yes' : 'no') : Array.isArray(v) ? `[${v.length}]` : typeof v === 'object' ? '{…}' : clip(String(v)));
  window.PAGES = window.PAGES || [];
  window.PAGES.push({
    id: 'system', name: 'SYSTEM', buttons: 'systemButtons',
    draw(ctx, L, S) {
      const { w, h, dpr } = L, C = D.C, T = S.tele, cfg = S.cfg;
      bg.drawFn = (c, W, H) => { D.grid(c, W, H); D.panel(c, 10, 10, W - 20, H - 56, 'system'); };
      ctx.drawImage(bg.get(w, h, dpr), 0, 0, w, h);
      const conn = T.conn || {};
      const blocks = [
        ['link', [['version', S.hello ? S.hello.version : '—'], ['source', S.demo ? 'demo (in-browser)' : conn.source], ['bus', conn.bus], ['ecu', conn.ecu], ['imu', conn.imu], ['pid rate', `${U.fmt(conn.pid_hz, 1)} Hz`], ['frame rate', `${U.fmt(conn.frame_hz, 0)} Hz`], ['ws url', S.link ? S.link.url : '—']]],
        ['vehicle', [['name', cfg.vehicle], ['transmission', cfg.transmission], ['gears', cfg.gears], ['redline', `${cfg.redline_rpm} rpm`], ['shift light', `${cfg.shift_light_rpm} rpm`], ['brake full scale', `${cfg.max_brake_bar} bar`], ['efficiency', cfg.drivetrain_efficiency == null ? '—' : cfg.drivetrain_efficiency]]],
        ['veh', Object.entries(T.veh || {})],
        ['g · brake', [...Object.entries(T.g || {}).filter(([k]) => k !== 'trail'), ...Object.entries(T.brake || {}).filter(([k]) => k !== 'trace' && k !== 'events').map(([k, v]) => [`brake.${k}`, v])]],
        ['shift · dyno · session', [...Object.entries((T.shift || {}).session || {}).map(([k, v]) => [`shift.${k}`, v]), ['shift.state', (T.shift || {}).state], ['dyno.state', (T.dyno || {}).state], ['dyno.runs', ((T.dyno || {}).runs || []).length], ...Object.entries(T.session || {}).map(([k, v]) => [`session.${k}`, v]), ['alerts', (T.alerts || []).join(', ') || 'none']]],
      ];
      const cols = w >= 1100 ? 5 : w >= 760 ? 4 : 2, cw = (w - 40) / cols, lh = 13, top = 64, bottom = h - 56;
      let col = 0, y = top;
      const nextCol = () => { col++; y = top; return col < cols; };
      blocks.forEach(([title, rows]) => {
        if (col >= cols) return;
        if (y + 3 * lh > bottom && !nextCol()) return;
        D.label(ctx, title, 24 + col * cw, y, { size: 9, color: C.ink }); y += lh + 2;
        for (const [k, v] of rows) {
          if (y > bottom && !nextCol()) return;
          const x = 24 + col * cw;
          D.text(ctx, k, x, y, { size: 10, color: C.muted }); D.text(ctx, fmtv(v), x + cw - 16, y, { size: 10, color: C.text, align: 'right' }); y += lh;
        }
        y += lh;
      });
    },
  });
})();
