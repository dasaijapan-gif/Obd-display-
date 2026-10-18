/* obddash app shell: data source (WebSocket or in-browser demo), page switching, input, event overlays, render loop. */
(function () {
  'use strict';
  const PAGE_ORDER = ['drive', 'shift', 'dyno', 'brake', 'handling', 'system'];
  const canvas = document.getElementById('main');
  const ctx = canvas.getContext('2d');
  const $ = (id) => document.getElementById(id);
  const C = U.colors();
  D.init(C);
  const pages = PAGE_ORDER.map((id) => (window.PAGES || []).find((p) => p.id === id)).filter(Boolean);

  const S = {
    tele: null, lastTeleAt: -1e9, hello: null, demo: false, link: null, sim: null,
    cfg: { vehicle: '—', redline_rpm: 6500, shift_light_rpm: 6000, idle_rpm: 800, gears: 6, max_brake_bar: 100, transmission: 'manual', drivetrain_efficiency: null },
    sm: { speed: new U.Smooth(0.10), rpm: new U.Smooth(0.07), thr: new U.Smooth(0.06), brk: new U.Smooth(0.06), press: new U.Smooth(0.06), lat: new U.Smooth(0.08), lon: new U.Smooth(0.08), steer: new U.Smooth(0.08), bal: new U.Smooth(0.3) },
    pageIdx: 0, lastShiftAt: -1e9, userNavAt: 0, returnTo: null, returnAt: 0, dynoStartedAt: 0,
    send(cmd, args) { if (S.demo && S.sim) return S.sim.command(cmd); return S.link ? S.link.send(cmd, args) : false; },
  };
  window.OBD = S; // for debugging in the browser console
  Object.assign(S, window.OBD_HOOKS || {}); // an embedding page may pass onPage / onEventExtra

  // ---- DOM --------------------------------------------------------------------------
  const dots = $('dots');
  pages.forEach(() => { const sp = document.createElement('span'); dots.appendChild(sp); });
  function go(idx, user) {
    const n = pages.length; S.pageIdx = ((idx % n) + n) % n;
    if (user) { S.userNavAt = U.now(); S.returnTo = null; S.returnAt = 0; }
    const p = pages[S.pageIdx];
    [...dots.children].forEach((d, i) => d.classList.toggle('on', i === S.pageIdx));
    $('tbSheet').textContent = `SHT ${S.pageIdx + 1}/${n}`; $('tbPage').textContent = p.name;
    document.querySelectorAll('.pagebtns').forEach((el) => { el.hidden = el.id !== p.buttons; });
    U.store.set('obddash.page', p.id);
    if (typeof S.onPage === 'function') S.onPage(S.pageIdx, p);
  }
  S.go = go; S.pages = pages;
  const savedPage = U.store.get('obddash.page', 'drive');
  go(Math.max(0, pages.findIndex((p) => p.id === savedPage)), false);

  // ---- data sources -------------------------------------------------------------------
  const params = new URLSearchParams(location.search);
  const forceDemo = params.get('demo') === '1' || window.OBD_FORCE_DEMO === true || location.protocol === 'file:';
  function applyConfig(data) {
    const v = (data && (data.vehicle || data)) || {};
    const cfg = S.cfg;
    if (v.name) cfg.vehicle = v.name;
    if (U.isNum(v.redline_rpm)) cfg.redline_rpm = v.redline_rpm;
    if (U.isNum(v.shift_light_rpm)) cfg.shift_light_rpm = v.shift_light_rpm;
    if (U.isNum(v.idle_rpm)) cfg.idle_rpm = v.idle_rpm;
    if (Array.isArray(v.gear_ratios) && v.gear_ratios.length) cfg.gears = v.gear_ratios.length;
    if (v.brake && U.isNum(v.brake.max_pressure_bar)) cfg.max_pressure_bar = cfg.max_brake_bar = v.brake.max_pressure_bar;
    if (v.transmission) cfg.transmission = v.transmission;
    if (U.isNum(v.drivetrain_efficiency)) cfg.drivetrain_efficiency = v.drivetrain_efficiency;
  }
  function onTelemetry(t) { S.tele = t; S.lastTeleAt = U.now(); }
  function onEvent(msg) {
    const now = U.now();
    if (typeof S.onEventExtra === 'function') { try { S.onEventExtra(msg); } catch (e) { console.error(e); } }
    if (msg.name === 'shift.scored' && msg.data) { S.lastShiftAt = now; showShift(msg.data); }
    else if (msg.name === 'dyno.started') { if (pages[S.pageIdx].id !== 'dyno') { S.returnTo = S.pageIdx; S.dynoStartedAt = now; go(pages.findIndex((p) => p.id === 'dyno'), false); } }
    else if (msg.name === 'dyno.done') { if (S.returnTo != null) S.returnAt = now + 8; }
  }
  function startDemo() {
    if (S.sim) return;
    if (S.link) { S.link.stop(); S.link = null; }
    S.demo = true; S.sim = new DemoSim({ seed: 7 });
    Object.assign(S.cfg, S.sim.cfg, { drivetrain_efficiency: 0.86 });
    S.hello = { version: 'demo' };
    // optional warm-up so the first frame already has history (log hook only, no overlays / page switches)
    const warm = Number(params.get('warmup') || window.OBD_DEMO_WARMUP || 0);
    if (warm > 0) S.sim.warmup(warm, (msg) => { if (typeof S.onEventExtra === 'function') S.onEventExtra(msg); });
    S.sim.start(onTelemetry, onEvent);
    $('tbSrc').textContent = 'DEMO'; $('btnToggleDemo').textContent = 'Live link';
  }
  function startLink() {
    if (S.sim) { S.sim.stop(); S.sim = null; S.demo = false; S.tele = null; }
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const url = `${proto}://${location.host}/ws`;
    $('wsurl').textContent = url;
    S.link = new Link(url);
    S.link.onTelemetry = onTelemetry; S.link.onEvent = onEvent;
    S.link.onState = () => { S.hello = S.link.hello; };
    S.link.start();
    fetch('/api/config').then((r) => r.json()).then(applyConfig).catch(() => {});
    $('btnToggleDemo').textContent = 'Demo';
  }
  if (forceDemo) startDemo(); else startLink();

  // ---- overlays -------------------------------------------------------------------------
  let shiftTimer = null;
  function showShift(r) {
    const el = $('shiftOverlay');
    el.querySelector('.grade').textContent = r.grade; el.querySelector('.score').textContent = Math.round(r.score);
    el.querySelector('.what').textContent = `${U.gearLabel(r.from_gear)}→${U.gearLabel(r.to_gear)} · ${r.heel_toe ? 'heel-toe' : r.blip ? 'blip' : 'no blip'} · ${U.signed(r.error_pct, 1)} %`;
    el.classList.toggle('good', r.grade === 'S' || r.grade === 'A'); el.classList.toggle('bad', r.grade === 'C' || r.grade === 'D');
    el.hidden = false; clearTimeout(shiftTimer); shiftTimer = setTimeout(() => { el.hidden = true; }, 3000);
  }

  // ---- buttons --------------------------------------------------------------------------
  $('btnDemo').addEventListener('click', startDemo);
  $('btnArm').addEventListener('click', () => S.send('dyno.arm'));
  $('btnClear').addEventListener('click', () => S.send('dyno.clear'));
  $('btnToggleDemo').addEventListener('click', () => { if (S.demo) { if (location.protocol.startsWith('http')) startLink(); } else startDemo(); });
  $('btnResetSession').addEventListener('click', () => S.send('session.reset'));
  $('btnReload').addEventListener('click', () => location.reload());

  // ---- input: keys, taps, swipes ----------------------------------------------------------
  window.addEventListener('keydown', (e) => {
    const tag = e.target && e.target.tagName;
    if (tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA') return;
    if (e.key === 'ArrowRight' || e.key === 'PageDown') go(S.pageIdx + 1, true);
    else if (e.key === 'ArrowLeft' || e.key === 'PageUp') go(S.pageIdx - 1, true);
    else if (/^[1-9]$/.test(e.key) && Number(e.key) <= pages.length) go(Number(e.key) - 1, true);
    else if (e.key === 'a') S.send('dyno.arm');
  });
  let p0 = null;
  canvas.addEventListener('pointerdown', (e) => { p0 = { x: e.clientX, y: e.clientY, t: U.now() }; });
  canvas.addEventListener('pointerup', (e) => {
    if (!p0) return;
    const dx = e.clientX - p0.x, dt = U.now() - p0.t; p0 = null;
    if (Math.abs(dx) > 60 && dt < 0.8) { go(S.pageIdx + (dx < 0 ? 1 : -1), true); return; }
    if (Math.abs(dx) < 12) { const r = canvas.getBoundingClientRect(), fx = (e.clientX - r.left) / r.width; if (fx < 0.15) go(S.pageIdx - 1, true); else if (fx > 0.85) go(S.pageIdx + 1, true); }
    if (navigator.wakeLock && !S.wakeLock) navigator.wakeLock.request('screen').then((l) => { S.wakeLock = l; }).catch(() => {});
  });

  if (document.fonts && document.fonts.ready) document.fonts.ready.then(() => { U.epoch++; });

  // ---- render loop ------------------------------------------------------------------------
  let last = U.now(), clockAt = 0;
  function drawWaiting(c, L) {
    D.grid(c, L.w, L.h);
    D.text(c, 'WAITING FOR TELEMETRY', L.w / 2, L.h / 2 - 8, { size: 16, color: C.muted, align: 'center' });
    D.text(c, S.demo ? 'starting demo…' : 'start `obddash sim` or `obddash run`, or press Demo', L.w / 2, L.h / 2 + 14, { size: 11, color: C.faint, align: 'center' });
  }
  function frame() {
    const now = U.now(), dt = Math.min(0.1, now - last); last = now;
    const { w, h, dpr } = U.fitCanvas(canvas);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const T = S.tele;
    if (T) {
      const v = T.veh || {}, g = T.g || {}, b = T.brake || {}, sm = S.sm;
      sm.speed.to(v.speed_kph, dt); sm.rpm.to(v.rpm, dt); sm.thr.to(v.throttle_pct, dt);
      sm.brk.to(b.pressure_bar, dt); sm.press.to(b.pressure_pct, dt);
      sm.lat.to(g.lat, dt); sm.lon.to(g.lon, dt); sm.steer.to(v.steering_deg, dt);
      sm.bal.to(U.isNum((T.handling || {}).balance) ? T.handling.balance : 0, dt);
    }
    ctx.fillStyle = C.bg; ctx.fillRect(0, 0, w, h);
    const L = { w, h, dpr, s: U.scale(w, h), dt, now };
    if (T) { try { pages[S.pageIdx].draw(ctx, L, S); } catch (err) { console.error(err); drawWaiting(ctx, L); } } else drawWaiting(ctx, L);
    // link state & housekeeping at ~4 Hz
    if (now - clockAt > 0.25) {
      clockAt = now;
      const stale = !S.demo && now - S.lastTeleAt > 2;
      $('nolink').hidden = !stale;
      if (!S.demo) $('tbSrc').textContent = stale ? 'NO LINK' : ((T && T.conn && T.conn.source) || 'LIVE').toUpperCase();
      $('tbClock').textContent = T ? U.fmtTime((T.session || {}).duration_s) : '--:--';
      const alerts = (T && T.alerts) || [];
      $('alertStrip').hidden = !alerts.length; if (alerts.length) $('alertStrip').textContent = alerts.join('  ·  ');
      if (S.returnAt && now > S.returnAt) { if (S.userNavAt < S.dynoStartedAt && S.returnTo != null) go(S.returnTo, false); S.returnAt = 0; S.returnTo = null; }
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
})();
