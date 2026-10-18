/* obddash in-browser demo: a small vehicle simulation + driver script + the same metrics the Python engine
   computes, emitting telemetry in the exact JSON shape of docs/architecture.md.  Lets anyone preview the UI
   with no hardware and no server (index.html?demo=1).  Physics are deliberately simple: longitudinal force
   balance (engine torque through the gearbox, aero drag, rolling resistance, brakes), a free-revving engine
   when the clutch is down, and a bicycle-model lateral response. */
(function () {
  'use strict';
  const G = 9.81;
  const VEH = {
    name: 'Sim Sportscar (6MT)', mass: 1130, cd: 0.33, area: 1.85, crr: 0.011, circ: 1.93,
    ratios: [5.087, 2.991, 1.594, 1.0, 0.713, 0.6], fd: 2.866, eta: 0.86, k: 1.08,
    idle: 850, redline: 7500, shiftLight: 7000, maxBar: 120, wheelbase: 2.31, steerRatio: 15.5, brakeNPerBar: 140,
  };
  const rpmPerKph = (g) => (g >= 1 && g <= 6 ? ((1000 / 60) / VEH.circ) * VEH.ratios[g - 1] * VEH.fd : 0);
  let torqueScale = 1;
  const torqueCurve = (rpm) => { if (rpm > VEH.redline) return 0; const x = (rpm - 4000) / 3500; return Math.max(60, 205 * torqueScale * (1 - 0.25 * x * x)); };
  const grade = (s) => (s >= 95 ? 'S' : s >= 85 ? 'A' : s >= 70 ? 'B' : s >= 50 ? 'C' : 'D');

  class Rng {
    constructor(seed) { this.s = (seed >>> 0) || 1; }
    next() { this.s = (Math.imul(this.s, 1664525) + 1013904223) >>> 0; return this.s / 4294967296; }
    gauss() { const u = 1 - this.next(), v = this.next(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }
  }

  class DemoSim {
    constructor(opts = {}) {
      this.rng = new Rng(opts.seed || 7);
      this.speed = opts.speed || 1;
      this.cfg = { vehicle: VEH.name, redline_rpm: VEH.redline, shift_light_rpm: VEH.shiftLight, idle_rpm: VEH.idle, gears: 6, max_brake_bar: VEH.maxBar, transmission: 'manual' };
      this.reset();
    }
    reset() {
      const s = this;
      s.t = 0; s.v = 0; s.rpm = VEH.idle; s.rpmFree = VEH.idle; s.gear = 1; s.lastGear = 1; s.clutch = 0; s.engaged = true;
      s.throttle = 0; s.brakeBar = 0; s.brakeTarget = 0; s.steer = 0; s.latG = 0; s.latTarget = 0; s.lonG = 0; s.yaw = 0; s.jolt = 0;
      s.dist = 0; s.lap = 0; s.phase = 'launch'; s.pt = 0; s.shiftSeq = null; s.pending = null;
      s.hist = []; // [t, rpm, throttle, brake]
      s.shift = { state: 'idle', last: null, session: { count: 0, downshifts: 0, upshifts: 0, avg_score: 0, best_score: 0, heel_toe_count: 0, history: [] } };
      s.dyno = { state: 'idle', armed: false, bins: null, startRpm: 0, curve: [], runs: [], peaks: null, doneAt: -1, wantPull: false, live: [0, 0, 0] };
      s.brake = { trace: [], events: [], ev: null, offSince: null, peak: 0, count: 0, score: 0, lastSample: -1 };
      s.g = { trail: [], latRing: [], peakLat: 0, peakAcc: 0, peakBrk: 0, maxTotal: 0, env: 0.8, inCorner: false, corners: 0, cornerPeaks: [], lastSample: -1, cornerPeak: 0 };
      s.sess = { maxSpeed: 0, maxRpm: 0, fuel: 0 };
      s.slow = { coolant: 60, oil: 55, intake: 28 };
      s.events = [];
      s.emitT = 0;
    }
    // ------------------------------------------------------------ public
    start(onTelemetry, onEvent) {
      this.onTelemetry = onTelemetry; this.onEvent = onEvent || (() => {});
      this._last = U.now();
      this._timer = setInterval(() => this._tick(), 33);
    }
    stop() { clearInterval(this._timer); }
    // run the simulation instantly for `seconds` of sim time (used to pre-fill history before display)
    warmup(seconds, onEvent) {
      const n = Math.round(seconds / 0.01);
      for (let i = 0; i < n; i++) { this._step(0.01); while (this.events.length) { const e = this.events.shift(); if (onEvent) onEvent(e); } }
    }
    command(cmd) {
      const d = this.dyno;
      if (cmd === 'dyno.arm') { if (d.state === 'idle' || d.state === 'done' || d.state === 'rejected') { d.state = 'armed'; d.armed = true; d.wantPull = true; } return true; }
      if (cmd === 'dyno.cancel') { d.armed = false; d.wantPull = false; if (d.state === 'armed') d.state = 'idle'; return true; }
      if (cmd === 'dyno.clear') { d.runs = []; d.curve = []; d.peaks = null; d.state = d.armed ? 'armed' : 'idle'; return true; }
      if (cmd === 'session.reset') { this.reset(); return true; }
      if (cmd === 'brake.reset_peak') { this.brake.peak = 0; return true; }
      if (cmd === 'shift.reset') { this.shift.last = null; this.shift.session = { count: 0, downshifts: 0, upshifts: 0, avg_score: 0, best_score: 0, heel_toe_count: 0, history: [] }; return true; }
      return false;
    }
    // ------------------------------------------------------------ loop
    _tick() {
      const now = U.now();
      let dt = Math.min(0.1, now - this._last) * this.speed; this._last = now;
      while (dt > 0) { const h = Math.min(0.01, dt); this._step(h); dt -= h; }
      this.onTelemetry(this._telemetry());
      while (this.events.length) this.onEvent(this.events.shift());
    }
    _step(dt) {
      const s = this; s.t += dt; s.pt += dt;
      s._drive(dt);
      s._shifting(dt);
      s._physics(dt);
      s._metrics(dt);
    }
    // ------------------------------------------------------------ driver script
    _drive(dt) {
      const s = this, vk = s.v * 3.6;
      const canShift = !s.shiftSeq;
      const upshiftIfNeeded = () => { if (canShift && s.rpm >= 7000 && s.gear < 6) s._startUpshift(); };
      const downshiftIfNeeded = (minGear) => { if (canShift && s.gear > minGear && vk * rpmPerKph(s.gear - 1) <= 5600) s._startDownshift(); };
      switch (s.phase) {
        case 'launch':
        case 'straight2':
          s.throttle = 100; s.brakeTarget = 0; s.latTarget = 0; upshiftIfNeeded();
          if (vk >= 185) { s.phase = 'brake1'; s.pt = 0; s.lap++; }
          break;
        case 'brake1':
          s.throttle = 0; s.brakeTarget = 85 + s.rng.gauss() * 1.5; s.latTarget = 0; downshiftIfNeeded(2);
          if (vk <= 62) { s.phase = 'corner1'; s.pt = 0; }
          break;
        case 'corner1': // right-hander, trail braking in
          s.latTarget = -0.9 * Math.min(1, s.pt / 0.5); s.brakeTarget = Math.max(0, 55 * (1 - s.pt / 1.2)); s.throttle = s.pt < 1.3 ? 0 : 25;
          if (s.pt >= 3.5) { s.phase = 'exit1'; s.pt = 0; }
          break;
        case 'exit1':
          s.latTarget = -0.9 * Math.max(0, 1 - s.pt / 0.8); s.throttle = 100; s.brakeTarget = 0; upshiftIfNeeded();
          if (vk >= 125) { s.phase = 'corner2'; s.pt = 0; }
          break;
        case 'corner2': // medium left with a brake tap on entry
          s.latTarget = 0.6 * Math.min(1, s.pt / 0.6); s.brakeTarget = s.pt < 0.6 ? 35 : 0; s.throttle = s.pt < 0.6 ? 0 : 30;
          if (s.pt >= 3.0) { s.pt = 0; s.phase = (s.dyno.wantPull || s.lap % 2 === 1) ? 'dynoPrep' : 'chicane'; }
          break;
        case 'chicane':
          s.latTarget = s.pt < 1.0 ? 0.5 * Math.sin(Math.PI * s.pt) : -0.5 * Math.sin(Math.PI * (s.pt - 1)); s.brakeTarget = s.pt < 0.4 ? 30 : 0; s.throttle = s.pt < 0.4 ? 0 : 60; downshiftIfNeeded(3);
          if (s.pt >= 2.0) { s.phase = 'straight2'; s.pt = 0; }
          break;
        case 'dynoPrep': // get into 3rd at ~2500 rpm
          s.latTarget = 0; s.throttle = 0; s.brakeTarget = 25; downshiftIfNeeded(3);
          if (vk <= 64 && s.gear === 3 && canShift) { s.phase = 'dynoHold'; s.pt = 0; }
          break;
        case 'dynoHold':
          s.brakeTarget = 0; s.throttle = 14;
          if (s.pt >= 1.0) { s.phase = 'dynoPull'; s.pt = 0; s._dynoStart(); }
          break;
        case 'dynoPull':
          s.throttle = 100; s.brakeTarget = 0; s.latTarget = 0;
          if (s.rpm >= 7350) { s._dynoEnd(); s.phase = 'brake1'; s.pt = 0; s.lap++; }
          break;
      }
    }
    _drawQuality() {
      const r = this.rng.next();
      if (r < 0.15) return null; // lazy: no blip
      if (r < 0.25) return 1.12; // big over-rev
      return 1 + this.rng.gauss() * 0.035;
    }
    _startUpshift() { this.shiftSeq = { kind: 'up', t0: this.t, from: this.gear, to: this.gear + 1, dur: 0.35 }; }
    _startDownshift() {
      const q = this._drawQuality();
      this.shiftSeq = { kind: 'down', t0: this.t, from: this.gear, to: this.gear - 1, dur: 0.35 + this.rng.next() * 0.45, blip: q != null, q: q || 1, maxThr: 0, brakeDuringBlip: false, done: false };
    }
    _shifting(dt) {
      const s = this, q = s.shiftSeq; if (!q) return;
      const el = s.t - q.t0;
      s.clutch = 1;
      if (q.kind === 'up') {
        s.throttle = 0;
        if (el >= 0.22 && s.gear !== q.to) s.gear = q.to;
        if (el >= q.dur) { s.clutch = 0; s.shiftSeq = null; s._shiftDone(q, false); }
        return;
      }
      // downshift: blip toward target rpm for the new gear (scaled by the driver's "quality")
      const target = s.v * 3.6 * rpmPerKph(q.to) * q.q;
      if (q.blip && el > 0.05) s.throttle = s.rpmFree < target - 150 ? 100 : s.rpmFree < target ? 35 : 0; else s.throttle = 0;
      if (s.throttle > q.maxThr) q.maxThr = s.throttle;
      if (s.throttle > 25 && s.brakeBar > 2) q.brakeDuringBlip = true;
      if (el >= q.dur) {
        const actual = s.rpmFree, tgt = s.v * 3.6 * rpmPerKph(q.to);
        s.gear = q.to; s.clutch = 0; s.shiftSeq = null;
        s.jolt = U.clamp(((actual - tgt) / tgt) * 2.0, -0.6, 0.4); // engine/wheel speed mismatch at engagement
        s._shiftDone(q, true, { target: tgt, actual });
      }
    }
    _shiftDone(q, isDown, m) {
      const s = this, ss = s.shift.session;
      ss.count++;
      if (!isDown) { ss.upshifts++; return; }
      ss.downshifts++;
      const err = ((m.actual - m.target) / m.target) * 100;
      const e = Math.abs(err) * (err < 0 ? 1.5 : 1);
      const match = e <= 2 ? 100 : e >= 15 ? 0 : 100 * (1 - (e - 2) / 13);
      const timing = q.dur <= 0.5 ? 5 : q.dur >= 1.5 ? -10 : U.lerp(5, -10, (q.dur - 0.5) / 1.0);
      const blip = q.maxThr >= 25;
      const joltG = Math.abs(s.jolt);
      const score = U.clamp(match + timing + (blip ? 0 : -15) - Math.min(15, 40 * Math.max(0, joltG - 0.15)), 0, 100);
      const res = {
        t: s.t, from_gear: q.from, to_gear: q.to, kind: 'down', duration_s: q.dur, target_rpm: m.target, actual_rpm: m.actual,
        error_pct: err, blip, heel_toe: blip && q.brakeDuringBlip, jolt_g: joltG, score: Math.round(score * 10) / 10, grade: grade(score),
        rpm_trace: [], throttle_trace: [], brake_trace: [],
      };
      if (res.heel_toe) ss.heel_toe_count++;
      ss.history.push(res.score); if (ss.history.length > 20) ss.history.shift();
      ss.best_score = Math.max(ss.best_score, res.score);
      ss.avg_score = ss.history.reduce((a, b) => a + b, 0) / ss.history.length;
      s.pending = { res, t0: q.t0 - 1.5, finalizeAt: s.t + 0.8 };
    }
    // ------------------------------------------------------------ physics
    _physics(dt) {
      const s = this, vk = s.v * 3.6;
      // brake pedal follows the target with a short lag
      s.brakeBar += (s.brakeTarget - s.brakeBar) * (1 - Math.exp(-dt / 0.12));
      if (s.brakeBar < 0.3) s.brakeBar = 0;
      s.engaged = s.clutch < 0.5 && s.gear >= 1;
      let T = 0, Fw = 0;
      if (s.engaged) {
        s.rpm = Math.max(VEH.idle, vk * rpmPerKph(s.gear)); s.rpmFree = s.rpm; s.lastGear = s.gear;
        T = (s.throttle / 100) * torqueCurve(s.rpm) - (s.throttle < 5 ? 12 + 0.004 * s.rpm : 0);
        if (s.rpm >= VEH.redline) T = Math.min(T, 0);
        Fw = (T * VEH.ratios[s.gear - 1] * VEH.fd * VEH.eta) / (VEH.circ / (2 * Math.PI));
      } else {
        s.rpmFree = U.clamp(s.rpmFree + ((s.throttle / 100) * 9000 - 2200) * dt, VEH.idle, VEH.redline);
        s.rpm = s.rpmFree;
      }
      s.crankTorque = Math.max(0, T);
      const Fdrag = 0.5 * 1.225 * VEH.cd * VEH.area * s.v * s.v;
      const Frr = s.v > 0.1 ? VEH.crr * VEH.mass * G : 0;
      const Fb = s.v > 0.05 ? s.brakeBar * VEH.brakeNPerBar : 0;
      let a = (Fw - Fdrag - Frr - Fb) / (VEH.mass * VEH.k);
      const v1 = s.v + a * dt;
      if (v1 < 0) { a = -s.v / dt; s.v = 0; } else s.v = v1;
      s.accel = a; s.wheelPowerW = (VEH.mass * VEH.k * a + Fdrag + Frr) * s.v;
      s.jolt *= Math.exp(-dt / 0.12);
      s.lonG = a / G + s.jolt;
      s.dist += s.v * dt;
      // lateral: approach the commanded lateral g, derive steering & yaw from the bicycle model
      s.latG += (s.latTarget - s.latG) * (1 - Math.exp(-dt / 0.25));
      if (s.v > 3) {
        const delta = (s.latG * G * VEH.wheelbase) / (s.v * s.v); // road wheel angle, rad
        s.steer = U.deg(delta) * VEH.steerRatio;
        const yawAck = U.deg((s.v * delta) / VEH.wheelbase);
        s.yaw = yawAck * (0.93 + 0.02 * Math.sin(s.t * 1.7)); // mild understeer
        s.yawAck = yawAck;
      } else { s.steer *= 0.9; s.yaw = 0; s.yawAck = 0; }
      // slow signals
      s.slow.coolant = 60 + 32 * (1 - Math.exp(-s.t / 90));
      s.slow.oil += (s.slow.coolant - 4 - s.slow.oil) * dt * 0.05;
      s.slow.intake += (28 + 10 * (s.throttle / 100) - s.slow.intake) * dt * 0.3;
      s.hist.push([s.t, s.rpm, s.throttle, s.brakeBar]); if (s.hist.length > 400) s.hist.shift();
    }
    // ------------------------------------------------------------ metrics (mirrors obddash.metrics)
    _metrics(dt) {
      const s = this, vk = s.v * 3.6;
      s.sess.maxSpeed = Math.max(s.sess.maxSpeed, vk); s.sess.maxRpm = Math.max(s.sess.maxRpm, s.rpm);
      // --- pending shift result: finalise traces 0.8 s after engagement
      if (s.pending && s.t >= s.pending.finalizeAt) {
        const p = s.pending, pts = s.hist.filter((h) => h[0] >= p.t0);
        const stride = Math.max(1, Math.ceil(pts.length / 120));
        const ds = pts.filter((_, i) => i % stride === 0);
        p.res.rpm_trace = ds.map((h) => [h[0], h[1]]); p.res.throttle_trace = ds.map((h) => [h[0], h[2]]); p.res.brake_trace = ds.map((h) => [h[0], h[3]]);
        s.shift.last = p.res; s.pending = null;
        s.events.push({ type: 'event', name: 'shift.scored', data: p.res, t: s.t });
      }
      // --- dyno capture
      const d = s.dyno;
      if (d.state === 'pulling') {
        const bin = Math.round(s.rpm / 100);
        const w = s.rpm * 2 * Math.PI / 60;
        const Pw = s.wheelPowerW / 1000, Tw = w > 1 ? s.wheelPowerW / w : 0;
        let b = d.bins.get(bin); if (!b) { b = { n: 0, p: 0, t: 0, e: 0 }; d.bins.set(bin, b); }
        b.n++; b.p += Pw; b.t += Tw; b.e += s.crankTorque;
        d.live = [s.rpm, Pw, Tw];
        if (s.t - d.lastBuild > 0.2) { d.curve = s._dynoCurve(1); d.lastBuild = s.t; }
      } else if (d.state === 'done' && s.t - d.doneAt > 10) d.state = d.armed ? 'armed' : 'idle';
      // --- brakes
      const B = s.brake, active = s.brakeBar >= 2;
      if (s.t - B.lastSample >= 0.04) { B.lastSample = s.t; B.trace.push([s.t, s.brakeBar]); while (B.trace.length && s.t - B.trace[0][0] > 8) B.trace.shift(); }
      B.peak = Math.max(B.peak, s.brakeBar);
      if (active) {
        if (!B.ev) B.ev = { t_start: s.t, entry: vk, peak: 0, decel: 0, n: 0, trail: 0, dp2: 0, dpn: 0, lastBar: s.brakeBar, lastT: s.t };
        const e = B.ev; e.peak = Math.max(e.peak, s.brakeBar); e.decel += Math.max(0, -s.lonG); e.n++; if (Math.abs(s.latG) > 0.25) e.trail++;
        if (s.t - e.lastT >= 0.04) { const dp = (s.brakeBar - e.lastBar) / (s.t - e.lastT); e.dp2 += dp * dp; e.dpn++; e.lastBar = s.brakeBar; e.lastT = s.t; }
        B.offSince = null;
      } else if (B.ev) {
        if (B.offSince == null) B.offSince = s.t;
        if (s.t - B.offSince > 0.2) {
          const e = B.ev, dur = B.offSince - e.t_start;
          if (dur >= 0.3) {
            const ev = { t_start: e.t_start, duration_s: dur, peak_bar: e.peak, entry_kph: e.entry, exit_kph: vk, avg_decel_g: e.decel / Math.max(1, e.n), trail_pct: (100 * e.trail) / Math.max(1, e.n), smoothness: U.clamp(100 - 0.4 * Math.sqrt(e.dp2 / Math.max(1, e.dpn)), 0, 100) };
            B.events.push(ev); if (B.events.length > 20) B.events.shift(); B.count++;
            const recent = B.events.slice(-10); B.score = recent.reduce((a, x) => a + 0.6 * x.smoothness + 0.4 * Math.min(100, (x.peak_bar / VEH.maxBar) * 130), 0) / recent.length;
            s.events.push({ type: 'event', name: 'brake.event', data: ev, t: s.t });
          }
          B.ev = null; B.offSince = null;
        }
      }
      // --- g / handling
      const g = s.g, total = Math.hypot(s.latG, s.lonG);
      g.peakLat = Math.max(g.peakLat, Math.abs(s.latG)); g.peakAcc = Math.max(g.peakAcc, s.lonG); g.peakBrk = Math.max(g.peakBrk, -s.lonG);
      g.maxTotal = Math.max(g.maxTotal, total); g.env = Math.max(0.8, 0.98 * g.maxTotal);
      if (s.t - g.lastSample >= 0.05) {
        g.lastSample = s.t; g.trail.push([s.latG, s.lonG]); if (g.trail.length > 60) g.trail.shift();
        g.latRing.push(s.latG); if (g.latRing.length > 40) g.latRing.shift();
      }
      const al = Math.abs(s.latG);
      if (!g.inCorner && al > 0.3) { g.inCorner = true; g.cornerPeak = total; }
      else if (g.inCorner) { g.cornerPeak = Math.max(g.cornerPeak, total); if (al < 0.15) { g.inCorner = false; g.corners++; g.cornerPeaks.push(g.cornerPeak); if (g.cornerPeaks.length > 20) g.cornerPeaks.shift(); } }
      // slow-lane session bits
      s.sess.fuel += (s._fuelRate() / 3600) * dt;
    }
    _fuelRate() { const maf = this._maf(); return (maf / 14.7 / 0.745) * 3.6; }
    _maf() { const map = 30 + 71 * (this.throttle / 100); return (this.rpm / 120) * 2.0 * (map / 101.3) * 1.2 * 0.9; }
    _dynoStart() {
      const d = this.dyno; torqueScale = 1 + this.rng.gauss() * 0.012; d.state = 'pulling'; d.bins = new Map(); d.startRpm = this.rpm; d.curve = []; d.lastBuild = this.t; d.t0 = this.t;
      this.events.push({ type: 'event', name: 'dyno.started', data: { gear: this.gear, rpm: this.rpm }, t: this.t });
    }
    _dynoCurve(half) {
      const d = this.dyno, keys = [...d.bins.keys()].sort((a, b) => a - b), out = [];
      for (const k of keys) {
        let n = 0, p = 0, t = 0, e = 0;
        for (let j = k - half; j <= k + half; j++) { const b = d.bins.get(j); if (b) { n += b.n; p += b.p; t += b.t; e += b.e; } }
        if (n) out.push({ rpm: k * 100, power_kw: p / n, torque_nm: t / n, ecu_torque_nm: e / n });
      }
      return out;
    }
    _dynoEnd() {
      const d = this.dyno, cf = U.clamp(1.18 * (99.0 / 101.3) * Math.sqrt((this.slow.intake + 273) / 298) - 0.18, 0.9, 1.1);
      const curve = this._dynoCurve(2).map((p) => ({ rpm: p.rpm, power_kw: p.power_kw * cf, torque_nm: p.torque_nm * cf, ecu_torque_nm: p.ecu_torque_nm }));
      const pk = curve.reduce((m, p) => (p.power_kw > m.power_kw ? p : m), curve[0] || { power_kw: 0, rpm: 0 });
      const tk = curve.reduce((m, p) => (p.torque_nm > m.torque_nm ? p : m), curve[0] || { torque_nm: 0, rpm: 0 });
      d.curve = curve; d.peaks = { pkw: pk.power_kw, prpm: pk.rpm, tnm: tk.torque_nm, trpm: tk.rpm, cf };
      const stride = Math.max(1, Math.ceil(curve.length / 60));
      d.runs.push({ id: d.runs.length + 1, t: this.t, gear: 3, rpm_start: d.startRpm, rpm_end: this.rpm, peak_power_kw: pk.power_kw, peak_power_rpm: pk.rpm, peak_torque_nm: tk.torque_nm, peak_torque_rpm: tk.rpm, correction: cf, curve: curve.filter((_, i) => i % stride === 0) });
      if (d.runs.length > 8) d.runs.shift();
      d.state = 'done'; d.doneAt = this.t; d.armed = false; d.wantPull = false; torqueScale = 1;
      this.events.push({ type: 'event', name: 'dyno.done', data: { peak_power_kw: pk.power_kw, peak_power_rpm: pk.rpm, peak_torque_nm: tk.torque_nm, peak_torque_rpm: tk.rpm, wheel_hp: pk.power_kw * 1.34102 }, t: this.t });
    }
    // ------------------------------------------------------------ telemetry assembly
    _telemetry() {
      const s = this, vk = s.v * 3.6, d = s.dyno, B = s.brake, g = s.g;
      const map = 30 + 71 * (s.throttle / 100), baro = 101.3;
      const nextGear = s.lastGear - 1, nextTarget = nextGear >= 1 ? vk * rpmPerKph(nextGear) : null;
      const sm = g.latRing.length > 2 ? (() => { let acc = 0; for (let i = 1; i < g.latRing.length; i++) { const j = (g.latRing[i] - g.latRing[i - 1]) / 0.05; acc += j * j; } return U.clamp(100 - 25 * Math.sqrt(acc / (g.latRing.length - 1)), 0, 100); })() : 100;
      const cornerScore = g.cornerPeaks.length ? (g.cornerPeaks.reduce((a, b) => a + b, 0) / g.cornerPeaks.length / g.env) * 100 : 0;
      const balance = s.v > 5.5 && Math.abs(s.yawAck) > 1 ? U.clamp((s.yaw - s.yawAck) / Math.max(Math.abs(s.yawAck), 3), -1, 1) : null;
      const curveNow = d.state === 'pulling' ? d.curve : d.curve;
      const pk = d.peaks;
      const msg = { idle: 'WOT in 3rd or 4th from ~2500 rpm to redline', armed: 'ARMED — floor it in 3rd', pulling: 'PULLING … hold it', done: pk ? `PEAK ${(pk.pkw * 1.34102).toFixed(0)} whp @ ${pk.prpm}` : 'done', rejected: 'pull too short' }[d.state];
      return {
        t: s.t, ts: Date.now() / 1000,
        conn: { bus: true, ecu: true, imu: true, source: 'demo', pid_hz: 46 + Math.sin(s.t) * 3, frame_hz: 212 + Math.sin(s.t * 0.7) * 6 },
        veh: {
          speed_kph: vk, rpm: s.rpm, gear: s.engaged ? s.gear : s.lastGear, gear_conf: s.engaged ? 0.97 : 0.2, gear_source: 'ratio',
          throttle_pct: s.throttle, load_pct: U.clamp((s.crankTorque / torqueCurve(Math.max(s.rpm, 1000))) * 100, 0, 100), coolant_c: s.slow.coolant, intake_c: s.slow.intake, oil_temp_c: s.slow.oil,
          maf_gps: s._maf(), map_kpa: map, boost_kpa: map - baro, baro_kpa: baro, fuel_pct: 62 - s.t * 0.0015, fuel_rate_lph: s._fuelRate(),
          batt_v: 14.1 + Math.sin(s.t * 3.1) * 0.03, timing_adv_deg: 12 + (s.rpm / 7500) * 18 - (s.throttle / 100) * 8, lambda: s.throttle > 85 ? 0.87 : 1.0,
          steering_deg: s.steer, yaw_dps: s.yaw, brake_switch: s.brakeBar > 1, clutch_switch: s.clutch > 0.5, ambient_c: 21,
        },
        g: { lat: s.latG, lon: s.lonG, vert: 1.0, total: Math.hypot(s.latG, s.lonG), peak_lat: g.peakLat, peak_accel: g.peakAcc, peak_brake: g.peakBrk, source: 'imu', trail: g.trail },
        brake: {
          pressure_bar: s.brakeBar, pressure_pct: U.clamp((s.brakeBar / VEH.maxBar) * 100, 0, 100), front_bar: s.brakeBar, rear_bar: Math.min(s.brakeBar, s.brakeBar * 0.6 + (s.brakeBar > 0 ? 4 : 0)),
          active: s.brakeBar >= 2, peak_bar: B.peak, source: 'can', trail_pct: B.ev ? (100 * B.ev.trail) / Math.max(1, B.ev.n) : 0, events: B.events, trace: B.trace, score: B.score, count: B.count,
        },
        shift: {
          state: s.shiftSeq ? (s.throttle > 25 ? 'blip' : 'disengaged') : 'idle', target_rpm: nextTarget != null && nextTarget <= VEH.redline ? nextTarget : null, target_gear: nextTarget != null && nextTarget <= VEH.redline ? nextGear : null,
          shift_light: s.rpm >= VEH.shiftLight && s.engaged, last: s.shift.last, session: s.shift.session,
        },
        dyno: {
          state: d.state, message: msg, gear: d.state === 'pulling' || d.state === 'done' ? 3 : 0, live_rpm: d.state === 'pulling' ? d.live[0] : 0, live_power_kw: d.state === 'pulling' ? d.live[1] : 0, live_torque_nm: d.state === 'pulling' ? d.live[2] : 0,
          progress: d.state === 'pulling' ? U.clamp((s.rpm - d.startRpm) / (VEH.redline - d.startRpm), 0, 1) : d.state === 'done' ? 1 : 0,
          curve: curveNow, peak_power_kw: pk ? pk.pkw : 0, peak_power_rpm: pk ? pk.prpm : 0, peak_torque_nm: pk ? pk.tnm : 0, peak_torque_rpm: pk ? pk.trpm : 0, runs: d.runs, wheel_hp: pk ? pk.pkw * 1.34102 : 0,
        },
        handling: { score: g.cornerPeaks.length ? 0.6 * sm + 0.4 * cornerScore : 0, smoothness: sm, balance, friction_used_pct: (Math.hypot(s.latG, s.lonG) / g.env) * 100, max_lat_g: g.peakLat, corner_count: g.corners },
        session: { duration_s: s.t, distance_km: s.dist / 1000, max_speed_kph: s.sess.maxSpeed, max_rpm: s.sess.maxRpm, fuel_used_l: s.sess.fuel },
        alerts: s.slow.coolant >= 110 ? ['COOLANT HIGH'] : [],
      };
    }
  }
  window.DemoSim = DemoSim;
})();
