"""Shift detection and downshift rev-match scoring.

State machine (driven by the :class:`~obddash.metrics.gears.GearEstimate`)::

    idle (locked in gear G) --unlock or clutch switch--> disengaged
    disengaged --locked again in G' != G--> engaged (1 s of post-engagement capture)
    engaged --1 s later--> idle   (the ShiftResult is returned on this tick)
    disengaged --locked again in G--> idle   (clutch dip / aborted shift, nothing counted)

``Shift.state`` also reports ``"blip"`` while disengaged with the throttle at or
above ``blip_min_throttle_pct``.  Every tick is recorded in a short ring
(rpm, throttle, brake, speed, lon_g) from which the shift is analysed when the new
gear locks.

Disengagement time
    ``t_dis`` is the tick the analyzer saw the unlock.  Without a clutch switch the
    ratio only unlocks once the free-revving engine has moved ~6 % away from the
    old gear, so ``t_dis`` is refined back to just after the last tick whose ratio
    was still within ``max(1.5 %, 60 / speed_kph %)`` of the old gear (at most
    0.5 s): that is when the clutch went down.

Blip / heel-toe (over ``[t_dis, t_eng]``)
    ``blip = max_throttle >= blip_min_throttle_pct and max_throttle >
    min_throttle_before + 5`` where ``min_throttle_before`` is the lowest throttle
    in the 0.5 s before disengagement (so a flat-foot clutch-in at WOT is not a
    blip, but a blip that starts a moment before the ratio unlocks still is).
    ``heel_toe = blip`` and the brake was active on a tick with blip-level throttle.

Engagement (G' locked)
    ``t_eng`` = ``GearEstimate.match_t`` (start of the stable, agreeing run).  The
    rpm that counts is the free-revving engine *as the clutch starts to bite*:
    after that the clutch drags it onto the gearbox speed and the error always
    reads ~0.  The reference is searched over *real* rpm samples (``rpm_fresh``
    ticks), each timed at the tick it arrived (a ~15 Hz OBD rpm is seen about three
    times by the 50 Hz loop, and rates must be measured between real samples):

    1. If the rpm current at the lock was already there before it (the lock was
       triggered by the clutch switch or a speed update), that rpm was presented.
       Otherwise the rpm changed into the lock: step to the sample before it -
       unless that step was the engine rising into the window on the throttle
       (throttle >= blip level, slower than a free-revving engine can rise:
       <= 150 %/s) while no clutch switch was in use.  Then the lock began while
       the driver was still holding the blip (ratio-only cars cannot see the
       pedal) and the held rpm *is* what was presented: use the sample of the
       tick that confirmed the lock.
    2. Walk back (at most 0.4 s, never before disengagement) while the error was
       converging on zero faster than 40 %/s - the clutch pulling - and, for an rpm
       *rising* toward the target, only while the throttle was below blip level (a
       rising engine with the throttle shut can only be the clutch; with it open it
       is the blip).  With a clutch switch, the newest pedal-down sample (the
       switch transition, where the clutch may already be biting: switches rarely
       sit exactly at the bite point) is judged the same way.  Each step back
       *past* it needs more than 150 %/s (``MAX_FREE_REV_PCT_S``): with the pedal
       down the clutch is open, so only a change no free-revving engine can make is
       drag.  This matters because a closed-throttle engine in neutral decays at
       about 40-70 %/s (the sim: ~55 %/s), which 40 %/s alone cannot tell from
       drag.  Without it, the common technique - over-blip, then let the revs fall
       to the target before releasing the clutch - is scored at the peak of the
       blip.  Ratio-only cars have no such signal (see "Known limits")::

        target_rpm = speed_kph(ref) * rpm_per_kph(G')
        actual_rpm = rpm(ref)
        error_pct  = (actual - target) / target * 100            (signed)
        duration_s = t_eng - t_dis

    With the rpm held until the clutch closes (a clean synthetic shift) the
    reference is simply the last sample before the lock.

Downshift score (0..100)::

    e      = |error_pct| * (under_rev_penalty if error_pct < 0 else 1)
             (engine too slow -> the car lurches / rear axle can lock, so it costs more)
    match  = 100                        if e <= perfect_pct
             0                          if e >= zero_pct
             100 * (zero - e) / (zero - perfect)  in between
    timing = +5 if duration <= fast_shift_s, -10 if >= slow_shift_s, linear in between
    blip   = -15 if no blip was seen (a blip-less downshift only matches by luck;
             not applied when no throttle signal exists at all)
    jolt   = -min(15, 40 * max(0, jolt_g - 0.15))          (only with an IMU lon_g)
    score  = clamp(match + timing + blip + jolt, 0, 100)
    grade  = S >= 95, A >= 85, B >= 70, C >= 50, else D

``jolt_g`` = the largest deviation of lon_g from its level while disengaged (mean
over the disengaged phase before the bite) from 0.05 s before the reference
sample to 0.5 s after ``t_eng``: the driveline shunt felt when the clutch closes on
an rpm mismatch.  The window closes early as soon as the driver moves a pedal
(brake +-5 bar or throttle +-15 % from its value at ``t_eng``), so releasing the
brake after a downshift is not mistaken for a shunt.

Upshifts are counted (``kind = "up"``) with ``score = 0`` and ``grade = "-"``; they
never enter the averages, best score or history and do not replace ``last``.
``ShiftSession.avg_score`` is the mean of every scored downshift this session;
``history`` keeps the last ``cfg.history`` scores.

Ignored entirely (not counted): engagement below ``min_speed_kph``, a disengaged
phase longer than 4 s (coasting in neutral, not a shift), a disengaged phase with
rpm or speed missing on any tick (an ECU dropout: the engagement itself was not
seen, so the rpm "presented" would really be the already-coupled rpm after the
signal came back), a start from no known gear, and anything involving reverse.

``transmission: auto``: ratio changes are still detected and counted, nothing is
scored and ``state`` stays ``"idle"``.

Traces: rpm / throttle / brake from 1.5 s before disengagement to 1.0 s after
engagement, downsampled to <= 120 points each, as ``[[t, value], ...]`` in the
caller's time base.  The brake trace uses ``brake_bar`` (pressure or estimate)
when given, else 1/0 from ``brake_active``.

Live hint: ``target_gear = current gear - 1`` and ``target_rpm`` = its rpm at the
current speed (None for no lower gear, above redline, unknown speed, or auto).
``shift_light = rpm >= vehicle.shift_light_rpm``.

Known limits
    * The drag that happens while the clutch switch still reads "pressed" (the
      switch flips after the bite) is only partly walked back.  In the sim (switch
      at half pedal, bite at 30 % engagement) this reads mismatches 0.5 % closer to
      zero on average (worst ~2 %).
    * Ratio-only cars (no clutch switch, no gear signal) cannot tell a closed-
      throttle decay through the new gear's +-6 % window from an engagement.  With
      1 km/h OBD speed steps, the decaying ratio can look stable for three ticks.
      An over-blip that the driver lets fall back before releasing the clutch can
      then be scored at the rpm where it entered the window (sim: +6 % read for a
      true -2 %).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from obddash.config import RevMatchConfig, VehicleConfig
from obddash.model import ShiftResult, ShiftSession

from .filters import TimedRing, clamp, downsample, lerp
from .gears import GearEstimate

log = logging.getLogger(__name__)

PRE_S = 1.5  # trace before disengagement
POST_S = 1.0  # trace (and jolt capture) after engagement
NEUTRAL_S = 4.0  # disengaged longer than this = neutral, not a shift
TRACE_MAX = 120
BLIP_RISE_PCT = 5.0
BLIP_BASE_S = 0.5  # look-back for the pre-shift throttle
DIS_REFINE_S = 0.5
COUPLED_BAND_PCT = 1.5
COUPLED_BAND_KPH = 0.6  # speed quantisation allowance for the coupled band
WALKBACK_MAX_S = 0.4
WALKBACK_MIN_RATE_PCT_S = 40.0
MAX_FREE_REV_PCT_S = 150.0  # faster than this, an rpm change is the clutch, not the engine
JOLT_PRE_S = 0.05
JOLT_POST_S = 0.5
JOLT_PEDAL_BRAKE_BAR = 5.0
JOLT_PEDAL_THROTTLE_PCT = 15.0
JOLT_FREE_G = 0.15
JOLT_GAIN = 40.0
JOLT_MAX_PENALTY = 15.0
NO_BLIP_PENALTY = 15.0

# ring entry value layout:
# (rpm, throttle, brake_value, speed_kph, lon_g, brake_active, rpm_fresh, clutch_pressed)
_RPM, _THR, _BRK, _SPD, _LON, _BACT, _FRESH, _CLU = range(8)


def grade_for(score: float) -> str:
    if score >= 95:
        return "S"
    if score >= 85:
        return "A"
    if score >= 70:
        return "B"
    if score >= 50:
        return "C"
    return "D"


def match_score(error_pct: float, cfg: RevMatchConfig) -> float:
    """The rev-match part of the score (0..100) for a signed rpm error in %."""
    e = abs(error_pct) * (cfg.under_rev_penalty if error_pct < 0 else 1.0)
    if e <= cfg.perfect_pct:
        return 100.0
    if e >= cfg.zero_pct:
        return 0.0
    return 100.0 * (cfg.zero_pct - e) / (cfg.zero_pct - cfg.perfect_pct)


def timing_score(duration_s: float, cfg: RevMatchConfig) -> float:
    """+5 for a quick shift, -10 for a slow one, linear in between."""
    if duration_s <= cfg.fast_shift_s:
        return 5.0
    if duration_s >= cfg.slow_shift_s:
        return -10.0
    span = max(1e-6, cfg.slow_shift_s - cfg.fast_shift_s)
    return lerp(5.0, -10.0, (duration_s - cfg.fast_shift_s) / span)


def jolt_penalty(jolt_g: float | None) -> float:
    if jolt_g is None:
        return 0.0
    return min(JOLT_MAX_PENALTY, JOLT_GAIN * max(0.0, jolt_g - JOLT_FREE_G))


def score_downshift(
    error_pct: float,
    duration_s: float,
    blip: bool,
    jolt_g: float | None,
    cfg: RevMatchConfig,
    *,
    throttle_known: bool = True,
) -> float:
    s = match_score(error_pct, cfg) + timing_score(duration_s, cfg)
    if throttle_known and not blip:
        s -= NO_BLIP_PENALTY
    s -= jolt_penalty(jolt_g)
    return clamp(s, 0.0, 100.0)


@dataclass(slots=True)
class _Pending:
    """An engaged shift waiting for its post-engagement window to fill."""

    t_dis: float
    t_eng: float
    t_ref: float
    from_gear: int
    to_gear: int
    kind: str
    duration_s: float
    target_rpm: float
    actual_rpm: float
    error_pct: float
    blip: bool
    heel_toe: bool
    throttle_known: bool
    finalize_at: float


class ShiftAnalyzer:
    def __init__(self, cfg: RevMatchConfig, vehicle: VehicleConfig) -> None:
        self.cfg = cfg
        self.vehicle = vehicle
        self.auto = vehicle.transmission == "auto"
        self._ring = TimedRing(PRE_S + DIS_REFINE_S + NEUTRAL_S + POST_S + 1.0, max_n=1200)
        self.reset()

    # -- public state --------------------------------------------------------------------
    def reset(self) -> None:
        self._ring.clear()
        self._disengaged = False
        self._gear = 0  # G: gear of the last locked tick
        self._t_dis = 0.0
        self._blipping = False
        self._clutch_seen = False  # a clutch switch was pressed during this disengagement
        self._data_gap = False  # rpm or speed went missing during this disengagement
        self._pending: _Pending | None = None
        self._scores_sum = 0.0
        self._n_scored = 0
        self.session = ShiftSession()
        self.last: ShiftResult | None = None
        self.target_rpm: float | None = None
        self.target_gear: int | None = None
        self.shift_light = False

    @property
    def state(self) -> str:
        if self.auto:
            return "idle"
        if self._disengaged:
            return "blip" if self._blipping else "disengaged"
        if self._pending is not None:
            return "engaged"
        return "idle"

    def session_snapshot(self) -> ShiftSession:
        s = self.session
        return ShiftSession(
            count=s.count,
            downshifts=s.downshifts,
            upshifts=s.upshifts,
            avg_score=s.avg_score,
            best_score=s.best_score,
            heel_toe_count=s.heel_toe_count,
            history=list(s.history),
        )

    # -- main ----------------------------------------------------------------------------
    def update(
        self,
        t: float,
        rpm: float | None,
        speed_kph: float | None,
        throttle_pct: float | None,
        brake_active: bool,
        clutch_switch: bool | None,
        est: GearEstimate,
        lon_g: float | None,
        *,
        brake_bar: float | None = None,
        rpm_fresh: bool = True,
    ) -> ShiftResult | None:
        """One tick.  ``rpm_fresh``: a new rpm sample arrived this tick (False when a
        slower signal is merely being re-read); ``brake_bar``: pressure or estimate
        for the brake trace.  Returns a ShiftResult once per completed shift."""
        brk = brake_bar if brake_bar is not None else (1.0 if brake_active else 0.0)
        entry = (
            rpm,
            throttle_pct,
            brk,
            speed_kph,
            lon_g,
            bool(brake_active),
            rpm_fresh,
            clutch_switch is True,
        )
        self._ring.append(t, entry)
        result: ShiftResult | None = None
        if self._pending is not None and t >= self._pending.finalize_at:
            result = self._finalize()

        locked = est.locked and clutch_switch is not True and est.gear != 0
        if not self._disengaged:
            if locked:
                self._gear = est.gear
            elif self._gear != 0:
                if self._pending is not None:  # a new shift starts inside the capture window
                    result = self._finalize()
                self._disengaged = True
                self._t_dis = t
                self._clutch_seen = False
                self._data_gap = False
        if self._disengaged:
            if clutch_switch is True:
                self._clutch_seen = True
            if rpm is None or speed_kph is None:
                self._data_gap = True  # dropout: whatever happens next was not observed
            if not locked:
                self._blipping = (
                    throttle_pct is not None and throttle_pct >= self.cfg.blip_min_throttle_pct
                )
            else:
                self._disengaged = False
                self._blipping = False
                if est.gear != self._gear:
                    self._pending = self._engage(t, est)
                self._gear = est.gear

        self._update_live(rpm, speed_kph, est)
        return result

    # -- analysis ------------------------------------------------------------------------
    def _engage(self, t: float, est: GearEstimate) -> _Pending | None:
        from_gear, new_gear = self._gear, est.gear
        if from_gear < 1 or new_gear < 1:
            return None  # from nothing / reverse
        t_eng = est.match_t if est.match_t is not None and est.match_t <= t else t
        t_dis = self._refine_disengage(from_gear)
        duration = max(0.0, t_eng - t_dis)
        if duration > NEUTRAL_S:
            log.debug("ignored %d->%d: disengaged %.1fs (neutral)", from_gear, new_gear, duration)
            return None
        if self._data_gap:
            log.debug("ignored %d->%d: rpm / speed missing while disengaged", from_gear, new_gear)
            return None
        rpk = self.vehicle.rpm_per_kph(new_gear)
        if not rpk:
            return None
        ref = self._reference_sample(t_dis, t_eng, t, rpk)
        if ref is None:
            return None
        rpm, speed = ref[1][_RPM], ref[1][_SPD]
        if rpm is None or speed is None or speed < self.cfg.min_speed_kph:
            return None
        target = speed * rpk
        if target <= 0:
            return None
        error = (rpm - target) / target * 100.0

        # blip / heel-toe over the disengaged window
        blip_min = self.cfg.blip_min_throttle_pct
        before = [
            v[_THR]
            for _, v in self._ring.between(t_dis - BLIP_BASE_S, t_dis)
            if v[_THR] is not None
        ]
        base_thr = min(before) if before else 0.0
        max_thr = 0.0
        thr_seen = False
        heel_toe = False
        for _, v in self._ring.between(t_dis, t_eng):
            thr = v[_THR]
            if thr is None:
                continue
            thr_seen = True
            if thr > max_thr:
                max_thr = thr
            if thr >= blip_min and v[_BACT]:
                heel_toe = True
        blip = thr_seen and max_thr >= blip_min and max_thr > base_thr + BLIP_RISE_PCT
        return _Pending(
            t_dis=t_dis,
            t_eng=t_eng,
            t_ref=ref[0],
            from_gear=from_gear,
            to_gear=new_gear,
            kind="down" if new_gear < from_gear else "up",
            duration_s=duration,
            target_rpm=target,
            actual_rpm=rpm,
            error_pct=error,
            blip=blip,
            heel_toe=blip and heel_toe,
            throttle_known=thr_seen,
            finalize_at=t_eng + POST_S,
        )

    def _refine_disengage(self, from_gear: int) -> float:
        """Move ``t_dis`` back to just after the last tick still coupled in the old gear."""
        t_dis = self._t_dis
        rpk = self.vehicle.rpm_per_kph(from_gear)
        if not rpk:
            return t_dis
        later = t_dis
        for tt, v in reversed(self._ring.between(t_dis - DIS_REFINE_S, t_dis)):
            if tt >= t_dis:
                continue
            rpm, speed = v[_RPM], v[_SPD]
            if rpm is None or speed is None or speed <= 0:
                break
            err = (rpm / (speed * rpk) - 1.0) * 100.0
            band = max(COUPLED_BAND_PCT, 100.0 * COUPLED_BAND_KPH / speed)
            if abs(err) <= band:
                return later  # first tick after the last coupled one
            later = tt
        return t_dis

    def _reference_sample(
        self, t_dis: float, t_eng: float, t_now: float, rpk: float
    ) -> tuple[float, tuple] | None:
        """The sample whose rpm is scored (see the module docstring, "Engagement")."""
        ticks = self._ring.between(t_dis, t_now)
        if not ticks:
            return self._ring.before(t_eng) or self._ring.last()
        # real rpm samples only, timed at the tick they arrived (a ~15 Hz OBD rpm is seen
        # ~3 times by a 50 Hz loop; rates must be measured between real samples)
        pts = [e for i, e in enumerate(ticks) if i == 0 or e[1][_FRESH]]

        def err(e: tuple[float, tuple]) -> float | None:
            rpm, speed = e[1][_RPM], e[1][_SPD]
            if rpm is None or speed is None or speed <= 0:
                return None
            return (rpm / (speed * rpk) - 1.0) * 100.0

        blip_min = self.cfg.blip_min_throttle_pct
        k = 0
        for i, e in enumerate(pts):
            if e[0] <= t_eng + 1e-9:
                k = i
        s0 = pts[k]
        e0 = err(s0)
        if e0 is None:
            return s0
        # step 1: the step into the lock
        if k == 0 or s0[0] < t_eng - 1e-9:
            best, e_best = s0, e0  # this rpm was already there before the lock
        else:
            prev = pts[k - 1]
            e_prev = err(prev)
            if e_prev is None:
                return s0
            rate = abs(e_prev - e0) / max(1e-3, s0[0] - prev[0])
            thr = prev[1][_THR]
            on_throttle = e_prev < e0 - 0.2 and thr is not None and thr >= blip_min
            if on_throttle and not self._clutch_seen and rate <= MAX_FREE_REV_PCT_S:
                # the ratio locked while the driver was still holding the blip: the held
                # rpm (as of the tick that confirmed the lock) is what was presented
                return ticks[-1]
            best, e_best = prev, e_prev
            k -= 1
        # step 2: walk back through the clutch drag.  The newest sample with the clutch
        # pedal down is the switch transition (the clutch may already be biting) and is
        # judged like any other; behind it the clutch was open, so only a change faster
        # than any free-revving engine counts as drag there.  A slower one - e.g. the
        # closed-throttle decay after an over-blip - is the engine itself.
        t_limit = best[0] - WALKBACK_MAX_S
        pedal_down_seen = bool(best[1][_CLU])
        for cand in reversed(pts[:k]):
            if cand[0] < t_limit:
                break
            e = err(cand)
            if e is None:
                break
            if e * e_best < 0 and abs(e_best) > 0.5:
                break  # crossed the target: not a one-sided pull
            if abs(e) <= abs(e_best) + 0.2:
                break  # not converging
            rate = (abs(e) - abs(e_best)) / max(1e-3, best[0] - cand[0])
            min_rate = MAX_FREE_REV_PCT_S if pedal_down_seen else WALKBACK_MIN_RATE_PCT_S
            if rate < min_rate:
                break  # as slow as a free-revving engine: not the clutch
            pedal_down_seen = pedal_down_seen or cand[1][_CLU]
            thr = cand[1][_THR]
            if e < 0 and thr is not None and thr >= blip_min:
                break  # rpm rising on the throttle: that is the blip
            best, e_best = cand, e
        return best

    def _jolt(self, p: _Pending) -> float | None:
        t0 = p.t_ref - JOLT_PRE_S
        base_vals = [v[_LON] for _, v in self._ring.between(p.t_dis, t0) if v[_LON] is not None]
        if not base_vals:
            near = self._ring.before(t0 + 1e-6)
            if near is not None and near[1][_LON] is not None:
                base_vals = [near[1][_LON]]
        at_eng = self._ring.before(p.t_eng + 1e-6)
        thr0 = at_eng[1][_THR] if at_eng is not None else None
        brk0 = at_eng[1][_BRK] if at_eng is not None else None
        win: list[float] = []
        for tt, v in self._ring.between(t0, p.t_eng + JOLT_POST_S):
            if tt > p.t_eng:
                thr, brk = v[_THR], v[_BRK]
                if (
                    thr0 is not None
                    and thr is not None
                    and abs(thr - thr0) > JOLT_PEDAL_THROTTLE_PCT
                ):
                    break
                if brk0 is not None and brk is not None and abs(brk - brk0) > JOLT_PEDAL_BRAKE_BAR:
                    break
            if v[_LON] is not None:
                win.append(v[_LON])
        if not base_vals or not win:
            return None
        base = sum(base_vals) / len(base_vals)
        return max(abs(x - base) for x in win)

    def _finalize(self) -> ShiftResult | None:
        p = self._pending
        self._pending = None
        if p is None:
            return None
        pts = downsample(self._ring.between(p.t_dis - PRE_S, p.t_eng + POST_S), TRACE_MAX)
        rpm_tr = [[round(tt, 3), round(v[_RPM], 1)] for tt, v in pts if v[_RPM] is not None]
        thr_tr = [[round(tt, 3), round(v[_THR], 1)] for tt, v in pts if v[_THR] is not None]
        brk_tr = [[round(tt, 3), round(v[_BRK], 2)] for tt, v in pts if v[_BRK] is not None]
        jolt = self._jolt(p)
        scored = p.kind == "down" and not self.auto
        if scored:
            raw_score = score_downshift(
                p.error_pct, p.duration_s, p.blip, jolt, self.cfg, throttle_known=p.throttle_known
            )
            score = round(raw_score, 1)
            grade = grade_for(score)
        else:
            score, grade = 0.0, "-"
        res = ShiftResult(
            t=round(p.t_eng, 3),
            from_gear=p.from_gear,
            to_gear=p.to_gear,
            kind=p.kind,
            duration_s=round(p.duration_s, 3),
            target_rpm=round(p.target_rpm, 1),
            actual_rpm=round(p.actual_rpm, 1),
            error_pct=round(p.error_pct, 2),
            blip=p.blip,
            heel_toe=p.heel_toe,
            jolt_g=None if jolt is None else round(jolt, 3),
            score=score,
            grade=grade,
            rpm_trace=rpm_tr,
            throttle_trace=thr_tr,
            brake_trace=brk_tr,
        )
        self._record(res, scored)
        return res

    def _record(self, res: ShiftResult, scored: bool) -> None:
        s = self.session
        s.count += 1
        if res.kind == "down":
            s.downshifts += 1
        else:
            s.upshifts += 1
        if not scored:
            return
        if res.heel_toe:
            s.heel_toe_count += 1
        self._scores_sum += res.score
        self._n_scored += 1
        s.avg_score = round(self._scores_sum / self._n_scored, 2)
        s.best_score = max(s.best_score, res.score)
        s.history.append(res.score)
        keep = max(1, self.cfg.history)
        if len(s.history) > keep:
            del s.history[: len(s.history) - keep]
        self.last = res

    # -- live ----------------------------------------------------------------------------
    def _update_live(self, rpm: float | None, speed_kph: float | None, est: GearEstimate) -> None:
        self.shift_light = rpm is not None and rpm >= self.vehicle.shift_light_rpm
        cur = est.gear if est.locked else self._gear
        self.target_gear = None
        self.target_rpm = None
        if self.auto or speed_kph is None or cur < 2:
            return
        rpk = self.vehicle.rpm_per_kph(cur - 1)
        if not rpk:
            return
        target = speed_kph * rpk
        if target > self.vehicle.redline_rpm:
            return
        self.target_gear = cur - 1
        self.target_rpm = round(target, 1)
