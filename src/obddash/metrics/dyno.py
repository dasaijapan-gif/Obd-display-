"""Virtual dyno: wheel power and torque from a full-throttle pull in one gear.

Pull detection
--------------
Runs whether or not the dyno is armed (arming only sets the UI hint / state
``"armed"``, lets a pull in *any* gear count, and makes the pull announce itself
with a ``dyno.started`` event).  Unarmed, only pulls in ``cfg.preferred_gears``
are captured, silently, so normal spirited driving in 1st / 2nd does not replace
the curve.

* candidate: throttle >= ``min_throttle_pct``, gear locked, rpm rising
  (filtered d(rpm)/dt > 15 rpm/s); all must hold continuously for 0.4 s, then the
  state becomes ``"pulling"``.
* the pull ends when the throttle drops below ``min_throttle_pct - 10``, the gear
  changes or unlocks (clutch), a signal is lost, or rpm stops rising for 0.5 s
  (then the samples after the last rising tick are discarded).
* shorter than ``min_duration_s`` or spanning less than ``min_rpm_span`` (raw rpm,
  start to end) -> ``"rejected"`` with the reason as the message; back to
  idle / armed after 3 s.  A good pull -> ``"done"`` (for 10 s, then idle; the
  curve stays) and the arm is consumed.  ``cancel()`` / ``clear()`` during a pull
  abort it, and nothing new is detected until the throttle has been lifted.

Physics (per tick while pulling)
--------------------------------
Speed (m/s) and rpm go through *identical* filter chains - a low-pass with
``tau = cfg.speed_smoothing_s`` followed by a least-squares slope over a 0.3 s
window - so acceleration, speed and rpm all describe the same instant
(about ``tau + 0.15 s`` ago)::

    v   = centre of the low-passed speed in the window              [m/s]
    a   = dv/dt  (least-squares slope)                              [m/s^2]
    F_wheel = m*k*a + 0.5*rho*Cd*A*v^2 + Crr*m*g                     [N]
              (k = rotational_inertia_factor, rho = cfg.air_density_kgm3, g = 9.81)
    P_wheel = F_wheel * v                                           [W]
    w       = rpm * 2*pi/60                                         [rad/s]
    T_wheel_ref = P_wheel / w     torque referred to engine rpm (what a chassis
                                  dyno prints; = crank torque x drivetrain efficiency)
    crank estimate = wheel / drivetrain_efficiency   (run.peak_crank_kw)
    ECU torque = engine_torque_pct / 100 * engine_ref_torque_nm   (crank, overlay only)

Samples from the first ``3 * tau + 0.15 s`` after the throttle opened are skipped
(the low-pass is still converging on the new acceleration and would read low), as
are samples within 50 rpm of the redline (limiter).

Curve: samples are binned by ``round(rpm / rpm_bin)``; each output point is the
count-weighted mean of the bins within ``+-smoothing_bins``.  ``power_kw`` is
WHEEL kW and ``torque_nm`` wheel-referred Nm; ``ecu_torque_nm`` when reported.

SAE J1349-style correction (``cfg.correction == "sae"`` with baro and intake
temperature known, averaged over the pull)::

    cf = 1.18 * (99.0 / baro_kpa) * sqrt((intake_c + 273) / 298) - 0.18,   clamped 0.9..1.1

applied to power and torque (not to ECU torque); ``run.correction = cf``.
Peaks come from the smoothed, corrected curve; ``wheel_hp = peak_power_kw * 1.34102``.
The last ``cfg.max_runs`` runs are kept (curves downsampled to <= 40 points, so
eight runs still fit a < 60 kB telemetry frame); the current curve is <= 120 points.  ``progress = (max rpm so far - start rpm) /
(redline - start rpm)``.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from typing import Any

from obddash.config import DynoConfig, VehicleConfig
from obddash.model import Dyno, DynoPoint, DynoRun

from .filters import Differentiator, clamp, downsample
from .gears import GearEstimate

log = logging.getLogger(__name__)

G = 9.81
HP_PER_KW = 1.34102
DIFF_WINDOW_S = 0.3
RISE_MIN_RPM_S = 15.0
CONFIRM_S = 0.4
NOT_RISING_END_S = 0.5
LIFT_HYSTERESIS_PCT = 10.0
REJECT_HOLD_S = 3.0
DONE_HOLD_S = 10.0
LIMITER_MARGIN_RPM = 50.0
SETTLE_TAUS = 3.0
MAX_PULL_S = 60.0
MAX_SAMPLES = 4000
LIVE_REBUILD_S = 0.25
CURVE_MAX = 120
RUN_CURVE_MAX = 40  # <= 60 per the contract; 8 runs x 40 points keeps the JSON < 60 kB


# kept as an alias: peak_crank_kw now lives on model.DynoRun itself
DynoRunRecord = DynoRun


def sae_correction(baro_kpa: float | None, intake_c: float | None) -> float:
    """SAE J1349-style correction factor (1.0 when inputs are unknown)."""
    if baro_kpa is None or intake_c is None or baro_kpa <= 0:
        return 1.0
    cf = 1.18 * (99.0 / baro_kpa) * math.sqrt(max(0.0, intake_c + 273.0) / 298.0) - 0.18
    return clamp(cf, 0.9, 1.1)


def ordinal(n: int) -> str:
    if 10 <= n % 100 <= 20:
        suf = "th"
    else:
        suf = {1: "st", 2: "nd", 3: "rd"}.get(n % 10, "th")
    return f"{n}{suf}"


def _gears_text(gears: list[int]) -> str:
    gs = [ordinal(g) for g in gears] or ["3rd", "4th"]
    if len(gs) == 1:
        return gs[0]
    return ", ".join(gs[:-1]) + " or " + gs[-1]


class _Pull:
    __slots__ = (
        "gear",
        "t_start",
        "rpm_start",
        "rpm_last",
        "confirmed",
        "last_rising_t",
        "rpm_at_last_rising",
        "n_at_last_rising",
        "samples",
        "bins",
        "baro_sum",
        "baro_n",
        "intake_sum",
        "intake_n",
        "live_power_kw",
        "live_torque_nm",
    )

    def __init__(self, gear: int, t: float, rpm: float) -> None:
        self.gear = gear
        self.t_start = t
        self.rpm_start = rpm
        self.rpm_last = rpm
        self.confirmed = False
        self.last_rising_t = t
        self.rpm_at_last_rising = rpm
        self.n_at_last_rising = 0
        # (t, rpm, power_kw, torque_nm, ecu_torque_nm | None)
        self.samples: list[tuple[float, float, float, float, float | None]] = []
        self.bins: dict[int, list[float]] = {}
        self.baro_sum = 0.0
        self.baro_n = 0
        self.intake_sum = 0.0
        self.intake_n = 0
        self.live_power_kw = 0.0
        self.live_torque_nm = 0.0

    def correction(self, mode: str) -> float:
        if mode != "sae" or not self.baro_n or not self.intake_n:
            return 1.0
        return sae_correction(self.baro_sum / self.baro_n, self.intake_sum / self.intake_n)


class VirtualDyno:
    def __init__(self, cfg: DynoConfig, vehicle: VehicleConfig) -> None:
        self.cfg = cfg
        self.vehicle = vehicle
        self.events: list[tuple[str, dict[str, Any]]] = []
        tau = max(0.0, cfg.speed_smoothing_s)
        self._vd = Differentiator(DIFF_WINDOW_S, tau)
        self._rd = Differentiator(DIFF_WINDOW_S, tau)
        self._settle_s = SETTLE_TAUS * tau + DIFF_WINDOW_S / 2.0
        mass = vehicle.mass_kg
        self._m_eff = mass * vehicle.rotational_inertia_factor
        self._aero = 0.5 * cfg.air_density_kgm3 * vehicle.cd * vehicle.frontal_area_m2
        self._roll = vehicle.crr * mass * G
        self._runs: deque[DynoRunRecord] = deque(maxlen=max(1, cfg.max_runs))
        self._next_id = 1
        self._reset_state()

    # -- lifecycle -----------------------------------------------------------------------
    def _reset_state(self) -> None:
        self._state = "idle"
        self._armed = False
        self._pull: _Pull | None = None
        self._wot_since: float | None = None
        self._hold_off = False  # after a cancel / clear mid-pull: wait for a lift
        self._hold_until = 0.0
        self._message_override = ""
        self._curve: list[DynoPoint] = []
        self._live_curve: list[DynoPoint] = []
        self._live_built_t = -1e9
        self._last_run: DynoRunRecord | None = None
        self._gear = 0
        self._live_peaks = (0.0, 0.0, 0.0, 0.0)

    def reset(self) -> None:
        """Forget everything (session reset)."""
        self._vd.reset()
        self._rd.reset()
        self._runs.clear()
        self.events.clear()
        self._reset_state()

    @property
    def state(self) -> str:
        return self._state

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def runs(self) -> list[DynoRunRecord]:
        return list(self._runs)

    def drain_events(self) -> list[tuple[str, dict[str, Any]]]:
        ev, self.events = self.events, []
        return ev

    # -- commands ------------------------------------------------------------------------
    def arm(self) -> None:
        self._armed = True
        if self._state != "pulling":
            self._state = "armed"

    def cancel(self) -> None:
        self._armed = False
        self._abort("cancelled")
        if self._state in ("armed", "pulling"):
            self._state = "idle"

    def clear(self) -> None:
        self._abort("cleared")
        self._runs.clear()
        self._curve = []
        self._live_curve = []
        self._last_run = None
        self._gear = 0
        self._state = "armed" if self._armed else "idle"

    def _abort(self, reason: str) -> None:
        p = self._pull
        self._pull = None
        if p is not None:
            self._hold_off = True  # the rest of this WOT run is not a new pull
            if p.confirmed:
                self.events.append(("dyno.done", {"ok": False, "reason": reason, "gear": p.gear}))

    # -- main ----------------------------------------------------------------------------
    def update(
        self,
        t: float,
        rpm: float | None,
        speed_kph: float | None,
        throttle_pct: float | None,
        est: GearEstimate,
        baro_kpa: float | None = None,
        intake_c: float | None = None,
        ecu_torque_pct: float | None = None,
        ecu_ref_torque_nm: float | None = None,
    ) -> None:
        cfg = self.cfg
        if self._state in ("done", "rejected") and t >= self._hold_until:
            self._state = "armed" if self._armed else "idle"
        if rpm is None or speed_kph is None:
            if self._pull is not None:
                self._end(t, "signal lost")
            self._vd.reset()
            self._rd.reset()
            self._wot_since = None
            return

        a = self._vd.update(speed_kph / 3.6, t)
        drpm = self._rd.update(rpm, t)
        wot = throttle_pct is not None and throttle_pct >= cfg.min_throttle_pct
        lifted = throttle_pct is None or throttle_pct < cfg.min_throttle_pct - LIFT_HYSTERESIS_PCT
        if wot:
            if self._wot_since is None:
                self._wot_since = t
        elif lifted:
            self._wot_since = None
            self._hold_off = False
        rising = drpm is not None and drpm > RISE_MIN_RPM_S
        gear = est.gear if est.locked and est.gear >= 1 else None

        p = self._pull
        if p is None:
            if self._hold_off or not (
                wot and rising and gear is not None and self._gear_allowed(gear)
            ):
                return
            p = self._pull = _Pull(gear, t, rpm)
        elif not p.confirmed:
            if not (wot and rising and gear == p.gear):
                self._pull = None  # never became a pull
                return
            if t - p.t_start >= CONFIRM_S:
                self._confirm(p)
        else:
            reason = None
            if lifted:
                reason = "lifted"
            elif gear != p.gear:
                reason = "gear changed" if gear is not None else "clutch / gear unlocked"
            elif not rising and t - p.last_rising_t >= NOT_RISING_END_S:
                reason = "rpm stopped rising"
            elif t - p.t_start > MAX_PULL_S or len(p.samples) >= MAX_SAMPLES:
                reason = "too long"
            if reason is not None:
                self._end(t, reason)
                return

        p.rpm_last = rpm
        if baro_kpa is not None:
            p.baro_sum += baro_kpa
            p.baro_n += 1
        if intake_c is not None:
            p.intake_sum += intake_c
            p.intake_n += 1
        self._sample(p, t, a, rpm, ecu_torque_pct, ecu_ref_torque_nm)
        if rising:
            p.last_rising_t = t
            p.rpm_at_last_rising = rpm
            p.n_at_last_rising = len(p.samples)
        if p.confirmed and t - self._live_built_t >= LIVE_REBUILD_S:
            self._live_built_t = t
            self._live_curve = self._build_curve(p.bins, p.correction(cfg.correction))
            self._live_peaks = _peaks(self._live_curve)

    def _gear_allowed(self, gear: int) -> bool:
        pref = self.cfg.preferred_gears
        return self._armed or not pref or gear in pref

    def _sample(
        self,
        p: _Pull,
        t: float,
        a: float | None,
        rpm: float,
        ecu_pct: float | None,
        ecu_ref: float | None,
    ) -> None:
        v = self._vd.center
        rpm_c = self._rd.center
        if a is None or v is None or rpm_c is None or rpm_c <= 0 or v <= 0:
            return
        if self._wot_since is None or t - self._wot_since < self._settle_s:
            return
        if rpm >= self.vehicle.redline_rpm - LIMITER_MARGIN_RPM:
            return
        force = self._m_eff * a + self._aero * v * v + self._roll
        power_w = force * v
        omega = rpm_c * 2.0 * math.pi / 60.0
        torque = power_w / omega
        ecu = ecu_pct / 100.0 * ecu_ref if ecu_pct is not None and ecu_ref is not None else None
        p.samples.append((t, rpm_c, power_w / 1000.0, torque, ecu))
        self._bin_add(p.bins, rpm_c, power_w / 1000.0, torque, ecu)
        p.live_power_kw = power_w / 1000.0
        p.live_torque_nm = torque

    def _bin_add(
        self, bins: dict[int, list[float]], rpm: float, kw: float, nm: float, ecu: float | None
    ) -> None:
        k = int(round(rpm / max(1.0, self.cfg.rpm_bin)))
        b = bins.get(k)
        if b is None:
            b = bins[k] = [0.0, 0.0, 0.0, 0.0, 0.0]
        b[0] += 1.0
        b[1] += kw
        b[2] += nm
        if ecu is not None:
            b[3] += 1.0
            b[4] += ecu

    def _build_curve(self, bins: dict[int, list[float]], cf: float) -> list[DynoPoint]:
        half = max(0, int(self.cfg.smoothing_bins))
        width = max(1.0, self.cfg.rpm_bin)
        out: list[DynoPoint] = []
        for k in sorted(bins):
            n = sp = st = ne = se = 0.0
            for j in range(k - half, k + half + 1):
                b = bins.get(j)
                if b is not None:
                    n += b[0]
                    sp += b[1]
                    st += b[2]
                    ne += b[3]
                    se += b[4]
            if n <= 0:
                continue
            out.append(
                DynoPoint(
                    rpm=round(k * width, 1),
                    power_kw=round(sp / n * cf, 1),
                    torque_nm=round(st / n * cf, 1),
                    ecu_torque_nm=round(se / ne, 1) if ne else None,
                )
            )
        return downsample(out, CURVE_MAX)

    def _confirm(self, p: _Pull) -> None:
        p.confirmed = True
        self._state = "pulling"
        self._gear = p.gear
        self._live_curve = []
        self._live_peaks = (0.0, 0.0, 0.0, 0.0)
        self._live_built_t = -1e9
        if self._armed:
            self.events.append(
                ("dyno.started", {"gear": p.gear, "rpm": round(p.rpm_start), "armed": True})
            )
        log.info("dyno pull started in %s at %.0f rpm", ordinal(p.gear), p.rpm_start)

    def _end(self, t: float, reason: str) -> None:
        p = self._pull
        self._pull = None
        if p is None or not p.confirmed:
            return
        cfg = self.cfg
        if reason == "rpm stopped rising":
            samples = p.samples[: p.n_at_last_rising]
            t_end, rpm_end = p.last_rising_t, p.rpm_at_last_rising
        else:
            samples = p.samples
            t_end, rpm_end = t, p.rpm_last
        duration = t_end - p.t_start
        span = rpm_end - p.rpm_start
        bins = p.bins
        if len(samples) != len(p.samples):
            bins = {}
            for _t, r, kw, nm, ecu in samples:
                self._bin_add(bins, r, kw, nm, ecu)
        cf = p.correction(cfg.correction)
        curve = self._build_curve(bins, cf)
        if duration < cfg.min_duration_s:
            self._reject(t, p, f"pull too short ({duration:.1f} s < {cfg.min_duration_s:g} s)")
            return
        if span < cfg.min_rpm_span:
            self._reject(t, p, f"rpm span too small ({span:.0f} < {cfg.min_rpm_span:.0f} rpm)")
            return
        if len(curve) < 3:
            self._reject(t, p, "not enough data in the pull")
            return
        pk_kw, pk_kw_rpm, pk_nm, pk_nm_rpm = _peaks(curve)
        eff = self.vehicle.drivetrain_efficiency or 1.0
        run = DynoRunRecord(
            id=self._next_id,
            t=round(t, 3),
            gear=p.gear,
            rpm_start=round(p.rpm_start, 1),
            rpm_end=round(rpm_end, 1),
            peak_power_kw=pk_kw,
            peak_power_rpm=pk_kw_rpm,
            peak_torque_nm=pk_nm,
            peak_torque_rpm=pk_nm_rpm,
            correction=round(cf, 4),
            curve=downsample(curve, RUN_CURVE_MAX),
            peak_crank_kw=round(pk_kw / eff, 2),
        )
        self._next_id += 1
        self._runs.append(run)
        self._last_run = run
        self._curve = curve
        self._state = "done"
        self._hold_until = t + DONE_HOLD_S
        self._armed = False
        self.events.append(
            (
                "dyno.done",
                {
                    "ok": True,
                    "id": run.id,
                    "gear": run.gear,
                    "peak_power_kw": run.peak_power_kw,
                    "peak_power_rpm": run.peak_power_rpm,
                    "peak_torque_nm": run.peak_torque_nm,
                    "peak_torque_rpm": run.peak_torque_rpm,
                    "wheel_hp": round(run.peak_power_kw * HP_PER_KW, 1),
                    "peak_crank_kw": run.peak_crank_kw,
                    "correction": run.correction,
                },
            )
        )
        log.info(
            "dyno run %d (%s): %.1f kW @ %.0f, %.0f Nm @ %.0f, cf %.3f",
            run.id,
            ordinal(run.gear),
            pk_kw,
            pk_kw_rpm,
            pk_nm,
            pk_nm_rpm,
            cf,
        )

    def _reject(self, t: float, p: _Pull, reason: str) -> None:
        self._state = "rejected"
        self._message_override = reason
        self._hold_until = t + REJECT_HOLD_S
        self._gear = self._last_run.gear if self._last_run is not None else 0
        self.events.append(("dyno.done", {"ok": False, "reason": reason, "gear": p.gear}))
        log.info("dyno pull rejected: %s", reason)

    # -- output --------------------------------------------------------------------------
    def message(self) -> str:
        st = self._state
        if st == "armed":
            return "ARMED — floor it"
        if st == "pulling":
            return "PULLING … hold it"
        if st == "done" and self._last_run is not None:
            hp = self._last_run.peak_power_kw * HP_PER_KW
            return f"PEAK {hp:.0f} whp @ {self._last_run.peak_power_rpm:.0f}"
        if st == "rejected":
            return self._message_override
        return f"WOT in {_gears_text(list(self.cfg.preferred_gears))} from ~2500 rpm to redline"

    def snapshot(self) -> Dyno:
        pulling = self._state == "pulling" and self._pull is not None
        p = self._pull
        if pulling and p is not None:
            curve = self._live_curve
            pk_kw, pk_kw_rpm, pk_nm, pk_nm_rpm = self._live_peaks
            redline = self.vehicle.redline_rpm
            denom = redline - p.rpm_start
            progress = clamp((p.rpm_last - p.rpm_start) / denom, 0.0, 1.0) if denom > 0 else 0.0
            live = (p.rpm_last, p.live_power_kw, p.live_torque_nm)
        else:
            curve = self._curve
            r = self._last_run
            if r is not None:
                pk_kw, pk_kw_rpm = r.peak_power_kw, r.peak_power_rpm
                pk_nm, pk_nm_rpm = r.peak_torque_nm, r.peak_torque_rpm
            else:
                pk_kw = pk_kw_rpm = pk_nm = pk_nm_rpm = 0.0
            progress = 1.0 if self._state == "done" else 0.0
            live = (0.0, 0.0, 0.0)
        return Dyno(
            state=self._state,
            message=self.message(),
            gear=self._gear,
            live_rpm=round(live[0], 1),
            live_power_kw=round(live[1], 2),
            live_torque_nm=round(live[2], 2),
            progress=round(progress, 3),
            curve=list(curve),
            peak_power_kw=pk_kw,
            peak_power_rpm=pk_kw_rpm,
            peak_torque_nm=pk_nm,
            peak_torque_rpm=pk_nm_rpm,
            runs=list(self._runs),
            wheel_hp=round(pk_kw * HP_PER_KW, 1),
        )


def _peaks(curve: list[DynoPoint]) -> tuple[float, float, float, float]:
    """(peak kW, its rpm, peak Nm, its rpm) of a curve; zeros when empty."""
    if not curve:
        return 0.0, 0.0, 0.0, 0.0
    pk = max(curve, key=lambda q: q.power_kw)
    tk = max(curve, key=lambda q: q.torque_nm)
    return pk.power_kw, pk.rpm, tk.torque_nm, tk.rpm
