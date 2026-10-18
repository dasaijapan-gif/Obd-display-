"""Gear estimation: which gear is engaged, and is the driveline actually coupled?

Two sources, best first:

1. ``"can"`` - the car broadcasts the selected gear (``Sig.GEAR``).  That is the
   lever position, so it is reported as ``gear`` whatever the clutch is doing, with
   ``conf = 1``.  Whether the driveline is *coupled* (``locked``) still comes from
   the checks below (clutch switch, speed, rpm, and the ratio of that gear when it
   can be computed), because the rev-match analyzer needs the moment the clutch
   really bites, not the moment the lever reaches the gate.  Reverse is ``-1``.
   Values outside ``-1 .. gear_count`` (the "invalid" / between-gates codes some
   cars broadcast while the lever moves) are ignored: that tick uses the ratio.
2. ``"ratio"`` - the observed ``rpm / speed_kph`` against
   ``VehicleConfig.rpm_per_kph(g)`` for every forward gear::

       err_g = (rpm / (speed_kph * rpm_per_kph(g)) - 1) * 100      [%]

   The gear with the smallest ``|err_g|`` is the per-tick *candidate* if
   ``|err_g| <= TOL_PCT`` (6 %).  ``conf = 1 - |err| / TOL_PCT``.

A tick has no candidate (``locked = False``) when the clutch switch is pressed,
speed < 5 km/h, rpm < idle - 100, a signal is missing, or no ratio is within the
tolerance (coasting in neutral, clutch slipping, free-revving engine).

Hysteresis
----------
* Staying in the locked gear needs nothing more than a matching tick.
* Locking into a *different* gear (or the first lock) needs ``CONFIRM_TICKS`` (3)
  consecutive *fresh* ticks agreeing on it **and** a stable ratio over those ticks:
  the spread of ``err`` must be within ``max(2 %, 120 / speed_kph %)`` (the second
  term allows for 1 km/h OBD speed quantisation at low speed).  The stability check
  is what stops a free-revving engine that sweeps *through* a gear's ±6 % window
  during a blip from being taken for an engagement.  "Fresh" (``update(...,
  fresh=)``, default True) means a new rpm sample arrived this tick: an OBD rpm PID
  answers at ~15 Hz, so a 50 Hz loop sees each value about three times, and three
  identical repeats must not count as three agreeing observations.  Ticks that are
  not fresh neither advance nor reset the agreeing run.
* Re-locking into the *same* gear within ``RELOCK_S`` (0.3 s) of losing it is
  immediate, so a single noisy tick never costs more than that one tick.

``match_t`` on the estimate is when the current lock began: the first tick of the
last ``CONFIRM_TICKS`` stable, agreeing observations (so a free-revving rpm still
settling into the window does not date the engagement early).  The shift analyzer
uses it as the engagement instant.

Display gear (``GearEstimate.gear``): the locked gear; while unlocked only because
the ratio briefly stopped matching, the previous gear is held for ``HOLD_S`` (0.3 s)
to avoid a flickering "N"; otherwise 0 (clutch in, stopped, neutral, unknown).
For the CAN source it is always the lever position.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from obddash.config import VehicleConfig

from .filters import clamp


@dataclass(slots=True)
class GearEstimate:
    gear: int = 0  # -1 R, 0 N / unknown, 1..n (display value, see module docstring)
    conf: float = 0.0  # 0..1
    source: str = "none"  # "can" | "ratio" | "none"
    locked: bool = False  # driveline coupled in ``gear`` (ratio matches, clutch up)
    ratio_err_pct: float | None = None  # signed error vs the (candidate) gear's ratio
    match_t: float | None = None  # first tick of the agreeing run behind the current lock


class GearEstimator:
    TOL_PCT = 6.0
    CONFIRM_TICKS = 3
    RELOCK_S = 0.3
    HOLD_S = 0.3
    MIN_SPEED_KPH = 5.0
    IDLE_MARGIN_RPM = 100.0
    STABLE_SPREAD_PCT = 2.0
    STABLE_SPREAD_KPH = 1.2  # speed quantisation allowance, as km/h of error

    def __init__(
        self,
        vehicle: VehicleConfig,
        *,
        tol_pct: float | None = None,
        confirm_ticks: int | None = None,
    ) -> None:
        self.vehicle = vehicle
        self.tol_pct = float(tol_pct if tol_pct is not None else self.TOL_PCT)
        self.confirm_ticks = max(1, int(confirm_ticks or self.CONFIRM_TICKS))
        self._rpk: list[tuple[int, float]] = []
        for g in range(1, vehicle.gear_count() + 1):
            r = vehicle.rpm_per_kph(g)
            if r and r > 0:
                self._rpk.append((g, r))
        self._min_rpm = float(vehicle.idle_rpm) - self.IDLE_MARGIN_RPM
        self.reset()

    # -- state -------------------------------------------------------------------------------
    def reset(self) -> None:
        self._gear = 0  # gear of the current / last lock
        self._locked = False
        self._lock_t: float | None = None
        self._unlock_t: float | None = None
        self._run_gear: int | None = None
        self._run_t: float | None = None
        self._run_n = 0
        self._run_errs: deque[tuple[float, float]] = deque(maxlen=self.confirm_ticks)  # (t, err)
        self.last_engaged_gear = 0
        self.estimate = GearEstimate()

    # -- helpers -----------------------------------------------------------------------------
    def ratio_error_pct(self, gear: int, rpm: float, speed_kph: float) -> float | None:
        """Signed % error of the observed rpm against ``gear``'s rpm at this speed."""
        rpk = self.vehicle.rpm_per_kph(gear)
        if not rpk or speed_kph <= 0:
            return None
        return (rpm / (speed_kph * rpk) - 1.0) * 100.0

    def nearest_gear(self, rpm: float, speed_kph: float) -> tuple[int | None, float | None]:
        """Nearest configured gear and its signed error; gear is None outside the
        tolerance (the error of the nearest gear is still returned)."""
        if speed_kph <= 0 or not self._rpk:
            return None, None
        observed = rpm / speed_kph
        best_g, best_e = 0, 0.0
        best_abs = float("inf")
        for g, rpk in self._rpk:
            e = (observed / rpk - 1.0) * 100.0
            ae = abs(e)
            if ae < best_abs:
                best_g, best_e, best_abs = g, e, ae
        if best_abs <= self.tol_pct:
            return best_g, best_e
        return None, best_e

    def _stable_tol(self, speed_kph: float) -> float:
        return max(self.STABLE_SPREAD_PCT, 100.0 * self.STABLE_SPREAD_KPH / max(speed_kph, 1.0))

    # -- main --------------------------------------------------------------------------------
    def update(
        self,
        t: float,
        rpm: float | None,
        speed_kph: float | None,
        clutch_switch: bool | None,
        gear_signal: int | None,
        *,
        fresh: bool = True,
    ) -> GearEstimate:
        clutch_in = clutch_switch is True
        moving = speed_kph is not None and speed_kph >= self.MIN_SPEED_KPH
        running = rpm is not None and rpm >= self._min_rpm
        candidate: int | None = None
        err: float | None = None
        lever: int | None = None

        if gear_signal is not None and not -1 <= round(gear_signal) <= len(
            self.vehicle.gear_ratios
        ):
            # an "invalid" / between-gates code (many cars send 7 or 15 while the lever moves):
            # not a gear - fall back to the ratio method rather than "lock" into it
            gear_signal = None
        if gear_signal is not None:
            source = "can"
            lever = int(round(gear_signal))
            if lever >= 1 and rpm is not None and speed_kph is not None and moving:
                err = self.ratio_error_pct(lever, rpm, speed_kph)
            if lever != 0 and not clutch_in:
                if rpm is None or speed_kph is None:
                    candidate = lever  # nothing to cross-check against
                elif moving and running and (err is None or abs(err) <= self.tol_pct):
                    candidate = lever
        elif rpm is not None and speed_kph is not None:
            source = "ratio"
            if moving and running:
                g, err = self.nearest_gear(rpm, speed_kph)
                if not clutch_in:
                    candidate = g
        else:
            source = "none"

        # -- lock state machine ------------------------------------------------------------
        was_locked = self._locked
        if candidate is None:
            self._run_gear = None
            self._run_n = 0
            self._run_errs.clear()
            self._set_unlocked(t)
        elif was_locked and candidate == self._gear:
            pass  # still in gear
        elif (
            candidate == self._gear
            and self._unlock_t is not None
            and t - self._unlock_t <= self.RELOCK_S
        ):
            self._set_locked(candidate, t)  # quick re-lock after a glitch / clutch dip
        else:
            if candidate != self._run_gear:
                self._run_gear = candidate
                self._run_t = t
                self._run_n = 0
                self._run_errs.clear()
                fresh = True  # the first observation of a run always counts
            if fresh or self._gear == 0:  # the session's first lock is not a shift
                self._run_n += 1
                if err is not None:
                    self._run_errs.append((t, err))
            confirmed = self._run_n >= self.confirm_ticks
            since = self._run_t if self._run_t is not None else t
            if confirmed and len(self._run_errs) >= 2 and speed_kph is not None:
                errs = [e for _, e in self._run_errs]
                confirmed = max(errs) - min(errs) <= self._stable_tol(speed_kph)
                # the lock dates from the start of the *stable* stretch, not from when a
                # still-moving rpm first entered the window
                since = max(since, self._run_errs[0][0])
            if confirmed:
                self._set_locked(candidate, since)
            else:
                self._set_unlocked(t)

        # -- estimate for display / consumers ------------------------------------------------
        locked = self._locked
        if locked:
            if source == "ratio" and err is not None:
                conf = clamp(1.0 - abs(err) / self.tol_pct, 0.0, 1.0)
            else:
                conf = 1.0
            gear = self._gear
        else:
            conf = 0.0
            gear = 0
            mismatch_only = not clutch_in and moving and running
            if (
                source == "ratio"
                and mismatch_only
                and self._gear != 0
                and self._unlock_t is not None
                and t - self._unlock_t <= self.HOLD_S
            ):
                gear = self._gear
        if source == "can":
            gear = lever if lever is not None else 0
            conf = 1.0 if gear != 0 else 0.0
        self.estimate = GearEstimate(
            gear=gear,
            conf=conf,
            source=source,
            locked=locked,
            ratio_err_pct=err,
            match_t=self._lock_t if locked else None,
        )
        return self.estimate

    def _set_locked(self, gear: int, since: float) -> None:
        if not self._locked or gear != self._gear:
            self._lock_t = since
        self._locked = True
        self._gear = gear
        self._unlock_t = None
        self._run_gear = None
        self._run_n = 0
        self._run_errs.clear()
        self.last_engaged_gear = gear

    def _set_unlocked(self, t: float) -> None:
        if self._locked:
            self._unlock_t = t
        self._locked = False
