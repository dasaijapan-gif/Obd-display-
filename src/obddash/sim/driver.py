"""Scripted drivers for the simulator.

A :class:`Driver` turns time + the current :class:`VehicleState` into
:class:`Controls` each step.  Scripts are selected by name with
:func:`make_driver`:

* ``"lap"`` (default) - loops forever: launch, full-throttle run up through the
  gears to ~165 km/h, a cruise in 5th, a heavy braking zone with heel-toe
  downshifts 5-4-3-2, a 0.9 g right-hander entered under trail braking, a
  throttle-on exit, a 0.6 g left, a short straight, a braked chicane entry with
  one more downshift, the chicane, a long sweeper, repeat.
* ``"dyno"`` - idle, climb to 3rd, hold ~2500 rpm, WOT pull to ~7300 rpm, lift,
  slow down, repeat (~30 s cycle).  Exercises the virtual dyno.
* ``"idle"`` - parked: idling with occasional blips, brake presses, steering wheel
  movement and clutch presses.  Handy for UI work.
* ``"highway"`` - steady 110 km/h in top gear (6th) with gentle lane changes and
  the odd mild brake.

The scripts are tuned for ``config/vehicles/sim_sportscar.yaml`` but key their gear
choices off ``VehicleConfig.gear_count()``, so a 4- or 5-speed car still laps.

Gear changes are choreographed by :class:`ShiftSequencer`, which overrides the
throttle / clutch / gear fields of the script's intent while a shift is in
progress:

* upshift: throttle off, clutch down (0.25 s), gear + 1, clutch up over 0.15 s;
* downshift (heel-toe): clutch down, blip the throttle toward
  ``target = speed_kph * rpm_per_kph(new_gear) * quality`` with a small
  closed-loop controller, then release the clutch over ~0.2 s while holding the
  steady-state throttle for that rpm.  ``quality ~ N(1.0, 0.04)``; ~15 % of
  downshifts are "lazy" (no blip, quicker clutch dump) and ~10 % over-rev by
  +12 %.  Total duration 0.35-0.9 s depending on how far the revs have to climb.

All randomness comes from a seeded ``random.Random`` so a given ``(script, seed)``
replays identically; steering and brake inputs are rate-limited so they look
like a human rather than a step function.
"""

from __future__ import annotations

import logging
import math
import random
from dataclasses import dataclass

from obddash.config import VehicleConfig
from obddash.sim.vehicle import Controls, VehicleState

log = logging.getLogger(__name__)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _toward(cur: float, target: float, max_delta: float) -> float:
    """Move ``cur`` toward ``target`` by at most ``max_delta``."""
    if target > cur:
        return min(target, cur + max_delta)
    return max(target, cur - max_delta)


def blip_throttle(target_rpm: float, rpm: float) -> float:
    """Throttle (%) to drive a free engine toward ``target_rpm``: floor it while far
    below, then settle on an estimate of the steady-state throttle plus a P term."""
    err = target_rpm - rpm
    if err > 250.0:
        return 100.0
    return _clamp(steady_throttle(target_rpm) + 0.1 * err, 0.0, 100.0)


def steady_throttle(rpm: float) -> float:
    """Rough throttle (%) that holds ``rpm`` in neutral on the sim engine (where WOT
    torque balances engine braking).  Only a feed-forward guess; the P term fixes it."""
    return _clamp(4.0 + 0.004 * rpm, 0.0, 60.0)


# --------------------------------------------------------------------------------------
# Shift choreography
# --------------------------------------------------------------------------------------
@dataclass
class ShiftRecord:
    t: float
    from_gear: int
    to_gear: int
    kind: str  # "up" | "down"
    style: str  # "normal" | "lazy" | "overrev" | "up"
    duration_s: float
    target_rpm: float | None  # the (quality-scaled) rpm the driver aimed for


class ShiftSequencer:
    """Owns the gear lever.  Call :meth:`upshift` / :meth:`downshift` to start a shift,
    then :meth:`apply` each step to overlay it on the script's controls."""

    UP_CLUTCH_IN_S = 0.05
    UP_GEAR_AT_S = 0.12
    UP_HOLD_S = 0.25
    UP_CLUTCH_OUT_S = 0.15
    DN_CLUTCH_IN_S = 0.06
    DN_GEAR_AT_S = 0.10
    DN_MAX_S = 0.9

    def __init__(
        self,
        vehicle: VehicleConfig,
        rng: random.Random,
        *,
        lazy_p: float = 0.15,
        overrev_p: float = 0.10,
        quality_sd: float = 0.04,
        overrev_factor: float = 1.12,
    ) -> None:
        self.vehicle = vehicle
        self.rng = rng
        self.lazy_p = lazy_p
        self.overrev_p = overrev_p
        self.quality_sd = quality_sd
        self.overrev_factor = overrev_factor
        self.gear = 0
        self.history: list[ShiftRecord] = []
        self.last_t = -10.0  # end time of the last shift
        self._kind: str | None = None
        self._t0 = 0.0
        self._from = 0
        self._to = 0
        self._style = "normal"
        self._quality = 1.0
        self._min_s = 0.4
        self._release_s = 0.2
        self._release_t: float | None = None
        self._target = 0.0

    @property
    def active(self) -> bool:
        return self._kind is not None

    @property
    def n_up(self) -> int:
        return sum(1 for r in self.history if r.kind == "up")

    @property
    def n_down(self) -> int:
        return sum(1 for r in self.history if r.kind == "down")

    def set_gear(self, gear: int) -> None:
        """Select a gear directly (standing start with the clutch down)."""
        self.gear = gear

    def upshift(self, t: float) -> bool:
        if self.active or self.gear >= self.vehicle.gear_count():
            return False
        self._kind, self._t0 = "up", t
        self._from, self._to = self.gear, self.gear + 1
        self._style = "up"
        return True

    def downshift(self, t: float, state: VehicleState) -> bool:
        if self.active or self.gear <= 1:
            return False
        self._kind, self._t0 = "down", t
        self._from, self._to = self.gear, self.gear - 1
        roll = self.rng.random()
        if roll < self.lazy_p:
            self._style = "lazy"
        elif roll < self.lazy_p + self.overrev_p:
            self._style = "overrev"
        else:
            self._style = "normal"
        self._quality = self.rng.gauss(1.0, self.quality_sd)
        if self._style == "overrev":
            self._quality *= self.overrev_factor
        self._min_s = self.rng.uniform(0.35, 0.6)
        self._release_s = 0.12 if self._style == "lazy" else 0.2
        self._release_t = None
        self._target = self._target_rpm(state)
        return True

    def _target_rpm(self, state: VehicleState) -> float:
        rpk = self.vehicle.rpm_per_kph(self._to) or 0.0
        return state.speed_kph * rpk * self._quality

    def apply(self, t: float, state: VehicleState, ctrl: Controls) -> Controls:
        """Overlay the shift in progress on ``ctrl`` (mutated and returned)."""
        ctrl.gear = self.gear
        if self._kind == "up":
            self._apply_up(t, ctrl)
        elif self._kind == "down":
            self._apply_down(t, state, ctrl)
        return ctrl

    # -- upshift ---------------------------------------------------------------------------
    def _apply_up(self, t: float, ctrl: Controls) -> None:
        tau = t - self._t0
        plan_throttle = ctrl.throttle_pct
        if tau >= self.UP_GEAR_AT_S:
            self.gear = self._to
        ctrl.gear = self.gear
        if tau < self.UP_HOLD_S:
            ctrl.clutch = min(1.0, tau / self.UP_CLUTCH_IN_S)
            ctrl.throttle_pct = 0.0
        elif tau < self.UP_HOLD_S + self.UP_CLUTCH_OUT_S:
            ctrl.clutch = 1.0 - (tau - self.UP_HOLD_S) / self.UP_CLUTCH_OUT_S
            ctrl.throttle_pct = 0.5 * plan_throttle  # feeding the power back in
        else:
            ctrl.clutch = 0.0
            self._finish(t, tau)

    # -- downshift (heel-toe) --------------------------------------------------------------
    def _apply_down(self, t: float, state: VehicleState, ctrl: Controls) -> None:
        tau = t - self._t0
        if tau >= self.DN_GEAR_AT_S:
            self.gear = self._to
        ctrl.gear = self.gear
        # the match target follows the live speed (the car is braking hard)
        self._target = self._target_rpm(state)
        blipping = self._style != "lazy"

        if self._release_t is None:
            ctrl.clutch = min(1.0, tau / self.DN_CLUTCH_IN_S)
            if blipping and tau >= 0.05:
                ctrl.throttle_pct = blip_throttle(self._target, state.rpm)
            else:
                ctrl.throttle_pct = 0.0
            earliest = self._min_s - self._release_s
            latest = self.DN_MAX_S - self._release_s
            matched = (not blipping) or state.rpm >= self._target - 60.0
            if tau >= latest or (tau >= earliest and matched):
                self._release_t = tau
            return

        rel = tau - self._release_t
        if rel < self._release_s:
            ctrl.clutch = 1.0 - rel / self._release_s
            ctrl.throttle_pct = steady_throttle(self._target) if blipping else 0.0
        else:
            ctrl.clutch = 0.0
            self._finish(t, tau)

    def _finish(self, t: float, duration: float) -> None:
        self.history.append(
            ShiftRecord(
                t=t,
                from_gear=self._from,
                to_gear=self._to,
                kind=self._kind or "",
                style=self._style,
                duration_s=duration,
                target_rpm=self._target if self._kind == "down" else None,
            )
        )
        if len(self.history) > 200:
            del self.history[:-200]
        log.debug(
            "shift %s %d->%d (%s) in %.2fs", self._kind, self._from, self._to, self._style, duration
        )
        self.last_t = t
        self._kind = None


# --------------------------------------------------------------------------------------
# Driver base
# --------------------------------------------------------------------------------------
class Driver:
    """Base class.  Subclasses implement :meth:`plan`; :meth:`get` adds the shift overlay
    and human-like rate limits on steering and brake."""

    name = "base"
    steer_rate_dps = 250.0  # steering wheel deg/s
    brake_rise_bar_s = 250.0
    brake_fall_bar_s = 350.0

    def __init__(self, vehicle: VehicleConfig, seed: int = 1) -> None:
        self.vehicle = vehicle
        self.rng = random.Random(seed)
        self.shift = ShiftSequencer(vehicle, self.rng)
        self._steer = 0.0
        self._brake = 0.0
        self._last_t: float | None = None

    @property
    def gear(self) -> int:
        return self.shift.gear

    def plan(self, t: float, state: VehicleState) -> Controls:  # pragma: no cover - abstract
        raise NotImplementedError

    def get(self, t: float, state: VehicleState) -> Controls:
        dt = 0.01 if self._last_t is None else _clamp(t - self._last_t, 1e-3, 0.2)
        self._last_t = t
        try:
            ctrl = self.plan(t, state)
        except Exception:  # never let a script bug kill the sim loop
            log.exception("driver %s failed; coasting", self.name)
            ctrl = Controls(gear=self.shift.gear, clutch=1.0)
        ctrl = self.shift.apply(t, state, ctrl)
        self._steer = _toward(self._steer, ctrl.steering_deg, self.steer_rate_dps * dt)
        rate = self.brake_rise_bar_s if ctrl.brake_bar > self._brake else self.brake_fall_bar_s
        self._brake = _toward(self._brake, ctrl.brake_bar, rate * dt)
        ctrl.steering_deg = self._steer
        ctrl.brake_bar = self._brake
        ctrl.throttle_pct = _clamp(ctrl.throttle_pct, 0.0, 100.0)
        ctrl.clutch = _clamp(ctrl.clutch, 0.0, 1.0)
        return ctrl

    # -- shared helpers --------------------------------------------------------------------
    def _launch(self, lt: float, ctrl: Controls, throttle: float = 60.0) -> bool:
        """Standing start: clutch down, 1st, release over ~0.9 s.  Returns True while busy."""
        if lt < 0.3:
            self.shift.set_gear(1)
            ctrl.clutch = 1.0
            ctrl.throttle_pct = 0.0
        elif lt < 1.3:
            ctrl.clutch = max(0.0, 1.0 - (lt - 0.3) / 0.9)
            ctrl.throttle_pct = throttle
        else:
            return False
        ctrl.gear = self.shift.gear
        return True

    def _hold_speed(
        self, kph: float, target_kph: float, base: float = 30.0, gain: float = 6.0
    ) -> float:
        return _clamp(base + gain * (target_kph - kph), 0.0, 100.0)


# --------------------------------------------------------------------------------------
# "lap"
# --------------------------------------------------------------------------------------
class LapDriver(Driver):
    """A short circuit, repeated forever.  Phase durations depend on the physics (the
    phase ends when a speed is reached), so lap time is ~65 s on the Sim Sportscar."""

    name = "lap"
    PHASES = (
        "straight",
        "brake_zone",
        "corner_right",
        "exit",
        "corner_left",
        "short_straight",
        "chicane_brake",
        "chicane",
        "sweeper",
    )

    def __init__(self, vehicle: VehicleConfig, seed: int = 1) -> None:
        super().__init__(vehicle, seed)
        self.upshift_rpm = min(7000.0, vehicle.redline_rpm - 300.0)
        # 5th on the Sim Sportscar (so the brake zone can go 5-4-3-2); top gear on a
        # shorter box, otherwise the straight would never end on a 4-speed car
        self.cruise_gear = max(1, min(5, vehicle.gear_count()))
        self.straight_kph = 165.0
        self.cruise_s = 15.0
        self.brake_zone_bar = 70.0
        self.sweeper_s = 8.0
        self.lap_count = 0
        self.lap_times: list[float] = []
        self._phase_idx = 0
        self._t0: float | None = None
        self._lap_t0 = 0.0
        self._launch_t0: float | None = None
        self._cruise_t0: float | None = None

    @property
    def phase(self) -> str:
        return self.PHASES[self._phase_idx]

    def _enter(self, t: float, idx: int) -> None:
        self._phase_idx = idx % len(self.PHASES)
        self._t0 = t
        self._cruise_t0 = None
        if self._phase_idx == 0 and t > self._lap_t0:
            if self.lap_count > 0 or t - self._lap_t0 > 5.0:
                self.lap_times.append(t - self._lap_t0)
                self.lap_count += 1
            self._lap_t0 = t
        log.debug("lap phase -> %s at t=%.1f", self.phase, t)

    def plan(self, t: float, state: VehicleState) -> Controls:
        if self._t0 is None:
            self._t0 = t
            self._lap_t0 = t
        ctrl = Controls(gear=self.shift.gear)
        handler = getattr(self, "_p_" + self.phase)
        done = handler(t, t - self._t0, state, ctrl)
        if done and not self.shift.active:
            self._enter(t, self._phase_idx + 1)
        return ctrl

    # phase handlers: (t, time-in-phase, state, ctrl) -> done ------------------------------
    def _p_straight(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        kph = s.speed_kph
        g = self.shift.gear
        if g == 0 and kph < 1.0 and self._launch_t0 is None:
            self._launch_t0 = tp
        if self._launch_t0 is not None:
            if self._launch(tp - self._launch_t0, c):
                return False
            self._launch_t0 = None
        if kph < self.straight_kph:
            c.throttle_pct = 100.0
        else:
            c.throttle_pct = self._hold_speed(kph, self.straight_kph, base=38.0, gain=8.0)
        if not self.shift.active:
            if g < 3 and s.rpm >= self.upshift_rpm:
                self.shift.upshift(t)
            elif (
                3 <= g < self.cruise_gear
                and kph >= self.straight_kph - 4.0
                and t - self.shift.last_t > 1.5
            ):
                self.shift.upshift(t)  # short-shift up to the cruise gear (5th)
            elif g < 6 and s.rpm >= self.vehicle.redline_rpm - 150.0:
                self.shift.upshift(t)  # never sit on the limiter
        if g >= self.cruise_gear and not self.shift.active and self._cruise_t0 is None:
            self._cruise_t0 = tp
        return self._cruise_t0 is not None and tp - self._cruise_t0 >= self.cruise_s

    def _p_brake_zone(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        kph = s.speed_kph
        c.throttle_pct = 0.0
        c.brake_bar = self.brake_zone_bar if kph > 75.0 else 25.0
        thresholds = {6: 160.0, 5: 150.0, 4: 118.0, 3: 78.0}
        g = self.shift.gear
        if not self.shift.active and g in thresholds and kph < thresholds[g]:
            self.shift.downshift(t, s)
        return (kph <= 62.0 and not self.shift.active) or tp > 12.0

    def _p_corner_right(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        c.steering_deg = -65.0  # right-hander: negative
        c.brake_bar = 25.0 * max(0.0, 1.0 - tp / 1.2)  # trail off the brake
        c.throttle_pct = 0.0 if tp < 1.0 else _clamp(30.0 + 5.0 * (60.0 - s.speed_kph), 0.0, 60.0)
        return tp >= 3.8

    def _p_exit(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        c.steering_deg = -65.0 * max(0.0, 1.0 - tp / 1.2)
        c.throttle_pct = min(100.0, 40.0 + 40.0 * tp)
        if not self.shift.active and self.shift.gear < 3 and s.rpm >= self.upshift_rpm:
            self.shift.upshift(t)
        return (s.speed_kph >= 100.0 and not self.shift.active) or tp > 10.0

    def _p_corner_left(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        c.steering_deg = 17.0 if tp < 3.6 else 17.0 * max(0.0, 1.0 - (tp - 3.6) / 0.6)
        if tp < 0.6:
            c.brake_bar = 20.0
            c.throttle_pct = 0.0
        else:
            c.throttle_pct = _clamp(25.0 + 5.0 * (95.0 - s.speed_kph), 0.0, 70.0)
        return tp >= 4.2

    def _p_short_straight(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        c.throttle_pct = 100.0
        if not self.shift.active and self.shift.gear < 6 and s.rpm >= self.upshift_rpm:
            self.shift.upshift(t)
        return s.speed_kph >= 135.0 or tp > 6.0

    def _p_chicane_brake(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        c.throttle_pct = 0.0
        c.brake_bar = 55.0
        if not self.shift.active and self.shift.gear >= 3 and s.speed_kph < 88.0:
            self.shift.downshift(t, s)
        return (s.speed_kph <= 82.0 and not self.shift.active) or tp > 6.0

    def _p_chicane(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        if tp < 0.9:
            c.steering_deg = -30.0
        elif tp < 1.9:
            c.steering_deg = 32.0
        elif tp < 2.6:
            c.steering_deg = -22.0
        else:
            c.steering_deg = 0.0
        c.throttle_pct = 30.0 + 10.0 * tp
        return tp >= 3.0

    def _p_sweeper(self, t: float, tp: float, s: VehicleState, c: Controls) -> bool:
        ramp_in = min(1.0, tp / 0.6)
        ramp_out = _clamp((self.sweeper_s - tp) / 0.6, 0.0, 1.0)
        c.steering_deg = -11.0 * ramp_in * ramp_out
        kph = s.speed_kph
        c.throttle_pct = 80.0 if kph < 105.0 else self._hold_speed(kph, 105.0, base=30.0)
        if not self.shift.active and self.shift.gear < 3 and s.rpm >= self.upshift_rpm:
            self.shift.upshift(t)
        return tp >= self.sweeper_s


# --------------------------------------------------------------------------------------
# "dyno"
# --------------------------------------------------------------------------------------
class DynoDriver(Driver):
    """Repeated 3rd-gear pulls: idle 2 s, climb to 3rd at ~2500 rpm, hold 1 s, WOT to
    ``pull_to_rpm``, lift, brake back down, repeat (~30 s per cycle)."""

    name = "dyno"

    def __init__(self, vehicle: VehicleConfig, seed: int = 1) -> None:
        super().__init__(vehicle, seed)
        self.pull_gear = 3
        self.pull_from_rpm = 2500.0
        self.pull_to_rpm = min(7300.0, vehicle.redline_rpm - 200.0)
        self.climb_shift_rpm = 4000.0
        self.pulls = 0
        self._state = "idle"
        self._t0: float | None = None
        self._hold_t0: float | None = None

    def _go(self, state: str, t: float) -> None:
        self._state = state
        self._t0 = t
        self._hold_t0 = None
        log.debug("dyno -> %s at t=%.1f", state, t)

    def plan(self, t: float, state: VehicleState) -> Controls:
        if self._t0 is None:
            self._t0 = t
        tp = t - self._t0
        c = Controls(gear=self.shift.gear)
        rpm, kph = state.rpm, state.speed_kph
        st = self._state
        if st == "idle":
            if tp >= 2.0:
                self._go("launch", t)
        elif st == "launch":
            if not self._launch(tp, c, throttle=45.0):
                self._go("climb", t)
        elif st == "climb":
            c.throttle_pct = 50.0
            if self.shift.gear >= self.pull_gear:
                self._go("settle", t)
            elif not self.shift.active and rpm >= self.climb_shift_rpm:
                self.shift.upshift(t)
        elif st == "settle":
            c.throttle_pct = _clamp(20.0 + 0.05 * (self.pull_from_rpm - rpm), 0.0, 60.0)
            if abs(rpm - self.pull_from_rpm) < 120.0:
                if self._hold_t0 is None:
                    self._hold_t0 = tp
                elif tp - self._hold_t0 >= 1.0:
                    self._go("pull", t)
            else:
                self._hold_t0 = None
        elif st == "pull":
            c.throttle_pct = 100.0
            if rpm >= self.pull_to_rpm or tp > 25.0:
                self.pulls += 1
                self._go("lift", t)
        elif st == "lift":
            c.throttle_pct = 0.0
            if tp >= 0.8:
                self._go("slow", t)
        elif st == "slow":
            c.throttle_pct = 0.0
            c.brake_bar = 22.0
            if rpm <= self.pull_from_rpm + 100.0 or kph < 20.0:
                self._go("settle", t)
        return c


# --------------------------------------------------------------------------------------
# "idle"
# --------------------------------------------------------------------------------------
class IdleDriver(Driver):
    """Parked, engine running: blips, brake presses, steering and clutch movement."""

    name = "idle"
    steer_rate_dps = 120.0

    def __init__(self, vehicle: VehicleConfig, seed: int = 1) -> None:
        super().__init__(vehicle, seed)
        r = self.rng
        self._next_blip = r.uniform(2.0, 5.0)
        self._blip_until = -1.0
        self._blip_target = 3000.0
        self._next_brake = r.uniform(3.0, 8.0)
        self._brake_until = -1.0
        self._brake_bar = 20.0
        self._next_steer = r.uniform(1.0, 4.0)
        self._steer_target = 0.0
        self._next_clutch = r.uniform(6.0, 12.0)
        self._clutch_until = -1.0

    def plan(self, t: float, state: VehicleState) -> Controls:
        r = self.rng
        c = Controls(gear=0)
        if t >= self._next_blip:
            self._blip_target = r.uniform(2000.0, 4500.0)
            self._blip_until = t + r.uniform(0.5, 0.9)
            self._next_blip = self._blip_until + r.uniform(3.0, 8.0)
        if t < self._blip_until:
            c.throttle_pct = blip_throttle(self._blip_target, state.rpm)
        if t >= self._next_brake:
            self._brake_bar = r.uniform(10.0, 50.0)
            self._brake_until = t + r.uniform(1.0, 2.5)
            self._next_brake = self._brake_until + r.uniform(4.0, 10.0)
        if t < self._brake_until:
            c.brake_bar = self._brake_bar
        if t >= self._next_steer:
            self._steer_target = r.uniform(-120.0, 120.0) if r.random() < 0.7 else 0.0
            self._next_steer = t + r.uniform(2.0, 6.0)
        c.steering_deg = self._steer_target
        if t >= self._next_clutch:
            self._clutch_until = t + r.uniform(1.0, 2.0)
            self._next_clutch = self._clutch_until + r.uniform(8.0, 15.0)
        if t < self._clutch_until:
            c.clutch = 1.0
        return c


# --------------------------------------------------------------------------------------
# "highway"
# --------------------------------------------------------------------------------------
class HighwayDriver(Driver):
    """Accelerate to 110 km/h in top gear (6th), then cruise with gentle lane changes and the odd
    mild brake application."""

    name = "highway"

    def __init__(self, vehicle: VehicleConfig, seed: int = 1) -> None:
        super().__init__(vehicle, seed)
        self.cruise_kph = 110.0
        self.climb_shift_rpm = 3800.0
        self._state = "accel"
        self._t0: float | None = None
        self._lane_t0: float | None = None
        self._lane_dir = 1.0
        self._next_lane = self.rng.uniform(8.0, 15.0)
        self._brake_until = -1.0
        self._next_brake = self.rng.uniform(20.0, 35.0)

    def plan(self, t: float, state: VehicleState) -> Controls:
        if self._t0 is None:
            self._t0 = t
        tp = t - self._t0
        c = Controls(gear=self.shift.gear)
        kph, rpm = state.speed_kph, state.rpm
        g = self.shift.gear
        top = self.vehicle.gear_count()  # 6th on the Sim Sportscar
        if self._state == "accel":
            if g == 0 and kph < 1.0:
                self._launch(tp, c, throttle=45.0)
                return c
            if self._launch(tp, c, throttle=45.0):
                return c
            c.throttle_pct = 55.0
            if not self.shift.active and g < top:
                if rpm >= self.climb_shift_rpm or (kph >= self.cruise_kph - 2.0 and g >= 4):
                    self.shift.upshift(t)
            if g == top and not self.shift.active and kph >= self.cruise_kph - 5.0:
                self._state = "cruise"
                self._next_lane += t
                self._next_brake += t
            return c

        # cruise
        c.throttle_pct = _clamp(24.0 + 4.0 * (self.cruise_kph - kph), 0.0, 70.0)
        if t >= self._next_brake:
            self._brake_until = t + 1.5
            self._next_brake = self._brake_until + self.rng.uniform(20.0, 40.0)
        if t < self._brake_until:
            c.brake_bar = 15.0
            c.throttle_pct = 0.0
        if self._lane_t0 is None and t >= self._next_lane:
            self._lane_t0 = t
            self._lane_dir = 1.0 if self.rng.random() < 0.5 else -1.0
        if self._lane_t0 is not None:
            lt = t - self._lane_t0
            if lt < 3.0:
                # one full sine: steer one way then back = a lane change
                c.steering_deg = self._lane_dir * 5.0 * math.sin(2.0 * math.pi * lt / 3.0)
            else:
                self._lane_t0 = None
                self._next_lane = t + self.rng.uniform(12.0, 25.0)
        if not self.shift.active and g == top and kph < 85.0:
            self.shift.downshift(t, state)
        elif not self.shift.active and g == top - 1 and kph > 100.0:
            self.shift.upshift(t)
        return c


SCRIPTS: dict[str, type[Driver]] = {
    "lap": LapDriver,
    "dyno": DynoDriver,
    "idle": IdleDriver,
    "highway": HighwayDriver,
}


def make_driver(script: str, vehicle: VehicleConfig, seed: int = 1) -> Driver:
    """Instantiate a driver by script name (``lap`` | ``dyno`` | ``idle`` | ``highway``)."""
    try:
        cls = SCRIPTS[script.lower()]
    except KeyError:
        raise ValueError(f"unknown sim script {script!r}; choose from {sorted(SCRIPTS)}") from None
    return cls(vehicle, seed)
