"""obddash.sim - a fake car on a CAN bus so the whole stack runs without hardware.

::

    bus_ecu = can.Bus(interface="virtual", channel="obddash-sim")   # the ECU side
    bus_app = can.Bus(interface="virtual", channel="obddash-sim")   # the reader side
    sim = Simulator(vehicle, bus_ecu, script="lap", seed=1)
    sim.start()            # physics thread + fake ECU (OBD responder + broadcaster)
    ...
    sim.stop()

:class:`Simulator` owns a :class:`VehicleModel` (physics), a scripted
:class:`Driver` and a :class:`FakeEcu`.  With ``realtime=True`` a thread steps the
model at ``rate_hz`` in wall-clock time (``sim.speed`` is a time multiplier, at most
50x: what one tick cannot step is dropped, never carried as a backlog); with
``realtime=False`` nothing advances until the owner calls :meth:`Simulator.step`,
which is how the deterministic tests drive it.  The bus is shared with the reader
through python-can's virtual interface, so the ECU must receive and send on the
very bus object it is given; ``receive_own_messages`` stays False.
"""

from __future__ import annotations

import logging
import threading
import time

from obddash.config import VehicleConfig
from obddash.sim.driver import SCRIPTS, Driver, ShiftSequencer, make_driver
from obddash.sim.ecu import FakeEcu, encode_pid, pack_be
from obddash.sim.vehicle import Controls, SimParams, VehicleModel, VehicleState

__all__ = [
    "SCRIPTS",
    "Controls",
    "Driver",
    "FakeEcu",
    "ShiftSequencer",
    "SimParams",
    "Simulator",
    "VehicleModel",
    "VehicleState",
    "encode_pid",
    "make_driver",
    "pack_be",
]

log = logging.getLogger(__name__)

_MAX_STEPS_PER_TICK = 50  # realtime loop: physics steps per tick, i.e. the top `speed`


class Simulator:
    """Physics + driver + fake ECU.  ``bus`` may be None for a pure-model simulation
    (no CAN traffic), e.g. in driver tests."""

    def __init__(
        self,
        vehicle: VehicleConfig,
        bus=None,
        *,
        script: str = "lap",
        seed: int = 1,
        realtime: bool = True,
        rate_hz: float = 100.0,
        params: SimParams | None = None,
        latency_s: float = 0.004,
    ) -> None:
        self.vehicle = vehicle
        self.bus = bus
        self.script = script
        self.seed = seed
        self.realtime = realtime
        self.rate_hz = max(1.0, float(rate_hz))
        self.speed = 1.0  # time multiplier for the realtime thread (effective max 50x)
        self.model = VehicleModel(vehicle, params, seed=seed)
        self.driver: Driver = make_driver(script, vehicle, seed=seed)
        self.ecu: FakeEcu | None = None
        if bus is not None:
            self.ecu = FakeEcu(vehicle, bus, lambda: self.model.state, latency_s=latency_s)
        self.steps = 0
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    # -- state ---------------------------------------------------------------------------------
    @property
    def state(self) -> VehicleState:
        return self.model.state

    @property
    def t(self) -> float:
        return self.model.state.t

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # -- stepping ------------------------------------------------------------------------------
    def step(self, dt: float) -> VehicleState:
        """Advance the car by ``dt`` seconds (driver then physics).  Deterministic."""
        s = self.model.state
        ctrl = self.driver.get(s.t, s)
        self.steps += 1
        return self.model.step(dt, ctrl)

    def run(self, duration_s: float, dt: float | None = None) -> VehicleState:
        """Step for ``duration_s`` of simulated time (tests / fast forward)."""
        dt = dt or 1.0 / self.rate_hz
        n = int(round(duration_s / dt))
        for _ in range(n):
            self.step(dt)
        return self.model.state

    # -- lifecycle -----------------------------------------------------------------------------
    def start(self) -> None:
        if self.ecu is not None:
            self.ecu.start()
        if self.realtime and self._thread is None:
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="sim-physics", daemon=True)
            self._thread.start()
        log.info(
            "simulator started: script=%s seed=%d realtime=%s",
            self.script,
            self.seed,
            self.realtime,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self.ecu is not None:
            self.ecu.stop()

    def _loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_t = time.monotonic()
        debt = 0.0  # simulated seconds owed (speed multiplier accumulates here)
        while not self._stop.is_set():
            next_t += period
            now = time.monotonic()
            if next_t < now - 0.5:  # suspended / overloaded: resync instead of bursting
                next_t = now
            delay = next_t - now
            if delay > 0:
                time.sleep(delay)
            debt += period * max(0.0, self.speed)
            n = 0
            try:
                while debt >= period and n < _MAX_STEPS_PER_TICK:
                    self.step(period)
                    debt -= period
                    n += 1
            except Exception:
                log.exception("simulator step failed; continuing")
                debt = 0.0
            # Whatever could not be paid this tick is dropped, not carried: otherwise a
            # speed above _MAX_STEPS_PER_TICK (or a slow host) piles up an unbounded
            # backlog and the sim keeps fast-forwarding long after `speed` is lowered.
            if debt > period:
                debt = period
