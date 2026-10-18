"""obddash.metrics - turns raw signals into the telemetry the UI draws.

:class:`MetricsEngine` (``engine.py``) reads a :class:`~obddash.model.RawState` once
per tick and returns a :class:`~obddash.model.Telemetry`.  The analyzers it drives
take plain floats (or ``None``) so each can be unit-tested on its own:

* ``gears``    - :class:`GearEstimator`: CAN gear or rpm/speed ratio, with hysteresis
* ``shifts``   - :class:`ShiftAnalyzer`: shift detection and downshift rev-match score
* ``dyno``     - :class:`VirtualDyno`: wheel power / torque from a WOT pull
* ``brakes``   - :class:`BrakeAnalyzer`: pressure, braking events, brake score
* ``handling`` - :class:`HandlingAnalyzer`: g-forces, friction circle, balance, corners
* ``session``  - :class:`SessionTracker`: duration, distance, maxima, fuel
* ``filters``  - low-pass, differentiator, bounded rings shared by the above

Every formula is documented in the docstring of the module that implements it.
"""

from .brakes import BrakeAnalyzer
from .dyno import VirtualDyno
from .engine import MetricsEngine
from .gears import GearEstimate, GearEstimator
from .handling import HandlingAnalyzer
from .session import SessionTracker
from .shifts import ShiftAnalyzer

__all__ = [
    "BrakeAnalyzer",
    "GearEstimate",
    "GearEstimator",
    "HandlingAnalyzer",
    "MetricsEngine",
    "SessionTracker",
    "ShiftAnalyzer",
    "VirtualDyno",
]
