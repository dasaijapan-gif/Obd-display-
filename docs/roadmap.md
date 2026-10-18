# Roadmap

Where obddash could go next, roughly in order of value per effort. Each item says
why it is worth doing and where it fits in the existing code, so anyone can pick one
up. The architecture makes most of these additive: a new data source writes
`RawState` names, a new metric adds an analyzer to the engine and a field to
`Telemetry`, and a new view is a page in `src/obddash/web/js/pages/`
([architecture.md](architecture.md)).

## Next: small, high value

**GPS lap timing.** A 10-25 Hz GNSS receiver (u-blox M8/M9/M10 class, USB or UART) via
`gpsd`, a `sensors/gps.py` source writing latitude, longitude, GPS speed and heading,
and a `metrics/laps.py` analyzer: start/finish line crossing (interpolated between
fixes), sectors, best lap, and a live predictive delta against the best lap. A new
LAPS page. GPS speed is also a *true* speed for the dyno and for calibrating
`tire_circumference_m` automatically.

**Data logging and export.** Recording already exists (`--record`, raw JSONL that
replays through the metrics). Missing: export to CSV and to the formats lap-analysis
tools import (MoTeC i2 `.ld`, RaceChrono / RaceRender CSV), a list of recordings in the
UI with download links (`GET /api/recordings`), and a per-session summary (shift grades,
best dyno run, brake score) written next to each recording.

**`--dbc` on the command line.** `obddash.bus.signals.load_dbc()` already turns a
`.dbc` file plus a name map into signal definitions (with the optional `cantools`
package), but the CLI has no flag for it yet, although `config/vehicles/README.md`
mentions one. Add `obddash run --dbc FILE --map dbc_name=sig_name ...` and a
`dbc:` section in the vehicle file.

**More IMU drivers.** The MPU-6050 is discontinued and its clones vary. Add
ICM-20948, ICM-42688 and LSM6DSOX drivers behind the same `Imu` interface
(`imu.driver`), and use the car's own accelerometer when the vehicle file maps one.

**Per-gear inertia and a coast-down test.** Replace the single
`rotational_inertia_factor` with an optional per-gear list (defaulting to the
textbook estimate in [metrics.md](metrics.md#3-virtual-dyno)). Add a "coast-down" dyno
mode: in neutral from 110 to 30 km/h the car's deceleration is pure drag and rolling
resistance, which calibrates `cd × frontal_area` and `crr` for *your* car, and with
them every dyno number.

**Dyno resolution.** In a locked gear the engine rpm is a far finer speed sensor than
the 1 km/h OBD speed PID. Deriving speed from rpm while the pull is in a known gear
(cross-checked against wheel speed for slip) would make curves much less noisy on cars
without wheel-speed signals.

## Then: bigger features

**Per-corner analysis.** Corners are already detected for the handling score. With
GPS position, each corner of a track gets an identity, and per-corner numbers become
possible: braking point, entry / minimum / exit speed, time to throttle pickup, trail-
braking %, and a comparison against your best lap through the same corner.

**Signal-mapping wizard and configuration from the UI.** Edit the app and vehicle
config from a CONFIG page (validated by the same pydantic models), upload a DBC, and
turn `obddash scan --diff` candidates into vehicle-file entries: "this byte moved when
you pressed the brake: call it `brake_pressure_bar`?". Changes over the air from a
phone instead of SSH.

**ELM327 data source.** A fallback for people without a CAN interface: poll PIDs
through the ELM327 AT protocol and write the same `RawState` names. Clearly marked as
slow in the UI, with rev-match scoring disabled below a minimum sample rate
([alternatives.md](../hardware/alternatives.md#elm327-bluetooth-the-slow-fallback)).

**CAN FD.** The MCP2518FD HATs are FD-capable and newer cars are starting to use CAN
FD internally. Needs `fd` / `data_bitrate` in `BusConfig`, python-can's FD mode, FD
frames in `obddash scan`, and `FDMode=yes` / `DataBitRate=` in the `.network` file.
OBD-II itself stays classic CAN on most cars.

**Second CAN channel.** The Waveshare 2-CH HAT can listen to a second bus (e.g. a
chassis bus tapped behind a gateway) at the same time: `bus` becomes a list, and each
signal names its channel.

**Automatic gearboxes.** Read the gear from the transmission (OBD PID 0xA4 on cars that
support it, or a TCU broadcast), show converter lock-up, and score shift *quality*
(jolt) instead of rev-matching.

## Hardware

**3D-printed housing.** An enclosure for Pi 5 + HAT + cooler with cable strain relief,
and mounts for the 7.9" bar display (gauge pod, vent clip, A-pillar), published as STL
plus editable FreeCAD sources under `hardware/`.

**Ignition-aware power.** Read an ignition-sense GPIO from a shutdown controller, flush
the recorder and shut down cleanly, and dim the display from a light sensor or the
car's headlight signal.

**LED shift light.** A WS2812 strip on a GPIO pin driven from `shift.shift_light` and
the live target rpm, readable in peripheral vision.

**Round companion gauge.** An ESP32 with a round LCD subscribing to `/ws` over Wi-Fi,
showing one thing well (gear and shift light, or the last grade)
([alternatives.md](../hardware/alternatives.md#esp32--can-transceiver--round-lcd)).

## Quality of life

* A ready-to-flash SD-card image (pi-gen) with obddash, the services, the overlay file
  system and the kiosk already set up.
* Hardware-in-the-loop tests: replay recordings onto a real bus with a second adapter
  (`canplayer`) and assert on the telemetry.
* Night mode and a brightness control in the UI; metric/imperial units.
* Units and thresholds per user on the SYSTEM page, remembered in the browser.
