# obddash architecture

```
                 ┌─────────────────────────────── Raspberry Pi ───────────────────────────────┐
  OBD-II port    │  CAN HAT   ┌──────────────┐          ┌───────────────┐      ┌───────────┐ │
  ─ CAN-H/L ─────┼─ can0 ───▶ │  bus.reader  │ RawState │ metrics.engine│ Telem│  server   │ │   Chromium
                 │            │ obd + signals│ ───────▶ │ gears/shifts/ │ ────▶│ FastAPI   │─┼─▶ kiosk
  MPU6050 (I2C) ─┼──────────▶ │ sensors.imu  │          │ dyno/brakes/  │      │ WebSocket │ │   web/
                 │            └──────────────┘          │ handling      │      └───────────┘ │
                 │  `obddash sim` replaces can0 with a   └───────────────┘  recorder.jsonl    │
                 │  virtual bus + sim.ecu (fake car)                                          │
                 └────────────────────────────────────────────────────────────────────────────┘
```

Three stages, two contracts. Everything upstream of the metrics engine writes raw
signals into a `RawState` under the canonical names in `obddash.model.Sig`. The
metrics engine turns that into a `Telemetry` snapshot. The server serialises it
as JSON over a WebSocket and the browser draws it.

## Packages and responsibilities

| package | owns | must not |
|---|---|---|
| `obddash.model` | `Sig`, `RawState`, `Telemetry` (+ nested dataclasses), `Event` | import anything else from obddash |
| `obddash.config` | pydantic config models, YAML loaders | — |
| `obddash.bus.obd` | OBD-II over CAN: request builder, ISO-TP (single + multi-frame) assembly, PID decode table (mode 01, 09 VIN), supported-PID discovery | touch `RawState` |
| `obddash.bus.signals` | DBC-style bit extraction for `SignalConfig` lists; optional `.dbc` via cantools | touch `RawState` |
| `obddash.bus.reader` | `CanReader`: opens the python-can bus, runs the PID scheduler, feeds decoded values into `RawState`, tracks stats (pid_hz, frame_hz, ecu alive) | compute metrics |
| `obddash.bus.scan` | `obddash scan` tooling: live id table, change highlighting, candidate finder | — |
| `obddash.sensors.imu` | MPU6050 over I2C → `acc_lat_g/lon/vert` into `RawState`; axis mapping, low-pass, auto-level; a `FakeImu` for tests | — |
| `obddash.metrics.*` | `MetricsEngine.tick(raw, t) -> Telemetry`; sub-modules gears / shifts / dyno / brakes / handling / session | do I/O |
| `obddash.sim.*` | vehicle physics + scripted driver + fake ECU that answers OBD requests and broadcasts the `sim_sportscar.yaml` signals on a python-can bus | import metrics |
| `obddash.server.*` | `Runtime` (owns reader/imu/engine/broadcast loop), FastAPI app, WebSocket, REST, recorder/replayer | compute metrics |
| `obddash.web` | static schematic UI (vanilla JS + Canvas/SVG, no build step) | need the network beyond the Pi |
| `obddash.cli` | `obddash run | sim | replay | scan | pids` | — |

## Contract 1: raw signals (`RawState`)

`RawState.set(name, value, t=None)` / `.update({name: value})` from any thread;
`.get(name, default, max_age)` / `.snapshot()` from the engine. Names are the
constants on `model.Sig`; units are in the names. Sign conventions are in the
`model` docstring (SAE: +lon = accelerating, +lat = left turn, +steering = left).

## Contract 2: telemetry JSON (WebSocket `/ws`)

Server → client, at `server.ws_rate_hz` (default 30 Hz):

```jsonc
{"type": "telemetry", "data": {
  "t": 123.45, "ts": 1760000000.0,
  "conn": {"bus": true, "ecu": true, "obd": true, "imu": false, "source": "sim", "pid_hz": 48.0, "frame_hz": 210.0},
  "veh": {"speed_kph": 87.3, "rpm": 4210, "gear": 3, "gear_conf": 0.97, "gear_source": "ratio",
          "throttle_pct": 64.0, "load_pct": 71.0, "coolant_c": 92, "intake_c": 31, "oil_temp_c": null,
          "maf_gps": 41.2, "map_kpa": 88, "boost_kpa": -13, "baro_kpa": 101, "fuel_pct": 48,
          "fuel_rate_lph": null, "batt_v": 14.1, "timing_adv_deg": 22.5, "lambda": 1.0,
          "steering_deg": -12.0, "yaw_dps": -4.1, "brake_switch": false, "clutch_switch": false,
          "ambient_c": 21},
  "g": {"lat": -0.31, "lon": 0.22, "vert": 1.0, "total": 0.38, "peak_lat": 0.92, "peak_accel": 0.48,
        "peak_brake": 1.05, "source": "imu", "trail": [[-0.30, 0.20], [-0.31, 0.22]]},
  "brake": {"pressure_bar": 0.0, "pressure_pct": 0.0, "front_bar": null, "rear_bar": null,
            "active": false, "peak_bar": 74.2, "source": "can", "trail_pct": 0.0,
            "events": [{"t_start": 98.1, "duration_s": 2.4, "peak_bar": 74.2, "entry_kph": 142,
                        "exit_kph": 61, "avg_decel_g": 0.86, "trail_pct": 32.0, "smoothness": 81}],
            "trace": [[120.0, 0.0], [120.02, 0.0]], "score": 78, "count": 6},
  "shift": {"state": "idle", "target_rpm": 5120, "target_gear": 2, "shift_light": false,
            "last": {"t": 99.3, "from_gear": 4, "to_gear": 3, "kind": "down", "duration_s": 0.42,
                     "target_rpm": 4480, "actual_rpm": 4310, "error_pct": -3.8, "blip": true,
                     "heel_toe": true, "jolt_g": 0.08, "score": 86, "grade": "A",
                     "rpm_trace": [[98.8, 3100], [98.82, 3150]], "throttle_trace": [], "brake_trace": []},
            "session": {"count": 14, "downshifts": 7, "upshifts": 7, "avg_score": 79.4,
                        "best_score": 94, "heel_toe_count": 5, "history": [72, 81, 94, 86]}},
  "dyno": {"state": "idle", "message": "Arm, then hold WOT in 3rd", "gear": 0, "live_rpm": 0,
           "live_power_kw": 0, "live_torque_nm": 0, "progress": 0,
           "curve": [{"rpm": 3000, "power_kw": 61.2, "torque_nm": 195.0, "ecu_torque_nm": 199.0}],
           "peak_power_kw": 128.4, "peak_power_rpm": 6900, "peak_torque_nm": 201.0, "peak_torque_rpm": 4300,
           "runs": [], "wheel_hp": 172.2},
  "handling": {"score": 74, "smoothness": 81, "balance": -0.12, "friction_used_pct": 41,
               "max_lat_g": 0.92, "corner_count": 9},
  "session": {"duration_s": 123.4, "distance_km": 2.1, "max_speed_kph": 161, "max_rpm": 7420, "fuel_used_l": null},
  "alerts": []
}}
```

Discrete events (same socket): `{"type": "event", "name": "shift.scored" | "dyno.started" |
"dyno.done" | "brake.event" | "alert", "data": {...}, "t": 123.4}`.

Client → server commands: `{"cmd": "dyno.arm"}`, `{"cmd": "dyno.cancel"}`,
`{"cmd": "dyno.clear"}`, `{"cmd": "session.reset"}`, `{"cmd": "brake.reset_peak"}`,
`{"cmd": "shift.reset"}`. The server answers `{"type": "ack", "cmd": "...", "ok": true}`.

REST: `GET /api/state` (latest telemetry), `GET /api/config` (app + vehicle config, ids
as hex strings), `GET /api/dyno/runs`, `GET /api/health`, `POST /api/cmd` (same payload as
the WS command).

## Python interfaces

```python
# bus
class CanReader:
    def __init__(self, bus_cfg: BusConfig, vehicle: VehicleConfig, raw: RawState, *, bus: can.BusABC | None = None): ...
    def start(self) -> None   # background thread(s); never raises on a missing interface, sets stats.bus=False
    def stop(self) -> None
    stats: ReaderStats          # .bus_ok, .ecu_ok, .pid_hz, .frame_hz, .frames, .pid_responses, .supported_pids

# sensors
class Imu:  # and FakeImu(raw, lat=..., lon=...)
    def __init__(self, cfg: ImuConfig, raw: RawState): ...
    def start(self) -> bool; def stop(self) -> None; available: bool

# metrics
class MetricsEngine:
    def __init__(self, cfg: MetricsConfig, vehicle: VehicleConfig): ...
    def tick(self, raw: RawState, t: float) -> Telemetry   # t = monotonic seconds; call at cfg.tick_hz
    def command(self, cmd: str, args: dict | None = None) -> bool
    def drain_events(self) -> list[Event]
    def set_connection(self, conn: Connection) -> None     # runtime passes reader/imu stats

# sim
class Simulator:
    def __init__(self, vehicle: VehicleConfig, bus: can.BusABC, *, script: str = "lap", seed: int = 1, realtime: bool = True): ...
    def start(self); def stop(self); def step(dt: float)    # step() exposed for deterministic tests
    state: VehicleState  # speed_mps, rpm, gear, throttle, brake_bar, clutch, steering_deg, lat_g, lon_g, ...

# server
class Runtime:  # wires everything; used by cli
    def __init__(self, cfg: AppConfig, *, source: str, bus=None, replay_path=None): ...
    async def run(self)            # engine tick loop + broadcast loop
    latest: Telemetry; engine: MetricsEngine; raw: RawState
def create_app(runtime: Runtime) -> FastAPI
```

## Threads & timing

* python-can `Notifier` thread delivers frames → decoders → `RawState.set` (fast, lock only).
* PID scheduler thread sends requests; `fast` PIDs round-robin as quickly as responses
  arrive (bounded by `obd_max_rate_hz`), `medium` every ~250 ms, `slow` every ~2 s.
* IMU thread samples at `imu.sample_hz`.
* asyncio loop (uvicorn): engine tick at `metrics.tick_hz` (50 Hz), broadcast at
  `server.ws_rate_hz` (30 Hz) sending the latest snapshot; commands from clients.

## Metric definitions (summary — details in each module's docstring)

* **Gear**: from `Sig.GEAR` if present, else nearest configured `rpm_per_kph` ratio
  within ±6 %; confidence from the distance; 0 when clutch is in / speed < 5 km/h.
* **Downshift & rev-match score**: state machine idle → disengaged (ratio unlocks or
  clutch switch) → engaged (ratio locks to a new, lower gear). `target_rpm = speed ×
  rpm_per_kph(new_gear)` at engagement; score from |error| (full marks ≤ 2 %, zero ≥
  15 %, under-rev weighted ×1.5), timing bonus/penalty, blip detection, jolt penalty if
  IMU. Grades S ≥ 95, A ≥ 85, B ≥ 70, C ≥ 50, D below.
* **Virtual dyno**: during a WOT pull in one gear, `a = d(v)/dt` on low-passed speed,
  `F = m·k·a + ½ρ·Cd·A·v² + Crr·m·g`, `P_wheel = F·v`, `T_engine ≈ P_wheel / ω_engine`
  (drivetrain efficiency applied for the crank estimate), binned by rpm, smoothed,
  SAE-corrected from baro/intake temp when available; ECU torque (PID 0x62×0x63)
  overlaid when present.
* **Brake**: pressure from CAN, else estimate `bar_per_g × |decel|` while brake switch is
  on (and flagged `source: estimate`). Events from threshold crossing; trail-braking %
  = share of samples with |lat g| > 0.25 while braking; smoothness from pressure jerk.
* **Handling**: lateral g from IMU, else bicycle model `v²·δ_road/(L)` from steering;
  friction circle with adaptive envelope; balance from measured yaw rate vs Ackermann
  yaw rate; smoothness from lateral jerk; corner counting.
