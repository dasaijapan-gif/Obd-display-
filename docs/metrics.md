# How the metrics work

Plain-language explanations of every number obddash shows, the formula behind it,
its limits, and the config keys that tune it. This page follows the definitions in
[architecture.md](architecture.md#metric-definitions-summary--details-in-each-modules-docstring);
the exact implementation, constant by constant, is in the docstring at the top of each
module in [`src/obddash/metrics/`](../src/obddash/metrics/).

Config keys are written as `metrics.revmatch.perfect_pct` (app config,
`config/default.yaml`) or `vehicle.mass_kg` (the vehicle file named by
`vehicle_file`, e.g. `config/vehicles/generic_obd.yaml`). Defaults are in
[`src/obddash/config.py`](../src/obddash/config.py).

## The basics

The metrics engine runs **50 times a second** (`metrics.tick_hz`). Each tick it reads
the latest value of every raw signal: OBD-II PIDs, the car's own broadcast signals
listed in the vehicle file, and the IMU. A value older than `metrics.stale_s` (2 s;
1 s for speed and rpm) counts as **missing**, and every metric is written to cope with
missing inputs: it shows less rather than something wrong.

Which signals each page needs:

| Page | Needs | Better with |
|---|---|---|
| DRIVE | rpm, speed (standard PIDs) | throttle, temperatures, MAP + baro for boost |
| SHIFT | rpm, speed, correct gear ratios | throttle (blip detection), brake (heel-toe), clutch switch, IMU (jolt) |
| DYNO | rpm, speed, throttle, correct mass / aero data | wheel-speed signals (resolution), baro + intake temp (correction), PIDs 0x62/0x63 (ECU torque) |
| BRAKE | a brake-pressure signal **or** a brake switch | pressure on CAN (otherwise ESTIMATED), IMU lateral G (trail braking) |
| HANDLING | IMU, or a yaw-rate / steering signal | IMU **and** yaw rate + steering (balance) |

Signs follow SAE vehicle axes: x forward, y left, z up. Positive longitudinal G is
accelerating, positive lateral G is a left turn, positive steering is to the left.

## 1. Gear estimate

**What you see:** the big gear number, and how sure obddash is (`gear_conf`).

If the car broadcasts the gear lever position (`gear` in the vehicle's signals),
that is shown directly. Otherwise obddash works it out from the engine speed and road
speed. In a given gear, rpm and speed are locked together by the gearbox:

```
rpm_per_kph(g) = (1000 / 60) / tire_circumference_m × gear_ratio[g] × final_drive
```

(`vehicle.rpm_per_kph()`; `obddash config-check` prints this table for your car.)
Each tick, the observed `rpm / speed` is compared with every gear:

```
error(g) = (rpm / (speed_kph × rpm_per_kph(g)) − 1) × 100 %
```

The nearest gear wins if it is within **±6 %**; confidence is `1 − |error| / 6 %`.
The display shows **N** (0) when the clutch switch is pressed, below 5 km/h, or when no
gear is within 6 % (coasting in neutral, clutch slipping, engine free-revving during a
shift).

To stop flicker, a *new* gear must agree on three consecutive fresh rpm samples with a
stable ratio. A brief mismatch keeps showing the previous gear for 0.3 s.

**How to tune**

* `vehicle.gear_ratios` and `vehicle.final_drive`: from the manufacturer's spec sheet
  (or a gearbox code lookup).
* `vehicle.tire_circumference_m`: the one people get wrong. Calibrate it against
  your car's own speed signal: drive steadily in a high gear, read rpm and speed off
  the DRIVE page, and solve the formula above for the circumference. For example, 4th
  gear (1.000) with a 2.866 final drive at 2500 rpm and 99 km/h gives
  `16.667 × 1.000 × 2.866 × 99 / 2500 = 1.89 m`.
* `obddash config-check` warns when two gears are within 12 % of each other, where the
  ±6 % windows overlap and the estimate can flicker.

## 2. Downshifts and the rev-match score

**What you see:** the SHIFT page, a pop-up grade (S/A/B/C/D) after each downshift, and
session statistics.

A good downshift brings the engine to exactly the rpm the lower gear will need
*before* the clutch is released, with a throttle blip, so the car doesn't lurch.
obddash measures how close you got.

**The state machine.** In gear (ratio locked) → **disengaged** (the ratio stops
matching, or the clutch switch goes down) → **engaged** when the ratio locks again in a
*different*, lower gear. A shift back into the same gear (a clutch dip) is ignored, and
so are shifts below `metrics.revmatch.min_speed_kph` and "shifts" where the clutch was
down for more than 4 s (that's coasting).

**The target.** At the moment the clutch starts to bite:

```
target_rpm = speed_kph × rpm_per_kph(new_gear)
error_pct  = (actual_rpm − target_rpm) / target_rpm × 100       (+ = over-rev, − = under-rev)
```

Getting "the moment the clutch starts to bite" right matters: a fraction of a second
later the clutch has dragged the engine onto the gearbox speed and every shift would
look perfect. Without a clutch switch, obddash finds that moment from the rpm trace
(it walks back from the lock while the error is closing faster than a free-revving
engine can change), as described in
[`shifts.py`](../src/obddash/metrics/shifts.py).

**The score** (0-100):

```
e      = |error_pct| × 1.5 if under-revved, else |error_pct|            (under_rev_penalty)
match  = 100 if e ≤ 2 %,   0 if e ≥ 15 %,   linear in between        (perfect_pct, zero_pct)
timing = +5 if the shift took ≤ 0.5 s, −10 if ≥ 1.5 s, linear between (fast_shift_s, slow_shift_s)
blip   = −15 if no throttle blip was seen
jolt   = −min(15, 40 × (jolt_g − 0.15))  if the IMU felt a shunt above 0.15 g
score  = clamp(match + timing + blip + jolt, 0, 100)
```

| Grade | S | A | B | C | D |
|---|---|---|---|---|---|
| Score | ≥ 95 | ≥ 85 | ≥ 70 | ≥ 50 | < 50 |

Under-revving costs 1.5× because it is the worse mistake: the engine has to be
accelerated by the drivetrain, which shunts the car and can briefly lock a rear-drive
car's driven wheels.

A **blip** is throttle at or above `metrics.revmatch.blip_min_throttle_pct` (25 %)
while disengaged, clearly higher than just before the clutch went down. **Heel-toe**
is a blip while the brake is active. The **jolt** is the largest change in
longitudinal G from 0.05 s before the bite to 0.5 s after, measured only with an IMU
and cut short as soon as you move a pedal.

*Worked example* (the simulator's car, 4th → 3rd at 100 km/h): 3rd gear needs
`100 × 39.45 = 3945 rpm`. Releasing the clutch at 3800 rpm is −3.7 %, under-revved,
so `e = 5.5 %` and `match = 73`. A 0.42 s shift adds 5, a blip was seen, no jolt:
**78, grade B**. The same miss as an *over*-rev would score 87 + 5 = **92, grade A**.
Without a blip, taking 1.0 s and with a 0.25 g shunt: 73 − 2.5 − 15 − 4 = **51.5, C**.

Upshifts are counted, but not scored. With `vehicle.transmission: auto` nothing is
scored.

**Live hint.** While driving, the SHIFT page shows the rpm you would need for one gear
down at the current speed (`shift.target_rpm`), hidden when that would be above the
redline. The shift light comes on at `vehicle.shift_light_rpm`.

**How to tune:** `metrics.revmatch.perfect_pct` / `zero_pct` (how strict),
`under_rev_penalty`, `fast_shift_s` / `slow_shift_s`, `blip_min_throttle_pct`,
`min_speed_kph`, `history` (scores kept for the session graph). Correct gear ratios and
tyre circumference matter far more than any of these: a 3 % error in
`tire_circumference_m` is a 3 % error in every rev-match.

## 3. Virtual dyno

**What you see:** a power and torque curve from a full-throttle pull, peak power in
wheel hp, and up to eight saved runs to compare.

A chassis dyno measures the force at the wheels. obddash uses the car itself as the
dyno: if you know how heavy the car is and how fast it is accelerating, you know the
force pushing it, and force × speed is power.

**Doing a pull.** Press **Arm** on the DYNO page (or not: pulls in
`metrics.dyno.preferred_gears`, 3rd and 4th by default, are always captured). Then, on
a flat, straight, empty road where it is legal and safe, hold full throttle in one gear
from low rpm to near the redline. The pull starts after 0.4 s of throttle above
`min_throttle_pct` with rising rpm, and ends when you lift, change gear or the rpm stops
rising. It is rejected if it lasted under `min_duration_s` or covered less than
`min_rpm_span`.

**The physics**, on low-passed speed (`speed_smoothing_s`) and its slope:

```
a        = dv/dt                                          road acceleration [m/s²]
F_wheel  = m·k·a  +  ½·ρ·Cd·A·v²  +  Crr·m·g               [N]
           accelerate   air drag      rolling resistance
P_wheel  = F_wheel · v                                    [W]
T        = P_wheel / ω_engine,   ω_engine = rpm · 2π / 60  torque referred to the engine [Nm]
```

| Symbol | Key | Meaning |
|---|---|---|
| m | `vehicle.mass_kg` | car + driver + fuel |
| k | `vehicle.rotational_inertia_factor` | extra "effective mass" of spinning parts (see below) |
| ρ | `metrics.dyno.air_density_kgm3` | air density, 1.225 kg/m³ at sea level, 15 °C |
| Cd·A | `vehicle.cd`, `vehicle.frontal_area_m2` | drag coefficient × frontal area |
| Crr | `vehicle.crr` | rolling-resistance coefficient |

The curve is binned every `rpm_bin` rpm and smoothed over `smoothing_bins` bins on
either side. `power_kw` is **wheel** power, like a chassis dyno printout;
`wheel_hp = peak kW × 1.341`. The UI also shows a **crank estimate**, wheel power ÷
`vehicle.drivetrain_efficiency`. If the ECU reports torque (PIDs 0x62 × 0x63), its
figure is drawn as a second line: it is the ECU's own model, not a measurement, but a
useful sanity check (wheel torque should be about ECU torque × drivetrain efficiency).

*Worked example* (the simulator's 1130 kg car in 3rd at 100 km/h, accelerating at
1.75 m/s²): `1130 × 1.08 × 1.75 = 2136 N` to accelerate, `289 N` of drag, `122 N` of
rolling resistance; `2546 N × 27.8 m/s = 70.7 kW` ≈ 95 wheel hp at 3945 rpm, i.e.
171 Nm at the wheels referred to the engine, about 82 kW at the crank with 0.86
driveline efficiency.

**SAE correction.** Engines make less power in thin, hot air. To compare runs made on
different days, the result is corrected to standard conditions with the SAE J1349-style
factor (`metrics.dyno.correction: sae`, when barometric pressure and intake temperature
are known):

```
cf = 1.18 × (99 / baro_kPa) × √((intake_°C + 273) / 298) − 0.18,   limited to 0.90-1.10
```

For example, 95 kPa (≈ 500 m altitude) and 35 °C intake air gives cf = 1.070 (+7 %);
sea level (101.3 kPa) at 20 °C gives 0.963. Two approximations to know about: the formula uses the
total barometric pressure rather than dry-air pressure, and the *intake* air
temperature, which is usually warmer than ambient after a few pulls (so the correction
reads slightly generous). `correction: none` turns it off.

### Why the number is not the brochure number, and its limits

* **Wheel vs crank.** Brochures quote crank power. 10-20 % is lost in the gearbox,
  differential and tyres; obddash's main number is at the wheels.
* **Mass.** The acceleration and rolling terms, most of the force below motorway
  speeds, are proportional to `mass_kg`. 50 kg too light (a passenger, a full tank) reads
  about 4 % low.
* **Slope.** A 1 % uphill grade costs this car 111 N, 3.1 kW at 100 km/h, that the
  dyno can't see. Do pulls in both directions on the same stretch and average them.
* **Wind.** The drag term uses ground speed. A 10 km/h headwind at 100 km/h adds 21 %
  more drag than computed, 1.7 kW here. Pick a calm day.
* **Tyre slip.** In low gears at full throttle the driven wheels slip a few percent.
  If the speed signal comes from the driven wheels, the measured acceleration is too
  high; if the slip is large, the gear lock breaks and the pull ends.
* **Rotational inertia.** Wheels, driveshafts, gears and the flywheel also have to be
  accelerated, and how much that matters depends on the gear: a textbook rule of thumb
  (Gillespie, *Fundamentals of Vehicle Dynamics*) is
  `k ≈ 1.04 + 0.0025 × (gear ratio × final drive)²`. For the simulator's car that is
  1.57 in 1st, 1.22 in 2nd, **1.09 in 3rd**, 1.06 in 4th. One `k` (1.08 by default) is
  only right for one or two gears, which is why `preferred_gears` is 3rd/4th. A pull in
  2nd with k = 1.08 reads low; in 5th, slightly high.
* **Speed resolution.** The standard OBD speed PID has 1 km/h steps, polled at about
  10-20 Hz; differentiating that is noisy, and the smoothing that tames it also blurs
  the curve. When the vehicle file lists two or more wheel-speed signals (0.01 km/h
  resolution, 50 Hz), the engine uses them instead. A speed signal that reads 2 % high
  makes power read about 4 % high.
* **Aero and rolling data** matter at high speed: at 150 km/h drag is most of the force.

**How to tune:** first `vehicle.mass_kg`, `cd`, `frontal_area_m2`, `crr` and
`tire_circumference_m`; then `rotational_inertia_factor` for the gear you pull in (the
rule of thumb above) and `drivetrain_efficiency` (≈ 0.85 RWD manual, 0.88 FWD manual,
0.80 AWD / automatic, as in `template_full.yaml`). Pull detection:
`metrics.dyno.min_throttle_pct`, `min_rpm_span`, `min_duration_s`; curve shape:
`rpm_bin`, `smoothing_bins`, `speed_smoothing_s`; `max_runs` saved runs.

## 4. Brake metrics

**What you see:** brake pressure (bar and % of full scale), a live pressure trace, a
list of braking events, a trail-braking percentage and a session brake score.

**Pressure.** If the vehicle file maps a pressure signal (`brake_pressure_bar`), that
is shown, with `source: can`. Standard OBD-II has no brake pressure, and many cars'
pressure signals are not documented. Without one, obddash **estimates** pressure from
how hard the car is slowing while the brake switch is on, and labels it
**ESTIMATED**:

```
pressure_bar ≈ estimate_bar_per_g × |deceleration in g|     (only while the brake switch is on)
```

So 0.8 g with the default 60 bar/g shows 48 bar. The estimate includes engine braking
and drag, and needs a brake-switch signal; without either a switch or pressure the BRAKE
page has nothing to show.

**Events.** A braking event starts when pressure rises above
`vehicle.brake.active_threshold_bar` (or the switch goes on) and ends when it has
been released for 0.2 s; events shorter than `metrics.brake.event_min_s` are dropped.
For each: entry and exit speed, peak pressure, average deceleration, duration, and:

* **Trail braking %**: the share of the event during which lateral G was above
  `metrics.brake.trail_lat_g` (0.25 g), i.e. you were still braking while turning in.
  More is not automatically better, but it shows the technique.
* **Smoothness** (0-100): `100 − 0.4 × RMS(dP/dt)`, with dP/dt the pressure rate in
  bar/s: how abruptly you squeeze and release. Stabbing the pedal scores low.

**Brake score** (session): the mean over the last 10 events of
`0.6 × smoothness + 0.4 × min(100, peak / max_pressure_bar × 130)`. The second term
rewards actually using the brakes (about 77 % of full scale counts as full use).

**How to tune:** `vehicle.brake.max_pressure_bar` (the full-scale of the gauge;
typical road cars reach 80-150 bar in a hard stop), `active_threshold_bar`,
`estimate_bar_per_g` (if you have a car with real pressure, fit pressure against
deceleration once; otherwise accept that the estimate's shape is more meaningful than
its absolute value); `metrics.brake.trail_lat_g`, `event_min_s`, `trace_s` (length of
the live trace), `max_events`.

## 5. Handling and G-forces

**What you see:** a friction circle with a 3-second trail, peak lateral, accelerating
and braking G, "% of grip used", a balance needle (understeer ↔ oversteer), smoothness,
a corner count and a handling score.

**Where the G-forces come from**, best first:

1. **IMU** (`source: imu`): an MPU-6050 on the Pi (see
   [wiring.md](../hardware/wiring.md#imu-mpu-6050-on-i2c)), or the car's own
   accelerometer if the vehicle file maps `acc_lat_g` / `acc_lon_g`.
2. **Derived** (`source: derived`), per axis:
   * longitudinal G from the change in speed (lags about 0.4 s, cannot see jolts);
   * lateral G from a yaw-rate signal, `v × yaw_rate / g` (exact for steady
     cornering), or else the **bicycle model** from the steering wheel angle:

     ```
     δ     = steering_deg / steering_ratio        road-wheel angle (radians)
     lat_g = v² × δ / (wheelbase × 9.81)
     ```

     45° of steering at 72 km/h with a 15:1 ratio and a 2.6 m wheelbase is 0.82 g. The
     bicycle model assumes the tyres go exactly where they point, so it overstates G
     when the car understeers and knows nothing about slides.

**Friction circle.** `total = √(lat² + lon²)`. The envelope is the largest total seen
(× 0.98), never less than `metrics.handling.peak_g_floor` (0.8 g), and it decays 1 %
a minute so one pothole doesn't set it for the whole session.
`friction_used_pct = total / envelope × 100`: how much of the grip you have shown you
can use, you are using right now.

**Balance** (needs yaw rate, steering angle and > 20 km/h): the car's real rotation
compared with what the steering asks for:

```
yaw_expected = v × δ / wheelbase                     (a neutral-steering car)
balance      = (yaw_measured − yaw_expected) / max(|yaw_expected|, 3 °/s), signed by the turn direction
```

clamped to −1…+1 and smoothed: negative is understeer (the car turns less than the
wheel asks), positive oversteer (more, including counter-steered slides). Without
yaw-rate and steering signals the balance bar reads "n/a"; going straight it reads 0.

**Smoothness** (0-100): `100 − 25 × RMS(lateral jerk over the last 2 s)`, jerk in g/s:
sawing at the wheel scores low. **Corners**: a corner starts when |lateral G| rises
above `metrics.handling.corner_lat_g` (0.3 g) and ends below half of that; each
finished corner scores its peak total G against the envelope. **Handling score** =
`0.6 × smoothness + 0.4 × mean(last 20 corner scores)`.

**How to tune:** `metrics.handling.corner_lat_g`, `g_trail_s`, `peak_g_floor`;
`vehicle.wheelbase_m` and `vehicle.steering_ratio` (bicycle model and balance); for
the IMU `imu.axis_lon` / `axis_lat` / `axis_vert` (mounting), `imu.lowpass_hz`,
`imu.auto_level_s`.

## 6. Session totals and alerts

* **Distance** integrates speed over time; **fuel used** (`session.fuel_used_l`)
  integrates the fuel-rate PID (0x5E) when the ECU supports it (many don't; then it is
  `null`); gaps over 2 s are skipped, so a dropout never adds a jump. Max speed and max
  rpm are the session maxima. **Reset session** on the SYSTEM page clears everything.
* **Alerts** appear in a red strip: `COOLANT HIGH` at `metrics.coolant_alert_c`
  (110 °C), `OIL HOT` at `metrics.oil_alert_c` (130 °C), `BATT LOW` below
  `metrics.batt_low_v` (12.0 V), each with a little hysteresis, and `NO ECU` when the
  ECU stopped answering.

## Tuning checklist for a new car

1. `obddash config-check`: fix every warning.
2. Gear ratios, final drive, then calibrate `tire_circumference_m` (§1). Check that
   the gear display is steady in every gear.
3. Mass, Cd, frontal area: the manufacturer's figures plus driver and fuel.
4. Brake: `max_pressure_bar` from a hard stop if you have pressure; otherwise leave
   the estimate.
5. IMU: axis mapping until parked reads vert ≈ 1, braking reads lon < 0 and a left
   turn reads lat > 0.
6. A few dyno pulls in 3rd and 4th, both directions; compare with the ECU torque line.
