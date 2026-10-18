# Vehicle files

A vehicle file describes the car well enough for the gear estimator and the virtual dyno,
and lists the manufacturer-specific CAN broadcast signals (brake pressure, steering
angle, wheel speeds, ...) that standard OBD-II does not provide.

| file | use it when |
|---|---|
| `generic_obd.yaml` | any car, standard PIDs only. Brake pressure is **estimated** from deceleration, G-forces need the IMU. |
| `sim_sportscar.yaml` | the built-in simulator (`obddash sim`). Its broadcast ids are invented and emitted by the fake ECU. |
| `template_full.yaml` | starting point for your own car: every field documented, signal ids left blank. |

Finding your car's broadcast ids: run `obddash scan` while parked with the ignition on,
then press the brake / turn the wheel and watch which ids change. Open-source DBC files
(e.g. the opendbc project) cover many models and can be transcribed into the `signals:`
list below (`obddash.bus.signals.load_dbc` can convert one with `cantools` installed;
a `--dbc` command-line flag is on the roadmap).
