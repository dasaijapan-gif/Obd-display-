# FAQ

### Will this void my warranty?

Plugging something into the OBD-II port is what every workshop scan tool, insurance
dongle and fleet tracker does, and obddash only sends the same standard read-only
requests. In the US, the Magnuson-Moss Warranty Act generally means a manufacturer has
to show that an aftermarket part caused a failure before refusing a warranty claim
because of it, and the EU has similar rules. Where you can get into trouble is
**wiring**: a fuse tap or a spliced wire that coincides with an electrical fault gives
the dealer an easy explanation. Keep the install reversible (OBD plug, fuse tap, no cut
wires) and unplug it before a dealer visit. This is not legal advice.

### Can it damage the car?

Listening cannot: receiving CAN frames changes nothing. The real risks are
electrical and are covered in [wiring.md](../hardware/wiring.md#safety): a short on
the OBD pins, back-feeding power, an unfused 12 V tap, or a battery drained by a Pi left
on permanent power.

One CAN-specific risk is worth knowing: **an interface set to the wrong bitrate**
doesn't just fail to receive; it sees every frame as an error and signals error
frames onto the bus, which on some cars can light up warning lamps until the next
restart. The CAN controller gives up quickly (bus-off), but when you are experimenting
with bitrates on a new car, use listen-only mode first (`ListenOnly=yes` in
[`deploy/80-can0.network`](../deploy/80-can0.network) with `obddash run --no-obd`), and
switch it off once `candump` shows clean frames.

obddash never writes to ECUs, clears codes, or sends anything except OBD-II mode 01
(live data) and mode 09 (VIN) requests, at most `bus.obd_max_rate_hz` per second, plus
the ISO-TP flow-control frame a multi-frame answer such as the VIN requires.

### Will it work with my car?

If your car has OBD-II over CAN, the standard data will work: in the US that is every
car from model year 2008 on, and most European and Asian cars of a similar age.
`obddash pids` tells you in five seconds: it lists the PIDs the ECU supports and
their live values. Older cars that use K-line or J1850 protocols are not supported.

The manufacturer-specific signals (brake pressure, steering angle, wheel speeds,
clutch switch) depend on the car: they need a vehicle file entry and are only visible
on the OBD port if the car's gateway passes them through
([BOM.md](../hardware/BOM.md#4-car-connection)). Electric cars answer few standard
PIDs (there is no engine rpm), so most of the SHIFT and DYNO pages do not apply.

### Why does my brake pressure show ESTIMATED?

Standard OBD-II has no brake-pressure PID. Many cars broadcast the pressure on their
internal CAN bus, but the id and scaling differ per car and are rarely documented. Until
your vehicle file has a `brake_pressure_bar` signal, obddash estimates pressure from how
hard the car is decelerating while the brake switch is on, and says so
([metrics.md §4](metrics.md#4-brake-metrics)). To get the real thing: `obddash scan
--diff`, press the brake pedal progressively, and look for a 1-2 byte field that rises
smoothly with pedal force; or find your car in a community DBC collection such as
[opendbc](https://github.com/commaai/opendbc). If the page says NO SIGNAL, there is
neither a pressure signal nor a brake switch.

### Why is the dyno number lower than the brochure?

Mostly because they measure different things. The brochure is power at the crankshaft
on an engine dyno; obddash measures power at the wheels, after 10-20 % has been lost in
the gearbox, differential and tyres (the DYNO page also shows a crank estimate using
`drivetrain_efficiency`). After that, the usual suspects are `mass_kg` set too low,
a slight uphill or headwind, and the rotational-inertia factor not matching the gear
you pulled in. [metrics.md §3](metrics.md#why-the-number-is-not-the-brochure-number-and-its-limits)
goes through each one with numbers. Treat the dyno as excellent for *comparing* runs
on the same road and day (before / after a modification) and approximate as an
absolute figure.

### Does it work with automatics?

Mostly. Set `transmission: auto` in the vehicle file. DRIVE, BRAKE, HANDLING and SYSTEM
work as usual. Rev-match scoring is turned off (there is nothing to score); gear changes
are still detected. The gear display from rpm/speed is less steady than with a manual,
because the torque converter slips at low load; if the car broadcasts the selected gear,
map it as `gear`. The dyno only gives sensible numbers while the torque converter is
locked up: many automatics unlock it at full throttle for the torque multiplication, and
the slip then breaks the fixed rpm-to-speed relationship the dyno relies on. If the
gear confidence stays high through a pull, the converter was locked.

### Why does the gear indicator show N or flicker?

The gear comes from comparing rpm with speed, so it shows N with the clutch down,
below 5 km/h, while coasting in neutral, and for a moment during each shift. That is
intended. If it flickers or shows the wrong gear while you are plainly in gear, the
`gear_ratios`, `final_drive` or `tire_circumference_m` in the vehicle file are off
by more than a few percent: see [metrics.md §1](metrics.md#1-gear-estimate) for how to
calibrate them, and run `obddash config-check`.

### Why did a shift that felt perfect get a C?

Check the details on the SHIFT page. The usual reasons: no throttle blip was detected
(−15; a blip must reach `blip_min_throttle_pct`, 25 % by default, which a gentle blip on
a sensitive throttle may not); the shift took over 1.5 s (−10); or the clutch came up
with the engine 5-8 % low, which costs 1.5× because under-revving is the worse mistake.
And if *every* shift reads a few percent off in the same direction, the tyre
circumference is wrong.

### Why are my G-forces zero, or the wrong way round?

Zero: no IMU is enabled (`imu.enabled: true`, and `i2cdetect -y 1` should show the
sensor at 0x68) and the car broadcasts no yaw rate or steering angle, so lateral G
cannot be derived. Wrong sign or swapped axes: the board is mounted differently from the
config. Fix `imu.axis_lon` / `axis_lat` / `axis_vert` using the table in
[wiring.md](../hardware/wiring.md#mounting-orientation): parked → vert ≈ +1, braking →
lon negative, left turn → lat positive.

### Will it drain my battery?

Not if it is powered from an ignition-switched source, which is the recommended setup.
**Never power it from OBD pin 16**, which is live all the time. If you wire it to
permanent power through a shutdown controller, note that a Pi 5 still draws 1.2-1.6 W
after shutdown unless `POWER_OFF_ON_HALT=1` is set in its EEPROM
([pi-setup.md §13](../hardware/pi-setup.md#13-power-and-shutdown)).

### Can I use it without a Raspberry Pi, or without a screen?

Yes to both. On a laptop, a USB CAN adapter works with the same software
([alternatives.md](../hardware/alternatives.md#usb-can-adapters-development-on-a-laptop)).
On a Pi without a display, open `http://<pi-name>.local:8080` on a phone on the same
Wi-Fi network. And `obddash sim` runs everything against a simulated car on any
computer, no hardware at all.

### How fast does it update?

The metrics run at 50 Hz and the UI receives 30 frames per second. Standard OBD-II data
is limited by the request/answer protocol: about 60 requests per second in total by
default, so rpm, speed and throttle each update roughly 10-20 times a second, and
temperatures every couple of seconds. Broadcast signals from the vehicle file (brake
pressure, wheel speeds, steering) arrive at whatever rate the car sends them, typically
10-100 Hz.

### Does it send data anywhere?

No. Everything runs on the Pi; the UI is served locally and nothing is uploaded.
Recordings stay in `recordings/` on the SD card. Note that the web server has **no
login**: anyone on the same network can view the dashboard and press its buttons (arm
the dyno, reset the session). Don't put the Pi on a network you don't trust, or bind it
to localhost only (`--host 127.0.0.1`) if the display is the only client.

### Is it OK to use while driving?

The display is designed to be glanceable, with big numbers and automatic page switches
for dyno pulls, but it is still a screen. Mount it where it doesn't block your view or
an airbag, check your local rules on screens visible to the driver, and save the dyno
pulls and the shift practice for a track day or closed road. A full-throttle pull to
the redline in 4th is far above any public speed limit in most cars.

### My car sits in the sun. Will it survive?

A parked car can get much hotter inside than any electronics' rated operating
temperature. The usual failures are the LCD blanking until it cools, SD cards wearing
out faster, and the Pi throttling. Mount the Pi out of direct sun, use an active cooler
and a high-endurance SD card, and consider a sunshade. Avoid lithium-cell UPS boards in
the cabin; supercapacitors handle heat much better.
