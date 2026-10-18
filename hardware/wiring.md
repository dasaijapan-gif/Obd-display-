# Wiring

Pinouts and diagrams for connecting the Pi, CAN HAT, IMU and power to a car. Parts are
in [BOM.md](BOM.md); software setup is in [pi-setup.md](pi-setup.md).

> **Read [Safety](#safety) before you touch the car's wiring.** Most of this build plugs
> into the OBD-II port and needs no cutting at all; that is the recommended way to do it.

## Overview

```
  OBD-II port (J1962)                Raspberry Pi 5 + CAN HAT                 display
 ┌───────────────────┐   CAN-H (6) ┌──────────────────────────────┐  HDMI/DSI ┌────────┐
 │ 6  CAN-H          ├─────────────┤ H    CAN HAT (MCP2518FD /    │──────────▶│ 1280×  │
 │ 14 CAN-L          ├─────────────┤ L    MCP2515), SPI0, IRQ     │   USB     │  400   │
 │ 5  signal ground  ├─────────────┤ G    GPIO25                 │◀──────────│ touch  │
 │ 4  chassis ground │  (twisted)  │                              │           └────────┘
 │ 16 +12 V battery  │  NOT USED   │ I2C1: GPIO2 SDA, GPIO3 SCL ──┼──▶ MPU-6050
 └───────────────────┘             │ USB-C 5 V in                 │
                                   └──────────────▲───────────────┘
  ignition-switched fuse ──[3-5 A fuse]──▶ 12→5 V 5 A buck ┘   (or USB-C PD car adapter)
  chassis ground bolt ──────────────────────────▶ buck GND
```

Only **three wires** go to the car's data bus: CAN-H, CAN-L and signal ground. Power
comes from an ignition-switched source, not from OBD pin 16.

## OBD-II connector (SAE J1962)

Looking into the car's socket (female, usually under the dash near the steering
column), pins are numbered 1-8 on the top row and 9-16 on the bottom, left to right:

```
   ┌───────────────────────────────┐
   │  1   2   3   4   5   6   7   8 │
    \ 9  10  11  12  13  14  15  16/
     └───────────────────────────┘
```

| Pin | Signal | Used by obddash |
|---|---|---|
| 4 | Chassis ground | optional second ground |
| 5 | Signal ground | **yes**: CAN ground reference |
| 6 | CAN-H (ISO 15765-4) | **yes** |
| 14 | CAN-L (ISO 15765-4) | **yes** |
| 16 | +12 V battery, **always live** | **no**: drains the battery when parked ([BOM §5](BOM.md#5-power)) |
| 7, 15 | K-line / L-line (ISO 9141-2, ISO 14230 KWP2000) | no: pre-CAN protocols |
| 2, 10 | SAE J1850 bus + / − | no: pre-CAN protocols |
| 1, 3, 8, 9, 11, 12, 13 | manufacturer discretionary | no; some makes put a second CAN bus here (e.g. Ford's MS-CAN on 3 / 11) |

CAN on pins 6/14 has been mandatory for OBD-II on US cars since model year 2008
([ISO 15765-4 overview](https://ricksfreeautorepairadvice.com/obd-ii-connector-pinout-diagrams/)).
The standard allows **four** variants: 11-bit or 29-bit identifiers at 250 or
500 kbit/s. Most passenger cars use 11-bit at 500 kbit/s, which is obddash's default.
Some cars use 29-bit identifiers or 250 kbit/s, and you cannot tell which from the
outside, so the configuration exposes both (`bus.bitrate`, `bus.obd_extended_ids`)
and [pi-setup.md](pi-setup.md#troubleshooting) shows how to tell which one you have.

## DB9: two incompatible conventions

OBD-to-DB9 cables and CAN adapters use DB9 connectors, but **not with the same
pinout**. Plugging one convention into the other can put CAN-H on an adapter's ground
and +12 V on a pin the adapter does not expect. Check both ends with a multimeter
before connecting.

| DB9 pin | "OBD-II cable" style (SparkFun CAB-10087, PiCAN boards in OBD mode) | "CiA 303-1" style (Lawicel CANUSB, PEAK PCAN, Kvaser, most industrial adapters, PiCAN in CAN mode) |
|---|---|---|
| 1 | signal ground (OBD 5) | (not connected) |
| 2 | chassis ground (OBD 4) | **CAN-L** |
| 3 | **CAN-H** (OBD 6) | CAN ground |
| 4 | K-line (OBD 7) | (reserved) |
| 5 | **CAN-L** (OBD 14) | (shield, optional) |
| 6 | J1850 bus − (OBD 10) | ground (optional) |
| 7 | J1850 bus + (OBD 2) | **CAN-H** |
| 8 | L-line (OBD 15) | (reserved) |
| 9 | **+12 V battery** (OBD 16, always live) | V+ supply (optional) |

Sources: the [SparkFun cable's pin table](https://sparkfun.com/products/10087)
(OBD 6 → DB9 3, OBD 14 → 5, OBD 5 → 1, OBD 4 → 2, OBD 16 → 9, OBD 7 → 4, OBD 15 → 8,
OBD 10 → 6, OBD 2 → 7), and SK Pang's
[PiCAN FD user guide](https://copperhilltech.com/content/PICAN_FD_UGB_15.pdf), which
documents both layouts (OBD: 2 GND, 3 CAN-H, 5 CAN-L; standard CAN: 3 GND, 7 CAN-H,
2 CAN-L) and the solder bridges SJ1-SJ3 that switch a PiCAN between them.

Rules of thumb:

* A HAT with **screw terminals** (Waveshare): use a DB9 male screw-terminal breakout on
  the OBD cable and wire DB9 3 → H, DB9 5 → L, DB9 1 (or 2) → G. Leave DB9 9 unconnected
  and insulated.
* A **PiCAN** with its DB9: set the solder bridges to the OBD side to match an OBD cable.
  On a PiCAN with an on-board 12 V supply (SMPS), the DB9 can also feed that supply,
  which is how write-ups power "both the Pi and the PiCAN from the OBD port"
  ([Elektor](https://www.elektormagazine.com/news/pican-2-can-bus-board-for-raspberry-pi)).
  Through an OBD-to-DB9 cable that feed is DB9 9 = OBD 16, the **always-live** battery
  pin, so the Pi would never switch off with the car. Check in the board's user guide
  which pin feeds the SMPS, and unless you want permanent power, break that pin (a DB9
  breakout with pin 9 left unconnected) and power the Pi from a switched source.
* A **CiA-pinout USB adapter** (PCAN, Kvaser, Lawicel) needs an OBD-to-DB9 cable wired
  for CiA (sold for those adapters), or a breakout with the wires moved.

## CAN HAT terminals

| HAT | Terminal labels | Connect |
|---|---|---|
| Waveshare 2-CH CAN FD HAT | per channel: H, L, G (isolated side) | CAN_0 H ← OBD 6, L ← OBD 14, G ← OBD 5. The board is isolated, so its G must go to the car's signal ground or the transceiver has no reference. |
| Waveshare RS485 CAN HAT | CAN H, CAN L (+ RS485 A/B; ignore those) | H ← OBD 6, L ← OBD 14; this board is not isolated, so its ground is the Pi's ground, shared through the power supply. Connecting OBD 5 to a Pi GND pin still keeps the reference clean. |
| PiCAN2 / PiCAN3 / PiCAN FD | DB9 male (and a screw terminal block on some versions) | an OBD-to-DB9 cable with the solder bridges in OBD mode |

Termination: **open the HAT's 120 Ω jumper / switch.** With the car asleep (key out,
locked for a few minutes, so no module is driving the bus), measure between OBD pins
6 and 14: ≈ 60 Ω is the car's own two terminators in parallel. With your interface
connected it should still read ≈ 60 Ω. If it reads ≈ 40 Ω your interface is adding a
third terminator: open its jumper.

Keep the CAN wires **twisted** (two turns per inch is plenty) and the stub from the
OBD port to the HAT **short**: under a metre is fine for one listener at 500 kbit/s;
do not daisy-chain extension cables.

## IMU (MPU-6050) on I2C

| MPU-6050 pin | Raspberry Pi header pin | GPIO |
|---|---|---|
| VCC | pin 1 (3V3) | power: use **3.3 V**, so the board's I2C pull-ups go to 3.3 V (Pi GPIOs are not 5 V tolerant) |
| GND | pin 6 (or 9, 14, 20, 25) | ground |
| SDA | pin 3 | GPIO2 (I2C1 SDA) |
| SCL | pin 5 | GPIO3 (I2C1 SCL) |
| AD0 | leave open / GND → address 0x68; to 3V3 → 0x69 | `imu.address` |
| INT, XDA, XCL | not connected | |

The CAN HAT uses SPI0 (GPIO 7-11) plus its interrupt pin (GPIO25, and GPIO24 for a
second channel), so I2C1 is free. If the HAT covers the header without passing the
pins through, use a stacking header or a GPIO splitter. Check the sensor with
`i2cdetect -y 1` (it should show `68`) and set `imu.enabled: true` in
`config/default.yaml`.

Use short wires (≤ 30 cm); I2C was designed for one circuit board, not a car.

### Mounting orientation

The software uses SAE vehicle axes: **x forward, y left, z up** (`src/obddash/model.py`).
At rest a correctly mapped IMU reads `vert ≈ +1.00 g`, accelerating gives `lon > 0`,
and a **left** turn gives `lat > 0`.

Mount the board rigidly (vibration from a floppy bracket shows up as noise), as close
to level and as square to the car's centre line as you can. A board rotated a few
degrees around the vertical axis mixes braking into lateral G; a tilt is mostly
removed by the start-up auto-level (`imu.auto_level_s`, done while standing still).

Map the board's printed axes to the car with `imu.axis_lon`, `imu.axis_lat`,
`imu.axis_vert` (each `+x`, `-x`, `+y`, `-y`, `+z` or `-z`):

| How the board is mounted | `axis_lon` | `axis_lat` | `axis_vert` |
|---|---|---|---|
| flat, components up, printed **X arrow forward** | `+x` | `+y` | `+z` (the default) |
| flat, components up, X arrow pointing **backward** | `-x` | `-y` | `+z` |
| flat, components up, X arrow pointing **left** | `-y` | `+x` | `+z` |
| flat, **upside down** (components down), X arrow forward | `+x` | `-y` | `-z` |
| **vertical**, components facing the rear seats, X arrow pointing **up** | `-z` | `+y` | `+x` |

Verify with `obddash run` on the HANDLING page (or in the SYSTEM page's telemetry dump;
`obddash pids` only polls the CAN bus and does not read the IMU): parked → vert ≈ 1,
lon ≈ 0, lat ≈ 0; brake gently → lon goes negative; turn left → lat goes positive. If
a sign is wrong, flip it in the config rather than remounting.

## Power

Pick one of these (trade-offs in [BOM §5](BOM.md#5-power)):

**A. Ignition-switched fuse tap + buck converter** (clean install, hard power cut)

```
 fuse box: ACC/IGN fuse ── add-a-circuit tap ──[3-5 A fuse]──▶ buck IN+
 chassis ground bolt (bare metal) ─────────────────────────────▶ buck IN−
                                     buck 5 V OUT ──▶ USB-C pigtail ──▶ Pi 5 USB-C
```

Find a fuse that is dead with the key out and live with ignition on (radio, 12 V
socket and cigarette lighter fuses usually are): probe both sides of each candidate
with a multimeter, key off and key on. Use the overlay file system
([pi-setup.md](pi-setup.md#12-survive-the-ignition-cutting-the-power)), since the
power simply disappears.

**B. USB-C PD adapter in the 12 V socket** (no wiring). Check that the socket switches
off with the ignition: some stay live. A Pi 5 will report that the supply cannot
deliver 5 A unless the adapter offers 5 V / 5 A; that only matters for power-hungry
USB devices.

**C. Permanent power + ignition sense + shutdown controller** (clean shutdown)

```
 battery-fed fuse ──[fuse]──▶ controller 12 V IN        (e.g. CarPiHAT PRO 5,
 ACC/IGN fuse ─────[1 A]────▶ controller IGN sense        or a relay + delay-off timer,
 chassis ground ────────────▶ controller GND              or a supercap UPS HAT)
                               controller 5 V ──▶ Pi
 ignition off → controller signals the Pi (GPIO) → Pi shuts down → controller cuts power
```

For Pi-side shutdown on a GPIO, Raspberry Pi OS has `dtoverlay=gpio-shutdown`
(see `/boot/firmware/overlays/README`); the controller's documentation names the pin.

## Safety

* **Fuse every 12 V tap** (3-5 A), as close to the source as you can. An unfused wire
  shorted to the body is a fire.
* **Don't back-feed.** Power the Pi from exactly one source: not USB-C *and* the GPIO
  5 V pins, not a HAT's SMPS *and* a USB supply (unless the board says it handles
  that). Never connect the Pi's 5 V or 3.3 V to anything in the car.
* **Never cut, splice or probe airbag (SRS) wiring** (usually yellow looms and
  connectors) or ABS / ESC / steering wiring. Use the OBD port, or a commercially made
  T-harness at a connector, if you need another bus.
* **Keep CAN stubs short and twisted**, and do not add termination ([above](#can-hat-terminals)).
* **Listening is passive.** Receiving frames does not change anything in the car (your
  controller acknowledges frames like every other node, which is normal). For
  strictly zero transmissions use `obddash run --no-obd` with listen-only mode
  (`ListenOnly=yes` in [`deploy/80-can0.network`](../deploy/80-can0.network)).
* **Active OBD requests are standard diagnostic traffic**: the same mode 01 / 09 queries
  any scan tool or insurance dongle sends. Apart from the ISO-TP flow-control frames
  that multi-frame answers (the VIN) require, obddash sends nothing else, waits for each
  answer, and never exceeds `bus.obd_max_rate_hz` (60/s by default). Keep it modest;
  a flood of requests could delay real diagnostic traffic on some ECUs.
* **Mount the display where it does not block your view** of the road or the airbag
  deployment path, and check your local rules on screens visible to the driver.
* Route cables away from pedals, the steering column and seat rails.
