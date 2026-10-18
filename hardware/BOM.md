# Bill of materials

What to buy to put obddash in a car, with two or three options per slot and why you
would pick each. Prices are approximate US dollars, checked in **October 2026**, and
they move: Raspberry Pi raised board prices four times between December 2025 and
October 2026 because LPDDR4 memory got so much more expensive
([Liliputing](https://liliputing.com/?p=198888),
[Gigazine](https://www.gigazine.net/gsc_news/en/20260402-raspberry-pi-price-increases),
[BGR](https://www.bgr.com/2150106/raspberry-pi-cost-ram-crisis/)). Check the vendor
before you order.

Wiring for all of this is in [wiring.md](wiring.md); OS setup is in
[pi-setup.md](pi-setup.md); other platforms (ESP32, USB adapters, ELM327) are in
[alternatives.md](alternatives.md).

## Quick picks

| | Primary build | Cheapest possible | No soldering |
|---|---|---|---|
| Computer | Raspberry Pi 5, 2 or 4 GB | Pi Zero 2 W | Raspberry Pi 5, 2 or 4 GB |
| CAN interface | Waveshare 2-CH CAN FD HAT (MCP2518FD, isolated) | USB CAN adapter (CANable-style, gs_usb or slcan) | Waveshare 2-CH CAN FD HAT (screw terminals) |
| Display | 7.9" 400×1280 HDMI touch "bar" display, rotated to landscape | **none**: your phone, over the Pi's Wi-Fi | Raspberry Pi Touch Display 2 (ribbon cable) |
| Car connection | OBD-II to DB9 cable + DB9 screw-terminal breakout | OBD-II pigtail into the adapter's terminals | OBD-II to DB9 cable + DB9 screw-terminal breakout |
| Power | Fused add-a-circuit on an ignition-switched fuse + 12→5 V 5 A buck | USB car charger in a switched 12 V socket | USB-C PD car adapter in a switched 12 V socket |
| G-forces | MPU-6050 breakout on I2C | none (G derived from speed / yaw rate) | MPU-6050 breakout with pre-soldered pins + jumper wires |
| Rough total | **$250-400** | **$60-90** | **$260-350** |
| Sampling | up to 60 OBD requests/s (the ECU sets the real pace) + every broadcast frame | same CAN path, slower CPU (untested on a Zero 2 W) | same as primary |

The rest of this page explains each slot.

## 1. Computer

| Option | Price | Pick it when |
|---|---|---|
| **Raspberry Pi 5**, 2 GB or 4 GB | 2 GB ≈ $65-78 (a further rise to $77.50 was reported for 1 Oct 2026 ([OMG! Ubuntu](https://www.omgubuntu.co.uk/2026/10/raspberry-pi-2gb-costs-how-much-now/amp)), unconfirmed); 4 GB $110; 8 GB $175 ([Liliputing](https://liliputing.com/?p=198888)) | Default. Chromium plus the Python server use well under 2 GB. The 1 GB model ($45, unaffected by the rises) is probably too tight for Chromium; we have not tested it. |
| **Raspberry Pi 4**, 2-4 GB | about the same as a Pi 5 with the same RAM since the 2026 rises | You already own one. Everything here works on a Pi 4. |
| **Pi Zero 2 W** | ≈ $15 ([Digi-Key](https://www.digikey.com/en/product-highlight/r/raspberry-pi/raspberry-pi-zero-2-w)); uses LPDDR2, so the price rises skipped it | Cheapest build only. 512 MB is not enough to run Chromium comfortably, so run the obddash server **headless** and open the UI on a phone. We have not measured whether it holds the 50 Hz metrics tick. |

Also buy an **active cooler** for a Pi 5 (≈ $5). A car in the sun gets far hotter
than your desk: the dash UI and the 50 Hz loop are light work, but a parked cabin can
exceed the Pi's rated ambient temperature, so mount the Pi out of direct sun.

## 2. CAN interface (the important part)

obddash talks to the car through Linux SocketCAN (`can0`) or any other
[python-can](https://python-can.readthedocs.io/) interface. Any of these work:

| Option | Chip | Price | Trade-offs |
|---|---|---|---|
| **Waveshare 2-CH CAN FD HAT** | MCP2518FD + MCP2562FD, galvanically isolated | $46.99 ([Waveshare](https://www.waveshare.com/product/2-ch-can-fd-hat.htm)) | Isolated (signal and power), TVS-protected, screw terminals, two channels (the second can tap another bus later), CAN FD-capable. Mainline `mcp251xfd` driver. Our first choice. |
| **Waveshare RS485 CAN HAT** | MCP2515 + SN65HVD230, not isolated | ≈ $13-20 ([Welectron](https://welectron.com/Waveshare-14882-RS485-CAN-HAT_1), [Core Electronics](https://core-electronics.com.au/rs485-can-hat-for-raspberry-pi.html)) | Cheap and fine for one 500 kbit/s bus. MCP2515 is classic CAN only and has small receive buffers; at high bus load it can drop frames, but OBD polling plus a few broadcast ids is well within its range. The crystal is 8, 12 or 16 MHz depending on the board revision, so **read the crystal** before writing the overlay line. |
| **PiCAN family** (Copperhill / SK Pang: PiCAN2, PiCAN3, PiCAN FD, Duo variants) | MCP2515 or MCP2518FD | ≈ €60 for a PiCAN2 with 12 V SMPS, ≈ €100 for the Duo with a 3 A SMPS ([Elektor](https://www.elektor.com/products/pican-2-can-bus-board-for-raspberry-pi-2-3-with-smps), [buyzero](https://buyzero.de/en/products/pican2-duo-can-bus-board-for-raspberry-pi-4-with-3a-smps)) | Long track record in car projects, DB9 connector with solder jumpers for the OBD or CiA pinout ([PiCAN FD guide](https://copperhilltech.com/content/PICAN_FD_UGB_15.pdf)), and optional on-board 12 V supply. The 1 A SMPS on the single PiCAN2 is too small for a Pi 4 or 5 (Elektor sends Pi 4 users to the PiCAN3), so check the SMPS rating against your Pi. Fed through an OBD-to-DB9 cable, the SMPS runs from OBD pin 16, which is **always live**: see [wiring.md](wiring.md#db9-two-incompatible-conventions) before using it. |
| **CarPiHAT PRO 5** | CAN port + 12 V supply + safe-shutdown circuit | ≈ £110 ([The Pi Hut](https://thepihut.com/products/carpihat-pro-5-car-interface-dac-for-raspberry-pi-5)) | Fewest boxes: a 12→5 V supply that can deliver 5 A to a Pi 5, ignition/12 V inputs and safe shutdown, *and* a CAN port. Costs more than a CAN HAT plus a buck converter, but it solves the power-management problem too (see §5). We have not tested its CAN overlay; follow its documentation. |
| USB adapter instead of a HAT | see [alternatives.md](alternatives.md) | $17-50 | Useful on a laptop, and the only option for the Pi Zero build. |

**Termination: do not add any.** The car's bus is already terminated at both ends
(two 120 Ω resistors, about 60 Ω between OBD pins 6 and 14 with the car off).
Most HATs have a 120 Ω termination jumper, DIP switch or solder bridge: **open it**.
A third terminator loads the bus (≈ 40 Ω) and is out of spec; on a short OBD stub it
often still works, which is why people get away with it, until it doesn't.

## 3. Display

The UI is drawn for **landscape 1280×400** (its primary design size, the "bar"
format) and lays out cleanly at **1024×600**; any other landscape size is scaled
between those two reference layouts (`U.scale` in `src/obddash/web/js/util.js`).
800×480 works, but the small text gets cramped.

| Option | Native | Use as | Price | Notes |
|---|---|---|---|---|
| **7.9" 400×1280 HDMI capacitive touch** (Waveshare) | portrait 400×1280 | rotate 90° → **1280×400**, exactly the design size | ≈ $90-110 ([RobotShop](https://www.robotshop.com/products/waveshare-79-capacitive-touch-screen-lcd-4001280-hdmi-ips-toughened-glass-cover), [openCircuit](https://opencircuit.shop/product/7.9inch-capacitive-touch-screen-lcd-400-1280)) | Fits under a dashboard brow or in a gauge pod. Rotation on current Raspberry Pi OS is done in Screen Configuration and rotates touch too ([Waveshare wiki](https://www.waveshare.com/wiki/7.9inch_HDMI_LCD)). A DSI version also exists. |
| **8.8" 480×1920 "side monitor"** (Waveshare and others) | portrait 480×1920 | rotate → 1920×480 (4:1), drawn at 1.2× the 1280×400 layout | ≈ $97 HDMI, no touch ([RobotShop](https://www.robotshop.com/products/waveshare-88-ips-side-monitor-4801920-hdmi-hifi-speaker-no-touch)); ≈ $130-155 DSI with touch ([OpenELAB](https://openelab.com/products/waveshare-8-8inch-dsi-capacitive)) | Wider and slightly letterboxed relative to the design. Without touch you change pages with a keyboard or not at all, since the UI's page buttons need touch. Generic 1280×400 / 1920×480 bar panels from marketplaces cost less, but quality, driver boards and brightness vary. |
| **Raspberry Pi Touch Display 2**, 7" or 5" | portrait 720×1280 | rotate → 1280×720 | $60 (7") / $40 (5") ([Raspberry Pi](https://raspberrypi.com/products/touch-display-2)) | No soldering, powered and driven over the DSI ribbon, 5-finger touch, supported until at least January 2030. It is not a bar shape, so the UI uses the 1024×600-style layout with spare height. The 5" has the same pixels in a smaller, sharper panel. |

Brightness matters more than resolution in a car: look for ≥ 400 nits or plan a hood.
An LCD left in a hot, sunlit car can blank until it cools down.

## 4. Car connection

| Part | Price | Notes |
|---|---|---|
| **OBD-II (J1962) to DB9 cable**, e.g. [SparkFun CAB-10087](https://sparkfun.com/products/10087) | ≈ $10-15 | DB9 female end in the "OBD" pinout (CAN-H pin 3, CAN-L pin 5, grounds 1 and 2, +12 V pin 9). There are two DB9 conventions and mixing them can short +12 V into an adapter: read [wiring.md](wiring.md#db9-two-incompatible-conventions) first. |
| **DB9 male to screw-terminal breakout** | ≈ $5-8 | Turns the cable's DB9 into three screw terminals for a HAT without a DB9 socket. No soldering. |
| *or* **OBD-II male pigtail** (16-pin plug with bare wires) | ≈ $8-12 | Cheapest and shortest option: use only pins 4/5 (ground), 6 (CAN-H) and 14 (CAN-L). Insulate every unused wire separately. |
| **OBD-II splitter / Y-cable** (optional) | ≈ $10 | Keeps the port free for a scan tool and lets you mount the plug somewhere your knee won't hit it. Each splitter adds stub length; one is fine. |

The OBD port is easiest, but on many modern cars it sits behind a **gateway** that
only answers diagnostic requests and does not pass broadcast traffic (brake
pressure, steering angle, wheel speeds). Community reports describe this on
VW Group MQB cars ([mqbcan](https://github.com/jrjoaoramos/mqbcan)); whether *your*
car does it shows up in five minutes of `candump` (see
[pi-setup.md](pi-setup.md#7-check-the-bus-with-candump)). Standard OBD-II data works
either way.

## 5. Power

**Do not power the Pi from OBD pin 16.** It is permanent battery +12 V: the Pi
(3-5 W running, and still 1.2-1.6 W after `shutdown` on a Pi 5 unless you change an
EEPROM setting, see [pi-setup.md](pi-setup.md#13-power-and-shutdown)) would run the
battery down while the car is parked. As a rough figure, 0.35 A from a 60 Ah battery
uses half of it in about three and a half days.

| Option | Price | Trade-offs |
|---|---|---|
| **Fused add-a-circuit tap on an ignition/ACC fuse + 12→5 V buck**, e.g. Pololu D36V50F5 (5 V, 5.5 A, up to 50 V in) | tap ≈ $8-10; buck ≈ $40 ([TME](https://www.tme.com/in/en/katalog/converter-modules_113706/), [The Pi Hut](https://thepihut.com/collections/voltage-regulators-buck-converters/brand_pololu)) | Clean, hidden, survives cranking dips if the buck's input range is wide. Power is cut hard at key-off, so use the overlay file system ([pi-setup.md](pi-setup.md#12-survive-the-ignition-cutting-the-power)). Feeding 5 V into the GPIO header bypasses the Pi's input protection and USB-PD negotiation: a Pi 5 then limits USB to 600 mA unless `usb_max_current_enable=1`. A USB-C pigtail from the buck avoids the protection bypass, but not the PD part. |
| **USB-C PD adapter in the 12 V socket** | ≈ $15-30 | No wiring. Most car chargers do not offer the optional 5 V / 5 A PD profile, so a Pi 5 shows a power warning and caps USB at 600 mA ([Raspberry Pi white paper](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)). That is fine for a DSI display and a CAN HAT; it is not fine for a USB-powered HDMI display plus SSD. Check that your socket really switches off with the ignition: some stay live. |
| **Permanent power + ignition sense + shutdown controller** (CarPiHAT PRO 5, or a relay off ACC with a delay-off timer, or a supercapacitor UPS HAT) | £110 for the CarPiHAT; $20-40 for relay/timer kits | The only way to get a clean shutdown. The controller watches ignition, tells the Pi to shut down, then cuts power. Avoid lithium-cell UPS HATs in a car that bakes in the sun; supercapacitors tolerate heat better. |

Always put an **inline fuse (3-5 A)** as close to the 12 V source as possible.

## 6. G-force sensor (optional, recommended)

Without an IMU, obddash derives longitudinal G from speed and lateral G from a yaw-rate
or steering signal (when the car broadcasts one), so it still works, only less
precisely ([metrics.md](../docs/metrics.md#5-handling-and-g-forces)).

| Option | Price | Notes |
|---|---|---|
| **MPU-6050 breakout** (GY-521 and clones) | a few dollars | What the software supports today (`imu.driver: mpu6050`). TDK has discontinued the MPU-6050, so new boards use clone, rejected or recycled stock ([Rapid](https://www.rapidonline.com/news/mpu6050-alternatives-equivalents), [SparkFun forum](https://community.sparkfun.com/t/mpu6050-obsolete/63218)): buy two, they are cheap. Many come with the header pins already soldered, which makes this the no-soldering option with four female-female jumper wires. |
| **MPU-6050 with STEMMA QT / Qwiic connector** | ≈ $10-15 | Plug-in cable instead of jumpers, if you can still find one in stock. |
| ICM-20948, ICM-42688, LSM6DSOX ([Adafruit](https://www.adafruit.com/products/4438)) | $10-20 | Better, current sensors, but **not supported yet**: each needs a small driver in `obddash.sensors` ([roadmap](../docs/roadmap.md)). |

## 7. The rest

| Part | Price | Notes |
|---|---|---|
| microSD, 32 GB, A1/A2, "high endurance" type | ≈ $10-20 | Endurance cards cope better with heat and constant writes. Or skip writes entirely with the overlay file system. |
| Case / mount | ≈ $10-30 | A case that fits the Pi *with* the HAT and cooler, plus a dash/vent/A-pillar mount for the display. A 3D-printed housing is on the [roadmap](../docs/roadmap.md). |
| Wire, ferrules, inline fuse holder, heat-shrink, zip ties | ≈ $15 | 0.5 mm² (20 AWG) is plenty for CAN; use 1 mm² (18 AWG) for 12 V. Twist CAN-H and CAN-L together. |
| A multimeter | you probably have one | For measuring the 60 Ω termination and checking which fuse is switched. |

## Cost ranges in detail

| Build | Low | High | What makes the difference |
|---|---|---|---|
| **Primary** (Pi 5 + Waveshare CAN HAT + 7.9" bar display + fused buck + MPU-6050 + OBD-DB9 cable) | ≈ $250 (Pi 5 2 GB, RS485 CAN HAT, Touch Display 2 5", USB-C car adapter) | ≈ $400 (Pi 5 4 GB, 2-CH CAN FD HAT, 7.9" HDMI, buck + fuse tap, case and mount) | RAM size (memory prices), display choice, and a shutdown controller instead of a buck converter (+ $100). |
| **Cheapest** (Pi Zero 2 W + USB CAN adapter + OBD pigtail + USB charger + SD card; display = your phone) | ≈ $60 | ≈ $90 | A clone adapter vs. a CANable Pro. You also need a micro-USB OTG adapter. |
| **No soldering** (Pi 5 + 2-CH CAN FD HAT + Touch Display 2 + SparkFun cable + DB9 breakout + USB-C PD car adapter + pre-soldered MPU-6050) | ≈ $260 | ≈ $350 | Same as primary, minus the buck converter wiring. |

An ELM327 Bluetooth dongle ($10-30) is cheaper still, but obddash cannot use one
today, and if it could it would get about 20 PID answers per second, too slow for
rev-match scoring and the dyno. See [alternatives.md](alternatives.md#elm327-bluetooth-the-slow-fallback).
