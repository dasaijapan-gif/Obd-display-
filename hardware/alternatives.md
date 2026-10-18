# Alternatives to the Pi + CAN HAT build

The [primary build](BOM.md) is a Raspberry Pi with a CAN HAT and a touch display,
because it runs the whole Python stack and the browser UI unchanged. This page covers
the other routes people ask about, and is honest about what each one can and cannot do
with obddash as it is today.

| Route | Runs obddash? | Good for | Main limitation |
|---|---|---|---|
| [USB CAN adapter + laptop](#usb-can-adapters-development-on-a-laptop) | yes, unchanged | development, logging, finding signals | a laptop is not a dash |
| [USB CAN adapter + Pi](#usb-can-adapters-development-on-a-laptop) | yes, unchanged | the Pi Zero build, or no free GPIO header | one more thing dangling off a USB port |
| [ESP32 + transceiver + round LCD](#esp32--can-transceiver--round-lcd) | no: a separate firmware | a single dedicated gauge that boots instantly | everything would have to be rewritten in C++ |
| [ELM327 Bluetooth dongle](#elm327-bluetooth-the-slow-fallback) | not today | standard PIDs at a few Hz | about 20 answers per second, no raw CAN |

## USB CAN adapters: development on a laptop

obddash opens the bus through [python-can](https://python-can.readthedocs.io/), so any
adapter python-can supports works, selected with `bus.interface` / `bus.channel`
in the config or `--interface` / `--channel` / `--bitrate` on the command line.
Two families matter:

**candleLight / gs_usb firmware** (CANable with candleLight firmware, Inno-Maker USB2CAN
and other "gs_usb" devices). On Linux the kernel's `gs_usb` driver turns the adapter
into a normal SocketCAN interface, exactly like a HAT:

```bash
sudo ip link set can0 up type can bitrate 500000
obddash run                       # interface: socketcan, channel: can0 (the defaults)
```

On macOS and Windows, python-can has a userspace `gs_usb` backend that needs pyusb and
libusb (`pip install "python-can[gs_usb]"`, not installed by obddash). See its
[documentation](https://python-can.readthedocs.io/en/main/interfaces/gs_usb.html) for
how to select the device; we have not tested it.

**slcan firmware** (the CANable's stock firmware, Lawicel-protocol adapters). The adapter
appears as a serial port, and python-can talks the text protocol itself:

```bash
obddash scan --interface slcan --channel /dev/ttyACM0 --bitrate 500000      # Linux
obddash run  --interface slcan --channel /dev/tty.usbmodem14101 --bitrate 500000   # macOS
obddash run  --interface slcan --channel COM5 --bitrate 500000              # Windows
```

On Linux you can also turn an slcan device into `can0` with can-utils:
`sudo slcand -o -c -s6 /dev/ttyACM0 can0 && sudo ip link set can0 up`
(`-s6` = 500 kbit/s, `-s5` = 250 kbit/s).

Adapters and prices (approximate, October 2026): CANable 2.0 ≈ $35 from its
open-source maker ([listing](https://acceptance.reach.dog/shop/openlight-labs/products/canable-2-0)),
clones from ≈ $17, the isolated CANable Pro ≈ $42-51
([RobotShop](https://www.robotshop.com/products/eyou-robotics-can-usb-adapter-canable-pro)).
PEAK PCAN-USB and Kvaser adapters cost far more and also work (`interface: pcan` /
`kvaser`, or SocketCAN on Linux). Note that most of those use the CiA DB9 pinout, not
the OBD-cable one ([wiring.md](wiring.md#db9-two-incompatible-conventions)).

A good development loop needs no car at all:

```bash
obddash sim                               # the built-in fake car, UI on http://localhost:8080
obddash scan --sim                        # practise finding signals on the simulator
```

and with a car, a laptop on the passenger seat:

```bash
obddash run --interface slcan --channel /dev/ttyACM0 --record    # drive; writes recordings/*.jsonl
obddash replay recordings/<file>.jsonl --speed 2                 # at home: replay through the metrics
```

Recordings hold the *raw* signals, so a replay re-runs every metric: an improved shift
scorer can be tried on last week's drive.

## ESP32 + CAN transceiver + round LCD

An ESP32 has a CAN 2.0 controller built in (Espressif calls it TWAI), so it needs only
a 3.3 V transceiver such as an SN65HVD230 to sit on the bus, and boards with round
240×240 or 412×412 displays are cheap. Hobby projects already do this, for example a
round-LCD gear indicator for a Mustang
([OBD2_Display_for_FordMustang_ESP32-S3](https://github.com/ClaudeMarais/OBD2_Display_for_FordMustang_ESP32-S3))
and ESP32-based OBD and dashboard builds
([overview](https://zbotic.in/esp32-can-bus-interface-automotive-diagnostics-projects/)).

What it **would** do well:

* boot in under a second and tolerate the power being cut (no file system to corrupt),
* draw almost nothing, so it can live on a switched 12 V feed with a tiny regulator,
* one beautiful single-purpose gauge: shift light and gear, boost, oil temperature,
  a G-ball, or the last rev-match grade,
* cost ≈ $20-40 in parts.

What it **would not** do:

* run obddash. None of the Python code (OBD scheduler, ISO-TP, the metrics engine, the
  FastAPI server) or the browser UI runs on a microcontroller. The metrics would have
  to be ported to C++ (or MicroPython, at much lower rates), and the six-page schematic
  UI redrawn for a 240-412 px circle.
* hold much history: dyno curves, brake traces and shift traces need RAM that a
  microcontroller doesn't have in any comfort; a Pi has gigabytes.
* make finding your car's signals easier: you would still use a Pi or laptop with
  `obddash scan` to discover the ids, then hard-code them.

A sensible split, if you want both: keep the Pi as the "brain" and screen, and treat an
ESP32 round gauge as a future second display that subscribes to obddash's WebSocket
over Wi-Fi (`/ws`, JSON documented in [docs/architecture.md](../docs/architecture.md)).
That is on the [roadmap](../docs/roadmap.md); nothing for it exists yet.

## ELM327 Bluetooth: the slow fallback

The $10-30 ELM327 dongles (and better-built compatibles such as the OBDLink range)
are not CAN interfaces. They speak a serial AT-command protocol: you send "010C", the
dongle sends the CAN request, waits for the answer, and returns it as text. That has
consequences:

* **Rate.** About 20 PID answers per second from a cheap clone over Bluetooth, and
  about 47/s from an OBDLink-class adapter, in one tuning-software developer's
  measurements ([VersaTune forum](https://www.versatune.net/forum/viewtopic.php?p=6943)).
  Shared between rpm, speed and throttle, that is about 7 Hz each with a cheap clone
  (less once temperatures and load are polled too). Over a CAN HAT the default
  60 requests/s give roughly 10-20 Hz each, depending on how fast the ECU answers and
  on `bus.obd_multi_pid`, plus every broadcast frame at its native rate.
* **No raw CAN.** Broadcast signals (brake pressure, steering angle, wheel speeds,
  the clutch switch) need the dongle's "monitor all" mode, and clones' buffers overflow
  on a busy 500 kbit/s bus. In practice you get standard PIDs only.

What that means for each metric:

| Metric | With an ELM327 |
|---|---|
| Speed, rpm, temperatures, the DRIVE page | fine, a little laggy |
| Gear estimate | works, a few hundred ms late |
| Rev-match scoring | poor: a downshift takes 0.3-0.6 s, so 7 Hz gives two or three rpm samples around the engagement and no reliable blip detection |
| Virtual dyno | poor: 1 km/h speed steps at 7 Hz make the differentiated acceleration very noisy |
| Brake | estimate only, and only if a brake-switch signal exists, which needs monitor mode |
| Handling | needs the IMU (unaffected by the dongle) |

**Status: obddash cannot use an ELM327 today.** python-can has no ELM327 interface, and
the reader is built on python-can. Supporting one would mean a small new data source
that polls PIDs through the AT protocol and writes the same `RawState` names as the CAN
reader; nothing downstream would change (the [model](../src/obddash/model.py) was
designed for exactly that). It is on the [roadmap](../docs/roadmap.md) as a "works
anywhere" fallback, with the limits above stated in the UI.
