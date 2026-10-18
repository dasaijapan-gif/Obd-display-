# Vehicle compatibility

Which cars obddash works with, with JDM and European performance cars first. Researched on
2026-10-18 by five parallel research passes, each followed by an independent fact-check
(see [How this list was made](#how-this-list-was-made) for the limits).

Before buying parts or opening trim, plug in and run `obddash scan` and `obddash pids` on
your own car. Model years, markets and option packs change what the OBD port carries.

## What "works" means

Every car that speaks **OBD-II over CAN at the port** gets the core features:

- speed, rpm, throttle, engine temperatures, boost where the ECU reports MAP
- gear estimate from the rpm-to-speed ratio
- downshift and rev-match scoring (manual gearboxes only; DCT, PDK and automatic cars get
  everything else)
- the virtual dyno
- **estimated** brake pressure from deceleration while the brake switch is on
- G-forces and handling from the IMU, on any car

Measured brake pressure, steering angle, yaw and wheel speeds come from the car's own
broadcast CAN frames, which need two things: the frames must reach the place you connect,
and someone must have decoded them. That gives these support levels:

| Level | Meaning |
|---|---|
| **Full at the port** | Plug into the OBD port. Broadcast frames, including brake pressure, are visible there and publicly decoded. |
| **Chassis at the port** | Steering, wheel speeds or yaw are at the port and decoded, but no brake pressure decode. Brake pressure is estimated. |
| **Full with a tap** | The OBD port only answers diagnostic requests (a gateway filters it). Core features work at the port. Connecting to a known internal connector adds measured brake pressure and chassis data. |
| **Core at the port** | Core features work at the port. Chassis data is unconfirmed or not decoded yet. |
| **Not CAN at the port** | OBD-II exists, but on K-line or another older protocol. Needs a future K-line driver, or a tap on an internal CAN bus where one exists. |
| **Pre-OBD** | No usable OBD-II, typical of 1990s JDM cars. Works through an aftermarket ECU's CAN stream or an added sensor box. See [1990s cars](#1990s-cars-pre-obd). |

Confidence: **high** means confirmed by an open source that was read directly. **Medium**
means a decode or report exists but was not cross-checked on a car. **Low** means inferred,
single-source or unverifiable.

---

## JDM

### Full at the port

| Car | Years | Manual | What you get | Notes | Confidence |
|---|---|---|---|---|---|
| Toyota GT86 / Subaru BRZ / Scion FR-S (ZN6/ZC6) | 2012-2021 | Yes | Brake pressure in bar, steering, yaw, lateral and longitudinal G, four wheel speeds, clutch | The reference car for this project. Main CAN on OBD pins 6/14. Brake pressure on 0xD1. | High |
| Mazda MX-5 NC | 2005-2015 | Yes | Brake pressure on DSC cars, steering, four wheel speeds, rpm, speed | Pressure frame 0x085; its scaling is marked "may be wrong" upstream, so calibrate once. Ford-era MS-CAN also sits on pins 3/11; obddash uses only 6/14. | Medium-high |
| Honda S2000 AP2 facelift | 2006-2009 | Yes | Brake pressure, steering, wheel speed | Brake pressure 0x6A at 0.1 bar from an AiM-derived DBC. Mode-01 OBD uses **29-bit** addressing. | Medium |
| Honda Civic Type R FN2 (EU) | 2007-2011 | Yes | Brake pressure, steering, wheel speeds | AiM-derived decode. Mode-01 OBD is 29-bit. | Medium |
| Honda Civic Type R FK2 | 2015-2017 | Yes | Brake pressure, steering, G, gear | AiM-derived decode. Steering on 0x156, not the 10th-gen 0x14A. | Medium |
| Honda Civic Type R FK8 | 2017-2019 | Yes | Brake pressure (0x1A4, raw x 0.239 - 24.4 bar), steering, four wheel speeds | 2020-2021 facelift cars may have the port muted by a gateway; check yours. | Medium |
| Honda Civic Si (8th gen) | 2006-2011 | Yes | Brake pressure (0x1AA), steering, four wheel speeds, gear | VSA-equipped cars only; a non-VSA ABS unit probably has no pressure sensor. | Medium |
| Honda Civic Si (9th gen) | 2012-2015 | Yes | Brake pressure, steering, four wheel speeds, gear | AiM decode exists; presence at the port inferred from the 8th gen. | Medium-low |
| Subaru WRX / WRX STI (VA) | 2015-2017 | Yes | Brake pressure, steering, yaw, wheel speeds | Port broadcast rests on a single source. 2018+ cars are filtered (see below). | Low-medium |

### Chassis at the port

| Car | Years | Manual | What you get | Notes | Confidence |
|---|---|---|---|---|---|
| Mazda MX-5 ND1/ND2 | 2015-2023 | Yes | Steering, speed, rpm, brake **pedal position** (0x078) | Public decodes give pedal position, not line pressure. The 2024+ ND3 isolates the port. | High |
| Mazda RX-8 | 2003-2012 | Yes | Steering (0x081), four wheel speeds (0x4B0), rpm, speed | No public brake pressure; 0x085 is a candidate. **Poll gently**: fast OBD polling glitches the dash and stops the engine revving. | Medium |
| Nissan 370Z (Z34) | 2009-2020 | Yes | Steering (0x002), four wheel speeds (0x284/0x285), yaw and G (0x292) | Racelogic and AiM decodes. Only a brake position value, not pressure. | Medium |
| Nissan GT-R (R35) | 2007-2025 | No (DCT) | Steering, wheel speeds, G, brake position % | AiM's R35 protocol is listen-only broadcast, so the frames are likely at the port. | Medium-low |
| Nissan 350Z (Z33) | 2003-2009 | Yes | Steering, four wheel speeds, yaw | AiM listen-only protocol. Whether early cars answer generic OBD over CAN is unconfirmed. | Low |
| Subaru WRX / STI (GR/GV) | 2008-2014 | Yes | Chassis broadcast confirmed at the port on '08 STI and '08-'09 WRX | Brake pressure frame not confirmed for this generation. | Medium |
| Honda Civic Type R FD2 (JDM) | 2007-2010 | Yes | rpm, speed, four wheel speeds, throttle from broadcast | Whether the JDM ECU answers generic OBD is unconfirmed; obddash can run on broadcast frames alone. | Medium-low |

### Full with a tap

| Car | Years | Manual | Where to connect | What the tap adds | Confidence |
|---|---|---|---|---|---|
| Toyota GR86 / Subaru BRZ (ZN8/ZD8) | 2021- | Yes | DCM connector behind the glovebox (GR86, EU BRZ) or the ASC connector (US BRZ) | Brake pressure (ID 313), steering, wheel speeds, G at 25-50 Hz | High |
| Toyota GR Yaris | 2020- | Yes | Option connector in the passenger kick panel (H62/I174; pin 1 CAN-L, pin 2 CAN-H, 500 kbit/s) | Brake pressure (0x101), steering (0x025), wheel speeds (0x0AA), G (0x024). Port polling peaks near 7 Hz; the tap gives about 50 Hz. | Medium-high |
| Toyota GR Corolla | 2022- | Yes | Same option connector (I174 / H62) | As GR Yaris. Do **not** connect to the CAN-FD bus near the AWD controller. | Medium-high |
| Toyota GR Supra (A90/A91) | 2019- | Some (6MT from MY2023) | BMW G-series PT-CAN | Brake pressure (0x0EF), steering (0x301), yaw (0x19F), G | Medium |
| Subaru WRX / STI (VA) | 2018-2021 | Yes | Behind the BIU/BCM; the port is filtered | Global-platform IDs: brake pressure 313 (gen-1 coefficient, needs calibration), steering, wheel speeds | Medium |
| Subaru WRX (VB) | 2022- | Some | As above | As above | Medium-low |
| Honda Civic Type R FL5 | 2022- | Yes | F-CAN behind the gateway | Brake pressure (0x1A4) on F-CAN. Honda diagnostic requests (VSA DID 4005) can also read it at the port, once obddash supports them. OBD is 29-bit. | Medium |
| Honda Civic Si (11th gen), Acura Integra / Type S (DE4/DE5) | 2022- | Yes | As FL5 | As FL5 (inferred from the FL5) | Low-medium |
| Mitsubishi Lancer Evolution X | 2008-2016 | Some (5MT GSR; MR is TC-SST) | Internal 500 kbit/s bus; the port carries no broadcast | Wheel speeds, steering (AiM EVO_10). Mitsubishi Mode 21 requests at the port also give wheel speeds and steering. | Medium |

### Core at the port

| Car | Years | Manual | Notes | Confidence |
|---|---|---|---|---|
| Lexus IS F | 2008-2014 | No | Commercial decodes suggest chassis data at the port; not independently verified | Low |
| Lexus IS (XE20) | 2006-2013 | Some | As IS F | Low |
| Lexus RC F, GS F, LC 500, IS (XE30) | 2014- | No | Toyota brake frame exists but is raw (needs calibration); port behaviour unknown | Low |
| Nissan Z (RZ34) | 2023- | Some | Port reported request-only (one source); IDs likely shared with the Z34 | Low |
| Mazda MX-5 ND3 | 2024- | Yes | Port no longer carries raw CAN traffic | Medium |
| Mazdaspeed3 / Mazda3 MPS (BK) | 2007-2009 | Yes | Probably chassis data at the port like the NC; unverified | Low |
| Mazdaspeed3 (BL), Mazda2 / Demio | 2009- | Yes | Unknown broadcast | Low |
| Mazda3 (BP) | 2019- | Some | Port showed no usable traffic | Medium |
| Subaru Legacy GT, Forester XT | 2007/08-2018 | Some | CAN cut-over depends on the ECU fitted; check your car | Low |
| Infiniti G37 / Skyline V36, Q50 / Q60 Red Sport, Juke Nismo | 2007- | Some | Nissan chassis IDs (0x002, 0x284/0x285) are a good template to try | Low |
| Honda Civic Si (10th gen), CR-Z, NSX (NC1), Acura TLX Type S | 2010- | Some | Port behaviour varies; OBD often 29-bit | Low |

### Not CAN at the port (2000s)

| Car | Years | Protocol | Notes |
|---|---|---|---|
| Subaru Impreza WRX STI (GD) | 2000-2007 | K-line / SSM | JDM cars may answer only Subaru SSM, not generic OBD |
| Subaru Legacy GT / Forester XT (early) | to ~2007 | K-line | Cut-over is ECU-dependent |
| Mitsubishi Lancer Evolution VII-IX | 2001-2007 | K-line | US/EU cars answer generic OBD; JDM cars probably need Mitsubishi MUT |
| Honda Integra Type R / RSX (DC5), Civic Type R (EP3) | 2001-2006 | K-line | Hondata KPro4 adds a CAN output |
| Honda S2000 AP1 and early AP2 | 1999-2005 | K-line | CAN from 2006 |
| Honda NSX (OBD-II era) | 1997-2005 | K-line | |
| Toyota Mark II (JZX110), Altezza / IS200 / IS300 | 1998-2005 | K-line | JZX110 needs maker-specific addressing |
| Mazda MX-5 NB | 1998-2005 | K-line | |

### 1990s cars (pre-OBD)

Skyline GT-R R32/R33/R34, Silvia S13/S14/S15, 180SX, 300ZX Z32, Supra JZA80, Chaser /
Mark II JZX90/JZX100, Soarer, MR2 SW20, Celica GT-Four ST205, RX-7 FD3S, MX-5 NA,
NSX NA1, Integra Type R DC2, Civic Type R EK9, Lancer Evolution I-VI, Impreza GC8.

None of these have OBD-II over CAN, and none broadcast brake pressure. Three routes:

1. **Aftermarket ECU CAN stream** (best, and common on modified cars). Haltech, Link,
   ECUMaster, MoTeC, AEM, Syvecs and Hondata publish their broadcast layouts. Connect
   obddash to the ECU's CAN port, not the OBD socket, switch OBD polling off and load a
   signal map for that stream. Several streams default to 1 Mbit/s, so the CAN HAT must
   support it (MCP2515 boards with an 8 MHz crystal cannot), or set the stream to 500 kbit/s
   where the ECU allows. A two-node ECU bus needs a 120 ohm terminator at each end.
2. **Factory diagnostic link** with a future driver: Nissan Consult (grey 14-pin, from about
   1989), Subaru SSM, Mitsubishi MUT. 90s Toyota ECUs and the stock FD3S give no live data.
3. **Sensor box**: tach signal, speed sensor or GPS, throttle position, clutch and brake
   switches and a brake pressure transducer, broadcast on CAN in an obddash-defined format.
   Works on any car.

Measured brake pressure on these cars always needs an added transducer on the brake line.
That is safety-critical work; have it fitted and bled by someone qualified.

---

## European

### Full at the port

| Car | Years | Manual | What you get | Notes | Confidence |
|---|---|---|---|---|---|
| Ford Focus RS (Mk3) | 2016-2018 | Yes | Steering, yaw, lateral and longitudinal G, four wheel speeds, clutch position, brake pressure | Public DBC (RS_HS.dbc) read at pins 6/14. Brake pressure (0x252) is a raw 12-bit value and needs calibrating to bar. | Medium-high |
| Ford Focus ST (Mk3) | 2012-2018 | Yes | As Focus RS Mk3 | The same DBC claims ST coverage; needs one capture to confirm | Medium |
| Renault Mégane RS 250 / 265 / 275 (Mk3) | 2010-2016 | Yes | Brake pressure (0x352), steering, four wheel speeds, pedals including clutch | Community DBC captured at the OBD port. Brake scaling marked uncertain. | Medium |
| Renault Clio RS 200 EDC / 220 Trophy (Clio IV) | 2013-2019 | No (EDC) | Brake pressure in bar (0x352), steering, wheel speeds, EDC gear | Same ID set as the Mégane III | Medium |
| Peugeot 308 GTi (T9) | 2015-2021 | Yes | Master-cylinder pressure in bar (0x3CD), yaw, lateral G, clutch pedal position | PSA-RE decode on the bus wired to the diagnostic socket | Medium-low |
| Peugeot 208 GTi (A9) | 2013-2019 | Yes | Steering; pressure frame probably present | | Low |

### Chassis at the port

| Car | Years | Manual | Notes | Confidence |
|---|---|---|---|---|
| Renault Mégane RS 225 / R26 (Mk2) | 2004-2009 | Yes | Main CAN on pins 6/14 with wheel speeds; no brake pressure decode | Medium-low |
| Abarth 500 / 595 / 695 | 2008-2024 | Yes | Engine and ABS probably on the CAN at pins 6/14, by analogy with the Fiat Panda. Never connect to pins 1/9. | Low |
| Abarth / Fiat 124 Spider | 2016-2020 | Some | MX-5 ND base; Fiat engine frames will differ | Low |

### Full with a tap

| Car | Years | Manual | Where to connect | What the tap adds | Confidence |
|---|---|---|---|---|---|
| VW Golf GTI / R Mk5, Mk6; Scirocco R; Audi S3 / RS3 (8P), TT S / TT RS (8J) | 2004-2014 | Yes | J533 gateway harness | PQ35 frames in opendbc vw_pq: steering, wheel speeds, brake pressure and yaw | Medium |
| VW Golf GTI / R Mk7, Mk7.5, Mk8; Polo GTI (AW); Audi S3 / RS3 (8V, 8Y), TT S / TT RS (8S) | 2013- | Some | J533 gateway harness (openpilot taps J533 pins 7/17) | opendbc vw_mqb / vw_mqbevo: brake pressure (ESP_05, 0.3 bar per bit, -30 offset), steering, wheel speeds, yaw, G, clutch switch | Medium-high |
| BMW M3 (E90/E92/E93), 135i / 1M, 335i | 2007-2013 | Yes | PT-CAN in the driver footwell corner, or F-CAN under the door-sill trim | Steering, wheel speeds; brake pressure byte is unscaled (needs calibration). One E92 M3 refused to start with a logger transmitting on PT-CAN, so tap listen-only. | Medium |
| BMW M3 / M4 (F80/F82), M2 (F87), M135i / M235i / 335i (F-series), M3 / M4 (G80/G82), M2 (G87), Z4 (G29) | 2012- | Yes | PT-CAN (under the gear lever, near the steering column or in the right footwell) | Brake pressure (0x0EF), steering (0x301), yaw (0x19F), G | Medium |
| BMW M3 (E46), Z4 M (E85/E86) | 2000-2008 | Yes | PT-CAN, 500 kbit/s. The OBD port is K-line, so everything comes from the tap. | Brake pressure (0x1F8, 1 bar per bit), steering, wheel speeds, G, rpm, clutch, gear | Medium |
| Porsche 911 (997.2, 991, 992), Boxster / Cayman (987.2, 981) | 2009- | Some (991.1 GT3, GT3 RS and Turbo are PDK only) | No tap needed once obddash can send diagnostic requests: brake pressure, steering and wheel speeds answer at the port | Measured brake pressure (992: 0.3 bar per bit, -30 offset; the 992 GT3 differs) | Medium |
| Alfa Romeo Giulia / Stelvio Quadrifoglio | 2016- | Some | Behind the security gateway (reading PIDs still works) | opendbc fca_giorgio: brake pressure (raw), steering, wheel speeds, yaw, G | Medium-low |
| Ford Focus ST (Mk4) | 2019-2025 | Some | Behind the gateway | Steering, wheel speeds, yaw; brake is a **torque** proxy (Nm), not pressure | Medium |
| Mini Cooper S / JCW / GP (R53) | 2002-2006 | Yes | Internal PT-CAN, 500 kbit/s (OBD is K-line) | E46-style frames | Low |

### Core at the port

| Car | Years | Manual | Notes | Confidence |
|---|---|---|---|---|
| Mercedes-AMG C 63 (W204/W205), A 45 / CLA 45, SLK 55 | 2005- | No | Port behind the central gateway; no public brake pressure decode | Low |
| Audi RS4 / RS5 (B8, B9), R8 (Type 42, 4S) | 2007- | Some | B9-generation chassis data is on FlexRay, which a CAN HAT cannot read | Low |
| Mini Cooper S / JCW (R56, F56) | 2007-2024 | Yes | F-series Minis use pins 3/11/12/13 for Ethernet; wire only 6/14 | Low |
| Lotus Elise / Exige S2 (black dash), S3, Evora, Emira | 2008- | Some | Pre-2008 "white dash" S2 cars are K-line (below) | Low |
| Renault Clio RS (Clio III), Mégane RS (Mk4), Alpine A110 | 2006- | Some | 2019+ Renault products may have a silent, gatewayed port | Low |
| Ford Focus ST / RS (Mk2), Fiesta ST (Mk7, Mk8) | 2005-2022 | Yes | Probably like the Mk3 Focus, but no capture yet | Low |
| Volvo C30 / S40 / V50 T5, S60 / V60 Polestar | 2004- | Some | P1 cars broadcast with 29-bit IDs; some early cars need a diagnostic relay switched on first | Low |
| Jaguar F-Type, Aston Martin Vantage (2018+), McLaren, Lamborghini Huracán, modern Ferrari | 2011- | Some | CAN OBD at the port; chassis data and gateways unknown | Low |

### Not CAN at the port

| Car | Years | Notes |
|---|---|---|
| BMW M5 (E60) built before 03/2007, Z3 M | to 2007 | K-line; D-CAN from 03/2007 builds |
| Porsche 911 (996), Boxster (986) | 1997-2005 | K-line |
| Porsche 911 (997.1), Boxster / Cayman (987.1) | 2005-2008 | K-line or CAN, unconfirmed; US MY2008 cars are CAN |
| Lotus Elise / Exige S2 "white dash" | 2004-2007 | K-line; CAN from 2008 |
| Renault Clio RS 172 / 182, Clio V6 | 2000-2006 | K-line |
| Volvo 850 R, S60 R / V70 R (early years) | 1995-2007 | K-line years vary |
| Ferrari 360 | 1999-2005 | K-line |

### Pre-OBD

Ford Escort and Sierra / Sapphire RS Cosworth, Lancia Delta HF Integrale, Fiat Coupé
20V Turbo, Lotus Elise S1, and many Caterhams and Ariel Atoms. The routes are the same as
for [1990s JDM cars](#1990s-cars-pre-obd); Caterham and Ariel cars often already run an
aftermarket ECU with a CAN stream.

---

## Also well covered

Not the focus of this list, but worth knowing:

- **Hyundai i30 N, Veloster N, Elantra N**: opendbc's Hyundai DBC decodes master-cylinder
  pressure (0.1 bar), steering, wheel speeds, yaw and G. Whether the port carries them is
  unverified.
- **Chevrolet Camaro (6th gen), Corvette C7**: believed to have no OBD gateway, and GM's
  DBC includes friction brake pressure. Unverified. The C8 is believed to be gatewayed.

---

## What this means for obddash

Changes the research points to, in rough priority order:

1. **Gentler OBD polling.** Some cars misbehave when polled fast. On 2004-2007 RX-8s it glitches
   the dash and stops the engine revving; the known fix is about 100 ms between requests. A
   2018 STI struggled with more than three PIDs at once. Done in part: vehicle files can set
   `obd_max_rate_hz` (use 10 on an RX-8). Still to do: a more conservative default.
2. **29-bit OBD auto-detection.** Many Hondas (S2000 AP2, FN2, FL5, 2010+ TSX) ignore 11-bit
   requests. obddash already supports 29-bit; it should try both.
3. **Listen-only mode for taps.** On an internal bus obddash must not transmit, acknowledge or
   add termination.
4. **Diagnostic-request polling** (UDS service 0x22, Mitsubishi Mode 21). This reads
   measured brake pressure through the gateway on Porsche 991/981/992 and Honda FL5, and wheel
   speeds and steering on the Evo X, without a tap.
5. **Brake-pressure kinds.** Decodes are bar-scaled (GT86, PSA, Clio IV), raw and uncalibrated
   (Focus RS, Giulia, BMW E9x, Lexus) or a torque proxy (Focus ST Mk4). Vehicle files need a
   calibration field and the UI should label raw values.
6. **More bit rates**: 1 Mbit/s for aftermarket ECU streams, 100 kbit/s for BMW K-CAN.
7. **Ready-made vehicle files**, starting with GT86/BRZ, MX-5 NC and ND, S2000 AP2, Civic
   Type R FK8, Focus RS Mk3, Mégane III RS, GR Yaris / GR Corolla (tap) and MQB Golf (tap).
8. **Aftermarket ECU profiles** for Haltech, Link, ECUMaster, MoTeC, AEM and Hondata streams.

## Wiring safety

- Connect only to OBD pins **6 (CAN-H), 14 (CAN-L)** and ground. Other pins carry buses a
  high-speed CAN transceiver must never touch: fault-tolerant 50 kbit/s B-CAN on pins 1/9
  (Fiat, Alfa Romeo), Ethernet on pins 3/11/12/13 (BMW F/G-series, Mini F56), PSA diagnostic
  CAN on pins 3/8, Ford MS-CAN on pins 3/11.
- Never join a **CAN-FD** bus with a classic CAN HAT (MCP2515). It answers FD frames with error
  frames and disturbs the bus. Known FD buses: near the GR Corolla's AWD controller, and several
  2023+ Hondas.
- On internal taps use listen-only mode, add no terminator, prefer existing option connectors
  to splicing, and never touch airbag (yellow) wiring.
- Do not add a terminator at the OBD port either; the car's bus is already terminated.

## How this list was made

Five research passes covered protocol rules and the pre-CAN JDM route, Toyota / Lexus /
Subaru / Mazda, Nissan / Honda / Mitsubishi, German cars, and other European cars. An
independent fact-checker then tried to refute each pass, and its corrections are applied
here. Limits: the fact-checkers' web search budget ran out and they could read only
GitHub-hosted sources, so claims from AiM documents, Racelogic files and owner forums were
not independently re-checked. Those rows are marked medium or low. Several decodes come
from AiM protocols converted to DBC by a third party; their licence is unclear, so check it
before bundling them.

## Sources

Decodes and captures:

- FT86 gen 1 and gen 2, MX-5 NC and ND: https://github.com/timurrrr/RaceChronoDiyBleDevice and https://github.com/timurrrr/ft86
- opendbc (Toyota, Subaru, Honda, Mazda, VW PQ / MQB / MLB, BMW E9x, PSA, Ford, Alfa Giorgio, Hyundai, GM): https://github.com/commaai/opendbc/tree/master/opendbc/dbc
- BMW E8x/E9x bus layout: https://github.com/dzid26/opendbc-BMW-E8x-E9x
- BMW E46 PT-CAN IDs: https://github.com/emdzej/dashx/blob/main/docs/bmw-e46-can-ids.md
- GR Yaris / GR Corolla option connector: https://github.com/ccoulton/GR-Corolla-Logging
- Honda FL5 signals: https://github.com/Caldis/fl5-can-signals
- Nissan and Honda notes: https://github.com/Knio/carhack
- AiM protocols converted to DBC (Honda, Nissan, Mitsubishi, BMW, Porsche, VW Group): https://github.com/billyjack2/CAN-DBC-Collection
- Focus RS / ST Mk3: https://github.com/RCP1990/mk3-focus-can and https://github.com/klexical/openRS_
- Mégane III RS: https://github.com/krsche/renault-megane-3-rs-can-dbc
- Clio IV RS: https://github.com/TheCosmicNebula/ClioIVDigitalDash
- PSA buses: https://github.com/prototux/PSA-RE and https://github.com/ludwig-v/arduino-psa-diag
- MX-5 ND3 isolated port: https://github.com/shifty35/2024-nd3-mazda-obdii
- Lotus T4e: https://github.com/Alcantor/LotusECU-T4e
- Mini R53 architecture: https://github.com/jackhumbert/mini-cooper-r53
- Volvo P1: https://github.com/Chuck3CZ/Volvo-P1-V50-S40-C30-C70-CAN-bus
- VW J533 tap: https://github.com/commaai/openpilot/wiki/VW-J533-%28Gateway%29-Cable
- Racelogic vehicle connection sheets: https://www.racelogic.co.uk/_downloads/vbox/Vehicles/Other/Docs/

Aftermarket ECU streams and factory protocols:

- Haltech CAN broadcast protocol: https://support.haltech.com/portal/en/kb/articles/haltech-can-ecu-broadcast-protocol
- Link Generic Dash DBC discussion: https://forums.linkecu.com/topic/23547-dbc-for-can
- ECUMaster EMU stream decoder: https://github.com/designer2k2/EMUcan
- MoTeC M1 dash DBC: https://forum.motec.com.au/viewtopic.php?p=16489
- Hondata KPro CAN output: https://www.hondata.com/help/kmanager/can_output.htm
- Many ECU streams as RealDash XML: https://github.com/janimm/RealDash-extras/tree/master/RealDash-CAN/XML-files
- Nissan Consult: https://github.com/jonsim/openconsult
- Subaru SSM: https://mailserver.romraider.com/index.php/RomRaider/SsmProtocol

Regulation:

- US ISO 15765-4 requirement from MY2008: 40 CFR 86.1806-05
- EU EOBD: Directive 98/69/EC (petrol new types 2000, all 2001); CAN not mandated
- Japan J-OBDII from October 2008 for new types; CAN never mandated
  (https://www.mlit.go.jp/common/000134815.pdf)
