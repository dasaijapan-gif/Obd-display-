# Raspberry Pi setup: from a blank microSD card to a dash that boots into obddash

About an hour, most of it waiting for downloads. Parts are in [BOM.md](BOM.md), wiring
in [wiring.md](wiring.md). The short version is:

```bash
git clone https://github.com/dasaijapan-gif/Obd-display-.git ~/obddash
cd ~/obddash
./deploy/setup-pi.sh --hat mcp251xfd      # or: --hat mcp2515 --oscillator 16000000
sudo reboot
```

The rest of this page explains what that script does, how to do each step by hand,
and what to do when something doesn't work.

## 1. Flash Raspberry Pi OS

Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and pick
**Raspberry Pi OS (64-bit)**, the desktop version (the kiosk needs the desktop's
compositor). As of October 2026 the current release is based on Debian 13 "trixie"
and uses the **labwc** Wayland compositor; the June 2026 update moved to Linux 6.18
LTS and labwc 0.9.7 ([Phoronix](https://phoronix.com/news/Raspberry-Pi-OS-2026-06-18),
[downloads page](https://www.raspberrypi.com/software/operating-systems/)).
Bookworm-based images (Debian 12, Python 3.11) also work; anything older than
bookworm does not, because obddash needs Python ≥ 3.11.

In Imager's OS customisation, set:

* a user name and password (the scripts work with any user name, not only `pi`),
* Wi-Fi, so you can SSH in from a laptop in the car park,
* **Enable SSH**,
* locale and time zone.

## 2. First boot

```bash
ssh <user>@<hostname>.local
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

## 3. Get obddash and run the setup script

```bash
sudo apt install -y git
git clone https://github.com/dasaijapan-gif/Obd-display-.git ~/obddash
cd ~/obddash
./deploy/setup-pi.sh --help
```

Pick the `--hat` that matches your interface:

| Your interface | Command |
|---|---|
| Waveshare 2-CH CAN FD HAT, PiCAN FD (MCP2518FD) | `./deploy/setup-pi.sh --hat mcp251xfd` |
| PiCAN2 / PiCAN3 (MCP2515, 16 MHz crystal) | `./deploy/setup-pi.sh --hat mcp2515 --oscillator 16000000` |
| Waveshare RS485 CAN HAT (MCP2515) | `./deploy/setup-pi.sh --hat mcp2515 --oscillator 12000000` (or `8000000`: **read the number on the crystal**) |
| USB adapter with candleLight / gs_usb firmware | `./deploy/setup-pi.sh --hat usb` |
| No CAN yet (try the simulator first) | `./deploy/setup-pi.sh` |

Add `--bitrate 250000` if your car turns out to use 250 kbit/s, and `--no-kiosk` for a
headless server you view from a phone. The script is idempotent: run it again with
different options and it only changes what differs. It never adds a second CAN overlay
if one is already in `config.txt`; it warns instead.

Then `sudo reboot` and continue at [§7](#7-check-the-bus-with-candump).

## 4. Enable SPI and I2C (by hand)

```bash
sudo raspi-config nonint do_spi 0      # 0 means "enable" (it is a shell exit status)
sudo raspi-config nonint do_i2c 0
```

or interactively: `sudo raspi-config` → *Interface Options* → *SPI* / *I2C*.
([background on the nonint convention](https://forums.pimoroni.com/t/purpose-of-sudo-raspi-config-nonint-do-i2c-0-for-trilobot/19054))

## 5. Add the CAN HAT overlay

The boot configuration lives in **`/boot/firmware/config.txt`** on bookworm and later
(it was `/boot/config.txt` before). Add the overlay at the very end, under an
**`[all]`** line: the stock file ends with conditional sections such as `[cm4]` /
`[cm5]`, and a line appended below one of those only applies to that model.

**MCP2515 boards** (PiCAN2/PiCAN3, Waveshare RS485 CAN HAT):

```ini
[all]
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
```

`oscillator` is the crystal frequency in Hz (16 MHz on PiCAN boards,
[PiCAN guide](https://copperhilltech.com/blog/tag/PICAN+board); 8, 12 or 16 MHz on
Waveshare boards depending on the revision,
[Waveshare wiki](https://www.waveshare.com/wiki/RS485_CAN_HAT)). `interrupt` is the
GPIO wired to the controller's INT pin (25 on all of these).

**MCP251xFD boards** (Waveshare 2-CH CAN FD HAT, PiCAN FD):

```ini
[all]
dtparam=spi=on
dtoverlay=mcp251xfd,spi0-0,interrupt=25
# second channel on the Waveshare 2-CH board (optional):
#dtoverlay=spi1-3cs
#dtoverlay=mcp251xfd,spi1-0,interrupt=24
```

The 40 MHz oscillator is the overlay's default. The Waveshare board's interrupt pins
are set by jumpers (CAN_0 on GPIO25, CAN_1 on GPIO24 by default)
([Waveshare wiki](https://www.waveshare.com/wiki/2-CH_CAN_FD_HAT),
[kewl.org notes](https://wiki.kewl.org/interfaces:canfdhat)). With both channels
enabled, check which one became `can0` with `ip -details link`.

Reboot, then check that the driver found the chip:

```bash
dmesg | grep -i -E 'mcp251|can'
#   mcp251x spi0.0 can0: MCP2515 successfully initialized.              (MCP2515)
#   mcp251xfd spi0.0 can0: MCP2518FD rev0.0 ... successfully initialized. (MCP2518FD)
ip link show can0
```

## 6. Bring up can0

Once, by hand:

```bash
sudo ip link set can0 up type can bitrate 500000 restart-ms 100
ip -details -statistics link show can0     # "state ERROR-ACTIVE", "bitrate 500000"
```

To change the bitrate later, take the link down first
(`sudo ip link set can0 down`), otherwise you get
`RTNETLINK answers: Device or resource busy`.

**At every boot**, with systemd-networkd and
[`deploy/80-can0.network`](../deploy/80-can0.network) (what `setup-pi.sh` does):

```bash
sudo install -m 644 deploy/80-can0.network /etc/systemd/network/80-can0.network
sudo systemctl enable --now systemd-networkd
sudo systemctl disable systemd-networkd-wait-online.service
networkctl status can0
```

The file sets `BitRate=500K` (with a commented `250K` line), `RestartSec=100ms` for
automatic recovery from bus-off, and `RequiredForOnline=no` so nothing waits for a CAN
"connection". Raspberry Pi OS keeps using NetworkManager for Wi-Fi and Ethernet;
networkd only touches interfaces a `.network` file matches.

Why disable `systemd-networkd-wait-online`: enabling networkd enables it too, and with
can0 as networkd's only link (and ignored for "online") it can never succeed, so it
would hold up anything that waits for `network-online.target` for its full 2-minute
timeout at every boot, then fail. NetworkManager's own wait-online service still
answers for Wi-Fi and Ethernet. Skip this step only if you also use networkd for other
interfaces.

**Alternative: `/etc/network/interfaces`**, if your image uses ifupdown instead:

```
auto can0
iface can0 inet manual
    pre-up /sbin/ip link set can0 type can bitrate 500000 restart-ms 100
    up /sbin/ip link set can0 up
    down /sbin/ip link set can0 down
```

Use one mechanism, not both.

## 7. Check the bus with candump

`can-utils` provides `candump` and `cansend`. With the HAT wired to the OBD port and
the **ignition on** (engine running is better):

```bash
candump -tz can0                  # every frame, with timestamps; Ctrl-C to stop
```

* **A wall of frames** (hundreds per second): the port carries broadcast traffic.
  `obddash scan` will help you find brake pressure, steering angle and friends.
* **Nothing at all**: many cars put the OBD port behind a gateway that only answers
  requests. Ask for engine rpm (a standard mode 01 request, PID 0x0C) and watch for the
  answer:

  ```bash
  candump can0,7E8:7F8 &          # only OBD responses 0x7E8-0x7EF
  cansend can0 7DF#02010C0000000000
  #   can0  7E8   [8]  04 41 0C 0B 54 00 00 00      <- 0x0B54 / 4 = 725 rpm
  kill %1
  ```

  An answer means OBD-II works and obddash will run; you just won't get the
  manufacturer broadcast signals from this port. No answer: see
  [Troubleshooting](#troubleshooting).

## 8. Python environment and first contact with the ECU

`setup-pi.sh` does this; by hand:

```bash
cd ~/obddash
python3 -m venv .venv
. .venv/bin/activate
pip install -e ".[imu]"          # smbus2 for the MPU-6050; add ,dbc for cantools
obddash --version
```

Then, in the car:

```bash
obddash pids             # 5 s of polling: VIN, supported PIDs, live values; exit code 1 if no ECU answered
obddash scan             # live table of every CAN id, changing bytes marked [xx]
obddash scan --diff      # baseline at rest, then press the brake / turn the wheel: candidates ranked
obddash config-check     # resolved config, per-gear rpm table, warnings about your vehicle file
```

Keep your own settings out of the git checkout so `git pull` stays clean:

```bash
cp config/default.yaml config/local.yaml
cp config/vehicles/template_full.yaml config/vehicles/my_car.yaml
# edit both; in local.yaml set  vehicle_file: vehicles/my_car.yaml
echo 'OBDDASH_CONFIG=/home/<user>/obddash/config/local.yaml' | sudo tee /etc/default/obddash
```

([config/vehicles/README.md](../config/vehicles/README.md) explains the vehicle file.)

## 9. Services

`deploy/obddash.service` runs `.venv/bin/obddash run -c $OBDDASH_CONFIG` as your
user, after the network is up, and restarts it if it fails. `setup-pi.sh` fills in
your user name and checkout path; by hand, edit `User=`, `Group=` and the paths, then:

```bash
sudo install -m 644 deploy/obddash.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now obddash
journalctl -u obddash -f
```

The service does not wait for `can0`: the CAN reader retries every 5 s and the UI
shows NO LINK meanwhile. Extra arguments go in `/etc/default/obddash`, e.g.
`OBDDASH_ARGS=--record` to log every drive to `recordings/`.

## 10. Kiosk: Chromium on the display

The browser runs inside the desktop session, so the Pi must **log into the desktop
automatically**:

```bash
sudo raspi-config nonint do_boot_behaviour B4    # B4 = desktop, auto-login
```

Then pick **one** way to start Chromium:

* **systemd** (what `setup-pi.sh` installs): `deploy/obddash-kiosk.service` runs
  [`deploy/kiosk-autostart.sh`](../deploy/kiosk-autostart.sh) and restarts the browser
  if it ever exits. Edit `User=` and the `/run/user/1000` uid if you install it by hand
  (`id -u` shows yours), then `sudo systemctl enable obddash-kiosk`.
* **labwc autostart**: the compositor runs `~/.config/labwc/autostart` once the
  desktop has loaded ([Raspberry Pi kiosk tutorial](https://www.raspberrypi.com/tutorials/how-to-use-a-raspberry-pi-in-kiosk-mode/)).
  Copy the system file first if you want the panel and desktop to keep starting, then
  add the script with a trailing `&`:

  ```bash
  mkdir -p ~/.config/labwc
  [ -f ~/.config/labwc/autostart ] || cp /etc/xdg/labwc/autostart ~/.config/labwc/ 2>/dev/null || true
  echo "$HOME/obddash/deploy/kiosk-autostart.sh &" >> ~/.config/labwc/autostart
  ```

`kiosk-autostart.sh` waits for the Wayland socket and for `/api/health` to answer,
clears Chromium's "didn't shut down correctly" state (the power is simply cut in a
car), keeps the browser cache in RAM, and starts Chromium with `--kiosk`, touch
events on, pinch-zoom and swipe-to-go-back off (the UI uses swipes), no infobars, no
translate prompt and `--ignore-gpu-blocklist`. The browser binary is `chromium` on
trixie and `chromium-browser` on bookworm; the script finds either. Extra flags go in
`OBDDASH_KIOSK_FLAGS`, e.g. `--force-device-scale-factor=1.25`.

To get the desktop back for maintenance: `sudo systemctl stop obddash-kiosk` (the
service restarts a closed browser within seconds). With the labwc autostart method,
plug in a keyboard and press Alt+F4.

## 11. Display: rotation, resolution, blanking

The UI wants **landscape**. Bar displays (400×1280, 480×1920) and the Touch Display 2
(720×1280) are portrait panels and must be rotated.

* **Desktop (what the kiosk uses)**: *Screen Configuration* → *Screen* → your output
  (`HDMI-A-1`, `DSI-1`, ...) → *Orientation*, then *Apply*. On current Raspberry Pi OS
  this rotates the touch input with it ([Waveshare 7.9" wiki](https://www.waveshare.com/wiki/7.9inch_HDMI_LCD)).
* **From the command line** (to test, or in `~/.config/labwc/autostart`):
  `wlr-randr` lists outputs and modes;
  `wlr-randr --output HDMI-A-1 --transform 90` rotates
  ([wlr-randr(1)](https://man.archlinux.org/man/wlr-randr.1.en)). If touch ends up
  rotated differently from the picture, map it in `~/.config/labwc/rc.xml` (the
  `<touch>` element, see labwc-config(5)).
* **Console only / Lite images**: append to the single line in
  `/boot/firmware/cmdline.txt`, e.g. `video=HDMI-A-1:400x1280M@60,rotate=90`. This is
  also how to force the mode if a cheap panel's EDID offers a wrong resolution.

Check the result with `wlr-randr` (the current mode is marked) and on the DRIVE page,
whose speed panel, rpm arc and bars should fill the screen edge to edge. If text is
too small or large, use `--force-device-scale-factor` (above) rather than a different
resolution.

**Screen blanking** must be off, or the dash goes dark after ten minutes of
"inactivity" while you drive:

```bash
sudo raspi-config nonint do_blanking 1    # 1 = disable blanking
```

(or *Display Options* → *Screen Blanking* in `raspi-config`).

## 12. Survive the ignition cutting the power

With switched power the Pi just loses power at key-off. The usual damage is a
corrupted file system or SD card, typically weeks later. Make the root file system
read-only with an overlay in RAM:

`sudo raspi-config` → *Performance Options* → *Overlay File System* → enable (and say
yes to write-protecting the boot partition), reboot.

What changes:

* Every write goes to RAM and disappears at power-off, including **recordings**. Set
  `recording.dir` to a separate partition or USB stick mounted read-write if you want
  to keep logs; only that partition is then at risk.
* To update obddash or change settings: disable the overlay, reboot, `git pull` /
  edit, re-enable, reboot.

If you use a shutdown controller ([wiring.md](wiring.md#power), option C) the overlay
is optional, but it still protects you from the day the controller doesn't get to
finish.

## 13. Power and shutdown

* **Pi 5 on permanent power**: after `shutdown` it still draws about 1.2-1.6 W
  unless you set `POWER_OFF_ON_HALT=1` with `sudo rpi-eeprom-config -e`, which brings
  it to about 0.01 W ([Jeff Geerling](https://www.jeffgeerling.com/blog/2023/reducing-raspberry-pi-5s-power-consumption-140x/)).
  Some HATs dislike having 3.3 V removed while 5 V stays on, which is why it is not
  the default.
* **"This power supply is not capable of supplying 5A"**: a Pi 5 without the 5 V / 5 A
  PD profile caps the USB ports at 600 mA in total. Harmless for a DSI display and a
  CAN HAT. Only if your supply really can deliver 5 A (a buck converter wired to the
  Pi), add `usb_max_current_enable=1` to `config.txt`
  ([Raspberry Pi white paper](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)).
* **Under-voltage during cranking**: `vcgencmd get_throttled` reports past
  under-voltage (bit 16 set). Use a buck converter with a wide input range, or a
  controller that holds the Pi up through the dip.

## Troubleshooting

**`ip link show can0`: "Device "can0" does not exist" (HAT not detected)**
* Is the overlay line under `[all]`, at the end of `/boot/firmware/config.txt`, and
  spelled exactly (`mcp2515-can0`, `mcp251xfd`)? Is SPI enabled?
* `dmesg | grep -i mcp`: "Cannot initialize MCP2515. Wrong wiring?" is usually a wrong
  `oscillator` value, a wrong `interrupt` GPIO, or the HAT not seated on all 40 pins.
* Two CAN overlays (e.g. one from an earlier attempt): keep one.

**can0 is UP but `candump` shows nothing**
* Ignition on? Many cars only talk with the ignition on, and some only with the engine
  running.
* Send the rpm request from [§7](#7-check-the-bus-with-candump): a gatewayed OBD port is
  silent until asked.
* Bitrate: a wrong bitrate gives no frames and rising error counters in
  `ip -details -statistics link show can0` (state ERROR-PASSIVE or BUS-OFF). Try
  `250000` (`./deploy/setup-pi.sh --hat ... --bitrate 250000`, or edit the `.network`
  file).
* Wiring: CAN-H and CAN-L swapped (the most common mistake), ground missing (an
  isolated HAT needs its G terminal on OBD pin 5), the wrong DB9 convention
  ([wiring.md](wiring.md#db9-two-incompatible-conventions)).
* Termination: ≈ 60 Ω between pins 6 and 14 with the car asleep. ≈ 40 Ω means your
  HAT's terminator is on; ≈ 120 Ω or open means you are not on the car's bus.

**Frames arrive, but no OBD answers (`obddash pids` exits with "no OBD-II response")**
* It is not the bitrate: at a wrong bitrate nothing decodes at all, so clean frames mean
  the bitrate is right (250 vs 500 kbit/s belongs in the section above).
* Your car may use **29-bit** OBD identifiers: set `bus.obd_extended_ids: true` in the
  config (requests then go to 0x18DB33F1 and answers come from 0x18DAF1xx).
* The frames you see may belong to a different bus (some cars expose a second CAN bus
  on discretionary pins); OBD-II is always on pins 6 and 14.
* Pre-2008 cars (US) and older European cars may speak K-line or J1850 instead of CAN:
  obddash cannot talk to those.
* The ECU is asleep: ignition fully on, not just accessory.

**"RTNETLINK answers: Device or resource busy"**
You changed the bitrate while can0 was up, or systemd-networkd already brought it up.
`sudo ip link set can0 down` first, or change `/etc/systemd/network/80-can0.network`
and run `sudo networkctl reload && sudo networkctl reconfigure can0`.

**The display shows the desktop, not the dash**
* `journalctl -u obddash-kiosk -b`: the script logs what it is waiting for.
* "waiting for the desktop session": desktop auto-login is off (`do_boot_behaviour B4`),
  or the uid in `/run/user/<uid>` in the unit is not yours.
* "waiting for obddash": `journalctl -u obddash -b`.
* Two browsers fighting: you enabled both the service and a labwc autostart line.

**The UI says NO LINK**
The page loaded but the WebSocket isn't delivering: `systemctl status obddash`,
`curl http://localhost:8080/api/health`.

**Brake pressure shows ESTIMATED, G-forces read zero, the gear flickers**
See the [FAQ](../docs/faq.md).
