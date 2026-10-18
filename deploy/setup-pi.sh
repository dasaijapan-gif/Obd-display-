#!/usr/bin/env bash
# obddash: set up a Raspberry Pi (Raspberry Pi OS bookworm / trixie) as an in-car display.
#
#   cd ~/obddash && ./deploy/setup-pi.sh --hat mcp251xfd        # Waveshare 2-CH CAN FD HAT, PiCAN FD
#   ./deploy/setup-pi.sh --hat mcp2515 --oscillator 16000000    # PiCAN2 / PiCAN3 (16 MHz)
#   ./deploy/setup-pi.sh --hat mcp2515 --oscillator 12000000    # Waveshare RS485 CAN HAT (read your crystal!)
#   ./deploy/setup-pi.sh --hat usb                              # candleLight / gs_usb USB adapter (can0)
#   ./deploy/setup-pi.sh --help
#
# Run it as your normal user from inside the checkout; it uses sudo where needed.  Every
# step checks before it changes anything, so running it again is safe (and is how you
# apply a changed option, e.g. --bitrate 250000).
#
# What it does
#   1. apt: git python3-venv can-utils i2c-tools curl (+ chromium for the kiosk)
#   2. enables SPI and I2C (raspi-config nonint), and appends the CAN HAT dtoverlay line
#      to config.txt under an [all] section - only if no such line exists yet
#   3. installs deploy/80-can0.network for systemd-networkd (can0 up at boot) and
#      turns off systemd-networkd-wait-online (with only can0 to manage it can never
#      succeed, and would time out after 2 minutes at every boot)
#   4. creates .venv and runs  pip install -e ".[imu]"
#   5. installs and enables obddash.service (+ obddash-kiosk.service, desktop autologin,
#      no screen blanking) with your user name and checkout path filled in
#   6. prints what is left to do (usually: reboot, then candump can0)
#
# Testing hooks (used by the project's own checks; not needed on a Pi):
#   OBDDASH_BOOT_CONFIG, OBDDASH_SYSTEMD_DIR, OBDDASH_NETWORK_DIR override the target
#   paths; --dry-run prints the privileged commands instead of running them.
set -euo pipefail

# --------------------------------------------------------------------------------------
# options
# --------------------------------------------------------------------------------------
HAT="none"
OSCILLATOR=""
INTERRUPT="25"
BITRATE="500000"
KIOSK=1
APT=1
VENV=1
DRY_RUN=0

usage() {
    sed -n '2,/^set -euo/p' "$0" | sed -e '$d' -e 's/^# \{0,1\}//'
    cat <<'EOF'
Options
  --hat TYPE          mcp251xfd | mcp2515 | usb | none   (default none: config.txt untouched)
  --oscillator HZ     MCP2515 crystal: 16000000 (PiCAN), 12000000 or 8000000 (Waveshare;
                      the value is printed on the silver crystal can).  Required for mcp2515.
  --interrupt GPIO    interrupt pin of the HAT (default 25)
  --bitrate BPS       CAN bitrate for can0 (default 500000; some cars use 250000)
  --no-kiosk          server only: no Chromium, no autologin (view on a phone / laptop)
  --no-apt            skip package installation
  --no-venv           skip the Python virtualenv / pip step
  --dry-run           print the privileged commands instead of running them
  -h, --help          this text
EOF
}

die() { echo "setup-pi: error: $*" >&2; exit 2; }

while (($#)); do
    case "$1" in
        --hat) HAT="${2:?--hat needs a value}"; shift 2 ;;
        --hat=*) HAT="${1#*=}"; shift ;;
        --oscillator) OSCILLATOR="${2:?--oscillator needs a value}"; shift 2 ;;
        --oscillator=*) OSCILLATOR="${1#*=}"; shift ;;
        --interrupt) INTERRUPT="${2:?--interrupt needs a value}"; shift 2 ;;
        --interrupt=*) INTERRUPT="${1#*=}"; shift ;;
        --bitrate) BITRATE="${2:?--bitrate needs a value}"; shift 2 ;;
        --bitrate=*) BITRATE="${1#*=}"; shift ;;
        --no-kiosk) KIOSK=0; shift ;;
        --no-apt) APT=0; shift ;;
        --no-venv) VENV=0; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        -h | --help) usage; exit 0 ;;
        *) usage >&2; die "unknown option: $1" ;;
    esac
done

case "$HAT" in mcp251xfd | mcp2515 | usb | none) ;; *) die "--hat must be mcp251xfd, mcp2515, usb or none" ;; esac
[[ "$INTERRUPT" =~ ^[0-9]+$ ]] || die "--interrupt must be a GPIO number"
[[ "$BITRATE" =~ ^[0-9]+$ ]] || die "--bitrate must be a number in bit/s, e.g. 500000"
if [[ "$HAT" == "mcp2515" ]]; then
    [[ -n "$OSCILLATOR" ]] || die "--hat mcp2515 needs --oscillator (16000000 for PiCAN, 12000000 or 8000000 for Waveshare: read the crystal)"
    [[ "$OSCILLATOR" =~ ^[0-9]+$ ]] || die "--oscillator must be in Hz, e.g. 16000000"
fi

# --------------------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------------------
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f "$REPO/pyproject.toml" && -d "$REPO/src/obddash" ]] || die "run this from an obddash checkout ($REPO has no pyproject.toml)"
# the path ends up in systemd ExecStart= lines and sed replacements
[[ ! "$REPO" =~ [[:space:]\&\|] ]] || die "move the checkout to a path without spaces, '&' or '|' (now: $REPO)"

if [[ $EUID -eq 0 ]]; then
    TARGET_USER="${SUDO_USER:-}"
    [[ -n "$TARGET_USER" && "$TARGET_USER" != "root" ]] ||
        die "run as your normal user (./deploy/setup-pi.sh), not as root: the app should not run as root"
else
    TARGET_USER="$(id -un)"
fi
TARGET_GROUP="$(id -gn "$TARGET_USER")"
TARGET_UID="$(id -u "$TARGET_USER")"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
TARGET_HOME="${TARGET_HOME:-$HOME}"

SYSTEMD_DIR="${OBDDASH_SYSTEMD_DIR:-/etc/systemd/system}"
NETWORK_DIR="${OBDDASH_NETWORK_DIR:-/etc/systemd/network}"
if [[ -n "${OBDDASH_BOOT_CONFIG:-}" ]]; then
    BOOT_CONFIG="$OBDDASH_BOOT_CONFIG"
elif [[ -f /boot/firmware/config.txt ]]; then
    BOOT_CONFIG=/boot/firmware/config.txt # bookworm and later
else
    BOOT_CONFIG=/boot/config.txt # bullseye and older
fi

REBOOT_NEEDED=0
NOTES=()

step() { printf '\n==> %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '    WARNING: %s\n' "$*" >&2; NOTES+=("$*"); }

# run a command as root (sudo when needed); --dry-run only prints it
as_root() {
    if ((DRY_RUN)); then
        printf '    [dry-run] %s\n' "$*"
        return 0
    fi
    if [[ $EUID -eq 0 ]]; then "$@"; else sudo "$@"; fi
}

# run a command as the target user (matters when invoked through sudo); --dry-run prints
as_user() {
    if ((DRY_RUN)); then
        printf '    [dry-run] %s\n' "$*"
        return 0
    fi
    if [[ $EUID -eq 0 ]]; then sudo -u "$TARGET_USER" -H "$@"; else "$@"; fi
}

have() { command -v "$1" >/dev/null 2>&1; }
unit_exists() { systemctl cat "$1" >/dev/null 2>&1; }

# install SRC as DEST (mode 644) only when the content differs; echoes "changed"/"unchanged"
install_file() {
    local src="$1" dest="$2"
    if [[ -f "$dest" ]] && cmp -s "$src" "$dest"; then
        info "unchanged: $dest"
        return 1
    fi
    as_root install -D -m 644 "$src" "$dest"
    info "installed: $dest"
    return 0
}

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "obddash setup: repo $REPO, user $TARGET_USER (uid $TARGET_UID), hat $HAT, bitrate $BITRATE"
if [[ -r /proc/device-tree/model ]]; then
    info "board: $(tr -d '\0' </proc/device-tree/model)"
else
    warn "this does not look like a Raspberry Pi (no /proc/device-tree/model); continuing anyway"
fi

# --------------------------------------------------------------------------------------
# 1. packages
# --------------------------------------------------------------------------------------
step "1/6 packages"
if ((APT)); then
    pkgs=(git python3-venv python3-pip can-utils i2c-tools curl)
    if ((KIOSK)); then
        if have chromium || have chromium-browser; then
            info "Chromium already installed"
        elif policy="$(apt-cache policy chromium 2>/dev/null)" && [[ "$policy" == *"Candidate: "[0-9]* ]]; then
            pkgs+=(chromium) # trixie and later
        else
            pkgs+=(chromium-browser) # bookworm
        fi
    fi
    as_root apt-get update
    as_root apt-get install -y --no-install-recommends "${pkgs[@]}"
else
    info "skipped (--no-apt)"
fi

# --------------------------------------------------------------------------------------
# 2. SPI, I2C and the CAN HAT overlay
# --------------------------------------------------------------------------------------
step "2/6 SPI / I2C / CAN overlay ($BOOT_CONFIG)"
if have raspi-config && [[ -z "${OBDDASH_BOOT_CONFIG:-}" ]]; then
    # 0 = enable (raspi-config's nonint convention: it is a shell exit status)
    if [[ "$(raspi-config nonint get_spi 2>/dev/null || echo 1)" != "0" ]]; then
        as_root raspi-config nonint do_spi 0
        REBOOT_NEEDED=1
    else
        info "SPI already enabled"
    fi
    if [[ "$(raspi-config nonint get_i2c 2>/dev/null || echo 1)" != "0" ]]; then
        as_root raspi-config nonint do_i2c 0
        REBOOT_NEEDED=1
    else
        info "I2C already enabled"
    fi
fi

boot_lines=()
# want LINE, unless an active (uncommented) line matching KEY_REGEX already exists
want_boot_line() {
    local line="$1" key="$2" existing
    if [[ ! -f "$BOOT_CONFIG" ]]; then
        warn "$BOOT_CONFIG not found: add '$line' to your boot config by hand"
        return
    fi
    existing="$(grep -E "^[[:space:]]*${key}" "$BOOT_CONFIG" || true)"
    if [[ -z "$existing" ]]; then
        boot_lines+=("$line")
    elif grep -Fxq "$line" "$BOOT_CONFIG"; then
        info "already present: $line"
    else
        warn "$BOOT_CONFIG already has '$(head -n1 <<<"$existing")' - left as is (wanted '$line'); fix it by hand if it is wrong"
    fi
}

want_boot_line "dtparam=spi=on" "dtparam=spi="
want_boot_line "dtparam=i2c_arm=on" "dtparam=i2c_arm="
case "$HAT" in
    mcp251xfd)
        # MCP2518FD / MCP251863 boards (Waveshare 2-CH CAN FD HAT channel 0, PiCAN FD).
        # The 40 MHz oscillator is the overlay's default.  The Waveshare board's second
        # channel needs "dtoverlay=spi1-3cs" + "dtoverlay=mcp251xfd,spi1-0,interrupt=24".
        want_boot_line "dtoverlay=mcp251xfd,spi0-0,interrupt=${INTERRUPT}" "dtoverlay=mcp25"
        ;;
    mcp2515)
        want_boot_line "dtoverlay=mcp2515-can0,oscillator=${OSCILLATOR},interrupt=${INTERRUPT}" "dtoverlay=mcp25"
        ;;
    usb | none) ;;
esac

if ((${#boot_lines[@]})); then
    {
        echo
        echo "[all]"
        echo "# --- added by obddash deploy/setup-pi.sh ---"
        printf '%s\n' "${boot_lines[@]}"
    } >"$TMP/boot-append"
    info "appending to $BOOT_CONFIG:"
    sed 's/^/      /' "$TMP/boot-append"
    if ((DRY_RUN)); then
        info "[dry-run] not written"
    elif [[ -w "$BOOT_CONFIG" ]]; then
        cat "$TMP/boot-append" >>"$BOOT_CONFIG"
    else
        # the temp file is ours to read; only the append needs root
        # shellcheck disable=SC2024
        sudo tee -a "$BOOT_CONFIG" <"$TMP/boot-append" >/dev/null
    fi
    REBOOT_NEEDED=1
fi
if [[ "$HAT" == "none" ]]; then
    info "no --hat given: no CAN overlay added (see hardware/pi-setup.md)"
fi
if [[ ! -e /dev/i2c-1 && -z "${OBDDASH_BOOT_CONFIG:-}" ]]; then
    # raspi-config loads i2c-dev itself; without raspi-config make it load at boot
    if ! have raspi-config; then
        echo "i2c-dev" >"$TMP/i2c-dev.conf"
        install_file "$TMP/i2c-dev.conf" /etc/modules-load.d/i2c-dev.conf || true
    fi
fi

# --------------------------------------------------------------------------------------
# 3. can0 at boot (systemd-networkd)
# --------------------------------------------------------------------------------------
step "3/6 can0 bring-up (systemd-networkd)"
if [[ "$HAT" == "none" ]]; then
    info "skipped (no --hat)"
else
    case "$BITRATE" in
        500000) br="500K" ;;
        250000) br="250K" ;;
        125000) br="125K" ;;
        1000000) br="1M" ;;
        *) br="$BITRATE" ;;
    esac
    sed "s/^BitRate=500K$/BitRate=${br}/" "$REPO/deploy/80-can0.network" >"$TMP/80-can0.network"
    if install_file "$TMP/80-can0.network" "$NETWORK_DIR/80-can0.network"; then
        NETWORK_CHANGED=1
    else
        NETWORK_CHANGED=0
    fi
    if [[ -z "${OBDDASH_NETWORK_DIR:-}" ]]; then
        if ! unit_exists systemd-networkd.service; then
            as_root apt-get install -y systemd-networkd || true
        fi
        if unit_exists systemd-networkd.service || ((DRY_RUN)); then
            if ! systemctl is-enabled --quiet systemd-networkd 2>/dev/null; then
                as_root systemctl enable --now systemd-networkd
            elif ((NETWORK_CHANGED)); then
                as_root networkctl reload
            fi
            # Enabling networkd also enables systemd-networkd-wait-online (Also= in its
            # [Install] section).  Here networkd manages only can0, which is
            # RequiredForOnline=no, and the Wi-Fi / Ethernet links belong to NetworkManager,
            # so wait-online never sees a link it counts as online: whenever something
            # pulls in network-online.target it would sit out its 2-minute timeout and
            # fail, every boot.  NetworkManager-wait-online already covers "online".
            # (Left alone if you use networkd for other interfaces too.)
            # (|| true: a missing directory in --dry-run, or SIGPIPE from head, must not
            # trip set -e / pipefail)
            other_network="$(find "$NETWORK_DIR" -maxdepth 1 -name '*.network' ! -name '80-can0.network' 2>/dev/null | head -n1 || true)"
            if [[ -n "$other_network" ]]; then
                info "networkd also manages $other_network: systemd-networkd-wait-online left as it is"
            else
                if ((DRY_RUN)) || systemctl is-enabled --quiet systemd-networkd-wait-online.service 2>/dev/null; then
                    as_root systemctl disable systemd-networkd-wait-online.service
                fi
                info "systemd-networkd manages can0 only; NetworkManager keeps Wi-Fi / Ethernet"
            fi
        else
            warn "systemd-networkd is not available: bring can0 up another way (see hardware/pi-setup.md, /etc/network/interfaces)"
        fi
    fi
fi

# --------------------------------------------------------------------------------------
# 4. Python environment
# --------------------------------------------------------------------------------------
step "4/6 Python virtualenv ($REPO/.venv)"
if ((VENV)); then
    py="$(command -v python3 || true)"
    [[ -n "$py" ]] || die "python3 not found"
    if ! "$py" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)'; then
        die "obddash needs Python >= 3.11 (Raspberry Pi OS bookworm or later); found $("$py" -V 2>&1)"
    fi
    if [[ ! -x "$REPO/.venv/bin/python" ]]; then
        as_user "$py" -m venv "$REPO/.venv"
        info "created $REPO/.venv"
    else
        info "venv exists"
    fi
    as_user "$REPO/.venv/bin/python" -m pip install --upgrade pip --quiet
    as_user "$REPO/.venv/bin/python" -m pip install -e "${REPO}[imu]" --quiet
    if ((!DRY_RUN)); then
        info "installed: $("$REPO/.venv/bin/obddash" --version 2>/dev/null || echo obddash)"
    fi
else
    info "skipped (--no-venv)"
fi

# --------------------------------------------------------------------------------------
# 5. services
# --------------------------------------------------------------------------------------
step "5/6 systemd services"
render_unit() {
    # the shipped units use user "pi", uid 1000 and /home/pi/obddash: fill in the real ones
    sed -e "s|/home/pi/obddash|${REPO}|g" \
        -e "s|^User=pi$|User=${TARGET_USER}|" \
        -e "s|^Group=pi$|Group=${TARGET_GROUP}|" \
        -e "s|^WorkingDirectory=/home/pi$|WorkingDirectory=${TARGET_HOME}|" \
        -e "s|/run/user/1000|/run/user/${TARGET_UID}|g" \
        "$1"
}
units_changed=0
render_unit "$REPO/deploy/obddash.service" >"$TMP/obddash.service"
install_file "$TMP/obddash.service" "$SYSTEMD_DIR/obddash.service" && units_changed=1
if ((KIOSK)); then
    chmod +x "$REPO/deploy/kiosk-autostart.sh" 2>/dev/null || true
    render_unit "$REPO/deploy/obddash-kiosk.service" >"$TMP/obddash-kiosk.service"
    install_file "$TMP/obddash-kiosk.service" "$SYSTEMD_DIR/obddash-kiosk.service" && units_changed=1
fi
if [[ -z "${OBDDASH_SYSTEMD_DIR:-}" ]]; then
    ((units_changed)) && as_root systemctl daemon-reload
    as_root systemctl enable obddash.service
    as_root systemctl restart obddash.service
    if ((KIOSK)); then
        as_root systemctl enable obddash-kiosk.service
        info "kiosk starts with the desktop after the next reboot"
    elif unit_exists obddash-kiosk.service; then
        as_root systemctl disable obddash-kiosk.service || true
    fi
fi

# --------------------------------------------------------------------------------------
# 6. kiosk desktop: autologin, no blanking
# --------------------------------------------------------------------------------------
step "6/6 desktop for the kiosk"
if ((KIOSK)) && have raspi-config && [[ -z "${OBDDASH_SYSTEMD_DIR:-}" ]]; then
    # B4 = boot to desktop, logged in automatically
    as_root raspi-config nonint do_boot_behaviour B4
    # 1 = disable screen blanking (0 would enable it)
    as_root raspi-config nonint do_blanking 1
    if [[ -f "$TARGET_HOME/.config/labwc/autostart" ]] && grep -q "kiosk-autostart.sh" "$TARGET_HOME/.config/labwc/autostart"; then
        warn "$TARGET_HOME/.config/labwc/autostart also starts kiosk-autostart.sh: remove that line, the service already does it"
    fi
    REBOOT_NEEDED=1
elif ((KIOSK)); then
    info "raspi-config not found: enable desktop autologin and disable screen blanking yourself"
else
    info "skipped (--no-kiosk)"
fi

# --------------------------------------------------------------------------------------
# done
# --------------------------------------------------------------------------------------
printf '\nDone.  Next steps:\n'
n=1
next_step() {
    printf '  %d. %s\n' "$n" "$*"
    n=$((n + 1))
}
if ((REBOOT_NEEDED)); then
    next_step "sudo reboot                              (config.txt / boot behaviour changed)"
fi
if [[ "$HAT" != "none" ]]; then
    next_step "ip -details link show can0               -> 'state UP ... bitrate $BITRATE'"
    next_step "candump can0                             -> frames with the ignition ON (Ctrl-C stops)"
fi
next_step "$REPO/.venv/bin/obddash pids           -> supported PIDs and live values from the ECU"
next_step "$REPO/.venv/bin/obddash config-check   -> sanity-check your vehicle file"
next_step "journalctl -u obddash -f                 -> server log; the UI is at http://localhost:8080/"
if ((${#NOTES[@]})); then
    echo
    echo "Warnings:"
    printf '  - %s\n' "${NOTES[@]}"
fi
