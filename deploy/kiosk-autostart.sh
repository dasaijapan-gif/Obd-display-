#!/usr/bin/env bash
# Start Chromium in kiosk mode on the obddash UI once the server answers.
#
# Used two ways (pick ONE, or you get two browsers):
#   * by deploy/obddash-kiosk.service (systemd; restarts Chromium if it ever exits), or
#   * from the desktop session, a line in ~/.config/labwc/autostart:
#         /home/pi/obddash/deploy/kiosk-autostart.sh &
#
# It (1) waits for a Wayland (or X11) display, because under systemd it may start before
# the desktop session, (2) waits until obddash answers /api/health, since a kiosk
# browser pointed at a dead URL would sit on an error page forever, (3) clears
# Chromium's "didn't shut down correctly" state - in a car the power is simply cut -
# and (4) execs Chromium, so systemd supervises the browser process itself.
#
# Environment (all optional):
#   OBDDASH_URL          page to open            (default http://localhost:8080/)
#   OBDDASH_KIOSK_FLAGS  extra Chromium flags, e.g. "--force-device-scale-factor=1.25"
#   OBDDASH_WAIT_LOG_S   how often to log while waiting (default 15)
#   CHROMIUM             browser binary          (default: chromium, else chromium-browser)
set -euo pipefail

URL="${OBDDASH_URL:-http://localhost:8080/}"
HEALTH_URL="${URL%/}/api/health"
LOG_EVERY="${OBDDASH_WAIT_LOG_S:-15}"
[[ "$LOG_EVERY" =~ ^[1-9][0-9]*$ ]] || LOG_EVERY=15

log() { echo "obddash-kiosk: $*" >&2; }

# --- 1. a display to draw on ------------------------------------------------------------
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

find_wayland() {
    # labwc creates $XDG_RUNTIME_DIR/wayland-0 (or -1, ...) plus a .lock file.
    local sock
    for sock in "$XDG_RUNTIME_DIR"/wayland-*; do
        [[ -S "$sock" ]] && { basename "$sock"; return 0; }
    done
    return 1
}

waited=0
while true; do
    if [[ -n "${WAYLAND_DISPLAY:-}" && -S "$XDG_RUNTIME_DIR/${WAYLAND_DISPLAY}" ]]; then
        break
    fi
    if sock="$(find_wayland)"; then
        export WAYLAND_DISPLAY="$sock"
        break
    fi
    if [[ -n "${DISPLAY:-}" ]]; then
        break # X11 session (older Raspberry Pi OS, or an X11 desktop chosen in raspi-config)
    fi
    if (( waited % LOG_EVERY == 0 )); then
        log "waiting for the desktop session (no Wayland socket in $XDG_RUNTIME_DIR yet)"
    fi
    sleep 1
    waited=$((waited + 1))
done

# --- 2. the obddash server --------------------------------------------------------------
server_up() {
    if command -v curl >/dev/null 2>&1; then
        curl -fsS -o /dev/null --max-time 2 "$HEALTH_URL"
    else
        python3 - "$HEALTH_URL" <<'EOF'
import sys, urllib.request
try:
    urllib.request.urlopen(sys.argv[1], timeout=2)
except Exception:
    sys.exit(1)
EOF
    fi
}

waited=0
until server_up 2>/dev/null; do
    if (( waited % LOG_EVERY == 0 )); then
        log "waiting for obddash at $HEALTH_URL (is obddash.service running?)"
    fi
    sleep 1
    waited=$((waited + 1))
done
log "server is up after ${waited}s"

# --- 3. a clean browser profile ---------------------------------------------------------
if [[ -n "${CHROMIUM:-}" ]]; then
    browser="$CHROMIUM"
elif command -v chromium >/dev/null 2>&1; then
    browser="chromium" # Raspberry Pi OS trixie (Debian 13) and later
elif command -v chromium-browser >/dev/null 2>&1; then
    browser="chromium-browser" # Raspberry Pi OS bookworm and earlier
else
    log "no Chromium found: sudo apt install chromium"
    exit 1
fi

profile="${XDG_CONFIG_HOME:-$HOME/.config}/obddash-kiosk"
mkdir -p "$profile/Default"
prefs="$profile/Default/Preferences"
if [[ -f "$prefs" ]]; then
    # After an ignition-off power cut Chromium thinks it crashed and shows a
    # "Restore pages?" bubble on top of the dash.  Mark the last exit as clean.
    sed -i -e 's/"exited_cleanly":false/"exited_cleanly":true/' \
        -e 's/"exit_type":"[A-Za-z]*"/"exit_type":"Normal"/' "$prefs" || true
fi
# Keep the HTTP cache in RAM (tmpfs): fewer SD-card writes, and nothing stale to load
# after an update.  The UI is served with no-cache headers anyway.
cache="$XDG_RUNTIME_DIR/obddash-kiosk-cache"
mkdir -p "$cache"

# --- 4. kiosk ---------------------------------------------------------------------------
# (the commas inside --disable-features are Chromium's list syntax, not array separators)
# shellcheck disable=SC2054
flags=(
    --kiosk
    --user-data-dir="$profile"
    --disk-cache-dir="$cache"
    --noerrdialogs
    --disable-infobars
    --no-first-run
    --no-default-browser-check
    --disable-session-crashed-bubble
    --disable-features=Translate,TranslateUI,MediaRouter
    --check-for-update-interval=31536000
    --password-store=basic
    # touch: real touch events, no pinch-zoom, and no swipe-to-go-back (the UI uses
    # horizontal swipes to change pages)
    --touch-events=enabled
    --disable-pinch
    --overscroll-history-navigation=0
    # the Pi's GPU is on Chromium's blocklist on some releases; canvas drawing is far
    # smoother with acceleration
    --ignore-gpu-blocklist
    --enable-gpu-rasterization
)
if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
    flags+=(--ozone-platform=wayland)
fi
if [[ -n "${OBDDASH_KIOSK_FLAGS:-}" ]]; then
    # intentional word splitting: a space-separated list of extra flags
    # shellcheck disable=SC2206
    flags+=(${OBDDASH_KIOSK_FLAGS})
fi

log "starting $browser on $URL (${WAYLAND_DISPLAY:+wayland $WAYLAND_DISPLAY}${DISPLAY:+x11 $DISPLAY})"
exec "$browser" "${flags[@]}" "$URL"
