"""The ``obddash`` command line.

::

    obddash run           live car: CAN interface (+ IMU) -> metrics -> web UI
    obddash sim | demo    the built-in fake car on a virtual CAN bus -> web UI
    obddash replay FILE   play a JSONL recording through the metrics -> web UI
    obddash scan          live CAN id table, to find vehicle-specific signals
    obddash pids          poll the ECU for a few seconds: supported PIDs, VIN, values
    obddash config-check  print the resolved configuration and the per-gear rpm table

Common options work before or after the subcommand (``obddash -v sim`` and
``obddash sim -v`` are the same): ``-c/--config``, ``--vehicle``, ``-v/--verbose``,
``--host``, ``--port``.  The subparsers declare them with ``default=SUPPRESS`` so a
value given before the subcommand is not overwritten by the subparser's default.

Server commands build a :class:`~obddash.server.runtime.Runtime` and hand the app to
``uvicorn.run``; everything heavy (FastAPI, python-can, the simulator) is imported
inside the command that needs it, so ``obddash config-check`` stays fast.

Exit codes: 0 ok, 1 command failed (e.g. no ECU answered), 2 usage / configuration
error, 130 interrupted.
"""

from __future__ import annotations

import argparse
import contextlib
import copy
import importlib.util
import logging
import os
import socket
import sys
import threading
import time
import webbrowser
from collections.abc import Iterator, Sequence
from pathlib import Path
from typing import Any, TextIO

from obddash import __version__
from obddash.config import CONFIG_DIR, AppConfig, BusConfig, load_config
from obddash.model import RawState, Sig

log = logging.getLogger(__name__)

DEFAULT_CONFIG = CONFIG_DIR / "default.yaml"
SIM_VEHICLE = CONFIG_DIR / "vehicles" / "sim_sportscar.yaml"
SIM_SCRIPTS = ("lap", "dyno", "idle", "highway")
# two adjacent gears closer than this (rpm/kph ratio) overlap in the +-6 % ratio window
# the gear estimator uses, so the estimate can flicker between them
GEAR_SEPARATION_MIN = 1.12


class CliError(Exception):
    """A user-facing error: printed without a traceback, exit code 2."""


# --------------------------------------------------------------------------------------
# argument parsing
# --------------------------------------------------------------------------------------
def _positive_float(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}") from None
    if not value > 0:
        raise argparse.ArgumentTypeError(f"must be > 0: {text!r}")
    return value


def _add_common(p: argparse.ArgumentParser, *, suppress: bool) -> None:
    def default(value: Any) -> Any:
        return argparse.SUPPRESS if suppress else value

    p.add_argument(
        "-c",
        "--config",
        metavar="PATH",
        default=default(None),
        help=f"app config YAML (default: {DEFAULT_CONFIG})",
    )
    p.add_argument(
        "--vehicle",
        metavar="PATH",
        default=default(None),
        help="vehicle YAML, overrides vehicle_file in the config "
        "(sim/demo default: config/vehicles/sim_sportscar.yaml)",
    )
    p.add_argument(
        "-v",
        "--verbose",
        action="count",
        default=default(0),
        help="more log output; repeat for debug",
    )
    p.add_argument("--host", default=default(None), help="web server bind address")
    p.add_argument("--port", type=int, default=default(None), help="web server port")


def _add_bus_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--interface", help="python-can interface (socketcan, slcan, virtual, ...)")
    p.add_argument("--channel", help="CAN channel, e.g. can0 or /dev/ttyACM0")
    p.add_argument("--bitrate", type=int, help="bit/s (OBD-II on CAN is normally 500000)")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="obddash",
        description="Schematic-style in-car telemetry display for OBD-II / CAN.",
    )
    parser.add_argument("--version", action="version", version=f"obddash {__version__}")
    _add_common(parser, suppress=False)
    common = argparse.ArgumentParser(add_help=False)
    _add_common(common, suppress=True)

    sub = parser.add_subparsers(dest="command", metavar="COMMAND")

    p = sub.add_parser(
        "run",
        parents=[common],
        help="live car over CAN -> web UI",
        description="Serve the dashboard from a live CAN interface (plus the IMU if imu.enabled).",
    )
    _add_bus_args(p)
    p.add_argument("--record", action="store_true", help="record raw signals to recording.dir")
    p.add_argument(
        "--no-obd", action="store_true", help="do not poll OBD-II PIDs (broadcast signals only)"
    )
    p.set_defaults(func=cmd_run, server=True)

    p = sub.add_parser(
        "sim",
        aliases=["demo"],
        parents=[common],
        help="built-in simulated car -> web UI",
        description="Serve the dashboard from the built-in fake car on a virtual CAN bus. "
        "--vehicle defaults to config/vehicles/sim_sportscar.yaml.",
    )
    p.add_argument("--script", choices=SIM_SCRIPTS, default="lap", help="driver script")
    p.add_argument("--seed", type=int, default=1, help="random seed for the driver")
    p.add_argument("--record", action="store_true", help="record raw signals to recording.dir")
    p.set_defaults(func=cmd_sim, server=True)

    p = sub.add_parser(
        "replay",
        parents=[common],
        help="play a recording -> web UI",
        description="Replay a JSONL recording (obddash run/sim --record) through the metrics.",
    )
    p.add_argument("file", metavar="FILE", help="recording (.jsonl)")
    p.add_argument("--speed", type=_positive_float, default=1.0, help="time multiplier")
    p.add_argument("--loop", action="store_true", help="start over at the end")
    p.set_defaults(func=cmd_replay, server=True)

    p = sub.add_parser(
        "scan",
        parents=[common],
        help="live CAN id table (find signals)",
        description="Show every CAN id with its rate and payload, marking bytes that change.",
    )
    _add_bus_args(p)
    p.add_argument("--duration", type=_positive_float, default=60.0, metavar="S", help="seconds")
    p.add_argument("--interval", type=_positive_float, default=0.5, metavar="S", help="redraw")
    p.add_argument(
        "--diff",
        action="store_true",
        help="record a resting baseline first, then list bytes that moved since",
    )
    p.add_argument("--sim", action="store_true", help="scan the built-in simulator instead")
    p.set_defaults(func=cmd_scan, server=False)

    p = sub.add_parser(
        "pids",
        parents=[common],
        help="poll the ECU: supported PIDs, VIN, values",
        description="Run the CAN reader for a few seconds and report what the ECU answers.",
    )
    _add_bus_args(p)
    p.add_argument("--seconds", type=_positive_float, default=5.0, metavar="S")
    p.add_argument("--sim", action="store_true", help="poll the built-in simulator instead")
    p.set_defaults(func=cmd_pids, server=False)

    p = sub.add_parser(
        "config-check", parents=[common], help="print the resolved config and gear table"
    )
    p.set_defaults(func=cmd_config_check, server=False)
    return parser


# --------------------------------------------------------------------------------------
# config helpers
# --------------------------------------------------------------------------------------
def _resolve_vehicle(value: str | None) -> Path | None:
    """A ``--vehicle`` path the user typed: relative to the cwd if it exists there,
    otherwise left for ``load_config`` to resolve next to the config / in config/."""
    if not value:
        return None
    p = Path(value).expanduser()
    return p.resolve() if p.is_file() else p


def load_app_config(args: argparse.Namespace, *, sim_vehicle_default: bool = False) -> AppConfig:
    """Load config + vehicle as the common options say; raise CliError on problems."""
    import pydantic
    import yaml

    cfg_path = Path(args.config).expanduser() if args.config else None
    if cfg_path is not None and not cfg_path.is_file():
        raise CliError(f"config file not found: {cfg_path}")
    vehicle = _resolve_vehicle(args.vehicle)
    if vehicle is None and sim_vehicle_default:
        vehicle = SIM_VEHICLE
    try:
        cfg = load_config(cfg_path, vehicle)
    except FileNotFoundError as exc:
        raise CliError(f"file not found: {exc.filename or exc}") from exc
    except yaml.YAMLError as exc:
        raise CliError(f"invalid YAML: {exc}") from exc
    except pydantic.ValidationError as exc:
        raise CliError(f"invalid configuration: {exc}") from exc
    if args.host:
        cfg.server.host = args.host
    if args.port:
        cfg.server.port = args.port
    return cfg


def _apply_bus_args(cfg: AppConfig, args: argparse.Namespace) -> None:
    update = {
        key: getattr(args, key)
        for key in ("interface", "channel", "bitrate")
        if getattr(args, key, None) is not None
    }
    if update:
        cfg.bus = cfg.bus.model_copy(update=update)


# --------------------------------------------------------------------------------------
# server commands
# --------------------------------------------------------------------------------------
def cmd_run(args: argparse.Namespace) -> int:
    cfg = load_app_config(args)
    _apply_bus_args(cfg, args)
    if args.no_obd:
        cfg.bus = cfg.bus.model_copy(update={"obd_enabled": False})
    if args.record:
        cfg.recording.enabled = True
    return serve(cfg, source="can", verbose=args.verbose)


def cmd_sim(args: argparse.Namespace) -> int:
    cfg = load_app_config(args, sim_vehicle_default=True)
    if args.record:
        cfg.recording.enabled = True
    return serve(
        cfg, source="sim", verbose=args.verbose, sim_script=args.script, sim_seed=args.seed
    )


def cmd_replay(args: argparse.Namespace) -> int:
    from obddash.server.recorder import read_meta

    path = Path(args.file).expanduser()
    if not path.is_file():
        raise CliError(f"recording not found: {path}")
    meta = read_meta(path) or {}
    recorded_vehicle = meta.get("vehicle_file")
    if args.vehicle is None and recorded_vehicle and Path(recorded_vehicle).is_file():
        # replay with the car it was recorded with unless told otherwise
        args.vehicle = recorded_vehicle
        log.info("using the recording's vehicle file %s", recorded_vehicle)
    cfg = load_app_config(args)
    if meta.get("vehicle") and meta["vehicle"] != cfg.vehicle.name:
        log.warning(
            "recording was made with vehicle %r but the config uses %r (see --vehicle)",
            meta["vehicle"],
            cfg.vehicle.name,
        )
    return serve(
        cfg,
        source="replay",
        verbose=args.verbose,
        replay_path=path,
        replay_speed=args.speed,
        replay_loop=args.loop,
    )


def serve(cfg: AppConfig, *, source: str, verbose: int = 0, **runtime_kwargs: Any) -> int:
    """Build the runtime + app and run uvicorn until Ctrl-C."""
    import uvicorn

    from obddash.server.app import create_app
    from obddash.server.runtime import Runtime

    try:
        runtime = Runtime(cfg, source=source, **runtime_kwargs)
    except (ImportError, ValueError, OSError) as exc:
        raise CliError(f"cannot set up the {source} runtime: {exc}") from exc
    app = create_app(runtime)
    local_url, lan_url = _urls(cfg.server.host, cfg.server.port)
    print(banner(cfg, source, runtime_kwargs, local_url, lan_url), flush=True)
    if cfg.server.open_browser:
        timer = threading.Timer(1.5, webbrowser.open, args=(local_url,))
        timer.daemon = True
        timer.start()
    uvicorn.run(
        app,
        host=cfg.server.host,
        port=cfg.server.port,
        log_config=None,  # use our logging setup
        log_level="debug" if verbose >= 2 else ("info" if verbose >= 1 else "warning"),
        access_log=verbose >= 1,
    )
    return 0


def banner(
    cfg: AppConfig,
    source: str,
    detail: dict[str, Any],
    local_url: str,
    lan_url: str | None = None,
) -> str:
    if source == "can":
        what = f"CAN {cfg.bus.interface}:{cfg.bus.channel}"
    elif source == "sim":
        what = f"simulator ({detail.get('sim_script', 'lap')}, seed {detail.get('sim_seed', 1)})"
    elif source == "replay":
        name = Path(str(detail.get("replay_path", "?"))).name
        what = f"replay {name} x{detail.get('replay_speed', 1.0):g}"
        if detail.get("replay_loop"):
            what += " (loop)"
    else:
        what = "no data source"
    line = f"obddash {__version__} | {what} | {cfg.vehicle.name} | {local_url}"
    if lan_url:
        line += f"  (LAN {lan_url})"
    return line


def _urls(host: str, port: int) -> tuple[str, str | None]:
    """The URL to open on the Pi itself, plus the LAN URL when bound to all interfaces."""
    if host in ("", "0.0.0.0", "::"):
        return f"http://localhost:{port}/", (
            f"http://{ip}:{port}/" if (ip := _lan_ip()) is not None else None
        )
    shown = f"[{host}]" if ":" in host else host
    return f"http://{shown}:{port}/", None


def _lan_ip() -> str | None:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("10.254.254.254", 1))  # UDP connect sends nothing; picks the route
            ip = s.getsockname()[0]
    except OSError:
        return None
    return None if ip.startswith("127.") else ip


# --------------------------------------------------------------------------------------
# bus tools
# --------------------------------------------------------------------------------------
@contextlib.contextmanager
def _bus_session(cfg: AppConfig, *, sim: bool) -> Iterator[tuple[Any, BusConfig, str]]:
    """Yield ``(bus, bus_cfg, description)``: the configured interface, or with
    ``sim`` a virtual bus with the simulator running on its twin."""
    import can

    buses: list[Any] = []
    simulator = None
    try:
        if sim:
            from obddash.sim import Simulator

            channel = f"obddash-cli-{os.getpid()}"
            ecu_bus = can.Bus(interface="virtual", channel=channel)
            buses.append(ecu_bus)
            bus = can.Bus(interface="virtual", channel=channel)
            buses.append(bus)
            simulator = Simulator(cfg.vehicle, ecu_bus, script="lap", seed=1, realtime=True)
            simulator.start()
            bus_cfg = cfg.bus.model_copy(
                update={
                    "interface": "virtual",
                    "channel": channel,
                    "obd_extended_ids": False,
                    "obd_request_id": 0x7DF,
                    "obd_response_ids": list(range(0x7E8, 0x7F0)),
                }
            )
            desc = f"simulator ({cfg.vehicle.name}, virtual:{channel})"
        else:
            bus_cfg = cfg.bus
            try:
                bus = can.Bus(
                    interface=bus_cfg.interface,
                    channel=bus_cfg.channel,
                    bitrate=bus_cfg.bitrate,
                    receive_own_messages=bus_cfg.receive_own_messages,
                )
            except Exception as exc:
                raise CliError(
                    f"cannot open CAN {bus_cfg.interface}:{bus_cfg.channel}: {exc}"
                ) from exc
            buses.append(bus)
            desc = f"{bus_cfg.interface}:{bus_cfg.channel} @ {bus_cfg.bitrate} bit/s"
        yield bus, bus_cfg, desc
    finally:
        if simulator is not None:
            simulator.stop()
        for b in buses:
            try:
                b.shutdown()
            except Exception:
                log.debug("bus shutdown failed", exc_info=True)


def cmd_scan(args: argparse.Namespace) -> int:
    from obddash.bus.scan import run_scan

    cfg = load_app_config(args, sim_vehicle_default=args.sim)
    _apply_bus_args(cfg, args)
    with _bus_session(cfg, sim=args.sim) as (bus, _bus_cfg, desc):
        print(f"scanning {desc} for {args.duration:g} s (Ctrl-C stops)", flush=True)
        run_scan(bus, args.duration, interval=args.interval, diff=args.diff, out=sys.stdout)
    return 0


def cmd_pids(args: argparse.Namespace) -> int:
    from obddash.bus.reader import CanReader

    cfg = load_app_config(args, sim_vehicle_default=args.sim)
    _apply_bus_args(cfg, args)
    raw = RawState()
    with _bus_session(cfg, sim=args.sim) as (bus, bus_cfg, desc):
        if not bus_cfg.obd_enabled:
            # polling is the point of this command, even for a broadcast-only config
            bus_cfg = bus_cfg.model_copy(update={"obd_enabled": True})
        reader = CanReader(bus_cfg, cfg.vehicle, raw, bus=bus)
        print(f"polling {desc} for {args.seconds:g} s ...", flush=True)
        reader.start()
        try:
            _sleep_interruptible(args.seconds)
        finally:
            stats = copy.deepcopy(reader.stats)
            reader.stop()
    print_pids_report(stats, raw, cfg)
    if stats.pid_responses == 0:
        hint = "no OBD-II response: ignition on? right bitrate (500000)? bus terminated?"
        if stats.frames:
            hint += f" ({stats.frames} other frames were seen, so the bus itself works)"
        print(hint, file=sys.stderr)
        return 1
    return 0


def _sleep_interruptible(seconds: float) -> None:
    try:
        time.sleep(seconds)
    except KeyboardInterrupt:
        print("interrupted; reporting what was collected", file=sys.stderr)


def print_pids_report(stats: Any, raw: RawState, cfg: AppConfig, out: TextIO | None = None) -> None:
    out = out or sys.stdout
    from obddash.bus import obd

    def p(text: str = "") -> None:
        print(text, file=out)

    rates = {req.pid: req.rate for req in cfg.vehicle.pids}
    p(
        f"bus {'ok' if stats.bus_ok else 'DOWN'} | frames {stats.frames} ({stats.frame_hz:.0f}/s)"
        f" | OBD responses {stats.pid_responses} ({stats.pid_hz:.1f}/s)"
        f" | requests {stats.requests}, timeouts {stats.timeouts}"
    )
    if stats.last_error:
        p(f"last error: {stats.last_error}")
    p(f"VIN: {stats.vin or '-'}")
    supported = sorted(stats.supported_pids)
    p(f"supported PIDs ({len(supported)}):")
    for pid in supported:
        d = obd.PID_TABLE.get(pid)
        if pid % 0x20 == 0:
            name, unit = f"(supports 0x{pid + 1:02X}-0x{pid + 0x20:02X})", ""
        elif d is None:
            name, unit = "(no decoder)", ""
        else:
            name, unit = d.sig or d.name, d.unit
        polled = f"[{rates[pid]}]" if pid in rates else ""
        p(f"  0x{pid:02X}  {name:<34} {unit:<6} {polled}")
    snap = raw.snapshot()
    counts = raw.counts()
    p(f"latest values ({len(snap)} signals):")
    if snap:
        p(f"  {'signal':<26} {'value':>11} {'age s':>7} {'updates':>8}")
        now = time.monotonic()
        for name in sorted(snap):
            rv = snap[name]
            p(f"  {name:<26} {rv.value:>11.3f} {now - rv.t:>7.2f} {counts.get(name, 0):>8}")


# --------------------------------------------------------------------------------------
# config-check
# --------------------------------------------------------------------------------------
def cmd_config_check(args: argparse.Namespace) -> int:
    cfg = load_app_config(args)
    config_path = Path(args.config).expanduser() if args.config else DEFAULT_CONFIG
    print(config_report(cfg, config_path))
    return 0


def config_report(cfg: AppConfig, config_path: Path | None = None) -> str:
    v, b = cfg.vehicle, cfg.bus
    lines: list[str] = []
    add = lines.append
    if config_path is not None:
        add(f"config     {config_path}{'' if config_path.is_file() else '  (missing: defaults)'}")
    add(f"vehicle    {cfg.vehicle_file or '(inline)'}  ->  {v.name}")
    obd = (
        f"OBD on (0x{b.obd_request_id:03X} -> {len(b.obd_response_ids)} response ids"
        f"{', 29-bit' if b.obd_extended_ids else ''}{', multi-PID' if b.obd_multi_pid else ''},"
        f" max {b.obd_max_rate_hz:g}/s)"
        if b.obd_enabled
        else "OBD off"
    )
    add(
        f"bus        {b.interface}:{b.channel} @ {b.bitrate} bit/s | {obd}"
        f" | broadcast signals {'on' if b.passive_enabled else 'off'}"
    )
    s = cfg.server
    add(f"server     http://{s.host}:{s.port}/  websocket {s.ws_rate_hz:g} Hz")
    m = cfg.metrics
    add(f"metrics    tick {m.tick_hz:g} Hz, stale after {m.stale_s:g} s")
    i = cfg.imu
    add(
        f"imu        {'enabled' if i.enabled else 'disabled'}"
        + (
            f" ({i.driver} on i2c-{i.i2c_bus} @ {i.address:#04x}, lon {i.axis_lon} "
            f"lat {i.axis_lat} vert {i.axis_vert})"
            if i.enabled
            else ""
        )
    )
    r = cfg.recording
    add(f"recording  {'on' if r.enabled else 'off'} (dir {r.dir})")
    add("")
    add(
        f"car        {v.transmission}, {v.mass_kg:g} kg, Cd {v.cd:g} x {v.frontal_area_m2:g} m2,"
        f" Crr {v.crr:g}, tyre {v.tire_circumference_m:g} m, final drive {v.final_drive:g},"
        f" driveline eff {v.drivetrain_efficiency:g}"
    )
    add(
        f"rpm        idle {v.idle_rpm:g} | shift light {v.shift_light_rpm:g}"
        f" | redline {v.redline_rpm:g}"
    )
    add("")
    add(
        f"{'gear':>4} {'ratio':>7} {'overall':>8} {'rpm/kph':>8} {'kph@1k':>7} "
        f"{'kph@shift':>9} {'kph@red':>8}"
    )
    for g in range(1, v.gear_count() + 1):
        rpk = v.rpm_per_kph(g)
        if not rpk:
            continue
        ratio = v.gear_ratios[g - 1]
        add(
            f"{g:>4} {ratio:>7.3f} {ratio * v.final_drive:>8.3f} {rpk:>8.2f} {1000 / rpk:>7.1f}"
            f" {v.shift_light_rpm / rpk:>9.1f} {v.redline_rpm / rpk:>8.1f}"
        )
    add("")
    by_rate: dict[str, list[str]] = {"fast": [], "medium": [], "slow": []}
    for req in v.pids:
        by_rate[req.rate].append(f"{req.pid:02X}")
    add(
        f"PIDs ({len(v.pids)})  "
        + " | ".join(f"{rate}: {' '.join(pids) or '-'}" for rate, pids in by_rate.items())
    )
    add(f"signals ({len(v.signals)})")
    for sig in v.signals:
        ident = f"0x{sig.id:08X}" if sig.extended else f"0x{sig.id:03X}"
        add(
            f"  {sig.name:<26} {ident:>10} bit {sig.start_bit:>2} len {sig.length:>2}"
            f" {'BE' if sig.byte_order == 'big_endian' else 'LE'}{' signed' if sig.signed else ''}"
            f" x{sig.scale:g}{f' {sig.offset:+g}' if sig.offset else ''} {sig.unit}".rstrip()
        )
    warnings = config_warnings(cfg)
    add("")
    if warnings:
        add(f"warnings ({len(warnings)})")
        lines.extend(f"  - {w}" for w in warnings)
    else:
        add("no warnings")
    return "\n".join(lines)


def config_warnings(cfg: AppConfig) -> list[str]:
    """Plausibility checks a typo in a vehicle file would otherwise only reveal on the road."""
    v = cfg.vehicle
    out: list[str] = []
    ratios = v.gear_ratios
    if any(a <= b for a, b in zip(ratios, ratios[1:], strict=False)):
        out.append(f"gear_ratios should decrease from 1st gear up: {ratios}")
    for g in range(1, len(ratios)):
        lo, hi = v.rpm_per_kph(g + 1), v.rpm_per_kph(g)
        if lo and hi and lo > 0 and 1.0 < hi / lo < GEAR_SEPARATION_MIN:
            out.append(
                f"gears {g} and {g + 1} are within {100 * (GEAR_SEPARATION_MIN - 1):.0f} % of each"
                " other; the ratio-based gear estimate may flicker between them"
            )
    if not v.idle_rpm < v.shift_light_rpm <= v.redline_rpm:
        out.append(
            f"expected idle_rpm < shift_light_rpm <= redline_rpm, got {v.idle_rpm:g} /"
            f" {v.shift_light_rpm:g} / {v.redline_rpm:g}"
        )
    known = set(Sig.all())
    seen: set[tuple[int, str]] = set()
    for sig in v.signals:
        if sig.name not in known:
            out.append(f"signal {sig.name!r} is not a known Sig name: decoded but unused")
        key = (sig.id, sig.name)
        if key in seen:
            out.append(f"signal {sig.name!r} on 0x{sig.id:X} is defined twice")
        seen.add(key)
    pids = [req.pid for req in v.pids]
    dupes = sorted({p for p in pids if pids.count(p) > 1})
    if dupes:
        out.append("PIDs listed more than once: " + " ".join(f"{p:02X}" for p in dupes))
    try:
        from obddash.bus.obd import PID_TABLE
    except ImportError:  # pragma: no cover - bus package missing
        PID_TABLE = None
    if PID_TABLE is not None:
        missing = [p for p in pids if p not in PID_TABLE]
        if missing:
            out.append(
                "PIDs without a decoder (will not be polled): "
                + " ".join(f"{p:02X}" for p in missing)
            )
    if (
        cfg.imu.enabled
        and cfg.imu.driver == "mpu6050"
        and importlib.util.find_spec("smbus2") is None
    ):
        out.append("imu.enabled but smbus2 is not installed (pip install 'obddash[imu]')")
    if cfg.bus.interface == "virtual" and not cfg.bus.obd_enabled and not v.signals:
        out.append("virtual bus with OBD off and no signals: nothing will be read")
    return out


# --------------------------------------------------------------------------------------
# entry point
# --------------------------------------------------------------------------------------
def _setup_logging(verbose: int, *, server: bool) -> None:
    """Server commands log at INFO (it is their only status output), tools at
    WARNING so tables stay readable; each -v goes one level further down."""
    base = logging.INFO if server else logging.WARNING
    level = max(logging.DEBUG, base - 10 * max(0, verbose))
    root = logging.getLogger()
    if root.handlers:  # embedded (or under pytest): leave the host's logging alone
        return
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    func = getattr(args, "func", None)
    if func is None:
        parser.print_help()
        return 2
    _setup_logging(args.verbose, server=bool(getattr(args, "server", False)))
    try:
        return int(func(args) or 0)
    except CliError as exc:
        print(f"obddash: error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
