"""Tests for the obddash command line (obddash.cli).  No web server is started:
``serve`` / ``uvicorn.run`` are replaced where a server command is exercised."""

from __future__ import annotations

import subprocess
import sys

import pytest

from obddash import __version__, cli
from obddash.config import CONFIG_DIR, AppConfig, load_vehicle
from obddash.model import RawState, Sig
from obddash.server.recorder import Recorder

SIM_VEHICLE = CONFIG_DIR / "vehicles" / "sim_sportscar.yaml"


def parse(*argv: str):
    return cli.build_parser().parse_args(list(argv))


@pytest.fixture
def served(monkeypatch):
    """Replace cli.serve; returns the list of (cfg, source, kwargs) it was called with."""
    calls: list[tuple[AppConfig, str, dict]] = []

    def fake_serve(cfg, *, source, verbose=0, **kwargs):
        calls.append((cfg, source, dict(kwargs, verbose=verbose)))
        return 0

    monkeypatch.setattr(cli, "serve", fake_serve)
    return calls


# --------------------------------------------------------------------------------------
# argument parsing
# --------------------------------------------------------------------------------------
def test_parse_run():
    a = parse(
        "run",
        "--interface",
        "slcan",
        "--channel",
        "/dev/ttyACM0",
        "--bitrate",
        "250000",
        "--record",
        "--no-obd",
    )
    assert a.func is cli.cmd_run and a.server
    assert (a.interface, a.channel, a.bitrate) == ("slcan", "/dev/ttyACM0", 250000)
    assert a.record and a.no_obd
    b = parse("run")
    assert (b.interface, b.channel, b.bitrate, b.record, b.no_obd) == (
        None,
        None,
        None,
        False,
        False,
    )


def test_parse_sim_and_demo_alias():
    a = parse("sim")
    assert a.func is cli.cmd_sim
    assert (a.script, a.seed, a.record) == ("lap", 1, False)
    b = parse("demo", "--script", "dyno", "--seed", "7", "--record")
    assert b.func is cli.cmd_sim
    assert (b.script, b.seed, b.record) == ("dyno", 7, True)
    with pytest.raises(SystemExit):
        parse("sim", "--script", "moon")


def test_parse_replay():
    a = parse("replay", "drive.jsonl", "--speed", "2.5", "--loop")
    assert a.func is cli.cmd_replay
    assert (a.file, a.speed, a.loop) == ("drive.jsonl", 2.5, True)
    assert parse("replay", "x.jsonl").speed == 1.0
    with pytest.raises(SystemExit):
        parse("replay", "x.jsonl", "--speed", "0")
    with pytest.raises(SystemExit):
        parse("replay")


def test_parse_scan_and_pids():
    a = parse("scan", "--sim", "--duration", "3", "--diff", "--interface", "virtual")
    assert a.func is cli.cmd_scan and not a.server
    assert (a.sim, a.duration, a.diff, a.interface, a.interval) == (True, 3.0, True, "virtual", 0.5)
    b = parse("pids")
    assert b.func is cli.cmd_pids
    assert (b.seconds, b.sim) == (5.0, False)
    assert parse("pids", "--seconds", "2", "--sim", "--channel", "can1").channel == "can1"


def test_parse_config_check():
    a = parse("config-check")
    assert a.func is cli.cmd_config_check


def test_common_options_before_or_after_the_subcommand():
    a = parse("-v", "--port", "9000", "-c", "my.yaml", "sim")
    assert (a.verbose, a.port, a.config, a.host, a.vehicle) == (1, 9000, "my.yaml", None, None)
    b = parse("sim", "-vv", "--host", "127.0.0.1", "--vehicle", "car.yaml")
    assert (b.verbose, b.host, b.vehicle, b.port, b.config) == (
        2,
        "127.0.0.1",
        "car.yaml",
        None,
        None,
    )
    c = parse("--port", "9000", "config-check", "--host", "::1")
    assert (c.port, c.host) == (9000, "::1")


def test_no_command_prints_help(capsys):
    assert cli.main([]) == 2
    assert "COMMAND" in capsys.readouterr().out


def test_version(capsys):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--version"])
    assert exc.value.code == 0
    assert __version__ in capsys.readouterr().out


# --------------------------------------------------------------------------------------
# config-check and config errors
# --------------------------------------------------------------------------------------
def test_config_check_default(capsys):
    assert cli.main(["config-check"]) == 0
    out = capsys.readouterr().out
    assert "rpm/kph" in out
    assert "default.yaml" in out


def test_config_check_sim_vehicle_gear_table(capsys):
    assert cli.main(["config-check", "--vehicle", str(SIM_VEHICLE)]) == 0
    out = capsys.readouterr().out
    v = load_vehicle(SIM_VEHICLE)
    assert v.name in out
    for g in range(1, v.gear_count() + 1):
        assert f"{v.rpm_per_kph(g):.2f}" in out
    assert "0x203" in out  # signals listed
    assert "no warnings" in out


def test_config_check_vehicle_relative_to_cwd(tmp_path, monkeypatch, capsys):
    (tmp_path / "mycar.yaml").write_text("name: My Car\ngear_ratios: [3.0, 2.0, 1.0]\n")
    monkeypatch.chdir(tmp_path)
    assert cli.main(["config-check", "--vehicle", "mycar.yaml"]) == 0
    assert "My Car" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("content", "message"),
    [
        (None, "config file not found"),
        ("bus: [unclosed\n", "invalid YAML"),
        ("server: {port: not-a-number}\n", "invalid configuration"),
        ("vehicle_file: no/such/car.yaml\n", "file not found"),
    ],
)
def test_config_errors_exit_2(tmp_path, capsys, content, message):
    path = tmp_path / "app.yaml"
    if content is not None:
        path.write_text(content)
    assert cli.main(["-c", str(path), "config-check"]) == 2
    assert message in capsys.readouterr().err


def test_config_warnings():
    cfg = AppConfig()
    cfg.vehicle = cfg.vehicle.model_validate(
        {
            "gear_ratios": [3.0, 2.9, 2.0, 2.2],
            "idle_rpm": 800,
            "shift_light_rpm": 7000,
            "redline_rpm": 6500,
            "signals": [
                {"name": "brake_pressure_bar", "id": 0x200, "byte": 0},
                {"name": "brake_pressure_bar", "id": 0x200, "byte": 0},
                {"name": "mystery", "id": 0x300, "byte": 1},
            ],
            "pids": [{"pid": 0x0C}, {"pid": 0x0C}, {"pid": 0xA6}],
        }
    )
    text = "\n".join(cli.config_warnings(cfg))
    assert "should decrease" in text
    assert "gears 1 and 2" in text
    assert "shift_light_rpm" in text
    assert "'mystery'" in text
    assert "defined twice" in text
    assert "more than once: 0C" in text
    assert "without a decoder" in text and "A6" in text


# --------------------------------------------------------------------------------------
# server commands -> runtime arguments (serve replaced)
# --------------------------------------------------------------------------------------
def test_run_maps_options_to_config(served):
    assert (
        cli.main(
            [
                "run",
                "--interface",
                "virtual",
                "--channel",
                "x",
                "--bitrate",
                "250000",
                "--no-obd",
                "--record",
                "--port",
                "9001",
                "--host",
                "127.0.0.1",
                "-v",
            ]
        )
        == 0
    )
    cfg, source, kw = served[0]
    assert source == "can"
    assert (cfg.bus.interface, cfg.bus.channel, cfg.bus.bitrate) == ("virtual", "x", 250000)
    assert cfg.bus.obd_enabled is False
    assert cfg.recording.enabled is True
    assert (cfg.server.host, cfg.server.port) == ("127.0.0.1", 9001)
    assert kw["verbose"] == 1


def test_sim_uses_sim_vehicle_by_default(served):
    assert cli.main(["sim", "--script", "dyno", "--seed", "4"]) == 0
    assert cli.main(["demo"]) == 0
    (cfg, source, kw), (_cfg2, source2, kw2) = served
    assert source == source2 == "sim"
    assert cfg.vehicle.name == load_vehicle(SIM_VEHICLE).name
    assert (kw["sim_script"], kw["sim_seed"]) == ("dyno", 4)
    assert (kw2["sim_script"], kw2["sim_seed"]) == ("lap", 1)
    assert cfg.recording.enabled is False


def test_replay_passes_file_and_picks_recorded_vehicle(served, tmp_path):
    path = tmp_path / "drive.jsonl"
    raw = RawState()
    with Recorder(
        path, meta={"vehicle": "Sim Sportscar (6MT)", "vehicle_file": str(SIM_VEHICLE)}
    ) as rec:
        raw.set(Sig.RPM, 900.0)
        rec.sample(raw, 0.0)
    assert cli.main(["replay", str(path), "--speed", "4", "--loop"]) == 0
    cfg, source, kw = served[0]
    assert source == "replay"
    assert kw["replay_path"] == path
    assert (kw["replay_speed"], kw["replay_loop"]) == (4.0, True)
    assert cfg.vehicle.name == "Sim Sportscar (6MT)"


def test_replay_missing_file_exits_2(served, capsys):
    assert cli.main(["replay", "/no/such/file.jsonl"]) == 2
    assert "recording not found" in capsys.readouterr().err
    assert served == []


def test_serve_builds_runtime_and_calls_uvicorn(monkeypatch, capsys):
    import uvicorn

    from obddash.server import runtime as runtime_mod

    class NullEngine:
        def set_connection(self, conn): ...
        def tick(self, raw, t): ...
        def command(self, cmd, args=None):
            return False

        def drain_events(self):
            return []

    seen = {}

    def fake_run(app, **kwargs):
        seen["app"] = app
        seen.update(kwargs)

    monkeypatch.setattr(runtime_mod, "_default_engine", lambda cfg: NullEngine())
    monkeypatch.setattr(uvicorn, "run", fake_run)
    cfg = AppConfig()
    cfg.server.port = 8123
    assert cli.serve(cfg, source="none", verbose=0) == 0
    assert (seen["host"], seen["port"]) == ("0.0.0.0", 8123)
    assert seen["app"].state.runtime.source == "none"
    assert seen["access_log"] is False and seen["log_config"] is None
    out = capsys.readouterr().out.strip().splitlines()
    assert len(out) == 1
    assert out[0].startswith(f"obddash {__version__} | no data source | ")
    assert "http://localhost:8123/" in out[0]


def test_banner_and_urls():
    cfg = AppConfig()
    assert cli._urls("127.0.0.1", 8080) == ("http://127.0.0.1:8080/", None)
    assert cli._urls("::1", 8080) == ("http://[::1]:8080/", None)
    local, _lan = cli._urls("0.0.0.0", 8080)
    assert local == "http://localhost:8080/"
    b = cli.banner(
        cfg, "sim", {"sim_script": "dyno", "sim_seed": 2}, local, "http://10.0.0.5:8080/"
    )
    assert "simulator (dyno, seed 2)" in b and "LAN http://10.0.0.5:8080/" in b
    assert "CAN socketcan:can0" in cli.banner(cfg, "can", {}, local)
    r = cli.banner(
        cfg,
        "replay",
        {"replay_path": "/x/drive.jsonl", "replay_speed": 2.0, "replay_loop": True},
        local,
    )
    assert "replay drive.jsonl x2 (loop)" in r


# --------------------------------------------------------------------------------------
# bus tools against the built-in simulator (short real runs)
# --------------------------------------------------------------------------------------
def test_scan_sim_prints_the_id_table(capsys):
    assert cli.main(["scan", "--sim", "--duration", "0.6", "--interval", "0.3"]) == 0
    out = capsys.readouterr().out
    assert "scanning simulator" in out
    assert "200" in out and "203" in out  # the sim's brake and gear frames


def test_pids_sim_reports_supported_pids_and_vin(capsys):
    assert cli.main(["pids", "--sim", "--seconds", "1.5"]) == 0
    out = capsys.readouterr().out
    assert "VIN: SIM0BDDASH0000001" in out
    assert "0x0C  rpm" in out
    assert "speed_kph" in out


def test_pids_bad_interface_exits_2(capsys):
    rc = cli.main(
        ["pids", "--interface", "socketcan", "--channel", "obddash_no_such_if0", "--seconds", "0.1"]
    )
    assert rc == 2
    assert "cannot open CAN" in capsys.readouterr().err


def test_python_dash_m_entry_point():
    r = subprocess.run(
        [sys.executable, "-m", "obddash", "config-check"],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert r.returncode == 0, r.stderr
    assert "rpm/kph" in r.stdout
