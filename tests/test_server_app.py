"""Tests for the FastAPI app (obddash.server.app): UI files, REST, WebSocket."""

from __future__ import annotations

import asyncio
import json
import logging
import time

import httpx
import pytest
from fastapi.testclient import TestClient

import obddash
from obddash.config import CONFIG_DIR, AppConfig, load_vehicle
from obddash.model import Connection, DynoRun, Event, Sig, Telemetry
from obddash.server.app import config_dict, create_app, handle_ws_command
from obddash.server.runtime import Runtime

KNOWN_COMMANDS = {"dyno.arm", "dyno.cancel", "dyno.clear", "session.reset"}
INDEX_HTML = "<!doctype html><title>stub dash</title><script src='js/app.js'></script>"


def _offer(q: asyncio.Queue, msg: str) -> None:
    if q.full():
        q.get_nowait()
    q.put_nowait(msg)


class FakeRuntime:
    """Just enough Runtime for the app: canned telemetry, no threads.  ``run()``
    publishes a telemetry frame every 20 ms and an event every 5th frame."""

    def __init__(self, cfg: AppConfig):
        self.cfg = cfg
        self.source = "sim"
        self.started = False
        self.stopped = False
        self.running = False
        self.commands: list[tuple[str, dict]] = []
        self.subs: set[asyncio.Queue] = set()
        tel = Telemetry(
            t=12.5,
            ts=1760000000.0,
            conn=Connection(bus=True, ecu=True, source="sim", pid_hz=48.0, frame_hz=210.0),
        )
        tel.veh.rpm = 4210.0
        tel.veh.gear = 3
        tel.veh.lambda_ = 1.0
        tel.dyno.runs = [
            DynoRun(
                id=1,
                t=3.0,
                gear=3,
                rpm_start=2500,
                rpm_end=7000,
                peak_power_kw=128.4,
                peak_power_rpm=6900,
                peak_torque_nm=201.0,
                peak_torque_rpm=4300,
                correction=1.01,
            )
        ]
        self.telemetry = tel

    def start(self):
        self.started = True

    async def run(self):
        self.running = True
        n = 0
        try:
            while True:
                n += 1
                tel = json.dumps({"type": "telemetry", "data": self.snapshot_dict()})
                ev = json.dumps({"type": "event", "name": "dyno.done", "data": {"id": 1}, "t": 1.0})
                for q in list(self.subs):
                    if n % 5 == 0:
                        _offer(q, ev)
                    _offer(q, tel)
                await asyncio.sleep(0.02)
        finally:
            self.running = False

    def stop(self):
        self.stopped = True

    def snapshot_dict(self):
        return self.telemetry.to_dict()

    def health(self):
        return {
            "ok": True,
            "source": "sim",
            "bus": True,
            "ecu": True,
            "imu": False,
            "pid_hz": 48.0,
            "uptime_s": 1.0,
        }

    def command(self, cmd, args=None):
        self.commands.append((cmd, args))
        return cmd in KNOWN_COMMANDS

    def subscribe(self):
        q: asyncio.Queue = asyncio.Queue(maxsize=2)
        self.subs.add(q)
        return q

    def unsubscribe(self, q):
        self.subs.discard(q)


@pytest.fixture(scope="module")
def cfg() -> AppConfig:
    c = AppConfig()
    c.vehicle = load_vehicle(CONFIG_DIR / "vehicles" / "sim_sportscar.yaml")
    return c


@pytest.fixture
def web_dir(tmp_path):
    d = tmp_path / "web"
    (d / "js").mkdir(parents=True)
    (d / "index.html").write_text(INDEX_HTML)
    (d / "js" / "app.js").write_text("console.log('hi');")
    return d


@pytest.fixture
def runtime(cfg):
    return FakeRuntime(cfg)


@pytest.fixture
def app(runtime, web_dir):
    return create_app(runtime, web_dir=web_dir)


@pytest.fixture
async def client(app):
    transport = httpx.ASGITransport(app=app)
    async with httpx.AsyncClient(transport=transport, base_url="http://dash") as c:
        yield c


# --------------------------------------------------------------------------------------
# UI files
# --------------------------------------------------------------------------------------
async def test_index_is_served_uncached(client):
    r = await client.get("/")
    assert r.status_code == 200
    assert r.text == INDEX_HTML
    assert r.headers["content-type"].startswith("text/html")
    assert r.headers["cache-control"] == "no-store"


async def test_static_assets_relative_and_under_static(client):
    for url in ("/js/app.js", "/static/js/app.js"):
        r = await client.get(url)
        assert r.status_code == 200, url
        assert r.text == "console.log('hi');"
        assert r.headers["cache-control"] == "no-cache"
    assert (await client.get("/static/index.html")).status_code == 200
    assert (await client.get("/js/missing.js")).status_code == 404
    assert (await client.get("/api/nothing")).status_code == 404


async def test_missing_index_and_missing_web_dir(runtime, tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    for d in (empty, tmp_path / "does-not-exist"):
        app = create_app(runtime, web_dir=d)
        transport = httpx.ASGITransport(app=app)
        async with httpx.AsyncClient(transport=transport, base_url="http://dash") as c:
            assert (await c.get("/")).status_code == 404
            assert (await c.get("/api/health")).status_code == 200


def test_default_web_dir_serves_the_real_ui(runtime):
    with TestClient(create_app(runtime)) as c:
        r = c.get("/")
        assert r.status_code == 200 and "<canvas" in r.text
        assert c.get("/js/app.js").status_code == 200
        assert c.get("/css/schematic.css").status_code == 200


# --------------------------------------------------------------------------------------
# REST
# --------------------------------------------------------------------------------------
async def test_health(client):
    r = await client.get("/api/health")
    assert r.status_code == 200
    assert r.json() == {
        "ok": True,
        "source": "sim",
        "bus": True,
        "ecu": True,
        "imu": False,
        "pid_hz": 48.0,
        "uptime_s": 1.0,
    }


async def test_state(client):
    data = (await client.get("/api/state")).json()
    assert data["t"] == 12.5
    assert data["veh"]["rpm"] == 4210.0
    assert data["veh"]["gear"] == 3
    assert data["veh"]["lambda"] == 1.0
    assert data["conn"]["source"] == "sim"


async def test_config_ids_are_hex_strings(client, cfg):
    data = (await client.get("/api/config")).json()
    assert data["version"] == obddash.__version__
    assert data["source"] == "sim"
    veh = data["vehicle"]
    # what web/js/app.js applyConfig() reads
    assert veh["name"] == cfg.vehicle.name
    assert veh["redline_rpm"] == 7500
    assert veh["gear_ratios"] == cfg.vehicle.gear_ratios
    assert veh["brake"]["max_pressure_bar"] == 120
    assert veh["transmission"] == "manual"
    assert data["bus"]["obd_request_id"] == "0x7DF"
    assert data["bus"]["obd_response_ids"][:2] == ["0x7E8", "0x7E9"]
    assert data["imu"]["address"] == "0x68"
    assert veh["signals"][0]["id"] == "0x200"
    assert veh["signals"][-1]["id"] == "0x203"
    assert veh["pids"][0]["pid"] == "0x0C"
    assert data["server"]["ws_rate_hz"] == 30


def test_config_dict_extended_ids():
    c = AppConfig()
    c.bus.obd_extended_ids = True
    c.bus.obd_request_id = 0x18DB33F1
    c.bus.obd_response_ids = [0x18DAF110]
    d = config_dict(c)
    assert d["bus"]["obd_request_id"] == "0x18DB33F1"
    assert d["bus"]["obd_response_ids"] == ["0x18DAF110"]


async def test_dyno_runs(client):
    data = (await client.get("/api/dyno/runs")).json()
    assert len(data["runs"]) == 1
    run = data["runs"][0]
    assert run["id"] == 1 and run["peak_power_kw"] == 128.4 and run["curve"] == []


async def test_post_cmd(client, runtime):
    r = await client.post("/api/cmd", json={"cmd": "dyno.arm"})
    assert r.status_code == 200
    assert r.json() == {"cmd": "dyno.arm", "ok": True}
    r = await client.post("/api/cmd", json={"cmd": "warp.drive", "args": {"factor": 9}})
    assert r.json() == {"cmd": "warp.drive", "ok": False}
    assert runtime.commands == [("dyno.arm", {}), ("warp.drive", {"factor": 9})]
    assert (await client.post("/api/cmd", json={"nope": 1})).status_code == 422


async def test_post_cmd_accepts_the_ws_payload_with_null_args(client, runtime):
    """POST /api/cmd takes the same payload as a WS command, including "args": null."""
    r = await client.post("/api/cmd", json={"cmd": "dyno.cancel", "args": None})
    assert r.status_code == 200 and r.json() == {"cmd": "dyno.cancel", "ok": True}
    assert json.loads(handle_ws_command(runtime, '{"cmd": "dyno.cancel", "args": null}'))["ok"]
    assert runtime.commands == [("dyno.cancel", {}), ("dyno.cancel", {})]
    assert (await client.post("/api/cmd", json={"cmd": "x", "args": [1]})).status_code == 422


async def test_cors_allows_any_origin(client):
    r = await client.options(
        "/api/cmd",
        headers={"Origin": "http://laptop:5173", "Access-Control-Request-Method": "POST"},
    )
    assert r.status_code == 200
    assert r.headers["access-control-allow-origin"] in ("*", "http://laptop:5173")
    r = await client.get("/api/state", headers={"Origin": "http://laptop:5173"})
    assert r.headers["access-control-allow-origin"] == "*"


# --------------------------------------------------------------------------------------
# WebSocket
# --------------------------------------------------------------------------------------
def _recv_until(ws, kind: str, limit: int = 100) -> dict:
    for _ in range(limit):
        msg = ws.receive_json()
        if msg["type"] == kind:
            return msg
    raise AssertionError(f"no {kind!r} message within {limit} messages")


def test_ws_hello_stream_and_command(runtime, web_dir):
    app = create_app(runtime, web_dir=web_dir)
    with TestClient(app) as c:
        assert runtime.started
        with c.websocket_connect("/ws") as ws:
            hello = ws.receive_json()
            assert hello["type"] == "hello"
            assert hello["version"] == obddash.__version__
            assert hello["source"] == "sim"
            first = ws.receive_json()  # current snapshot right away
            assert first["type"] == "telemetry"
            assert first["data"]["veh"]["rpm"] == 4210.0
            ev = _recv_until(ws, "event")
            assert ev == {"type": "event", "name": "dyno.done", "data": {"id": 1}, "t": 1.0}
            assert _recv_until(ws, "telemetry")["data"]["t"] == 12.5

            ws.send_json({"cmd": "dyno.arm"})
            assert _recv_until(ws, "ack") == {"type": "ack", "cmd": "dyno.arm", "ok": True}
            ws.send_json({"cmd": "warp.drive", "args": {"factor": 9}})
            assert _recv_until(ws, "ack") == {"type": "ack", "cmd": "warp.drive", "ok": False}
            ws.send_text("this is not json")
            bad = _recv_until(ws, "ack")
            assert bad["ok"] is False and bad["error"] == "invalid JSON"
            ws.send_json({"cmd": "dyno.arm", "args": [1, 2]})
            assert _recv_until(ws, "ack")["error"] == "args must be an object"
        assert runtime.commands == [("dyno.arm", {}), ("warp.drive", {"factor": 9})]
    assert runtime.stopped
    assert not runtime.running  # the lifespan cancelled run()


def test_ws_client_disconnect_is_silent(runtime, web_dir, caplog):
    app = create_app(runtime, web_dir=web_dir)
    with caplog.at_level(logging.WARNING, logger="obddash.server.app"):
        with TestClient(app) as c:
            for _ in range(3):
                with c.websocket_connect("/ws") as ws:
                    ws.receive_json()  # hello, then leave while frames are flowing
            deadline = time.monotonic() + 2.0
            while runtime.subs and time.monotonic() < deadline:
                time.sleep(0.01)
            assert runtime.subs == set()
    assert [r for r in caplog.records if r.name == "obddash.server.app"] == []


def test_ws_binary_frame_command_and_burst_of_commands(runtime, web_dir):
    """A command sent as a binary frame is still acked; a burst of commands is acked
    one for one while telemetry keeps streaming."""
    with TestClient(create_app(runtime, web_dir=web_dir)) as c:
        with c.websocket_connect("/ws") as ws:
            ws.receive_json()  # hello
            ws.send_bytes(b'{"cmd": "session.reset"}')
            assert _recv_until(ws, "ack") == {"type": "ack", "cmd": "session.reset", "ok": True}
            for _ in range(20):
                ws.send_json({"cmd": "dyno.clear"})
            acks = 0
            for _ in range(200):
                if ws.receive_json()["type"] == "ack":
                    acks += 1
                    if acks == 20:
                        break
            assert acks == 20
    assert runtime.commands == [("session.reset", {})] + [("dyno.clear", {})] * 20


async def test_ws_handler_cancelled_mid_cleanup_still_unsubscribes(app, runtime):
    """Servers (and Starlette's TestClient, via anyio cancel scopes) may cancel the
    handler on disconnect / shutdown, and keep cancelling every await after that.
    The client queue must be unsubscribed anyway, or a ghost client stays in the hub
    forever (counted in health, keeping telemetry serialisation busy)."""
    sent: list[dict] = []
    greeted = asyncio.Event()
    connected = False

    async def receive():
        nonlocal connected
        if not connected:
            connected = True
            return {"type": "websocket.connect"}
        await asyncio.Event().wait()  # the client never says anything else

    async def send(message):
        sent.append(message)
        if sum(m["type"] == "websocket.send" for m in sent) >= 2:  # hello + snapshot
            greeted.set()

    scope = {
        "type": "websocket",
        "asgi": {"version": "3.0"},
        "path": "/ws",
        "raw_path": b"/ws",
        "root_path": "",
        "scheme": "ws",
        "query_string": b"",
        "headers": [],
        "client": ("test", 1),
        "server": ("dash", 80),
        "subprotocols": [],
        "state": {},
    }
    handler = asyncio.create_task(app(scope, receive, send))
    await asyncio.wait_for(greeted.wait(), 2.0)
    assert len(runtime.subs) == 1
    handler.cancel()  # first cancel: lands in the pump/listen wait
    await asyncio.sleep(0)  # handler is now inside its cleanup awaiting the children
    handler.cancel()  # level-triggered cancellation: the cleanup await is cancelled too
    await asyncio.gather(handler, return_exceptions=True)
    assert runtime.subs == set()


def test_handle_ws_command_shapes(runtime):
    assert json.loads(handle_ws_command(runtime, '{"cmd": "session.reset"}')) == {
        "type": "ack",
        "cmd": "session.reset",
        "ok": True,
    }
    assert json.loads(handle_ws_command(runtime, "[1, 2]"))["ok"] is False
    assert json.loads(handle_ws_command(runtime, '{"cmd": 5}'))["ok"] is False

    class Exploding(FakeRuntime):
        def command(self, cmd, args=None):
            raise RuntimeError("engine fell over")

    assert json.loads(handle_ws_command(Exploding(runtime.cfg), '{"cmd": "dyno.arm"}')) == {
        "type": "ack",
        "cmd": "dyno.arm",
        "ok": False,
    }


# --------------------------------------------------------------------------------------
# the real Runtime behind the app
# --------------------------------------------------------------------------------------
class EchoEngine:
    def __init__(self):
        self.n = 0
        self.conn = None
        self.commands: list[tuple[str, dict]] = []
        self.pending: list[Event] = []

    def set_connection(self, conn):
        self.conn = conn

    def tick(self, raw, t):
        self.n += 1
        tel = Telemetry(t=float(self.n), conn=self.conn)
        tel.veh.speed_kph = raw.get(Sig.SPEED_KPH, 0.0) or 0.0
        return tel

    def command(self, cmd, args=None):
        self.commands.append((cmd, args))
        if cmd == "dyno.arm":
            self.pending.append(Event("dyno.started", {"gear": 3}, t=float(self.n)))
        return True

    def drain_events(self):
        out, self.pending = self.pending, []
        return out


def test_real_runtime_through_the_app(cfg, web_dir):
    engine = EchoEngine()
    rt = Runtime(cfg, source="none", engine=engine)
    rt.raw.set(Sig.SPEED_KPH, 87.5)
    with TestClient(create_app(rt, web_dir=web_dir)) as c:
        with c.websocket_connect("/ws") as ws:
            assert ws.receive_json()["type"] == "hello"
            ts = [ws.receive_json()["data"]["t"] for _ in range(4)]
            assert ts == sorted(ts) and ts[-1] > ts[0]  # a live, advancing stream
            ws.send_json({"cmd": "dyno.arm"})
            ev = _recv_until(ws, "event")
            assert ev["name"] == "dyno.started" and ev["data"] == {"gear": 3}
        assert engine.commands == [("dyno.arm", {})]
        h = c.get("/api/health").json()
        assert h["ok"] is True and h["source"] == "none" and h["ticks"] > 0
        assert c.get("/api/state").json()["veh"]["speed_kph"] == 87.5
        assert c.post("/api/cmd", json={"cmd": "session.reset"}).json()["ok"] is True
    assert not rt.running
