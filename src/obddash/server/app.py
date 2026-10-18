"""FastAPI app: the schematic UI, a small REST API and the ``/ws`` telemetry stream.

The app is a thin shell over a :class:`~obddash.server.runtime.Runtime`; it never
computes anything itself.  What it needs from the runtime (a test fake only has to
provide these)::

    runtime.cfg: AppConfig            runtime.source: str
    runtime.start() / runtime.stop()  async runtime.run()
    runtime.snapshot_dict() -> dict   runtime.health() -> dict
    runtime.command(cmd, args) -> bool
    runtime.subscribe() -> asyncio.Queue[str]  /  runtime.unsubscribe(q)

Routes
------
* ``GET /`` - ``index.html`` with ``Cache-Control: no-store`` so a kiosk reload always
  picks up a new UI.  The page loads ``css/...`` and ``js/...`` by relative URL, so
  the web directory is mounted at ``/`` (after every API route, so it never shadows
  one) and, for explicit references, at ``/static``.  Assets are sent with
  ``Cache-Control: no-cache``: the browser keeps them but revalidates (cheap 304s).
* ``GET /api/health``, ``GET /api/state``, ``GET /api/config`` (ids as hex strings),
  ``GET /api/dyno/runs``, ``POST /api/cmd`` ``{"cmd": ..., "args": {}}`` -> ``{"ok": ...}``.
* ``WS /ws`` - sends ``{"type": "hello", ...}`` and the current snapshot right away,
  then whatever the runtime publishes (``telemetry`` / ``event`` messages); accepts
  ``{"cmd": ..., "args": {...}}`` and answers ``{"type": "ack", "cmd": ..., "ok": ...}``.

Every handler is ``async def`` so it runs on the event-loop thread - the same thread
as the runtime's tick loop - and ``engine.command()`` can never race ``engine.tick()``.
CORS is open to every origin for development convenience (on the Pi the UI is
same-origin anyway).
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
from collections.abc import AsyncIterator
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse, Response
from pydantic import BaseModel, Field
from starlette.staticfiles import StaticFiles
from starlette.types import Scope

import obddash
from obddash.config import AppConfig

log = logging.getLogger(__name__)

DEFAULT_WEB_DIR = Path(obddash.__file__).resolve().parent / "web"
_JSON_SEPARATORS = (",", ":")


class CommandBody(BaseModel):
    cmd: str
    args: dict[str, Any] = Field(default_factory=dict)


class _UiStaticFiles(StaticFiles):
    """StaticFiles that makes browsers revalidate (ETag / Last-Modified -> 304)
    instead of trusting a heuristic cache lifetime after a UI update."""

    async def get_response(self, path: str, scope: Scope) -> Response:
        response = await super().get_response(path, scope)
        response.headers.setdefault("cache-control", "no-cache")
        return response


# --------------------------------------------------------------------------------------
# /api/config
# --------------------------------------------------------------------------------------
def _hex(value: Any, width: int) -> Any:
    return (
        f"0x{value:0{width}X}" if isinstance(value, int) and not isinstance(value, bool) else value
    )


def config_dict(cfg: AppConfig) -> dict[str, Any]:
    """``cfg.model_dump()`` with CAN ids, PIDs and the I2C address as hex strings
    (``"0x7DF"``, ``"0x0C"``), the way they are written in the YAML files."""
    d = cfg.model_dump(mode="json")
    bus = d.get("bus", {})
    ext = bool(bus.get("obd_extended_ids"))
    id_width = 8 if ext else 3
    if "obd_request_id" in bus:
        bus["obd_request_id"] = _hex(bus["obd_request_id"], id_width)
    if "obd_response_ids" in bus:
        bus["obd_response_ids"] = [_hex(x, id_width) for x in bus["obd_response_ids"]]
    imu = d.get("imu", {})
    if "address" in imu:
        imu["address"] = _hex(imu["address"], 2)
    veh = d.get("vehicle", {})
    for sig in veh.get("signals", []):
        sig["id"] = _hex(sig.get("id"), 8 if sig.get("extended") else 3)
    for pid in veh.get("pids", []):
        pid["pid"] = _hex(pid.get("pid"), 2)
    return d


# --------------------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------------------
def _run_command(runtime: Any, cmd: str, args: dict[str, Any] | None) -> bool:
    try:
        return bool(runtime.command(cmd, args or {}))
    except Exception:
        log.exception("command %r raised", cmd)
        return False


def _ack(cmd: Any, ok: bool, error: str | None = None) -> str:
    msg: dict[str, Any] = {"type": "ack", "cmd": cmd, "ok": ok}
    if error:
        msg["error"] = error
    return json.dumps(msg, separators=_JSON_SEPARATORS)


def handle_ws_command(runtime: Any, text: str) -> str:
    """Parse one client message, run it, and return the ack message (JSON text)."""
    try:
        payload = json.loads(text)
    except ValueError:
        return _ack(None, False, "invalid JSON")
    if not isinstance(payload, dict) or not isinstance(payload.get("cmd"), str):
        return _ack(None, False, 'expected {"cmd": "...", "args": {...}}')
    cmd = payload["cmd"]
    args = payload.get("args") or {}
    if not isinstance(args, dict):
        return _ack(cmd, False, "args must be an object")
    return _ack(cmd, _run_command(runtime, cmd, args))


def _is_disconnect(exc: BaseException) -> bool:
    # Starlette raises WebSocketDisconnect (also for OSError on send); a send after the
    # close handshake raises RuntimeError.  All of these just mean "client went away".
    return isinstance(exc, WebSocketDisconnect | OSError | RuntimeError)


# --------------------------------------------------------------------------------------
# app factory
# --------------------------------------------------------------------------------------
def create_app(runtime: Any, *, web_dir: Path | str | None = None) -> FastAPI:
    """Build the FastAPI app around ``runtime``.  The lifespan starts the runtime and
    runs its loops as a task; shutdown cancels the task and stops the runtime."""
    web_dir = Path(web_dir) if web_dir is not None else DEFAULT_WEB_DIR
    index_path = web_dir / "index.html"

    @contextlib.asynccontextmanager
    async def lifespan(_app: FastAPI) -> AsyncIterator[None]:
        runtime.start()
        task = asyncio.create_task(runtime.run(), name="obddash-runtime")
        task.add_done_callback(_log_runtime_exit)
        try:
            yield
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
            runtime.stop()

    app = FastAPI(
        title="obddash",
        version=obddash.__version__,
        lifespan=lifespan,
        docs_url="/api/docs",
        redoc_url=None,
        openapi_url="/api/openapi.json",
    )
    app.state.runtime = runtime
    app.add_middleware(
        CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
    )

    # -- UI -------------------------------------------------------------------------
    @app.get("/", include_in_schema=False)
    async def index() -> Response:
        if not index_path.is_file():
            return PlainTextResponse(f"obddash web UI not found in {web_dir}", status_code=404)
        return FileResponse(
            index_path, media_type="text/html", headers={"Cache-Control": "no-store"}
        )

    # -- REST -----------------------------------------------------------------------
    @app.get("/api/health")
    async def health() -> JSONResponse:
        return JSONResponse(runtime.health())

    @app.get("/api/state")
    async def state() -> JSONResponse:
        return JSONResponse(runtime.snapshot_dict())

    @app.get("/api/config")
    async def config() -> JSONResponse:
        out = {"version": obddash.__version__, "source": runtime.source}
        out.update(config_dict(runtime.cfg))
        return JSONResponse(out)

    @app.get("/api/dyno/runs")
    async def dyno_runs() -> JSONResponse:
        dyno = runtime.snapshot_dict().get("dyno") or {}
        return JSONResponse({"runs": dyno.get("runs") or []})

    @app.post("/api/cmd")
    async def command(body: CommandBody) -> JSONResponse:
        return JSONResponse({"cmd": body.cmd, "ok": _run_command(runtime, body.cmd, body.args)})

    # -- WebSocket ------------------------------------------------------------------
    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        queue = runtime.subscribe()
        send_lock = asyncio.Lock()  # the pump and the acks share one socket

        async def send(text: str) -> None:
            async with send_lock:
                await ws.send_text(text)

        async def pump() -> None:
            while True:
                await send(await queue.get())

        async def listen() -> None:
            while True:
                msg = await ws.receive()
                if msg["type"] == "websocket.disconnect":
                    return
                text = msg.get("text")
                if text is None and msg.get("bytes") is not None:
                    text = msg["bytes"].decode("utf-8", "replace")
                if text:
                    await send(handle_ws_command(runtime, text))

        tasks: list[asyncio.Task[None]] = []
        try:
            hello = {
                "type": "hello",
                "version": obddash.__version__,
                "source": runtime.source,
                "vehicle": runtime.cfg.vehicle.name,
            }
            await send(json.dumps(hello, separators=_JSON_SEPARATORS))
            first = {"type": "telemetry", "data": runtime.snapshot_dict()}
            await send(json.dumps(first, separators=_JSON_SEPARATORS))
            tasks = [
                asyncio.create_task(pump(), name="ws-pump"),
                asyncio.create_task(listen(), name="ws-listen"),
            ]
            done, _pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for t in done:
                exc = None if t.cancelled() else t.exception()
                if exc is not None and not _is_disconnect(exc):
                    log.warning("websocket client error: %r", exc)
        except Exception as exc:
            if not _is_disconnect(exc):
                log.warning("websocket handler error: %r", exc)
        finally:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            runtime.unsubscribe(queue)

    # -- static files: last, so they never shadow an API route ----------------------
    if web_dir.is_dir():
        app.mount("/static", _UiStaticFiles(directory=web_dir), name="static")
        app.mount("/", _UiStaticFiles(directory=web_dir), name="web")
    else:
        log.warning("web UI directory %s not found; serving the API only", web_dir)

    return app


def _log_runtime_exit(task: asyncio.Task[None]) -> None:
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        log.error("runtime loop crashed: %r", exc, exc_info=exc)
