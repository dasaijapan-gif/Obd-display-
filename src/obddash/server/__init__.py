"""obddash.server: the runtime that wires source -> metrics -> clients, the FastAPI app
(UI, REST, WebSocket) and the JSONL recorder / replayer.

::

    runtime = Runtime(cfg, source="sim")
    app = create_app(runtime)          # lifespan: runtime.start(); runtime.run(); runtime.stop()
    uvicorn.run(app, host=cfg.server.host, port=cfg.server.port)
"""

from obddash.server.app import config_dict, create_app
from obddash.server.recorder import (
    Recorder,
    Replayer,
    iter_recording,
    make_recording_path,
    read_meta,
    read_recording,
)
from obddash.server.runtime import SOURCES, Runtime, RuntimeStats

__all__ = [
    "SOURCES",
    "Recorder",
    "Replayer",
    "Runtime",
    "RuntimeStats",
    "config_dict",
    "create_app",
    "iter_recording",
    "make_recording_path",
    "read_meta",
    "read_recording",
]
