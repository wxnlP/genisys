"""
AIMRT runtime worker — runs the AIMRT session in a separate process.

Launched by ``monitor.core.aimrt_client``. Decoded messages are streamed to
the parent over the fd given by ``--data-fd``::

    record := uint32_be length | pickle((kind, payload))

    ("ready", {"topics": [...]})   subscriptions established
    ("data",  {…message dict…})    one decoded message
    ("error", "message")           fatal problem, worker exits

The worker is always killed, never asked to shut down: aimrt_py 1.7.0 crashes
in ``Core.Shutdown()`` as soon as a channel subscriber with a Python callback
has been declared, and that path cannot be avoided from Python.
"""

from __future__ import annotations

import argparse
import logging
import os
import pickle
import struct
import sys
import threading
from typing import Optional

import yaml

from monitor.core.msg_decode import MSGTYPE_BY_SOURCE, message_to_dict

logger = logging.getLogger("monitor.aimrt_worker")

# Must match the module name declared in aimrt.module.modules and the section
# holding this program's settings in the AIMRT config.
_MODULE_NAME = "MonitorSubscriberPyModule"


# Flush the pending buffer past this size so a stalled reader cannot block the
# AIMRT dispatch thread.
_MAX_PENDING_BYTES = 4 << 20


# ---------------------------------------------------------------------------
# Pipe transport
# ---------------------------------------------------------------------------

class PipeWriter:
    """Non-blocking, drop-oldest writer of length-prefixed pickles."""

    def __init__(self, fd: int) -> None:
        self._fd = fd
        self._buf = bytearray()
        self._lock = threading.Lock()
        self.dropped = 0
        try:
            os.set_blocking(fd, False)
        except OSError:
            pass

    def send(self, kind: str, payload: object) -> None:
        blob = pickle.dumps((kind, payload), protocol=pickle.HIGHEST_PROTOCOL)
        record = struct.pack(">I", len(blob)) + blob
        with self._lock:
            if len(self._buf) + len(record) > _MAX_PENDING_BYTES:
                # Reader is behind; drop at a record boundary to keep framing valid.
                self._buf.clear()
                self.dropped += 1
            self._buf += record
            self._flush_locked()

    def _flush_locked(self) -> None:
        while self._buf:
            try:
                n = os.write(self._fd, bytes(self._buf[:65536]))
            except BlockingIOError:
                return
            except OSError:
                self._buf.clear()
                return
            del self._buf[:n]


# ---------------------------------------------------------------------------
# Worker main
# ---------------------------------------------------------------------------

def _make_callback(source: str, writer: PipeWriter):
    counters = {"n": 0}

    def _callback(msg) -> None:
        try:
            result = message_to_dict(source, msg)
            if result is None:
                return
            result["source"] = source
            counters["n"] += 1
            writer.send("data", result)
        except Exception as exc:  # pragma: no cover - defensive
            print(f"callback error ({source}): {exc}", file=sys.stderr, flush=True)

    return _callback


def _read_module_topics(module) -> dict:
    """Read this module's ``topics`` from the AIMRT config.

    AIMRT writes the module's own config section to a file and exposes the path
    through the configurator, so the worker needs no knowledge of the main
    config file beyond what it was started with.
    """
    try:
        path = module.GetConfigurator().GetConfigFilePath()
    except Exception as exc:
        print(f"cannot read module config: {exc}", file=sys.stderr, flush=True)
        return {}
    if not path or not os.path.isfile(path):
        return {}
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    raw = data.get("topics")
    if not isinstance(raw, dict):
        return {}
    return {str(k): str(v) for k, v in raw.items()}


def run(data_fd: int, cfg_path: str) -> int:
    writer = PipeWriter(data_fd)
    cfg_path = os.path.abspath(cfg_path)
    if not os.path.isfile(cfg_path):
        message = f"AIMRT config not found: {cfg_path}"
        print(message, file=sys.stderr, flush=True)
        writer.send("error", message)
        return 2

    print(f"aimrt worker: cfg={cfg_path}", file=sys.stderr, flush=True)

    import aimrt_py

    try:
        core = aimrt_py.Core()
        opts = aimrt_py.CoreOptions()
        opts.cfg_file_path = cfg_path
        core.Initialize(opts)
    except Exception as exc:
        print(f"aimrt init failed: {exc}", file=sys.stderr, flush=True)
        writer.send("error", f"aimrt init failed: {exc}")
        return 3

    module = core.CreateModule(_MODULE_NAME)
    channel = module.GetChannelHandle()
    topics = _read_module_topics(module)

    subscribed: list[str] = []
    for source, topic in topics.items():
        msg_type = MSGTYPE_BY_SOURCE.get(source)
        if msg_type is None:
            print(f"no protobuf type mapped for {topic} ({source}) — skipped",
                  file=sys.stderr, flush=True)
            continue
        subscriber = channel.GetSubscriber(topic)
        if not subscriber:
            print(f"GetSubscriber('{topic}') returned an invalid handle",
                  file=sys.stderr, flush=True)
            continue
        aimrt_py.Subscribe(subscriber, msg_type, _make_callback(source, writer))
        subscribed.append(f"{source} ({topic})")

    if not subscribed:
        writer.send("error",
                    f"no AIMRT topic could be subscribed — check the "
                    f"'{_MODULE_NAME}: topics:' section of the AIMRT config")
        return 4

    writer.send("ready", {"topics": subscribed})
    print(f"aimrt worker: subscribed {', '.join(subscribed)}", file=sys.stderr, flush=True)

    # Blocks until the parent terminates this process; Core.Shutdown() is never
    # called (see the module docstring).
    core.Start()
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="AIMRT channel worker for ArmMonitor")
    parser.add_argument("--data-fd", type=int, required=True,
                        help="inherited fd used to stream decoded messages")
    parser.add_argument("--config", required=True,
                        help="path to the AIMRT runtime YAML")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        stream=sys.stderr,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    return run(args.data_fd, args.config)


if __name__ == "__main__":
    sys.exit(main())
