"""
AIMRT runtime worker — runs the AIMRT session in a separate process.

Why a subprocess
----------------
``aimrt_py`` 1.7.0 segfaults inside ``Core.Shutdown()`` as soon as any
channel subscriber with a Python callback has been declared (reproducible
with a minimal 1-topic subscriber; a core without subscribers shuts down
cleanly). Keeping the runtime in a child process lets the GUI:

* tear the session down by simply terminating the child (no native
  teardown, no crash);
* rebuild it from scratch on every "Connect";
* survive a bad zenoh configuration — the worker dies, the GUI reports the
  captured stderr instead of taking the whole application down.

Protocol
--------
Decoded messages travel over a dedicated pipe fd (``--data-fd``) as
length-prefixed pickles so AIMRT's own console logging, which shares
stdout/stderr, cannot corrupt the stream::

    record := uint32_be length | pickle((kind, payload))

    ("ready", {"topics": [...]})   subscription established
    ("data",  {…message dict…})    one decoded message
    ("error", "message")           fatal problem, worker exits next
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import pickle
import socket
import struct
import sys
import tempfile
import threading
from typing import Optional

import yaml

from monitor.core.config import config
from monitor.core.msg_decode import MSGTYPE_BY_SOURCE, message_to_dict

logger = logging.getLogger("monitor.aimrt_worker")

_ZENOH_PLUGIN_NAME = "libaimrt_zenoh_plugin.so"

# Drop the pending buffer once it exceeds this, so a stalled GUI can never
# block the AIMRT dispatch thread.
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
                # Reader is far behind — a monitor only needs fresh data, and
                # clearing at a record boundary keeps framing valid.
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
# Preflight / runtime config
# ---------------------------------------------------------------------------

def _assert_port_free(host: str, port: int, endpoint: str) -> None:
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    probe = socket.socket(family, socket.SOCK_STREAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind((host, port))
    except OSError as exc:
        raise OSError(
            f"listen endpoint {endpoint} is unavailable: {exc}. Another process "
            "(or an editor port-forward / stale instance) may be holding it."
        ) from exc
    finally:
        probe.close()


def preflight() -> None:
    """Fail with a readable message *before* touching the AIMRT runtime.

    The zenoh plugin only logs when the session fails to open and then keeps
    using the invalid session; the next declaration crashes. Validate what we
    can up front so a typo does not look like a mysterious crash.
    """
    native = config.aimrt_native_cfg_path
    if native:
        if not os.path.isfile(native):
            raise FileNotFoundError(f"native_cfg_path not found: {native}")
        return

    for ep in config.aimrt_listen_endpoints:
        proto, _, addr = ep.partition("/")
        if proto != "tcp":
            continue
        host, _, port_s = addr.partition(":")
        try:
            port = int(port_s)
        except ValueError:
            raise ValueError(f"bad listen endpoint (no port): {ep}")
        _assert_port_free(host, port, ep)


def write_native_zenoh_cfg(run_dir: str) -> Optional[str]:
    """Generate a zenoh native json5 from config.yaml.

    Returns ``None`` when everything stays at zenoh's defaults — the
    same-LAN zero-config case (peer mode + multicast scouting).
    """
    connect = config.aimrt_connect_endpoints
    listen = config.aimrt_listen_endpoints
    multicast = config.aimrt_multicast_scouting

    if not connect and not listen and multicast:
        return None

    native: dict = {"mode": "peer"}
    if connect:
        native["connect"] = {"endpoints": list(connect)}
    if listen:
        native["listen"] = {"endpoints": {"peer": list(listen)}}
    native["scouting"] = {"multicast": {"enabled": bool(multicast)}}

    path = os.path.join(run_dir, "zenoh_native.json5")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(native, f, indent=2)
    logger.info("zenoh native config: %s", native)
    return path


def write_runtime_config(run_dir: str) -> str:
    import aimrt_py

    pkg_dir = os.path.dirname(os.path.abspath(aimrt_py.__file__))
    plugin_path = os.path.join(pkg_dir, _ZENOH_PLUGIN_NAME)
    if not os.path.isfile(plugin_path):
        raise FileNotFoundError(
            f"AIMRT zenoh plugin not found at {plugin_path} — the aimrt_py "
            "package seems incomplete"
        )

    plugin_options: dict = {}
    native_path = config.aimrt_native_cfg_path
    if native_path:
        plugin_options["native_cfg_path"] = os.path.abspath(native_path)
    else:
        generated = write_native_zenoh_cfg(run_dir)
        if generated:
            plugin_options["native_cfg_path"] = generated

    cfg = {
        "aimrt": {
            "plugin": {
                "plugins": [{
                    "name": "zenoh_plugin",
                    "path": plugin_path,
                    "options": plugin_options,
                }]
            },
            "log": {
                "core_lvl": config.aimrt_log_level,
                "backends": [{"type": "console"}],
            },
            "executor": {
                "executors": [{
                    "name": "monitor_executor",
                    "type": "asio_thread",
                    "options": {"thread_num": 2},
                }]
            },
            "channel": {
                "backends": [{"type": "zenoh"}],
                "sub_topics_options": [{
                    "topic_name": "(.*)",
                    "enable_backends": ["zenoh"],
                }],
            },
        }
    }

    cfg_path = os.path.join(run_dir, "aimrt_cfg.yaml")
    with open(cfg_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
    return cfg_path


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


def run(data_fd: int) -> int:
    writer = PipeWriter(data_fd)

    try:
        preflight()
        run_dir = tempfile.mkdtemp(prefix="armmonitor_aimrt_")
        cfg_path = write_runtime_config(run_dir)
    except Exception as exc:
        print(f"config error: {exc}", file=sys.stderr, flush=True)
        writer.send("error", f"config error: {exc}")
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

    module = core.CreateModule("MonitorSubscriberPyModule")
    channel = module.GetChannelHandle()

    subscribed: list[str] = []
    for source, topic in config.topics.items():
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
        writer.send("error", "no AIMRT topic could be subscribed — check config.yaml")
        return 4

    writer.send("ready", {"topics": subscribed})
    print(f"aimrt worker: subscribed {', '.join(subscribed)}", file=sys.stderr, flush=True)

    # Blocks until the parent terminates this process. We deliberately never
    # call Core.Shutdown() — see the module docstring.
    core.Start()
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="AIMRT channel worker for ArmMonitor")
    parser.add_argument("--data-fd", type=int, required=True,
                        help="inherited fd used to stream decoded messages")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        stream=sys.stderr,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    return run(args.data_fd)


if __name__ == "__main__":
    sys.exit(main())
