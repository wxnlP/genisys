"""
AIMRT channel subscriber — the monitor's data source.

The AIMRT runtime lives in a **child process** (``monitor.core.aimrt_worker``)
and streams decoded messages back over a pipe. That isolation is not
cosmetic: ``aimrt_py`` 1.7.0 segfaults inside ``Core.Shutdown()`` whenever a
channel subscriber with a Python callback has been declared, so an in-process
session cannot be torn down or rebuilt. Terminating the child avoids the
broken teardown entirely, allows a clean "Connect" for every session, and
keeps a bad zenoh configuration from taking the GUI down with it.

Public interface is identical to the legacy ``ZenohSubscriber`` so the UI does
not care which backend is active::

    signals: data_received(dict), connection_changed(bool),
             error_occurred(str), raw_data_received(str, int)
    methods: start() -> bool, stop()
    props:   is_active, message_count
"""

from __future__ import annotations

import collections
import logging
import os
import pickle
import struct
import subprocess
import sys
import threading
from typing import Dict, Optional

from PyQt5 import QtCore

logger = logging.getLogger(__name__)

# Warn when the session is up but silent — 'connected' and 'receiving' are
# independent states in AIMRT/zenoh.
_NO_DATA_WARN_MS = 5000

# Sanity cap on a single framed record.
_MAX_RECORD_BYTES = 64 << 20


def _workspace_root() -> str:
    """.../monitor/core/aimrt_client.py -> .../ (so ``-m monitor.…`` works)."""
    here = os.path.abspath(__file__)
    return os.path.dirname(os.path.dirname(os.path.dirname(here)))


def _read_exact(f, n: int) -> Optional[bytes]:
    """Read exactly *n* bytes, or return ``None`` on EOF."""
    chunks = []
    remaining = n
    while remaining > 0:
        try:
            chunk = f.read(remaining)
        except (BlockingIOError, InterruptedError):
            continue
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class AimrtSubscriber(QtCore.QObject):
    """Supervises an AIMRT worker process and emits Qt signals."""

    data_received = QtCore.pyqtSignal(dict)
    raw_data_received = QtCore.pyqtSignal(str, int)  # kept for interface parity
    connection_changed = QtCore.pyqtSignal(bool)
    error_occurred = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._proc: Optional[subprocess.Popen] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._drain_thread: Optional[threading.Thread] = None
        self._active = False
        self._ready = False
        self._msg_count = 0
        self._source_counts: Dict[str, int] = {}
        self._source_logged: set = set()
        self._child_log: collections.deque = collections.deque(maxlen=60)
        self._no_data_timer: Optional[QtCore.QTimer] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> bool:
        if self._active:
            logger.warning("AimrtSubscriber: already running")
            return True

        try:
            import aimrt_py  # noqa: F401  (fail fast with a clear message)
        except Exception as exc:
            msg = ("aimrt_py is not importable — install the AIMRT python wheel "
                   f"matching your robot's AIMRT version ({exc})")
            logger.error(msg)
            self.error_occurred.emit(msg)
            return False

        python = sys.executable or "python3"
        root = _workspace_root()

        try:
            read_fd, write_fd = os.pipe()
        except OSError as exc:
            self.error_occurred.emit(f"cannot create data pipe: {exc}")
            return False

        env = dict(os.environ)
        env["PYTHONPATH"] = root + os.pathsep + env.get("PYTHONPATH", "")
        cmd = [python, "-u", "-m", "monitor.core.aimrt_worker",
               "--data-fd", str(write_fd)]

        try:
            proc = subprocess.Popen(
                cmd,
                cwd=root,
                env=env,
                pass_fds=(write_fd,),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
        except Exception as exc:
            os.close(read_fd)
            os.close(write_fd)
            logger.error("failed to spawn AIMRT worker: %s", exc)
            self.error_occurred.emit(f"cannot start AIMRT worker: {exc}")
            return False
        finally:
            try:
                os.close(write_fd)
            except OSError:
                pass

        self._proc = proc
        self._active = True
        self._ready = False
        self._msg_count = 0
        self._source_counts.clear()
        self._source_logged.clear()
        self._child_log.clear()

        self._reader_thread = threading.Thread(
            target=self._read_loop, args=(read_fd,),
            name="aimrt-reader", daemon=True,
        )
        self._reader_thread.start()
        self._drain_thread = threading.Thread(
            target=self._drain_child_output, args=(proc,),
            name="aimrt-drain", daemon=True,
        )
        self._drain_thread.start()

        logger.info("AIMRT worker started (pid=%d)", proc.pid)
        self.connection_changed.emit(True)
        self._arm_no_data_watchdog()
        return True

    def stop(self) -> None:
        was_active = self._active
        self._active = False
        self._disarm_no_data_watchdog()

        proc = self._proc
        self._proc = None
        if proc is not None:
            # SIGTERM is enough: the worker installs no handler, so it dies
            # without running the crashing native teardown.
            try:
                proc.terminate()
            except ProcessLookupError:
                pass
            except Exception as exc:
                logger.debug("terminate failed: %s", exc)
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                logger.warning("AIMRT worker ignored SIGTERM; killing it")
                try:
                    proc.kill()
                    proc.wait(timeout=2.0)
                except Exception:
                    pass
            except Exception:
                pass

        for thread in (self._reader_thread, self._drain_thread):
            if thread is not None:
                thread.join(timeout=2.0)
        self._reader_thread = None
        self._drain_thread = None

        if proc is not None:
            try:
                if proc.stdout is not None:
                    proc.stdout.close()
            except Exception:
                pass

        if was_active:
            if self._source_counts:
                summary = "  ".join(f"{s}={c}" for s, c in self._source_counts.items())
                logger.warning("AimrtSubscriber stopped  (total=%d  %s)",
                               self._msg_count, summary)
            else:
                logger.warning("AimrtSubscriber stopped  (total=%d)", self._msg_count)
            self.connection_changed.emit(False)

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def is_ready(self) -> bool:
        """True once the worker confirmed its subscriptions."""
        return self._ready

    @property
    def message_count(self) -> int:
        return self._msg_count

    # ------------------------------------------------------------------
    # Worker plumbing
    # ------------------------------------------------------------------

    def _read_loop(self, fd: int) -> None:
        try:
            with os.fdopen(fd, "rb", buffering=0) as stream:
                while True:
                    header = _read_exact(stream, 4)
                    if header is None:
                        break
                    size = struct.unpack(">I", header)[0]
                    if size <= 0 or size > _MAX_RECORD_BYTES:
                        logger.error("AIMRT worker sent a bad record size: %d", size)
                        break
                    blob = _read_exact(stream, size)
                    if blob is None:
                        break
                    try:
                        kind, payload = pickle.loads(blob)
                    except Exception as exc:
                        logger.error("cannot unpickle worker record: %s", exc)
                        continue
                    self._handle_record(kind, payload)
        except Exception as exc:  # pragma: no cover - defensive
            logger.error("AIMRT reader loop failed: %s", exc)
        finally:
            self._on_worker_gone()

    def _handle_record(self, kind: str, payload: object) -> None:
        if kind == "data":
            if not isinstance(payload, dict):
                return
            source = payload.get("source", "?")
            self._msg_count += 1
            self._source_counts[source] = self._source_counts.get(source, 0) + 1
            if source not in self._source_logged:
                self._source_logged.add(source)
                logger.info("Data flowing: %s", source)
            self.data_received.emit(payload)
            return

        if kind == "ready":
            self._ready = True
            topics = payload.get("topics") if isinstance(payload, dict) else None
            logger.info("AIMRT session ready  (%s)",
                        ", ".join(topics) if topics else "no topics")
            return

        if kind == "error":
            logger.error("AIMRT worker reported: %s", payload)
            self.error_occurred.emit(str(payload))
            return

    def _drain_child_output(self, proc: subprocess.Popen) -> None:
        """Keep the child's merged stdout/stderr drained and keep the tail.

        Draining matters: if the pipe fills, the worker blocks on write.
        """
        stream = proc.stdout
        if stream is None:
            return
        try:
            for raw in iter(stream.readline, b""):
                line = raw.decode("utf-8", "replace").rstrip()
                if line:
                    self._child_log.append(line)
        except Exception:
            pass

    def _on_worker_gone(self) -> None:
        """Worker exited — report it unless we asked for the shutdown."""
        proc = self._proc
        if not self._active or proc is None:
            return
        code = proc.poll()
        if code is None:
            return
        self._active = False
        self._disarm_no_data_watchdog()

        detail = ""
        if self._child_log:
            detail = " | ".join(list(self._child_log)[-4:])
        msg = (f"AIMRT worker exited (code={code})"
               + (f": {detail}" if detail else " — no output captured"))
        logger.error(msg)
        self.error_occurred.emit(msg)
        self.connection_changed.emit(False)

    # ------------------------------------------------------------------
    # Silent-failure watchdog
    # ------------------------------------------------------------------

    def _arm_no_data_watchdog(self) -> None:
        self._disarm_no_data_watchdog()
        timer = QtCore.QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(self._on_no_data_timeout)
        timer.start(_NO_DATA_WARN_MS)
        self._no_data_timer = timer

    def _disarm_no_data_watchdog(self) -> None:
        if self._no_data_timer is not None:
            self._no_data_timer.stop()
            self._no_data_timer.deleteLater()
            self._no_data_timer = None

    def _on_no_data_timeout(self) -> None:
        if self._msg_count == 0 and self._active:
            logger.warning(
                "AIMRT session is up but no data received in %d ms — check that "
                "the robot is publishing and that topics match config.yaml",
                _NO_DATA_WARN_MS,
            )
