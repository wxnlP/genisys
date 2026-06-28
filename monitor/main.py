#!/usr/bin/env python3
"""
ArmMonitor — real-time joint-state monitoring tool.

Usage::

    python -m monitor.main

Dependencies
------------
- PyQt5
- pyqtgraph
- eclipse-zenoh
- protobuf
"""

from __future__ import annotations

import logging
import os
import signal
import socket
import sys

from PyQt5 import QtCore, QtWidgets

from monitor.ui.main_window import MainWindow

# ── Ctrl+C → graceful quit via socket pair ──────────────────────────
# A plain signal.signal(…) / QTimer trick is unreliable when background
# threads (e.g. zenoh subscriber) spend time in native code, because Qt's
# event loop never wakes to process the Python-level signal.  Writing a
# byte into a socket from the C-level signal handler forces Qt to wake up.
_sig_r, _sig_w = socket.socketpair()
_sig_w.setblocking(False)

# Populated by main() so the signal handler can gracefully close the window.
_main_window: MainWindow | None = None


def _sigint_handler(signum: int, frame: object) -> None:
    """Minimal signal handler – just kick the socket."""
    _sig_w.send(b"\0")


def _graceful_shutdown() -> None:
    """Close the window (triggers closeEvent → zenoh.stop()) and force-
    exit after 1 s in case zenoh native threads keep the process alive."""
    if _main_window is not None:
        _main_window.close()
    # Fallback: if the process still hasn't exited, force it.
    QtCore.QTimer.singleShot(1000, lambda: os._exit(0))


signal.signal(signal.SIGINT, _sigint_handler)


def main() -> None:
    global _main_window

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("ArmMonitor")
    app.setApplicationVersion("0.1.0")

    # Wire the socket to Qt so the event loop sees Ctrl+C and quits
    notifier = QtCore.QSocketNotifier(
        _sig_r.fileno(), QtCore.QSocketNotifier.Read
    )
    notifier.activated.connect(_graceful_shutdown)

    # Global font hint
    font = app.font()
    font.setPointSize(11)
    app.setFont(font)

    _main_window = MainWindow()
    _main_window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
