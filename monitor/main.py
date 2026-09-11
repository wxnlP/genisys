#!/usr/bin/env python3
"""
ArmMonitor — real-time joint-state monitoring tool.

Usage::

    python -m monitor.main

Dependencies
------------
- PyQt5
- pyqtgraph
- aimrt_py (must match the robot's AIMRT version)
- protobuf
"""

from __future__ import annotations

import logging
import os
import signal
import socket
import sys

from PyQt5 import QtCore, QtWidgets

from monitor.core.config import config
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


_shutdown_in_progress = False


def _graceful_shutdown() -> None:
    """Close the window (triggers closeEvent → zenoh.stop()) and force-
    exit after 1 s in case zenoh native threads keep the process alive."""
    global _shutdown_in_progress
    if _shutdown_in_progress:
        return
    _shutdown_in_progress = True

    # Read from the socket to clear it, otherwise QSocketNotifier
    # keeps firing and calls _graceful_shutdown repeatedly.
    try:
        _sig_r.recv(4096)
    except Exception:
        pass

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

    # ── High-DPI / UI scaling ──────────────────────────────────────
    # 注意：AA_EnableHighDpiScaling 必须在 QApplication 创建之前设置。
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)

    # Windows 额外 DPI 感知（让系统告诉 Qt 真实 DPI）
    if sys.platform == "win32":
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    app.setApplicationName("ArmMonitor")
    app.setApplicationVersion("0.1.0")

    # Wire the socket to Qt so the event loop sees Ctrl+C and quits
    notifier = QtCore.QSocketNotifier(
        _sig_r.fileno(), QtCore.QSocketNotifier.Read
    )
    notifier.activated.connect(_graceful_shutdown)

    # Global font hint (base size before ui_scale)
    font = app.font()
    base_size = 11
    font.setPointSize(max(1, round(base_size * config.ui_scale)))
    app.setFont(font)

    if config.ui_scale != 1.0:
        logging.getLogger(__name__).info(
            "UI scale factor = %.2f", config.ui_scale
        )

    _main_window = MainWindow()
    _main_window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
