"""
Main application window — layout, styling, and wiring.
"""

from __future__ import annotations

import logging
import os
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from monitor.core.config import config
from monitor.core.data_model import DataModel
from monitor.core.tag_manager import PlotTagManager
from monitor.core.trajectory_manager import TrajectoryManager
from monitor.ui.data_panel import DataPanel
from monitor.ui.plot_panel import PlotPanel
from monitor.core.zenoh_client import ZenohSubscriber

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Blue-white tech colour palette
# ---------------------------------------------------------------------------
# Primary blue:  #2563eb
# Light blue:    #dbeafe
# Background:    #f8fafc
# Surface:       #ffffff
# Text primary:  #1e293b
# Text muted:    #64748b
# Border:        #e2e8f0
# Green:         #16a34a
# Red:           #dc2626
# ---------------------------------------------------------------------------
STYLE = """
/* --- Global --- */
QWidget {
    background-color: #f8fafc;
    color: #1e293b;
    font-family: "JetBrains Mono", "Fira Code", "Consolas", "Source Code Pro", monospace;
}

/* --- Main window --- */
QMainWindow::separator {
    width: 1px;
    background: #e2e8f0;
}

/* --- Table --- */
QTableWidget {
    background-color: #ffffff;
    alternate-background-color: #f1f5f9;
    gridline-color: #e2e8f0;
    border: 1px solid #e2e8f0;
    border-radius: 4px;
    selection-background-color: #dbeafe;
    selection-color: #1e293b;
    outline: none;
}
QTableWidget::item {
    padding: 4px 8px;
}
QHeaderView::section {
    background-color: #2563eb;
    color: #ffffff;
    border: none;
    border-right: 1px solid #1d4ed8;
    padding: 6px 8px;
    font-weight: 600;
}

/* --- Buttons --- */
QPushButton {
    background-color: #ffffff;
    border: 1px solid #cbd5e1;
    border-radius: 4px;
    padding: 5px 14px;
    color: #334155;
    min-width: 64px;
}
QPushButton:hover {
    background-color: #f1f5f9;
    border-color: #94a3b8;
    color: #1e293b;
}
QPushButton:pressed {
    background-color: #e2e8f0;
}
QPushButton:disabled {
    background-color: #f1f5f9;
    color: #94a3b8;
    border-color: #e2e8f0;
}

/* --- Accent (blue) button --- */
QPushButton[accent="true"] {
    background-color: #2563eb;
    border-color: #2563eb;
    color: #ffffff;
    font-weight: 600;
}
QPushButton[accent="true"]:hover {
    background-color: #1d4ed8;
    border-color: #1d4ed8;
    color: #ffffff;
}
QPushButton[accent="true"]:pressed {
    background-color: #1e40af;
}
QPushButton[accent="true"]:disabled {
    background-color: #93c5fd;
    border-color: #93c5fd;
    color: #ffffff;
}

/* --- Danger (disconnect) button --- */
QPushButton[danger="true"] {
    background-color: #ffffff;
    border: 1px solid #fca5a5;
    color: #dc2626;
}
QPushButton[danger="true"]:hover {
    background-color: #fef2f2;
    border-color: #ef4444;
}

/* --- Status bar --- */
QStatusBar {
    background-color: #ffffff;
    border-top: 1px solid #e2e8f0;
    color: #64748b;
}

/* --- Splitter handle --- */
QSplitter::handle {
    background-color: #e2e8f0;
}
QSplitter::handle:horizontal {
    width: 2px;
}

/* --- Menu bar --- */
QMenuBar {
    background-color: #ffffff;
    border-bottom: 1px solid #e2e8f0;
    padding: 2px 0;
}
QMenuBar::item {
    padding: 4px 12px;
    color: #64748b;
    border-radius: 4px;
    margin: 2px 2px;
}
QMenuBar::item:selected {
    background-color: #eff6ff;
    color: #2563eb;
}
QMenu {
    background-color: #ffffff;
    border: 1px solid #e2e8f0;
    border-radius: 6px;
    padding: 4px;
}
QMenu::item {
    padding: 6px 28px;
    border-radius: 4px;
    color: #334155;
}
QMenu::item:selected {
    background-color: #eff6ff;
    color: #2563eb;
}
QMenu::separator {
    height: 1px;
    background: #e2e8f0;
    margin: 4px 8px;
}

/* --- Tool bar --- */
QToolBar {
    background-color: #ffffff;
    border-bottom: 1px solid #e2e8f0;
    spacing: 6px;
    padding: 4px 8px;
}

/* --- Scrollbar --- */
QScrollBar:vertical {
    background-color: #f8fafc;
    width: 8px;
    border: none;
    border-radius: 4px;
}
QScrollBar::handle:vertical {
    background-color: #cbd5e1;
    border-radius: 4px;
    min-height: 24px;
}
QScrollBar::handle:vertical:hover {
    background-color: #94a3b8;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}

/* --- Labels in panels --- */
QLabel[heading="true"] {
    font-weight: 600;
    color: #1e293b;
    padding: 6px 0;
}
"""


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("ArmMonitor · Joint State Viewer")
        self.resize(1280, 800)
        self.setMinimumSize(960, 600)
        self.setStyleSheet(STYLE)

        # --- Config ---
        self._cfg = config

        # --- Core components ---
        self._model = DataModel(self)
        self._zenoh = ZenohSubscriber(parent=self)

        # --- UI ---
        self._data_panel = DataPanel()
        self._plot_panel = PlotPanel()
        self._setup_ui()
        self._setup_wiring()

        # --- State ---
        self._sample_count = 0
        self._last_update = QtCore.QTime.currentTime()
        self._connected_at: float | None = None

        # Explicit initial paused state (signals are now wired)
        self._plot_panel.set_paused(paused=True, readonly=False)
        self._model.pause()

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        # NOTE: Menu bar omitted — all actions are in the toolbar or
        #       the plot panel's own toolbar.

        # -- Toolbar --
        toolbar = self.addToolBar("Main")
        toolbar.setMovable(False)

        toolbar.addWidget(QtWidgets.QLabel("  "))

        self._btn_connect = QtWidgets.QPushButton("🟢  Connect")
        self._btn_connect.setProperty("accent", "true")
        self._btn_connect.clicked.connect(self._start_zenoh)
        topics_display = ", ".join(self._cfg.topics.values())
        self._btn_connect.setToolTip(f"Connect to zenoh ({topics_display})")
        toolbar.addWidget(self._btn_connect)

        self._btn_disconnect = QtWidgets.QPushButton("🔴  Disconnect")
        self._btn_disconnect.setProperty("danger", "true")
        self._btn_disconnect.clicked.connect(self._stop_zenoh)
        self._btn_disconnect.setEnabled(False)
        self._btn_disconnect.setToolTip("Disconnect from zenoh")
        toolbar.addWidget(self._btn_disconnect)

        toolbar.addSeparator()

        # Stretch → rate label on the right
        spacer = QtWidgets.QWidget()
        spacer.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Preferred,
        )
        toolbar.addWidget(spacer)

        self._toolbar_rate = QtWidgets.QLabel("— Hz")
        self._toolbar_rate.setStyleSheet("color: #64748b; padding-right: 4px;")
        toolbar.addWidget(self._toolbar_rate)

        # -- Data panel in scroll area — 两个表格共用同一垂直滚动条 --
        # 当窗口高度小于两表格总高时，滚动条出现；
        # 向下滚动时 Table 1 逐行上移隐藏，Table 2 逐渐显现。
        self._data_scroll = QtWidgets.QScrollArea()
        self._data_scroll.setWidget(self._data_panel)
        self._data_scroll.setWidgetResizable(True)
        self._data_scroll.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarAlwaysOff
        )
        self._data_scroll.setVerticalScrollBarPolicy(
            QtCore.Qt.ScrollBarAsNeeded
        )
        self._data_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)

        # -- Central splitter --
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.addWidget(self._data_scroll)
        splitter.addWidget(self._plot_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([420, 860])
        self.setCentralWidget(splitter)

        # -- Status bar --
        self._status = self.statusBar()
        topics_display = ", ".join(self._cfg.topics.values())
        self._status_label = QtWidgets.QLabel(f"●  Disconnected  |  {topics_display}")
        self._status_label.setStyleSheet("color: #64748b; padding-left: 4px;")
        self._status.addWidget(self._status_label)
        self._status.addWidget(QtWidgets.QWidget(), 1)
        self._count_label = QtWidgets.QLabel("Samples: 0")
        self._status.addPermanentWidget(self._count_label)
        self._uptime_label = QtWidgets.QLabel("")
        self._uptime_label.setStyleSheet("color: #94a3b8; padding-right: 8px;")
        self._status.addPermanentWidget(self._uptime_label)

    # ------------------------------------------------------------------
    # Wiring
    # ------------------------------------------------------------------

    def _setup_wiring(self) -> None:
        self._zenoh.data_received.connect(self._on_data_received)
        self._zenoh.connection_changed.connect(self._on_connection_changed)
        self._zenoh.error_occurred.connect(self._on_error)
        self._zenoh.raw_data_received.connect(self._on_raw_data)
        self._data_panel.add_to_plot_requested.connect(self._on_add_to_plot)
        self._plot_panel.clear_data_requested.connect(self._on_reset_data)
        self._plot_panel.clear_curves_requested.connect(self._on_clear_curves)
        self._plot_panel.save_csv_requested.connect(self._on_save_csv)
        self._plot_panel.save_csv_range_requested.connect(self._on_save_csv_range)
        self._plot_panel.pause_toggled.connect(self._on_pause_toggled)
        self._plot_panel.save_tag_config_requested.connect(self._on_save_tag_config)
        self._plot_panel.load_tag_config_requested.connect(self._on_load_tag_config)
        self._plot_panel.export_trajectory_requested.connect(self._on_export_trajectory)
        self._plot_panel.import_trajectory_requested.connect(self._on_import_trajectory)

        # UI refresh timer — drives plot redraw only (interval from config.yaml).
        # Data acquisition uses a separate timer below.
        self._ui_timer = QtCore.QTimer(self)
        self._ui_timer.timeout.connect(lambda: self._plot_panel.update_curves(self._model))
        self._ui_timer.start(self._cfg.refresh_interval_ms)

        # DAQ timer — strict 10 ms sampling clock, independent of UI refresh.
        # Drives sample_snapshot() which snapshots the ingest buffer into the
        # time series with a single shared timestamp per tick.
        self._daq_timer = QtCore.QTimer(self)
        self._daq_timer.timeout.connect(self._model.sample_snapshot)
        self._daq_timer.start(10)

        # Rate meter (1 Hz)
        rate_timer = QtCore.QTimer(self)
        rate_timer.timeout.connect(self._update_rate)
        rate_timer.start(1000)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _ensure_ext(path: str, ext: str) -> str:
        """Append *ext* to *path* if it doesn't already end with it."""
        return path if path.endswith(ext) else path + ext

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    def _on_data_received(self, msg: dict) -> None:
        self._model.ingest(msg)
        self._data_panel.update_from_message(msg)
        self._sample_count += 1
        self._last_update = QtCore.QTime.currentTime()

    def _on_raw_data(self, key: str, size: int) -> None:
        """Log raw data for debugging — shows in status bar briefly."""
        logger.info("Raw data: key=%s  size=%d", key, size)

    def _on_connection_changed(self, connected: bool) -> None:
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)
        topics_display = ", ".join(self._cfg.topics.values())
        if connected:
            self._connected_at = time.time()
            self._status_label.setText(f"🟢  Connected  |  {topics_display}")
            self._status_label.setStyleSheet(
                "color: #16a34a; padding-left: 4px; font-weight: 600;"
            )
        else:
            self._connected_at = None
            self._uptime_label.setText("")
            self._status_label.setText(f"⚪  Disconnected  |  {topics_display}")
            self._status_label.setStyleSheet("color: #64748b; padding-left: 4px;")

    def _on_error(self, error_text: str) -> None:
        self._status_label.setText(f"⚠  {error_text}")
        self._status_label.setStyleSheet(
            "color: #dc2626; padding-left: 4px; font-weight: 600;"
        )

    def _on_add_to_plot(self, key: str, label: str) -> None:
        # Determine the latest timestamp for this series so the curve
        # only shows data collected *after* it was added to the plot.
        series = self._model.get_series(key)
        start_time = series.times[-1] if (series and series.times) else None

        added = self._plot_panel.add_curve(key, label, start_time=start_time)
        if not added:
            self._plot_panel.remove_curve(key)
        self._sync_plotted_keys()

    def _on_reset_data(self) -> None:
        """Reset Data: clear accumulated data, keep curves.
        Preserve current pause state — let user decide when to resume.
        """
        paused = self._model.is_paused
        if paused:
            self._model.reset_and_pause()
        else:
            self._model.reset_and_resume()
        self._plot_panel.set_paused(paused=paused, readonly=False)
        # Force render so curves visually clear immediately (even when paused)
        self._plot_panel.update_curves(self._model, force=True)
        self._sync_plotted_keys()

    def _on_clear_curves(self) -> None:
        """Clear Curves: remove all curves & data, back to normal mode."""
        self._plot_panel.clear_all()
        self._model.reset_and_pause()
        self._plot_panel.set_paused(paused=True, readonly=False)
        self._sync_plotted_keys()

    def _on_pause_toggled(self, paused: bool) -> None:
        """Mirror plot-panel pause state to the data model."""
        if paused:
            self._model.pause()
        else:
            self._model.resume()

    def _on_save_csv(self) -> None:
        """Export plotted curves to a CSV file (full range) — modal progress."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save CSV", "", "CSV files (*.csv);;All files (*)"
        )
        if not path:
            return
        path = self._ensure_ext(path, ".csv")
        plotted = list(self._plot_panel.curve_keys)
        if not plotted:
            self._status_label.setText("⚠  Nothing to save — no curves plotted")
            return
        self._do_export_csv(path, plotted)

    def _on_save_csv_range(self, t_min: float, t_max: float) -> None:
        """Export plotted curves within the selected time range — modal progress."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save CSV (range)", "", "CSV files (*.csv);;All files (*)"
        )
        if not path:
            return
        path = self._ensure_ext(path, ".csv")
        plotted = list(self._plot_panel.curve_keys)
        if not plotted:
            self._status_label.setText("⚠  Nothing to save — no curves plotted")
            return
        self._do_export_csv(path, plotted, t_min=t_min, t_max=t_max)

    def _do_export_csv(self, path: str, plotted: list[str],
                       t_min: float | None = None,
                       t_max: float | None = None) -> None:
        """Show modal progress dialog and export CSV in background thread."""
        # ── custom modal dialog ──────────────────────────────────────
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("导出数据")
        dialog.setWindowModality(QtCore.Qt.WindowModal)
        dialog.setFixedSize(360, 140)
        layout = QtWidgets.QVBoxLayout(dialog)
        layout.setSpacing(16)

        label = QtWidgets.QLabel("正在导出 CSV …")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-size: 14px; padding: 8px;")
        layout.addWidget(label)

        progress = QtWidgets.QProgressBar(dialog)
        progress.setRange(0, 0)  # indeterminate
        progress.setFixedHeight(6)
        layout.addWidget(progress)

        btn = QtWidgets.QPushButton("完成")
        btn.setEnabled(False)
        btn.setFixedWidth(100)
        btn_layout = QtWidgets.QHBoxLayout()
        btn_layout.addStretch()
        btn_layout.addWidget(btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        dialog.show()
        QtWidgets.QApplication.processEvents()

        error_msg: list[str] = []
        loop = QtCore.QEventLoop(self)

        def on_done(fp: str) -> None:
            label.setText("✅  导出完成，请点击「完成」关闭")
            label.setStyleSheet(
                "font-size: 14px; padding: 8px; color: #16a34a;")
            progress.setRange(0, 100)
            progress.setValue(100)
            progress.setStyleSheet(
                "QProgressBar::chunk { background-color: #16a34a; }")
            btn.setEnabled(True)
            QtWidgets.QApplication.processEvents()

        def on_failed(msg: str) -> None:
            error_msg.append(msg)
            label.setText(f"❌  导出失败:\n{msg}")
            label.setStyleSheet(
                "font-size: 14px; padding: 8px; color: #dc2626;")
            progress.setVisible(False)
            btn.setEnabled(True)
            btn.setText("关闭")
            QtWidgets.QApplication.processEvents()

        btn.clicked.connect(loop.quit)
        self._model.csv_export_finished.connect(on_done)
        self._model.csv_export_failed.connect(on_failed)
        self._model.export_csv(path, plotted, t_min=t_min, t_max=t_max)

        loop.exec()

        # ── cleanup ─────────────────────────────────────────────────
        self._model.csv_export_finished.disconnect(on_done)
        self._model.csv_export_failed.disconnect(on_failed)
        dialog.close()

        if error_msg:
            self._status_label.setText(
                f"❌  CSV export failed: {error_msg[0]}")
            logger.error("CSV export failed: %s", error_msg[0])
        else:
            self._status_label.setText(f"✅  Saved {path}")
            logger.info("Saved CSV to %s", path)

    # ------------------------------------------------------------------
    # Tag configuration (plot layout save/load)
    # ------------------------------------------------------------------

    def _on_save_tag_config(self) -> None:
        """Export current plot layout as a JSON tag config file."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出标签配置", "", "Plot Tag (*.plottag);;All files (*)"
        )
        if not path:
            return
        path = self._ensure_ext(path, ".plottag")
        curves_info = self._plot_panel.get_curve_info()
        if not curves_info:
            self._status_label.setText("⚠  没有曲线可导出")
            return
        PlotTagManager.save(path, curves_info)
        self._status_label.setText(f"✅  标签配置已保存: {path}")
        logger.info("Saved %d tag(s) to %s", len(curves_info), path)

    def _on_load_tag_config(self) -> None:
        """Load a plot layout from a JSON tag config file."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载标签配置", "", "Plot Tag (*.plottag);;All files (*)"
        )
        if not path:
            return
        try:
            curves_info = PlotTagManager.load(path)
        except Exception as exc:
            self._status_label.setText(f"⚠  加载标签配置失败: {exc}")
            logger.error("Failed to load tag config: %s", exc)
            return
        self._plot_panel.restore_from_curve_info(curves_info, self._model)
        self._sync_plotted_keys()
        self._status_label.setText(f"✅  标签配置已加载: {path}")
        logger.info("Loaded %d tag(s) from %s", len(curves_info), path)

    # ------------------------------------------------------------------
    # Trajectory export/import (.traj / NPZ)
    # ------------------------------------------------------------------

    def _on_export_trajectory(self) -> None:
        """Export all plotted curve data as a compressed NPZ file."""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出轨迹", "", "Trajectory (*.npz);;All files (*)"
        )
        if not path:
            return
        path = self._ensure_ext(path, ".npz")
        plotted = list(self._plot_panel.curve_keys)
        if not plotted:
            self._status_label.setText("⚠  没有曲线可导出")
            return

        # Build curves dict and labels dict from the model
        curves = {}
        labels = {}
        for key in plotted:
            series = self._model.get_series(key)
            if series is not None:
                curves[key] = (series.times, series.values)
                # Derive a human-readable label from the plot panel
                labels[key] = self._plot_panel._labels.get(key, key)

        if not curves:
            self._status_label.setText("⚠  没有数据可导出")
            return

        # Collect basic metadata
        metadata = {
            "source": "arm_monitor",
            "curve_count": len(curves),
        }
        # Estimate sample count and duration from the first curve
        first_key = next(iter(curves))
        times = curves[first_key][0]
        if times:
            metadata["sample_count"] = len(times)
            metadata["duration_s"] = round(times[-1] - times[0], 3)

        TrajectoryManager.export(path, curves, labels, metadata=metadata)
        self._status_label.setText(f"✅  轨迹已导出: {path}")
        logger.info("Exported trajectory (%d curves) to %s", len(curves), path)

    def _on_import_trajectory(self) -> None:
        """Import a NPZ trajectory file and display its curves."""
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "导入轨迹", "", "Trajectory (*.npz);;All files (*)"
        )
        if not path:
            return
        try:
            result = TrajectoryManager.load(path)
        except Exception as exc:
            self._status_label.setText(f"⚠  导入轨迹失败: {exc}")
            logger.error("Failed to import trajectory: %s", exc)
            return

        curves = result["curves"]
        labels = result["labels"]

        if not curves:
            self._status_label.setText("⚠  轨迹文件为空")
            return

        # Load data into the model, then add curves to the plot
        max_t = self._model.load_external_data(curves)
        self._plot_panel.add_trajectory_curves(curves, labels)
        self._sync_plotted_keys()

        # Force an immediate render so curves appear right away
        self._plot_panel.update_curves(self._model, force=True)

        # Auto-focus the view to show the full imported trajectory
        if max_t > 0:
            self._plot_panel._plot_widget.setXRange(0, max_t * 1.05, padding=0)

        # Enter read-only mode — freeze data, disable the pause button
        self._model.pause()
        self._plot_panel.set_paused(paused=True, readonly=True)

        self._status_label.setText(
            f"✅  轨迹已导入: {os.path.basename(path)}  "
            f"({len(curves)} 曲线, {result['metadata'].get('sample_count', '?')} 采样点)"
        )
        logger.info("Imported trajectory from %s (%d curves)", path, len(curves))

    def _sync_plotted_keys(self) -> None:
        """Keep data-panel plotted-keys in sync with plot panel."""
        self._data_panel.set_plotted_keys(set(self._plot_panel.curve_keys))

    def _update_rate(self) -> None:
        elapsed = self._last_update.msecsTo(QtCore.QTime.currentTime()) / 1000.0
        rate = self._sample_count / max(elapsed, 0.001)
        self._toolbar_rate.setText(f"{rate:.0f} Hz")
        self._count_label.setText(f"Samples: {self._sample_count}")
        self._sample_count = 0
        self._last_update = QtCore.QTime.currentTime()

        # Sync plotted-keys (curves may have been removed via legend menu)
        self._sync_plotted_keys()

        # Update uptime display
        if self._connected_at is not None:
            uptime = time.time() - self._connected_at
            h, r = divmod(int(uptime), 3600)
            m, s = divmod(r, 60)
            if h:
                self._uptime_label.setText(f"⏱  {h}h{m:02d}m{s:02d}s")
            else:
                self._uptime_label.setText(f"⏱  {m:02d}m{s:02d}s")

    # ------------------------------------------------------------------
    # Zenoh lifecycle
    # ------------------------------------------------------------------

    def _start_zenoh(self) -> None:
        self._zenoh.start()

    def _stop_zenoh(self) -> None:
        self._zenoh.stop()

    # ------------------------------------------------------------------
    # Qt close
    # ------------------------------------------------------------------

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._zenoh.stop()
        super().closeEvent(event)
