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
from monitor.ui.data_panel import DataPanel
from monitor.ui.plot_panel import PlotPanel
from monitor.core.aimrt_client import AimrtSubscriber

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
        self._transport = AimrtSubscriber(parent=self)

        # --- UI ---
        self._data_panel = DataPanel()
        self._plot_panel = PlotPanel()
        # When True, skip updating DataPanel tables to reduce UI workload.
        # Initialize before _setup_ui so toolbar button text can read it.
        self._suspend_table_updates: bool = False
        self._setup_ui()
        self._setup_wiring()

        # --- State ---
        self._sample_count = 0
        self._last_update = QtCore.QTime.currentTime()
        self._connected_at: float | None = None
        self._record_pending_action: str | None = None
        self._record_config = {
            "action_name": "record_joint_state",
            "preparation_duration_s": 0,
            "record_duration_s": 0,
            "rpc_base_url": "http://127.0.0.1:50080",
        }

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
        self._btn_connect.clicked.connect(self._start_transport)
        topics_display = ", ".join(self._cfg.topics.values())
        self._btn_connect.setToolTip(f"Connect via AIMRT/zenoh ({topics_display})")
        toolbar.addWidget(self._btn_connect)

        self._btn_disconnect = QtWidgets.QPushButton("🔴  Disconnect")
        self._btn_disconnect.setProperty("danger", "true")
        self._btn_disconnect.clicked.connect(self._stop_transport)
        self._btn_disconnect.setEnabled(False)
        self._btn_disconnect.setToolTip("Disconnect the AIMRT session")
        toolbar.addWidget(self._btn_disconnect)

        self._btn_record_config = QtWidgets.QPushButton("🛠  录制配置")
        self._btn_record_config.clicked.connect(self._on_configure_start_record)
        self._btn_record_config.setToolTip("配置 StartRecord 参数与 RPC 地址")
        toolbar.addWidget(self._btn_record_config)

        # Button to suspend/resume table updates (reduces UI load during heavy
        # ingestion; plotting and recording remain active).
        # Initial text reflects the current suspend state
        init_text = "⏹  列表更新" if not self._suspend_table_updates else "▶️  列表已暂停"
        self._btn_suspend_tables = QtWidgets.QPushButton(init_text)
        self._btn_suspend_tables.setCheckable(True)
        self._btn_suspend_tables.setToolTip("切换：暂停/恢复表格更新以降低 UI 负载；绘图与录制不受影响")
        self._btn_suspend_tables.clicked.connect(self._on_toggle_suspend_tables)
        # Set checked state to match internal flag
        self._btn_suspend_tables.setChecked(self._suspend_table_updates)
        toolbar.addWidget(self._btn_suspend_tables)

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
        self._transport.data_received.connect(self._on_data_received)
        self._transport.connection_changed.connect(self._on_connection_changed)
        self._transport.error_occurred.connect(self._on_error)
        self._transport.raw_data_received.connect(self._on_raw_data)
        self._data_panel.add_to_plot_requested.connect(self._on_add_to_plot)
        self._plot_panel.clear_data_requested.connect(self._on_reset_data)
        self._plot_panel.clear_curves_requested.connect(self._on_clear_curves)
        self._plot_panel.record_toggle_requested.connect(self._on_record_toggle)
        self._plot_panel.pause_toggled.connect(self._on_pause_toggled)
        self._plot_panel.save_tag_config_requested.connect(self._on_save_tag_config)
        self._plot_panel.load_tag_config_requested.connect(self._on_load_tag_config)
        self._model.record_started.connect(self._on_record_started)
        self._model.record_stopped.connect(self._on_record_stopped)
        self._model.record_failed.connect(self._on_record_failed)

        # UI refresh timer — drives plot redraw and batched ingestion.
        # Cap interval to avoid extremely high-frequency updates that can
        # overload the GUI when the user set a very small value in config.
        interval_ms = max(int(self._cfg.refresh_interval_ms), 10)
        self._ui_timer = QtCore.QTimer(self)
        self._ui_timer.timeout.connect(self._on_ui_tick)
        self._ui_timer.start(interval_ms)

        # Sampling is message-driven in DataModel.ingest(); no DAQ snapshot timer.

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
        # Fast path: enqueue message for batched processing in the UI timer
        # tick. Avoid doing heavy work directly in the data-received slot.
        self._model.enqueue(msg)
        # Update a lightweight latest-sample indicator immediately so rate
        # meter and status feel responsive even when batches are processed
        # at the UI tick.
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

    def _on_record_toggle(self, start: bool) -> None:
        """Handle unified record toggle button (start/stop)."""
        action_name = str(self._record_config["action_name"]).strip() or "record_joint_state"
        rpc_base_url = str(self._record_config["rpc_base_url"]).strip() or "http://127.0.0.1:50080"
        self._model.set_record_rpc_base_url(rpc_base_url)
        self._record_pending_action = "start" if start else "stop"

        if start:
            self._model.export_recording(
                action_name=action_name,
                preparation_duration_s=int(self._record_config["preparation_duration_s"]),
                record_duration_s=int(self._record_config["record_duration_s"]),
            )
            self._status_label.setText("⏺  已请求 record_playback 插件开始录制…")
        else:
            self._model.stop_recording(action_name=action_name)
            self._status_label.setText("⏹  已请求 record_playback 插件停止录制…")

    def _on_configure_start_record(self) -> None:
        """Open a small dialog to configure StartRecord payload."""
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("配置 StartRecord")
        layout = QtWidgets.QVBoxLayout(dialog)

        form = QtWidgets.QFormLayout()

        action_edit = QtWidgets.QLineEdit(str(self._record_config["action_name"]))
        action_edit.setPlaceholderText("record_joint_state")
        form.addRow("action_name", action_edit)

        prep_spin = QtWidgets.QSpinBox()
        prep_spin.setRange(0, 86400)
        prep_spin.setValue(int(self._record_config["preparation_duration_s"]))
        prep_spin.setSuffix(" s")
        form.addRow("preparation_duration_s", prep_spin)

        duration_spin = QtWidgets.QSpinBox()
        duration_spin.setRange(0, 86400)
        duration_spin.setValue(int(self._record_config["record_duration_s"]))
        duration_spin.setSuffix(" s")
        form.addRow("record_duration_s", duration_spin)

        rpc_url_edit = QtWidgets.QLineEdit(str(self._record_config["rpc_base_url"]))
        rpc_url_edit.setPlaceholderText("http://127.0.0.1:50080")
        form.addRow("rpc_base_url", rpc_url_edit)

        tip = QtWidgets.QLabel("提示: record_duration_s=0 表示持续录制，直到 StopRecord。")
        tip.setStyleSheet("color: #64748b;")

        layout.addLayout(form)
        layout.addWidget(tip)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return

        action_name = action_edit.text().strip() or "record_joint_state"
        rpc_base_url = rpc_url_edit.text().strip() or "http://127.0.0.1:50080"
        self._record_config = {
            "action_name": action_name,
            "preparation_duration_s": prep_spin.value(),
            "record_duration_s": duration_spin.value(),
            "rpc_base_url": rpc_base_url,
        }
        self._status_label.setText(
            "✅  StartRecord 参数已更新: "
            f"action_name={action_name}, "
            f"preparation_duration_s={prep_spin.value()}, "
            f"record_duration_s={duration_spin.value()}, "
            f"rpc_base_url={rpc_base_url}"
        )

    def _on_record_started(self, output_dir: str) -> None:
        self._record_pending_action = None
        self._plot_panel.set_recording(True)
        self._status_label.setText(f"⏺  录制已开始: {output_dir}")

    def _on_record_stopped(self, output_dir: str) -> None:
        self._record_pending_action = None
        self._plot_panel.set_recording(False)
        self._status_label.setText(f"⏹  录制已停止: {output_dir}")

    def _on_record_failed(self, error_text: str) -> None:
        pending = self._record_pending_action
        self._record_pending_action = None
        if pending == "start":
            self._plot_panel.set_recording(False)
        elif pending == "stop":
            self._plot_panel.set_recording(True)
        self._status_label.setText(f"⚠  录制操作失败: {error_text}")

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

    def _on_ui_tick(self) -> None:
        """Called by the UI timer: process a bounded number of queued
        incoming messages, update UI tables and redraw plots.
        """
        # Process up to N messages per tick — keeps main thread responsive.
        processed = self._model.process_queue(max_items=200)
        if processed:
            # Update the lightweight UI table with the last received message
            latest = self._model.latest_message
            if latest and not self._suspend_table_updates:
                self._data_panel.update_from_message(latest)
            # Update sample count for rate meter
            self._sample_count += processed

        # Redraw plots (batch update)
        self._plot_panel.update_curves(self._model)

    def _on_toggle_suspend_tables(self, checked: bool) -> None:
        """Toggle suspension of DataPanel table updates.

        When resuming, immediately refresh the tables with the latest message
        so the UI reflects the current state.
        """
        self._suspend_table_updates = bool(checked)
        if self._suspend_table_updates:
            # When suspended, show ▶️ to indicate pressing will resume updates.
            self._btn_suspend_tables.setText("▶️  列表已暂停")
            self._status_label.setText("⚠  已暂停表格更新（绘图与录制不受影响）")
        else:
            # When active, show ⏹ to indicate pressing will pause updates.
            self._btn_suspend_tables.setText("⏹  列表更新")
            self._status_label.setText("✅  已恢复表格更新")
            # Immediately refresh the table with the latest message to catch up
            latest = self._model.latest_message
            if latest:
                self._data_panel.update_from_message(latest)

    # ------------------------------------------------------------------
    # AIMRT transport lifecycle
    # ------------------------------------------------------------------

    def _start_transport(self) -> None:
        self._transport.start()

    def _stop_transport(self) -> None:
        self._transport.stop()

    # ------------------------------------------------------------------
    # Qt close
    # ------------------------------------------------------------------

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._transport.stop()
        super().closeEvent(event)
