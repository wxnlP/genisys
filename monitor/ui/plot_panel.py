"""
Real-time plot panel built on pyqtgraph.

Manages a set of plotted curves.  Curves are added/removed manually
(right-click a cell in the data table → "Plot …").

Legend / curve interaction
--------------------------
- Left-click  legend sample  → highlight the curve (thicker line)
- Right-click legend sample  → per-curve menu: show / hide / remove

Axis zoom
---------
- Click the X or Y axis label area → axis turns blue (selected)
- Scroll wheel while an axis is selected → zoom only that axis
- Click the plot background → deselect the axis
- No axis selected → scroll zooms both axes uniformly (default)

Auto-follow
-----------
- X-axis always auto-scrolls to the latest data — no toggle needed.
- Current X zoom level (range width) is preserved while following.
- Manual drag temporarily shifts the view; the next frame snaps back.
"""

from __future__ import annotations

import bisect
from typing import Dict, Optional

import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets

from monitor.core.config import config

# Blue-white friendly curve colours
CURVE_COLORS = [
    (37, 99, 235),     # blue-600
    (220, 38, 38),     # red-600
    (22, 163, 74),     # green-600
    (217, 119, 6),     # amber-600
    (147, 51, 234),    # purple-600
    (8, 145, 178),     # cyan-600
    (219, 39, 119),    # pink-600
    (101, 163, 13),    # lime-600
    (225, 96, 0),      # orange-700
    (55, 65, 81),      # gray-700
    (2, 132, 199),     # sky-600
    (190, 24, 93),     # rose-700
]

HIGHLIGHT_WIDTH = 4.0
NORMAL_WIDTH = 1.5

# Axis visual constants
AXIS_PEN_NORMAL = pg.mkPen(color="#64748b", width=1)
AXIS_PEN_HIGHLIGHT = pg.mkPen(color="#ea580c", width=3)   # orange-600, thicker
AXIS_TEXT_NORMAL = pg.mkPen(color="#475569")
AXIS_TEXT_HIGHLIGHT = pg.mkPen(color="#ea580c")


# ---------------------------------------------------------------------------
# Custom AxisItem — click to select for single-axis zoom
# ---------------------------------------------------------------------------

class _ClickableAxis(pg.AxisItem):
    """Axis item that emits a signal when left-clicked for axis-specific zoom.

    - Left-click  → emit ``sigSelected(orientation)``, highlight toggled by owner.
    - Right-click → pass through to default (ViewBox context menu).
    """

    sigSelected = QtCore.pyqtSignal(str)  # 'bottom' or 'left'

    def __init__(self, orientation: str, plain_decimal: bool = False) -> None:
        super().__init__(orientation)
        self._plain_decimal = plain_decimal

    def tickStrings(self, values, scale, spacing):
        """Override to force plain-decimal labels (never ``1e-6`` etc.)."""
        if self._plain_decimal:
            return [f"{v:.10f}".rstrip("0").rstrip(".") for v in values]
        return super().tickStrings(values, scale, spacing)

    def mouseClickEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.sigSelected.emit(self.orientation)
            event.accept()
        else:
            super().mouseClickEvent(event)


# ---------------------------------------------------------------------------
# Custom ViewBox — single-axis zoom when an axis is selected
# ---------------------------------------------------------------------------

class _ZoomViewBox(pg.ViewBox):
    """ViewBox that restricts scroll-wheel zoom to one axis when selected.

    Set ``selected_axis`` to ``'x'``, ``'y'``, or ``None``.
    """

    sigPlotAreaClicked = QtCore.pyqtSignal()        # left-click on empty plot area
    sigDoubleClicked = QtCore.pyqtSignal()           # double-click → restore auto-Y

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._selected_axis: str | None = None

    def set_selected_axis(self, axis: str | None) -> None:
        self._selected_axis = axis

    def wheelEvent(self, ev, axis=None):
        if self._selected_axis == 'x':
            super().wheelEvent(ev, axis=0)
        elif self._selected_axis == 'y':
            super().wheelEvent(ev, axis=1)
        else:
            super().wheelEvent(ev, axis)

    def mouseClickEvent(self, ev):
        """Left-click on empty plot area → deselect axis."""
        if ev.button() == QtCore.Qt.LeftButton:
            self.sigPlotAreaClicked.emit()
            ev.accept()
            return
        super().mouseClickEvent(ev)

    def mouseDoubleClickEvent(self, ev):
        """Double-click on plot → restore Y auto-range."""
        self.sigDoubleClicked.emit()
        ev.accept()


class _Legend(pg.LegendItem):
    """Legend subclass with custom left/right-click handling.

    - Left-click  → highlight / unhighlight the curve.
    - Right-click → per-curve context menu (show / hide / remove).
    - Hover      → hand cursor on sample items.
    """

    sigSampleClicked = QtCore.pyqtSignal(object)       # PlotDataItem (left-click)
    sigSampleRightClicked = QtCore.pyqtSignal(object)   # PlotDataItem (right-click)

    def mousePressEvent(self, ev: QtGui.QMouseEvent) -> None:
        for sample, _label in self.items:
            local = sample.mapFromScene(ev.scenePos())
            if sample.boundingRect().contains(local):
                if ev.button() == QtCore.Qt.RightButton:
                    self.sigSampleRightClicked.emit(sample.item)
                    ev.accept()
                    return
                elif ev.button() == QtCore.Qt.LeftButton:
                    # Handle left-click ourselves to prevent the default
                    # ItemSample.mouseClickEvent (which toggles visibility)
                    # and to use a known signal signature.
                    self.sigSampleClicked.emit(sample.item)
                    ev.accept()
                    return
        super().mousePressEvent(ev)

    def hoverEvent(self, ev):
        """Change cursor shape when hovering over a legend sample.

        NOTE: ``HoverEvent`` with ``exitOnly=True`` has no ``scenePos()``,
        so we guard against that to avoid ``AttributeError``.
        """
        try:
            scene_pos = ev.scenePos()
        except AttributeError:
            # exit-only event — nothing to hit-test against
            self.setCursor(QtCore.Qt.ArrowCursor)
            super().hoverEvent(ev)
            return

        for sample, _label in self.items:
            local = sample.mapFromScene(scene_pos)
            if sample.boundingRect().contains(local):
                self.setCursor(QtCore.Qt.PointingHandCursor)
                return
        self.setCursor(QtCore.Qt.ArrowCursor)
        super().hoverEvent(ev)


class PlotPanel(QtWidgets.QWidget):
    """pyqtgraph-based plot panel for time-series traces."""

    # Emitted when the user clicks "↺  Reset Data" — the receiver
    # (MainWindow) should clear the DataModel so curves restart from 0.
    clear_data_requested = QtCore.pyqtSignal()
    # Emitted when the user clicks "✕  清除曲线".
    clear_curves_requested = QtCore.pyqtSignal()
    # Emitted when record toggle is clicked (True=start, False=stop).
    record_toggle_requested = QtCore.pyqtSignal(bool)
    # Emitted when the pause button is toggled (True = paused).
    pause_toggled = QtCore.pyqtSignal(bool)

    # Emitted for tag configuration (plot layout) save/load.
    save_tag_config_requested = QtCore.pyqtSignal()
    load_tag_config_requested = QtCore.pyqtSignal()

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._curves: Dict[str, pg.PlotDataItem] = {}
        self._labels: Dict[str, str] = {}
        self._visible: Dict[str, bool] = {}
        self._colors: Dict[str, tuple] = {}
        self._color_idx = 0
        self._highlighted: Optional[str] = None   # key of highlighted curve
        self._selected_axis: str | None = None     # 'x', 'y', or None
        self._paused = False                       # pause auto-follow
        self._readonly = False                     # read-only mode (trajectory import)
        self._auto_y_enabled = True                # Y auto-range on; user scroll = off
        self._curve_start_time: Dict[str, float] = {}  # t ≥ this → visible
        self._recording = False                    # recording state for toggle button icon/text

        self._setup_ui()
        self.set_recording(False)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # ---- toolbar ----
        bar = QtWidgets.QHBoxLayout()

        title = QtWidgets.QLabel("Plot View")
        title.setStyleSheet(
            "font-weight: 600; color: #1e293b; padding: 6px 0;"
        )
        bar.addWidget(title)
        bar.addStretch()

        # 「☰」global actions dropdown
        self._actions_btn = QtWidgets.QPushButton("⚙  操作")
        self._actions_btn.clicked.connect(self._show_actions_menu)
        bar.addWidget(self._actions_btn)

        # 「🔄」reset data — clear model data, curves restart from 0
        self._reset_data_btn = QtWidgets.QPushButton("🔄  重置数据")
        self._reset_data_btn.clicked.connect(self.clear_data_requested.emit)
        self._reset_data_btn.setToolTip("Clear accumulated data, curves start from 0")
        bar.addWidget(self._reset_data_btn)

        # 「🗑」clear curves — remove all curve items and legend entries
        self._clear_curves_btn = QtWidgets.QPushButton("🗑  清除曲线")
        self._clear_curves_btn.clicked.connect(self.clear_curves_requested.emit)
        self._clear_curves_btn.setToolTip("Clear curves & data, stay paused to add new curves")
        bar.addWidget(self._clear_curves_btn)

        self._record_toggle_btn = QtWidgets.QPushButton("▶️  开始录制")
        self._record_toggle_btn.clicked.connect(self._on_record_toggle_clicked)
        self._record_toggle_btn.setToolTip("Start recording via record_playback plugin")
        bar.addWidget(self._record_toggle_btn)

        self._pause_btn = QtWidgets.QPushButton("⏸  暂停")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        self._pause_btn.setChecked(True)   # start paused — time frozen at 0
        bar.addWidget(self._pause_btn)

        layout.addLayout(bar)

        # ---- pyqtgraph plot (custom axes + viewbox) ----
        self._x_axis = _ClickableAxis("bottom")
        self._y_axis = _ClickableAxis("left", plain_decimal=True)
        self._zoom_viewbox = _ZoomViewBox()

        self._plot_widget = pg.PlotWidget(
            viewBox=self._zoom_viewbox,
            axisItems={"bottom": self._x_axis, "left": self._y_axis},
        )
        self._plot_widget.setBackground("#ffffff")
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setLabel("bottom", "Time", units="s")
        self._plot_widget.getPlotItem().setMenuEnabled(False)
        self._plot_widget.getPlotItem().vb.setMenuEnabled(False)

        # Wire axis click → selection
        self._x_axis.sigSelected.connect(self._on_axis_clicked)
        self._y_axis.sigSelected.connect(self._on_axis_clicked)
        self._zoom_viewbox.sigPlotAreaClicked.connect(self._on_deselect_axes)

        # User manually changes Y range (scroll / drag) → disable auto-Y
        self._zoom_viewbox.sigRangeChangedManually.connect(self._on_y_range_changed)
        # Double-click plot → restore Y auto-range
        self._zoom_viewbox.sigDoubleClicked.connect(self._restore_auto_y)

        # Custom legend — replace the default one so plot(…, name=…) uses it
        self._legend = _Legend(offset=(-5, 5), labelTextSize="9pt")
        self._legend.setParentItem(self._plot_widget.getPlotItem())
        self._plot_widget.getPlotItem().legend = self._legend
        # NOTE: sigSampleClicked is our custom _Legend signal,
        # NOT pg.LegendItem.sigSampleClicked (which has a different signature).
        self._legend.sigSampleClicked.connect(self._on_legend_left_click)
        self._legend.sigSampleRightClicked.connect(self._on_legend_right_click)

        # Axis styling (default, no selection)
        self._x_axis.setPen(AXIS_PEN_NORMAL)
        self._x_axis.setTextPen(AXIS_TEXT_NORMAL)
        self._y_axis.setPen(AXIS_PEN_NORMAL)
        self._y_axis.setTextPen(AXIS_TEXT_NORMAL)

        layout.addWidget(self._plot_widget)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_curve(self, key: str, label: str,
                  start_time: float | None = None) -> bool:
        """Add a new curve.

        *start_time* — if given, only data with ``t ≥ start_time`` is
        plotted (so historical data before the curve was added is hidden).
        Pass ``None`` (default) to show all accumulated data.
        """
        if key in self._curves:
            return False

        color = CURVE_COLORS[self._color_idx % len(CURVE_COLORS)]
        self._color_idx += 1

        curve = self._plot_widget.plot(
            [], [],
            pen=pg.mkPen(color=color, width=NORMAL_WIDTH),
            name=label,
            autoDownsample=True,
            downsampleMethod='peak',
        )
        self._curves[key] = curve
        self._labels[key] = label
        self._visible[key] = True
        self._colors[key] = color
        self._curve_start_time[key] = start_time if start_time is not None else float("-inf")
        return True

    def remove_curve(self, key: str) -> bool:
        """Remove a curve by key. Returns False if not found."""
        curve = self._curves.pop(key, None)
        if curve is None:
            return False
        self._labels.pop(key, None)
        self._visible.pop(key, None)
        self._colors.pop(key, None)
        self._curve_start_time.pop(key, None)
        if self._highlighted == key:
            self._highlighted = None
        self._plot_widget.removeItem(curve)
        return True

    def toggle_curve(self, key: str) -> bool:
        """Toggle curve visibility. Returns new state."""
        if key not in self._curves:
            return False
        vis = not self._visible.get(key, True)
        self._visible[key] = vis
        self._curves[key].setVisible(vis)
        return vis

    # ------------------------------------------------------------------
    # Tag configuration (plot layout save/load)
    # ------------------------------------------------------------------

    def get_curve_info(self) -> list[dict]:
        """Return a list of dicts describing every currently plotted curve.

        Each dict has keys ``key``, ``label``, ``visible``.
        This is the format expected by :meth:`restore_from_curve_info`.
        """
        return [
            {"key": k, "label": self._labels.get(k, k),
             "visible": self._visible.get(k, True)}
            for k in self._curves
        ]

    def restore_from_curve_info(self, curves_info: list[dict],
                                 model) -> None:
        """Restore plotted curves from a saved tag configuration.

        Args:
            curves_info: List of dicts with ``key``, ``label``, ``visible``
                         (as returned by :meth:`get_curve_info`).
            model: The ``DataModel`` instance to pull data from.
        """
        for info in curves_info:
            key = info["key"]
            label = info.get("label", key)
            visible = info.get("visible", True)

            if key in self._curves:
                continue

            # Determine start_time from existing model data
            series = model.get_series(key)
            start_time = series.times[-1] if (series and series.times) else None
            self.add_curve(key, label, start_time=start_time)
            self.set_curve_visible(key, visible)

    # ------------------------------------------------------------------
    # Trajectory import — add curves for externally loaded data
    # ------------------------------------------------------------------

    def add_trajectory_curves(self, curves: dict,
                               labels: dict[str, str]) -> None:
        """Add curves for imported trajectory data.

        Each curve is added without a *start_time* filter so all historical
        data is immediately visible.

        Args:
            curves: ``{key: (times, values)}``.
            labels: ``{key: display_label}``.
        """
        for key in curves:
            label = labels.get(key, key)
            if key not in self._curves:
                self.add_curve(key, label, start_time=None)

    def set_curve_visible(self, key: str, visible: bool) -> None:
        if key not in self._curves:
            return
        self._visible[key] = visible
        self._curves[key].setVisible(visible)

    def has_curve(self, key: str) -> bool:
        return key in self._curves

    @property
    def curve_keys(self):
        return list(self._curves.keys())

    @property
    def curve_count(self) -> int:
        return len(self._curves)

    def update_curves(self, model, force: bool = False) -> None:
        """Push latest data from model into all curves.

        When *paused*, ``setData`` is skipped for performance — the
        visual is frozen.  Pass ``force=True`` to do a one-time render
        even while paused (used after importing a trajectory or after
        resetting data so the plot immediately reflects the new model
        state).

        Auto-follow (X/Y range) is always skipped when paused regardless
        of *force*, so the user's current viewport is preserved.
        """
        if not self._paused or force:
            for key, curve in self._curves.items():
                series = model.get_series(key)
                if series is None:
                    curve.setData([], [])  # clear visual when data gone
                    continue
                times, values = series.times, series.values
                start_t = self._curve_start_time.get(key, float("-inf"))
                if start_t > float("-inf") and times:
                    i = bisect.bisect_left(times, start_t)
                    if i > 0:
                        times, values = times[i:], values[i:]
                curve.setData(times, values)

        if not self._paused:
            self._auto_range_x()
            self._auto_range_y()

        # Always refresh the label so curve count is accurate
        self._refresh_actions_label()

    def _refresh_actions_label(self) -> None:
        count = self.curve_count
        self._actions_btn.setText(f"⚙  操作  ({count})" if count else "⚙  操作")

    def clear_all(self) -> None:
        for curve in list(self._curves.values()):
            self._plot_widget.removeItem(curve)
        self._curves.clear()
        self._labels.clear()
        self._visible.clear()
        self._colors.clear()
        self._curve_start_time.clear()
        self._highlighted = None
        self._color_idx = 0
        self._legend.clear()
        self._refresh_actions_label()

    # ------------------------------------------------------------------
    # Highlight
    # ------------------------------------------------------------------

    def _highlight(self, key: str) -> None:
        """Highlight one curve, reset others to normal width."""
        if self._highlighted == key:
            # Already highlighted — do nothing (toggle-off via clicking elsewhere)
            return
        # Reset previous highlight
        if self._highlighted is not None and self._highlighted in self._curves:
            prev = self._curves[self._highlighted]
            color = self._colors[self._highlighted]
            prev.setPen(pg.mkPen(color=color, width=NORMAL_WIDTH))
        # Apply new highlight
        self._highlighted = key
        curve = self._curves[key]
        color = self._colors[key]
        curve.setPen(pg.mkPen(color=color, width=HIGHLIGHT_WIDTH))
        # Push this curve to top of Z-order so it renders above others
        curve.setZValue(10)

    def _unhighlight(self) -> None:
        if self._highlighted is None:
            return
        key = self._highlighted
        self._highlighted = None
        if key in self._curves:
            curve = self._curves[key]
            color = self._colors[key]
            curve.setPen(pg.mkPen(color=color, width=NORMAL_WIDTH))
            curve.setZValue(0)

    # ------------------------------------------------------------------
    # Legend interaction
    # ------------------------------------------------------------------

    def _on_legend_left_click(self, item: pg.PlotDataItem) -> None:
        """Left-click legend sample → highlight / unhighlight curve."""
        for key, curve in self._curves.items():
            if curve is item:
                if self._highlighted == key:
                    self._unhighlight()
                else:
                    self._highlight(key)
                return

    def _on_legend_right_click(self, item) -> None:
        """Right-click legend sample → per-curve context menu."""
        # Find the key for this curve
        target_key = None
        for key, curve in self._curves.items():
            if curve is item:
                target_key = key
                break
        if target_key is None:
            return

        label = self._labels[target_key]
        vis = self._visible[target_key]

        menu = QtWidgets.QMenu(self)
        menu.addSection(f"📈  {label}")

        if vis:
            hide_act = menu.addAction("🙈  隐藏")
            hide_act.setData(("hide", target_key))
        else:
            show_act = menu.addAction("👁  显示")
            show_act.setData(("show", target_key))

        remove_act = menu.addAction("🗑  删除")
        remove_act.setData(("remove", target_key))

        action = menu.exec_(QtGui.QCursor.pos())
        if action is None or action.data() is None:
            return

        cmd, key = action.data()
        if cmd == "hide":
            self.set_curve_visible(key, False)
        elif cmd == "show":
            self.set_curve_visible(key, True)
        elif cmd == "remove":
            self.remove_curve(key)

    # ------------------------------------------------------------------
    # Global actions dropdown (⚙)
    # ------------------------------------------------------------------

    def _show_actions_menu(self) -> None:
        menu = QtWidgets.QMenu(self)

        show_all = menu.addAction("显示全部")
        show_all.setData("show_all")

        hide_all = menu.addAction("隐藏全部")
        hide_all.setData("hide_all")

        menu.addSeparator()

        save_tag = menu.addAction("导出标签配置")
        save_tag.setData("save_tag")
        load_tag = menu.addAction("加载标签配置")
        load_tag.setData("load_tag")

        action = menu.exec_(
            self._actions_btn.mapToGlobal(
                QtCore.QPoint(0, self._actions_btn.height())
            )
        )
        if action is None:
            return

        cmd = action.data()
        if cmd == "show_all":
            for k in self._curves:
                self.set_curve_visible(k, True)
        elif cmd == "hide_all":
            for k in self._curves:
                self.set_curve_visible(k, False)
        elif cmd == "save_tag":
            self.save_tag_config_requested.emit()
        elif cmd == "load_tag":
            self.load_tag_config_requested.emit()

    # ------------------------------------------------------------------
    # Axis selection (click-to-highlight → single-axis zoom)
    # ------------------------------------------------------------------

    def _on_axis_clicked(self, orientation: str) -> None:
        """Toggle axis selection on click.

        - ``'bottom'`` → toggle ``'x'`` selection.
        - ``'left'``   → toggle ``'y'`` selection.
        """
        target = 'x' if orientation == 'bottom' else 'y'
        self._set_selected_axis(None if self._selected_axis == target else target)

    def _on_deselect_axes(self) -> None:
        """Deselect both axes (e.g. click on empty plot area)."""
        self._set_selected_axis(None)

    def _set_selected_axis(self, axis: str | None) -> None:
        """Set which axis is selected and update visuals + ViewBox."""
        self._selected_axis = axis
        self._zoom_viewbox.set_selected_axis(axis)

        # Reset both axes to normal
        self._x_axis.setPen(AXIS_PEN_NORMAL)
        self._x_axis.setTextPen(AXIS_TEXT_NORMAL)
        self._y_axis.setPen(AXIS_PEN_NORMAL)
        self._y_axis.setTextPen(AXIS_TEXT_NORMAL)

        # Highlight the selected one
        if axis == 'x':
            self._x_axis.setPen(AXIS_PEN_HIGHLIGHT)
            self._x_axis.setTextPen(AXIS_TEXT_HIGHLIGHT)
        elif axis == 'y':
            self._y_axis.setPen(AXIS_PEN_HIGHLIGHT)
            self._y_axis.setTextPen(AXIS_TEXT_HIGHLIGHT)

    # ------------------------------------------------------------------
    # Pause / resume
    # ------------------------------------------------------------------

    def set_paused(self, paused: bool, readonly: bool = False) -> None:
        """Unified state transition.

        Args:
            paused: ``True`` = freeze sampling & rendering.
            readonly: ``True`` = trajectory-import read-only mode.
                      Only ``🗑 清除曲线`` and ``⚙ 操作`` stay enabled.
        """
        self._paused = paused
        self._readonly = readonly

        # ── Pause button ──────────────────────────────────────────
        self._pause_btn.blockSignals(True)
        self._pause_btn.setChecked(paused)
        self._pause_btn.setEnabled(not readonly)
        if readonly:
            self._pause_btn.setText("📖  只读")
            self._pause_btn.setToolTip("Read-only — click 清除曲线 to exit")
        elif paused:
            self._pause_btn.setText("▶  继续")
            self._pause_btn.setToolTip("Resume data sampling & auto-follow")
        else:
            self._pause_btn.setText("⏸  暂停")
            self._pause_btn.setToolTip("Pause data sampling & auto-follow")
        self._pause_btn.blockSignals(False)

        # ── Toolbar buttons in readonly mode ─────────────────────
        # Only 🗑 清除曲线 and ⚙ 操作 remain clickable.
        self._reset_data_btn.setEnabled(not readonly)
        self._record_toggle_btn.setEnabled(not readonly)

        # ── Sync model ───────────────────────────────────────────
        self.pause_toggled.emit(paused)

        # Auto-follow on resume
        if not paused:
            self._auto_range_x()

    def set_recording(self, recording: bool) -> None:
        """Update record toggle icon/text by recording state."""
        self._recording = recording
        if recording:
            self._record_toggle_btn.setText("⏹  停止录制")
            self._record_toggle_btn.setToolTip("Stop recording via record_playback plugin")
            self._record_toggle_btn.setStyleSheet("")
        else:
            self._record_toggle_btn.setText("▶️  开始录制")
            self._record_toggle_btn.setToolTip("Start recording via record_playback plugin")
            self._record_toggle_btn.setStyleSheet("")

    def _on_record_toggle_clicked(self) -> None:
        """Emit toggle intent; receiver decides start/stop."""
        self.record_toggle_requested.emit(not self._recording)

    def _on_pause_toggled(self, checked: bool) -> None:
        """Signal handler — user clicked the pause button."""
        self.set_paused(checked, readonly=False)

    # ------------------------------------------------------------------
    # Auto-follow (pausable)
    # ------------------------------------------------------------------

    def _auto_range_x(self) -> None:
        """Slide the X-axis viewport so the latest data is centred.

        Preserves the current zoom level (range-width) so the user's
        scroll-wheel zoom on X is not overwritten.
        """
        # Find the max time across all visible curves
        t_max = 0.0
        for key, curve in self._curves.items():
            if not self._visible.get(key, True):
                continue
            data = curve.getData()
            if data is None or data[0] is None or len(data[0]) == 0:
                continue
            t_max = max(t_max, data[0][-1])

        if t_max <= 0:
            return

        # Preserve current X range width instead of using a fixed window
        vb = self._plot_widget.getViewBox()
        x_range = vb.viewRange()[0]
        current_width = x_range[1] - x_range[0]
        if current_width <= 0:
            current_width = config.initial_x_window_s

        # Centre the latest data point in the viewport
        half = current_width / 2.0
        self._plot_widget.setXRange(t_max - half, t_max + half, padding=0)

    def _auto_range_y(self) -> None:
        """Ensure Y-axis fits all visible curves — unless the user has
        manually zoomed/panned Y, in which case we stay out of the way."""
        if not self._auto_y_enabled:
            return
        y_min = float("inf")
        y_max = float("-inf")
        for key, curve in self._curves.items():
            if not self._visible.get(key, True):
                continue
            data = curve.getData()
            if data is None or data[0] is None or len(data[0]) == 0:
                continue
            vals = data[1]
            y_min = min(y_min, min(vals))
            y_max = max(y_max, max(vals))
        if y_min < y_max:
            # Enforce minimum Y range so tiny deviations remain visible
            data_range = y_max - y_min
            min_range = config.y_range_min
            if min_range > 0 and data_range < min_range:
                mid = (y_max + y_min) / 2.0
                half = min_range / 2.0
                y_min, y_max = mid - half, mid + half
            vb = self._plot_widget.getViewBox()
            vb.setYRange(y_min, y_max, padding=0.1)

    def _on_y_range_changed(self, *args) -> None:
        """User scrolled or dragged the Y axis — disable auto-Y so the
        manual view is preserved.

        ``sigRangeChangedManually`` signature varies across pyqtgraph
        versions (1 or 2 args); *args absorbs both.
        """
        self._auto_y_enabled = False

    def _restore_auto_y(self) -> None:
        """Re-enable Y auto-range and immediately fit all visible curves."""
        self._auto_y_enabled = True
        self._auto_range_y()

