"""
Data display panel — tables showing the latest snapshot from each subscribed
topic: ``joint_state`` / ``joint_cmd`` (joint tables) and
``pose_current`` / ``pose_expected`` / ``flange_pose`` (pose tables).

Right-click a cell to add / remove the corresponding series from the plot.
"""

from __future__ import annotations

import math as _math
from typing import Optional

from PyQt5 import QtCore, QtGui, QtWidgets


# Fields shown in both tables (stiffness / damping are fixed values, omitted).
DISPLAY_FIELDS = ("pos", "vel", "eff")

# Mapping from short field name → full key used in the protobuf message.
# This lets us display short column headers while still reading the
# correct field from incoming data.
_FIELD_KEY: dict[str, str] = {
    "pos": "position",
    "vel": "velocity",
    "eff": "effort",
}

FIELD_UNITS = {
    "pos": "(rad)",
    "vel": "(rad/s)",
    "eff": "(Nm)",
}

COL_NAME = 0
_COL_FIELD_START = 1  # first data column


class _DataTableWidget(QtWidgets.QWidget):
    """A labelled table for one data source (e.g. ``joint_state``)."""

    _RAD2DEG = 180.0 / _math.pi
    # Which data columns hold angular values (position=col1, velocity=col2)
    _ANGLE_COLS = (1, 2)

    def __init__(
        self,
        source: str,
        title: str,
        accent_color: str = "#2563eb",
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._source = source
        self._use_degrees = True
        self._rows: dict[str, int] = {}   # joint_name → row index
        self._name_by_row: dict[int, str] = {}
        self._plotted_keys: set[str] = set()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # -- Title row: accent bar + label + unit toggle --
        title_row = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(title)
        lbl.setStyleSheet(
            f"QLabel {{ font-weight: 600;"
            f" color: {accent_color}; padding: 6px 0 6px 8px;"
            f" border-left: 3px solid {accent_color}; }}"
        )
        title_row.addWidget(lbl)
        title_row.addStretch()

        self._unit_btn = QtWidgets.QPushButton("°")
        self._unit_btn.setFixedWidth(36)
        self._unit_btn.setFixedHeight(22)
        self._unit_btn.setToolTip("切换角度/弧度显示 (pos / vel)")
        self._unit_btn.setStyleSheet(
            "QPushButton { font-size: 10pt; padding: 0;"
            " border: 1px solid #cbd5e1; border-radius: 3px; }"
        )
        self._unit_btn.setCheckable(True)
        self._unit_btn.setChecked(True)
        self._unit_btn.clicked.connect(self._toggle_unit)
        title_row.addWidget(self._unit_btn)

        layout.addLayout(title_row)

        # -- table --
        ncols = 1 + len(DISPLAY_FIELDS)
        self._table = QtWidgets.QTableWidget(0, ncols)
        self._headers_rad = ["Joint"] + [
            f"{f.capitalize()} {FIELD_UNITS.get(f, '')}"
            for f in DISPLAY_FIELDS
        ]
        self._headers_deg = ["Joint", "Pos (°)", "Vel (°/s)", "Eff (Nm)"]
        self._table.setHorizontalHeaderLabels(self._headers_deg)
        self._table.setAlternatingRowColors(True)
        self._table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectItems)
        self._table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self._table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._table.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._on_context_menu)
        self._table.verticalHeader().setVisible(False)

        # 禁用内部垂直滚动条 — 表格显示所有行，高度由内容决定。
        # 两个表格共用外层的 QScrollArea 统一滚动。
        self._table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        hdr = self._table.horizontalHeader()
        # 所有列等比例拉伸，随窗口缩放自动调整列宽
        for i in range(ncols):
            hdr.setSectionResizeMode(i, QtWidgets.QHeaderView.Stretch)

        layout.addWidget(self._table)

        self._fit_height()

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def _fit_height(self) -> None:
        """调整表格最小高度使其恰好容纳所有行，不出现内部滚动条。

        仅在行数变化时由调用方触发。
        """
        self._table.resizeRowsToContents()
        h = self._table.horizontalHeader().height()
        for r in range(self._table.rowCount()):
            h += self._table.rowHeight(r)
        h += 2 * self._table.frameWidth()
        self._table.setMinimumHeight(h)

    def _format_value(self, value: float, col: int) -> str:
        """Format a raw joint value for display, applying unit conversion."""
        if self._use_degrees and col in self._ANGLE_COLS:
            return f"{value * self._RAD2DEG:.4f}"
        return f"{value:.6f}"

    def _refresh_display(self) -> None:
        """Redraw all data cells with the current unit setting."""
        hdr = self._table.horizontalHeader()
        hdr.setVisible(False)
        headers = self._headers_deg if self._use_degrees else self._headers_rad
        self._table.setHorizontalHeaderLabels(headers)
        hdr.setVisible(True)

        for row in range(self._table.rowCount()):
            for col in self._ANGLE_COLS:
                item = self._table.item(row, col)
                if item is None:
                    continue
                raw = item.data(QtCore.Qt.UserRole)
                if raw is not None:
                    item.setText(self._format_value(raw, col))

    def _toggle_unit(self, checked: bool) -> None:
        self._use_degrees = checked
        self._unit_btn.setText("°" if checked else "rad")
        self._refresh_display()

    def update_from_message(self, msg: dict) -> None:
        joints = msg.get("joints", [])
        existing = set(self._rows.keys())
        seen: set[str] = set()

        for j in joints:
            name = j["name"]
            seen.add(name)

            if name not in self._rows:
                row = self._table.rowCount()
                self._table.insertRow(row)
                self._rows[name] = row
                self._name_by_row[row] = name

                name_item = QtWidgets.QTableWidgetItem(name)
                name_item.setFlags(name_item.flags() & ~QtCore.Qt.ItemIsEditable)
                name_item.setTextAlignment(
                    QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter
                )
                font = name_item.font()
                font.setBold(True)
                name_item.setFont(font)
                self._table.setItem(row, COL_NAME, name_item)

            row = self._rows[name]
            for idx, field in enumerate(DISPLAY_FIELDS, start=_COL_FIELD_START):
                # Map short display name to the actual protobuf key
                msg_key = _FIELD_KEY.get(field, field)
                if msg_key not in j:
                    continue
                value = j[msg_key]
                item = self._table.item(row, idx)
                if item is None:
                    item = QtWidgets.QTableWidgetItem()
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                    item.setTextAlignment(QtCore.Qt.AlignCenter)
                    self._table.setItem(row, idx, item)
                # Store raw value, display depends on _use_degrees
                item.setData(QtCore.Qt.UserRole, value)
                item.setText(self._format_value(value, idx))

        # Remove stale rows
        for name in existing - seen:
            row = self._rows.pop(name)
            self._name_by_row.pop(row, None)
            self._table.removeRow(row)
            for n, r in list(self._rows.items()):
                if r > row:
                    self._rows[n] = r - 1
            for r in list(self._name_by_row):
                if r > row:
                    self._name_by_row[r - 1] = self._name_by_row.pop(r)

        # 仅在行数变化时重算高度，避免高频数据更新时频繁布局导致卡顿
        if seen != existing:
            self._fit_height()

    def set_plotted_keys(self, keys: set[str]) -> None:
        self._plotted_keys = keys

    # ------------------------------------------------------------------
    # Context menu (delegated to parent via signal)
    # ------------------------------------------------------------------

    add_to_plot_requested = QtCore.pyqtSignal(str, str)

    def _on_context_menu(self, pos: QtCore.QPoint) -> None:
        item = self._table.itemAt(pos)
        if item is None:
            return

        row = item.row()
        col = item.column()
        name = self._name_by_row.get(row)
        if name is None:
            return

        if col == COL_NAME:
            fields = list(DISPLAY_FIELDS)
        elif _COL_FIELD_START <= col < _COL_FIELD_START + len(DISPLAY_FIELDS):
            fields = [DISPLAY_FIELDS[col - _COL_FIELD_START]]
        else:
            return

        plotted = self._plotted_keys
        menu = QtWidgets.QMenu(self)
        for field in fields:
            # Map short display name → original protobuf key for data lookup
            msg_key = _FIELD_KEY.get(field, field)
            key = f"{self._source}/{name}/{msg_key}"
            label = f"{self._source}_{name}_{field}"
            if key in plotted:
                action = menu.addAction(f"✕  Remove {label}")
            else:
                action = menu.addAction(f"📈  Plot {label}")
            action.setData((key, label))

        if menu.actions():
            action = menu.exec_(self._table.viewport().mapToGlobal(pos))
            if action is not None:
                key, label = action.data()
                self.add_to_plot_requested.emit(key, label)


class _PoseTableWidget(QtWidgets.QWidget):
    """Pose table: Position (X Y Z) + Orientation, RPY or quaternion.

    Quaternion from the data model is converted to Roll / Pitch / Yaw
    (ZYX convention) for human-readable display.  The ``RPY`` / ``QUAT``
    button in the title row switches the orientation block between the two
    representations (the position block is unaffected).

      ┌──────────────────────────────┐
      │          Position            │  ← span 3, blue bold
      ├──────┬──────┬────────────────┤
      │  X   │  Y   │       Z        │
      │0.123 │0.456 │     0.789      │
      ├──────┴──────┴────────────────┤
      │      Orientation (RPY)       │  ← span 3, blue bold
      ├──────┬──────┬────────────────┤
      │ Roll │ Pitch│      Yaw       │
      │45.23°│-12.1°│     89.9°     │
      └──────┴──────┴────────────────┘
    """

    _POS_KEYS   = [("position_x", "X"), ("position_y", "Y"), ("position_z", "Z")]
    _RPY_LABELS = ["Roll", "Pitch", "Yaw"]
    _RPY_KEYS   = ["roll", "pitch", "yaw"]
    _QUAT_LABELS = ["X", "Y", "Z", "W"]
    _QUAT_KEYS   = ["orientation_x", "orientation_y", "orientation_z", "orientation_w"]
    # Internal 12-column grid: RPY uses 3 equal columns (span 4), the
    # quaternion uses 4 equal columns (span 3) — one table, equal widths.
    NCOLS = 12
    _SPAN_RPY = 4
    _SPAN_QUAT = 3

    add_to_plot_requested = QtCore.pyqtSignal(str, str)

    def __init__(
        self,
        source: str,
        title: str,
        accent_color: str = "#2563eb",
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._source = source
        self._use_degrees = True  # display toggle: True→°  False→rad
        self._quat_mode = False   # orientation display: False→RPY  True→quaternion
        # For position: msg_key → (row, col)
        self._pos_cells: dict[str, tuple[int, int]] = {}
        # Current orientation cells: msg_key → (row, col)
        self._ori_cells: dict[str, tuple[int, int]] = {}
        self._ori_span = self._SPAN_RPY   # span width currently used (rows 4/5)
        self._last_rpy: list[float] | None = None    # radians
        self._last_quat: list[float] | None = None   # x, y, z, w

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # -- Title row: accent bar + label + unit toggle --
        title_row = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(title)
        lbl.setStyleSheet(
            f"QLabel {{ font-weight: 600;"
            f" color: {accent_color}; padding: 6px 0 6px 8px;"
            f" border-left: 3px solid {accent_color}; }}"
        )
        title_row.addWidget(lbl)
        title_row.addStretch()

        # Orientation mode toggle: ZYX Euler angles ↔ quaternion.
        # Text shows the mode currently in effect, like the °/rad button.
        self._ori_btn = QtWidgets.QPushButton("RPY")
        self._ori_btn.setFixedWidth(44)
        self._ori_btn.setFixedHeight(22)
        self._ori_btn.setToolTip("切换姿态显示：ZYX 欧拉角 (RPY) ↔ 四元数 (X Y Z W)")
        self._ori_btn.setStyleSheet(
            "QPushButton { font-size: 10pt; padding: 0;"
            " border: 1px solid #cbd5e1; border-radius: 3px; }"
        )
        self._ori_btn.setCheckable(True)
        self._ori_btn.setChecked(False)
        self._ori_btn.clicked.connect(self._toggle_orientation)
        title_row.addWidget(self._ori_btn)

        self._unit_btn = QtWidgets.QPushButton("°")
        self._unit_btn.setFixedWidth(36)
        self._unit_btn.setFixedHeight(22)
        self._unit_btn.setToolTip("切换角度/弧度显示")
        self._unit_btn.setStyleSheet(
            "QPushButton { font-size: 10pt; padding: 0;"
            " border: 1px solid #cbd5e1; border-radius: 3px; }"
        )
        self._unit_btn.setCheckable(True)
        self._unit_btn.setChecked(True)
        self._unit_btn.clicked.connect(self._toggle_unit)
        title_row.addWidget(self._unit_btn)

        layout.addLayout(title_row)

        # ── Build table (6 rows on a 12-column grid) ────────────
        self._table = QtWidgets.QTableWidget(6, self.NCOLS)
        self._table.setHorizontalHeaderLabels([""] * self.NCOLS)
        self._table.horizontalHeader().setVisible(False)
        # The 12 grid columns must be allowed to shrink below the style's
        # default minimum section width (36 px), otherwise 12 × 36 > panel
        # width and the table grows its own horizontal scrollbar.
        self._table.horizontalHeader().setMinimumSectionSize(8)
        self._table.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._table.verticalHeader().setVisible(False)
        self._table.setAlternatingRowColors(True)
        self._table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self._table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._table.setWordWrap(False)
        self._table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._table.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._on_context_menu)

        hdr = self._table.horizontalHeader()
        for i in range(self.NCOLS):
            hdr.setSectionResizeMode(i, QtWidgets.QHeaderView.Stretch)

        # Rows 0-2: position section, X/Y/Z labels, position values
        self._set_section(0, "Position (m)")
        for i, (key, label) in enumerate(self._POS_KEYS):
            col = i * self._SPAN_RPY
            self._table.setSpan(1, col, 1, self._SPAN_RPY)
            self._table.setSpan(2, col, 1, self._SPAN_RPY)
            self._set_cell(1, col, label, bold=True)
            self._set_cell(2, col, "—")
            self._pos_cells[key] = (2, col)

        # Row 3: orientation section header (text depends on the mode)
        self._set_section(3, "Orientation (°)")
        self._ori_header_item = self._table.item(3, 0)

        # Rows 4-5: orientation labels + values, built per mode
        self._build_orientation()

        layout.addWidget(self._table)
        self._fit_height()

    # ------------------------------------------------------------------
    # Quaternion → ZYX Euler (RPY)
    # ------------------------------------------------------------------

    @staticmethod
    def _quat_to_rpy(
        qx: float, qy: float, qz: float, qw: float
    ) -> tuple[float, float, float]:
        """Convert quaternion to ZYX Euler angles (Roll, Pitch, Yaw) in **radians**.

        Returns (roll, pitch, yaw) in radians.
        """
        # Roll  (φ) — rotation around X
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = _math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (θ) — rotation around Y
        sinp = 2.0 * (qw * qy - qz * qx)
        if sinp > 1.0:
            sinp = 1.0
        elif sinp < -1.0:
            sinp = -1.0
        pitch = _math.asin(sinp)

        # Yaw   (ψ) — rotation around Z
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = _math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # ------------------------------------------------------------------
    # Height
    # ------------------------------------------------------------------

    def showEvent(self, event: QtGui.QShowEvent) -> None:
        # The table font is only final once the widget is polished, so the
        # heights computed in __init__ can be off; re-fit on first show.
        super().showEvent(event)
        self._fit_height()

    def _fit_height(self) -> None:
        """Set deterministic row heights and the table's minimum height.

        ``resizeRowsToContents()`` cannot be used here: with a 12-column grid
        it measures each item in its own (narrow) column, ignoring spans, so
        wrapped text inflates the rows — and switching RPY ↔ quaternion would
        change the height.  Fixed heights keep the layout stable.
        """
        fm = self._table.fontMetrics()
        normal = fm.height() + 10
        band = fm.height() + 16       # blue section headers
        for row in range(self._table.rowCount()):
            self._table.setRowHeight(row, band if row in (0, 3) else normal)
        h = (band * 2 + normal * (self._table.rowCount() - 2)
             + 2 * self._table.frameWidth())
        self._table.setMinimumHeight(h)

    # ------------------------------------------------------------------
    # Cell helpers
    # ------------------------------------------------------------------

    def _set_section(self, row: int, text: str) -> None:
        """Full-width blue section header cell."""
        item = QtWidgets.QTableWidgetItem(text)
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        fnt = item.font()
        fnt.setBold(True)
        item.setFont(fnt)
        item.setBackground(QtGui.QColor("#2563eb"))
        item.setForeground(QtGui.QColor("#ffffff"))
        self._table.setSpan(row, 0, 1, self.NCOLS)
        self._table.setItem(row, 0, item)

    def _set_cell(self, row: int, col: int, text: str,
                  bold: bool = False) -> QtWidgets.QTableWidgetItem:
        item = QtWidgets.QTableWidgetItem(text)
        item.setTextAlignment(QtCore.Qt.AlignCenter)
        if bold:
            fnt = item.font()
            fnt.setBold(True)
            item.setFont(fnt)
        self._table.setItem(row, col, item)
        return item

    # ------------------------------------------------------------------
    # Orientation block (RPY ↔ quaternion)
    # ------------------------------------------------------------------

    def _apply_static_spans(self) -> None:
        """Span layout that does not depend on the orientation mode."""
        self._table.setSpan(0, 0, 1, self.NCOLS)          # "Position (m)"
        for i in range(len(self._POS_KEYS)):
            col = i * self._SPAN_RPY
            self._table.setSpan(1, col, 1, self._SPAN_RPY)
            self._table.setSpan(2, col, 1, self._SPAN_RPY)
        self._table.setSpan(3, 0, 1, self.NCOLS)          # orientation header

    def _build_orientation(self) -> None:
        """(Re)build orientation labels/values for the current mode."""
        # Drop the previous orientation cells, then rebuild every span:
        # clearSpans() is used because setSpan(…, 1, 1) warns on stderr.
        for col in range(0, self.NCOLS, self._ori_span):
            if self._table.item(4, col) is not None:
                self._table.takeItem(4, col)
            if self._table.item(5, col) is not None:
                self._table.takeItem(5, col)
        self._table.clearSpans()
        self._apply_static_spans()
        self._ori_cells.clear()

        if self._quat_mode:
            labels, keys, span = (self._QUAT_LABELS, self._QUAT_KEYS,
                                  self._SPAN_QUAT)
        else:
            labels, keys, span = (self._RPY_LABELS, self._RPY_KEYS,
                                  self._SPAN_RPY)
        self._ori_span = span

        for i, (label, key) in enumerate(zip(labels, keys)):
            col = i * span
            self._table.setSpan(4, col, 1, span)
            self._table.setSpan(5, col, 1, span)
            self._set_cell(4, col, label, bold=True)
            self._set_cell(5, col, "—")
            self._ori_cells[key] = (5, col)

        self._set_orientation_header()

    def _set_orientation_header(self) -> None:
        if self._ori_header_item is None:
            return
        if self._quat_mode:
            self._ori_header_item.setText("Orientation (quaternion)")
        else:
            self._ori_header_item.setText(
                "Orientation (°)" if self._use_degrees else "Orientation (rad)"
            )

    def _set_ori_text(self, key: str, text: str) -> None:
        cell = self._ori_cells.get(key)
        if cell is None:
            return
        item = self._table.item(*cell)
        if item is not None:
            item.setText(text)

    def _refresh_orientation(self) -> None:
        """Redraw rows 4/5 from cached values, honoring the current mode."""
        if self._quat_mode:
            if self._last_quat is None:
                for key in self._QUAT_KEYS:
                    self._set_ori_text(key, "—")
                return
            for key, value in zip(self._QUAT_KEYS, self._last_quat):
                self._set_ori_text(key, f"{value:.4f}")
            return

        if self._last_rpy is None:
            for key in self._RPY_KEYS:
                self._set_ori_text(key, "—")
            return
        for key, rad in zip(self._RPY_KEYS, self._last_rpy):
            if self._use_degrees:
                self._set_ori_text(key, f"{_math.degrees(rad):.2f}")
            else:
                self._set_ori_text(key, f"{rad:.4f}")

    def _toggle_unit(self, checked: bool) -> None:
        self._use_degrees = checked
        self._unit_btn.setText("°" if checked else "rad")
        self._set_orientation_header()
        self._refresh_orientation()

    def _toggle_orientation(self, quat: bool) -> None:
        """Switch orientation between ZYX Euler angles and quaternion."""
        self._quat_mode = bool(quat)
        self._ori_btn.setText("QUAT" if quat else "RPY")
        self._unit_btn.setEnabled(not quat)  # unit is meaningless for quaternions
        self._build_orientation()
        self._refresh_orientation()
        self._fit_height()

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def update_from_message(self, msg: dict) -> None:
        # -- Position values (direct pass-through) --
        for field, (row, col) in self._pos_cells.items():
            item = self._table.item(row, col)
            if item is not None:
                val = msg.get(field)
                item.setText(f"{val:.4f}" if val is not None else "—")

        # -- Orientation: cache both representations, show the active one --
        rpy = [msg.get(k) for k in self._RPY_KEYS]
        self._last_rpy = rpy if all(v is not None for v in rpy) else None
        quat = [msg.get(k) for k in self._QUAT_KEYS]
        self._last_quat = quat if all(v is not None for v in quat) else None
        self._refresh_orientation()

    # ------------------------------------------------------------------
    # Context menu — right-click to add/remove from plot
    # ------------------------------------------------------------------

    def _plot_target(self, row: int, col: int) -> Optional[tuple[str, str]]:
        """Map a cell (anchor column) to (series key, label) for the plot menu."""
        if row == 2:
            idx = col // self._SPAN_RPY
            if 0 <= idx < len(self._POS_KEYS):
                return self._POS_KEYS[idx]
        elif row == 5:
            if self._quat_mode:
                idx = col // self._SPAN_QUAT
                if 0 <= idx < len(self._QUAT_KEYS):
                    return (self._QUAT_KEYS[idx],
                            "Q" + self._QUAT_LABELS[idx])
            else:
                idx = col // self._SPAN_RPY
                if 0 <= idx < len(self._RPY_KEYS):
                    return self._RPY_KEYS[idx], self._RPY_LABELS[idx]
        return None

    def _on_context_menu(self, pos: QtCore.QPoint) -> None:
        """Right-click handler: plot position / RPY / quaternion fields."""
        index = self._table.indexAt(pos)
        if not index.isValid():
            return
        target = self._plot_target(index.row(), index.column())
        if target is not None:
            self._show_plot_menu(pos, *target)

    def _show_plot_menu(self, pos: QtCore.QPoint,
                        msg_key: str, short: str) -> None:
        """Build and show the right-click plot menu for *msg_key*."""
        key = f"{self._source}/{msg_key}"
        label = f"{self._source}_{short}"

        parent_panel = self.parent()
        plotted: set[str] = getattr(parent_panel, "_plotted_keys", set())

        menu = QtWidgets.QMenu(self)
        if key in plotted:
            action = menu.addAction(f"🗑  Remove {label}")
        else:
            action = menu.addAction(f"📈  Plot {label}")
        action.setData((key, label))

        if menu.actions():
            action = menu.exec_(self._table.viewport().mapToGlobal(pos))
            if action is not None:
                key, label = action.data()
                self.add_to_plot_requested.emit(key, label)


class DataPanel(QtWidgets.QWidget):
    """Container for all data source tables."""

    add_to_plot_requested = QtCore.pyqtSignal(str, str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._plotted_keys: set[str] = set()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # -- joint_state table (blue) --
        self._state_table = _DataTableWidget(
            source="joint_state",
            title="Joint State Data",
            accent_color="#2563eb",
        )
        self._state_table.add_to_plot_requested.connect(self._relay)
        layout.addWidget(self._state_table)

        # -- joint_cmd table --
        self._cmd_table = _DataTableWidget(
            source="joint_cmd",
            title="Joint Command Data",
            accent_color="#2563eb",
        )
        self._cmd_table.add_to_plot_requested.connect(self._relay)
        layout.addWidget(self._cmd_table)

        # -- pose_current table --
        self._pose_cur_table = _PoseTableWidget(
            source="pose_current",
            title="Pose Current",
            accent_color="#2563eb",
        )
        self._pose_cur_table.add_to_plot_requested.connect(self._relay)
        layout.addWidget(self._pose_cur_table)

        # -- pose_expected table --
        self._pose_exp_table = _PoseTableWidget(
            source="pose_expected",
            title="Pose Expected",
            accent_color="#2563eb",
        )
        self._pose_exp_table.add_to_plot_requested.connect(self._relay)
        layout.addWidget(self._pose_exp_table)

        # -- flange_pose table --
        self._flange_pose_table = _PoseTableWidget(
            source="flange_pose",
            title="Flange Pose",
            accent_color="#2563eb",
        )
        self._flange_pose_table.add_to_plot_requested.connect(self._relay)
        layout.addWidget(self._flange_pose_table)

        # 底部 stretch 吸收多余空间
        layout.addStretch(1)

    def _relay(self, key: str, label: str) -> None:
        self.add_to_plot_requested.emit(key, label)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def update_from_message(self, msg: dict) -> None:
        source = msg.get("source", "")
        if source == "joint_state":
            self._state_table.update_from_message(msg)
        elif source == "joint_cmd":
            self._cmd_table.update_from_message(msg)
        elif source == "pose_current":
            self._pose_cur_table.update_from_message(msg)
        elif source == "pose_expected":
            self._pose_exp_table.update_from_message(msg)
        elif source == "flange_pose":
            self._flange_pose_table.update_from_message(msg)

    def set_plotted_keys(self, keys: set[str]) -> None:
        self._plotted_keys = keys
        self._state_table._plotted_keys = keys
        self._cmd_table._plotted_keys = keys
