"""
Data display panel — two tables side by side showing the latest snapshot
from ``joint_state`` (top) and ``joint_cmd`` (bottom).

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
    """Pose table: Position (X Y Z) + Orientation as ZYX Euler angles (RPY).

    Quaternion from the data model is converted to Roll / Pitch / Yaw
    (ZYX convention) for human-readable display.

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
    NCOLS = 3

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
        # For position: msg_key → (row, col)
        self._pos_cells: dict[str, tuple[int, int]] = {}
        # For RPY: just three cell positions
        self._rpy_cells: list[tuple[int, int]] = []
        self._last_rpy: list[float] | None = None

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

        # ── Build table rows (6 rows × 3 cols) ──────────────────
        rows_data: list[list[Optional[tuple[str, str]]]] = []
        # Row 0: Position section header (span 3)
        rows_data.append([("section", "Position (m)")] + [None] * (self.NCOLS - 1))
        # Row 1: X / Y / Z field labels
        labels: list[Optional[tuple[str, str]]] = [None] * self.NCOLS
        for i, (key, label) in enumerate(self._POS_KEYS):
            labels[i] = ("label", label)
        rows_data.append(labels)
        # Row 2: position values
        val_row: list[Optional[tuple[str, str]]] = [None] * self.NCOLS
        for i, (key, label) in enumerate(self._POS_KEYS):
            val_row[i] = ("pos_value", key)
        rows_data.append(val_row)
        # Row 3: Orientation section header (span 3)
        rows_data.append([("section", "Orientation (°)")] + [None] * (self.NCOLS - 1))
        # Row 4: Roll / Pitch / Yaw labels
        labels2: list[Optional[tuple[str, str]]] = [None] * self.NCOLS
        for i, lab in enumerate(self._RPY_LABELS):
            labels2[i] = ("label", lab)
        rows_data.append(labels2)
        # Row 5: RPY values (computed from quaternion)
        rpy_row: list[Optional[tuple[str, str]]] = [None] * self.NCOLS
        for i in range(self.NCOLS):
            rpy_row[i] = ("rpy_value", str(i))
        rows_data.append(rpy_row)

        nrows = len(rows_data)
        self._table = QtWidgets.QTableWidget(nrows, self.NCOLS)
        self._table.setHorizontalHeaderLabels([""] * self.NCOLS)
        self._table.horizontalHeader().setVisible(False)
        self._table.verticalHeader().setVisible(False)
        self._table.setAlternatingRowColors(True)
        self._table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self._table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._table.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self._table.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._on_context_menu)

        hdr = self._table.horizontalHeader()
        for i in range(self.NCOLS):
            hdr.setSectionResizeMode(i, QtWidgets.QHeaderView.Stretch)

        for row, cells in enumerate(rows_data):
            for col, cell in enumerate(cells):
                if cell is None:
                    continue
                typ, val = cell
                item = QtWidgets.QTableWidgetItem(val)
                item.setTextAlignment(QtCore.Qt.AlignCenter)

                if typ == "section":
                    fnt = item.font()
                    fnt.setBold(True)
                    item.setFont(fnt)
                    item.setBackground(QtGui.QColor("#2563eb"))
                    item.setForeground(QtGui.QColor("#ffffff"))
                    self._table.setSpan(row, 0, 1, self.NCOLS)
                    self._table.setItem(row, 0, item)
                    break
                elif typ == "label":
                    fnt = item.font()
                    fnt.setBold(True)
                    item.setFont(fnt)
                    self._table.setItem(row, col, item)
                elif typ == "pos_value":
                    self._table.setItem(row, col, item)
                    self._pos_cells[val] = (row, col)
                elif typ == "rpy_value":
                    self._table.setItem(row, col, item)
                    self._rpy_cells.append((row, col))

        # Store reference to orientation section header for unit update
        self._ori_header_item = self._table.item(3, 0)

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

    def _fit_height(self) -> None:
        self._table.resizeRowsToContents()
        h = self._table.horizontalHeader().height()
        for r in range(self._table.rowCount()):
            h += self._table.rowHeight(r)
        h += 2 * self._table.frameWidth()
        self._table.setMinimumHeight(h)

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

    def _refresh_rpy_display(self) -> None:
        """Re-read stored RPY radians and redisplay based on _use_degrees."""
        if self._last_rpy is None:
            return
        rpy_rad = self._last_rpy
        for i, (row, col) in enumerate(self._rpy_cells):
            item = self._table.item(row, col)
            if item is None:
                continue
            if self._use_degrees:
                item.setText(f"{_math.degrees(rpy_rad[i]):.2f}")
            else:
                item.setText(f"{rpy_rad[i]:.4f}")

    def _toggle_unit(self, checked: bool) -> None:
        self._use_degrees = checked
        self._unit_btn.setText("°" if checked else "rad")
        # Update orientation section header unit
        if self._ori_header_item is not None:
            self._ori_header_item.setText(
                "Orientation (°)" if checked else "Orientation (rad)"
            )
        self._refresh_rpy_display()

    def update_from_message(self, msg: dict) -> None:
        # -- Position values (direct pass-through) --
        for field, (row, col) in self._pos_cells.items():
            item = self._table.item(row, col)
            if item is not None:
                val = msg.get(field)
                if val is not None:
                    item.setText(f"{val:.4f}")
                else:
                    item.setText("—")

        # -- Orientation: RPY values --
        rpy_rad = [msg.get("roll"), msg.get("pitch"), msg.get("yaw")]
        if all(v is not None for v in rpy_rad):
            self._last_rpy = rpy_rad
            self._refresh_rpy_display()
        else:
            self._last_rpy = None
            for row, col in self._rpy_cells:
                item = self._table.item(row, col)
                if item is not None:
                    item.setText("—")

    # ------------------------------------------------------------------
    # Context menu — right-click to add/remove from plot
    # ------------------------------------------------------------------

    def _on_context_menu(self, pos: QtCore.QPoint) -> None:
        """Right-click handler: plot position or quaternion fields."""
        item = self._table.itemAt(pos)
        if item is None:
            return

        row = item.row()
        col = item.column()

        # Row 2 = position values (X / Y / Z)
        if row == 2 and 0 <= col < len(self._POS_KEYS):
            msg_key, short = self._POS_KEYS[col]
            self._show_plot_menu(pos, msg_key, short)
        # Row 5 = RPY values → plot roll / pitch / yaw
        elif row == 5 and 0 <= col < 3:
            rpy_map = {
                0: ("roll", "Roll"),
                1: ("pitch", "Pitch"),
                2: ("yaw", "Yaw"),
            }
            msg_key, short = rpy_map[col]
            self._show_plot_menu(pos, msg_key, short)

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

    def set_plotted_keys(self, keys: set[str]) -> None:
        self._plotted_keys = keys
        self._state_table._plotted_keys = keys
        self._cmd_table._plotted_keys = keys
