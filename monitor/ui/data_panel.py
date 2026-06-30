"""
Data display panel — two tables side by side showing the latest snapshot
from ``joint_state`` (top) and ``joint_cmd`` (bottom).

Right-click a cell to add / remove the corresponding series from the plot.
"""

from __future__ import annotations

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

    def __init__(
        self,
        source: str,
        title: str,
        parent: Optional[QtWidgets.QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._source = source
        self._rows: dict[str, int] = {}   # joint_name → row index
        self._name_by_row: dict[int, str] = {}
        self._plotted_keys: set[str] = set()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # -- header label --
        lbl = QtWidgets.QLabel(title)
        lbl.setStyleSheet(
            "QLabel { font-weight: 600;"
            " color: #1e293b; padding: 6px 0; }"
        )
        layout.addWidget(lbl)

        # -- table --
        ncols = 1 + len(DISPLAY_FIELDS)
        self._table = QtWidgets.QTableWidget(0, ncols)
        headers = ["Joint"] + [
            f"{f.capitalize()} {FIELD_UNITS.get(f, '')}"
            for f in DISPLAY_FIELDS
        ]
        self._table.setHorizontalHeaderLabels(headers)
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
                text = f"{value:.6f}"
                item = self._table.item(row, idx)
                if item is None:
                    item = QtWidgets.QTableWidgetItem(text)
                    item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                    item.setTextAlignment(
                        QtCore.Qt.AlignCenter
                    )
                    self._table.setItem(row, idx, item)
                else:
                    item.setText(text)

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


class DataPanel(QtWidgets.QWidget):
    """Container for the two source tables."""

    add_to_plot_requested = QtCore.pyqtSignal(str, str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self._plotted_keys: set[str] = set()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # -- joint_state table --
        self._state_table = _DataTableWidget(
            source="joint_state",
            title="Joint State Data",
        )
        self._state_table.add_to_plot_requested.connect(self._relay)
        layout.addWidget(self._state_table)

        # -- joint_cmd table --
        self._cmd_table = _DataTableWidget(
            source="joint_cmd",
            title="Joint Command Data",
        )
        self._cmd_table.add_to_plot_requested.connect(self._relay)
        layout.addWidget(self._cmd_table)

        # 底部 stretch 吸收多余空间：窗口远大于表格时不会把表格拉高，
        # 表格保持自然高度，空白留在底部。
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

    def set_plotted_keys(self, keys: set[str]) -> None:
        self._plotted_keys = keys
        self._state_table._plotted_keys = keys
        self._cmd_table._plotted_keys = keys
