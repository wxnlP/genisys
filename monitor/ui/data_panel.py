"""
Data display panel — two tables side by side showing the latest snapshot
from ``joint_state`` (top) and ``joint_cmd`` (bottom).

Right-click a cell to add / remove the corresponding series from the plot.
"""

from __future__ import annotations

from typing import Optional

from PyQt5 import QtCore, QtGui, QtWidgets


# Fields shown in both tables (stiffness / damping are fixed values, omitted).
DISPLAY_FIELDS = ("position", "velocity", "effort")

FIELD_UNITS = {
    "position": "(rad)",
    "velocity": "(rad/s)",
    "effort": "(Nm)",
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

        hdr = self._table.horizontalHeader()
        hdr.setStretchLastSection(True)
        hdr.setSectionResizeMode(COL_NAME, QtWidgets.QHeaderView.Stretch)

        layout.addWidget(self._table)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

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
                if field not in j:
                    continue
                value = j[field]
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
            key = f"{self._source}/{name}/{field}"
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

        layout.addStretch()

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
