"""
Time-series storage for joint-state data.

Each data series is identified by a ``{joint_name}/{field}`` key
(e.g. ``shoulder_joint/position``).  Data accumulates without limit —
use the "Clear All" button in the plot toolbar to reset when needed.
"""

from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional

from PyQt5 import QtCore

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
# No rolling window — data accumulates until the user manually clears it.
# Memory usage depends on sample rate × uptime; for long sessions, click
# the "Clear All" button in the plot toolbar to reset.
DEFAULT_HISTORY_SECONDS = float("inf")
_MAX_SAMPLES_UNLIMITED = None  # deque maxlen=None → no cap


class JointStateSeries:
    """Per-series ring buffer."""

    __slots__ = ("_times", "_values")

    def __init__(self) -> None:
        # No maxlen — unlimited history until user clears manually.
        self._times: deque[float] = deque(maxlen=_MAX_SAMPLES_UNLIMITED)
        self._values: deque[float] = deque(maxlen=_MAX_SAMPLES_UNLIMITED)

    def append(self, t: float, v: float) -> None:
        self._times.append(t)
        self._values.append(v)

    def trim_older_than(self, t_min: float) -> None:
        """Discard samples with time < *t_min*."""
        while self._times and self._times[0] < t_min:
            self._times.popleft()
            self._values.popleft()

    @property
    def times(self) -> List[float]:
        return list(self._times)

    @property
    def values(self) -> List[float]:
        return list(self._values)

    @property
    def latest(self) -> Optional[float]:
        return self._values[-1] if self._values else None


class DataModel(QtCore.QObject):
    """Holds all joint-state series and emits signals on updates.

    A *series key* looks like ``"shoulder_joint/position"``.

    .. rubric:: Sampling strategy

    *ingest()* only writes into a transient buffer (``_latest_buffer``)
    — no timestamps, no series append.  *sample_snapshot()* is driven
    by an external 10 ms DAQ timer and snapshots the entire buffer into
    every series at the current ``time.time()`` instant.  All series share
    an identical relative timestamp per tick, so the CSV is naturally
    aligned.
    """

    # Emitted after a batch of samples is ingested.
    # payload: {"new_keys": list[str], "latest": dict[str, float]}
    data_updated = QtCore.pyqtSignal(dict)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._series: Dict[str, JointStateSeries] = {}
        self._latest_buffer: Dict[str, float] = {}   # transient — high-frequency overwrite
        self._latest_full: dict = {}                  # last raw message
        self._history_seconds = DEFAULT_HISTORY_SECONDS
        self._start_time: Optional[float] = None      # wall-clock anchor (CSV window filter)
        self._sample_step: float = 0.010              # 10 ms logical tick
        self._next_logical_time: float = 0.0           # logical frame accumulator
        self._sample_count: int = 0
        self._paused: bool = True                     # start paused — time frozen at 0

    # ------------------------------------------------------------------
    # public
    # ------------------------------------------------------------------

    def ingest(self, msg: dict) -> None:
        """Write incoming joint-field / pose values into the transient buffer.

        The buffer is overwritten in-place on every call — no timestamp
        is recorded and nothing is appended to the time series.  That
        happens exclusively in :meth:`sample_snapshot` on the DAQ timer.
        """
        source = msg.get("source", "unknown")
        new_keys: List[str] = []

        # ── Joint data (joints array) ─────────────────────────────
        for j in msg.get("joints", []):
            name = j["name"]
            for field, value in j.items():
                if field == "name":
                    continue
                key = f"{source}/{name}/{field}"

                if key not in self._series:
                    self._series[key] = JointStateSeries()
                    new_keys.append(key)

                self._latest_buffer[key] = value

        # ── Pose data (single pose, no joints) ────────────────────
        for field in ("position_x", "position_y", "position_z",
                      "orientation_x", "orientation_y", "orientation_z",
                      "orientation_w", "roll", "pitch", "yaw"):
            if field not in msg:
                continue
            key = f"{source}/{field}"
            if key not in self._series:
                self._series[key] = JointStateSeries()
                new_keys.append(key)
            self._latest_buffer[key] = msg[field]

        self._latest_full = msg
        if new_keys:
            self.data_updated.emit(
                {"new_keys": new_keys, "latest": dict(self._latest_buffer)}
            )

    def sample_snapshot(self) -> None:
        """Snapshot the transient buffer into every time series using a
        **logical frame counter**.

        Driven by the external 10 ms DAQ timer.  Timestamps are
        mathematically guaranteed to be exactly *sample_step* apart
        (0.000, 0.010, 0.020, …) regardless of OS jitter or timer drift.

        When *paused*, this method returns immediately — the logical
        clock is frozen and no data is written.
        """
        if self._paused or not self._latest_buffer:
            return

        # Anchor wall-clock time once so CSV window filtering still works.
        if self._start_time is None:
            import time as _time
            self._start_time = _time.time()

        t = self._next_logical_time
        for key, value in self._latest_buffer.items():
            self._series[key].append(t, value)

        self._next_logical_time += self._sample_step
        self._sample_count += 1

    # -- pause / resume ---------------------------------------------------

    def pause(self) -> None:
        """Freeze the logical clock.  :meth:`sample_snapshot` becomes a no-op."""
        self._paused = True

    def resume(self) -> None:
        """Unfreeze the logical clock.  Sampling continues from where it left off."""
        self._paused = False

    @property
    def is_paused(self) -> bool:
        return self._paused

    # -- session control --------------------------------------------------

    def reset_and_resume(self) -> None:
        """Clear all data, reset logical time to zero, and **auto-resume**
        so new data starts recording immediately.

        Use for: "Reset Data" — I want to start a fresh recording NOW.
        """
        self.clear_all()
        self._paused = False

    def reset_and_pause(self) -> None:
        """Clear all data, reset logical time to zero, and **stay paused**
        so the user can add curves before manually resuming.

        Use for: "Clear Curves" — I want to look at other data.
        """
        self.clear_all()
        self._paused = True

    @property
    def series_keys(self) -> List[str]:
        return sorted(self._series.keys())

    def get_series(self, key: str) -> Optional[JointStateSeries]:
        return self._series.get(key)

    def export_csv(self, filepath: str, plotted_keys: list[str],
                   t_min: float | None = None,
                   t_max: float | None = None) -> None:
        """Export plotted curves to CSV.

        All series share the same uniform time grid (guaranteed by
        :meth:`sample_snapshot`).  Pick one visible series as the
        global time axis and align every other column by row index.

        Only rows within *csv_export_window_s* (from ``config.yaml``) of
        the current system time are included; set to 0 to export everything.

        Args:
            filepath: Output path (e.g. ``/tmp/data.csv``).
            plotted_keys: Curve keys to include.
            t_min: Optional lower time bound for range export.
            t_max: Optional upper time bound for range export.
        """
        import csv
        import time as _time

        from monitor.core.config import config as _cfg

        window = _cfg.csv_export_window_s
        now_abs = _time.time()

        # Take the time axis from the first non-empty plotted series
        times: list[float] = []
        for key in plotted_keys:
            s = self._series.get(key)
            if s is not None and s.times:
                times = s.times
                break

        if window > 0 and self._start_time is not None:
            cutoff_rel = now_abs - self._start_time - window
            times = [t for t in times if t >= cutoff_rel]

        # Apply range selection (takes priority over window filter)
        if t_min is not None or t_max is not None:
            lo = t_min if t_min is not None else float("-inf")
            hi = t_max if t_max is not None else float("inf")
            times = [t for t in times if lo <= t <= hi]

        if not times:
            return

        columns = ["t"] + [k.replace("/", "_") for k in plotted_keys]

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(columns)
            for i, t in enumerate(times):
                row = [f"{t:.6f}"]
                for key in plotted_keys:
                    s = self._series.get(key)
                    if s is None or i >= len(s.values):
                        row.append("")
                    else:
                        row.append(f"{s.values[i]:.6f}")
                writer.writerow(row)

    def clear_all(self) -> None:
        """Discard all accumulated data and reset the logical clock."""
        self._series.clear()
        self._latest_buffer.clear()
        self._latest_full = {}
        self._start_time = None
        self._next_logical_time = 0.0

    def load_external_data(self, curves: dict) -> float:
        """Load externally recorded trajectory data (e.g. from NPZ import).

        This bypasses the normal ``ingest()`` / ``sample_snapshot()`` pipeline
        and writes directly into the time series.  The logical clock is
        advanced past the imported data so any subsequent live data will
        not collide.

        Args:
            curves: ``{key: (times_list, values_list)}``  where both lists
                    contain ``float`` values.

        Returns:
            The maximum timestamp found across all imported curves (so the
            caller can set the plot view accordingly).
        """
        max_t = 0.0
        for key, (times, values) in curves.items():
            if key not in self._series:
                self._series[key] = JointStateSeries()
            for t, v in zip(times, values):
                self._series[key].append(t, v)
                if t > max_t:
                    max_t = t

            # Also write the latest value into the buffer so the data tables
            # show something immediately.
            if values:
                self._latest_buffer[key] = values[-1]

        # Advance logical time past imported data to avoid overlap
        if max_t >= self._next_logical_time:
            self._next_logical_time = max_t + self._sample_step
        if self._start_time is None:
            import time as _time
            self._start_time = _time.time()

        self.data_updated.emit(
            {"new_keys": list(curves.keys()),
             "latest": dict(self._latest_buffer)}
        )
        return max_t

    @property
    def latest_values(self) -> Dict[str, float]:
        return dict(self._latest_buffer)

    @property
    def latest_message(self) -> dict:
        return self._latest_full

    @property
    def history_seconds(self) -> float:
        return self._history_seconds

    @history_seconds.setter
    def history_seconds(self, value: float) -> None:
        self._history_seconds = max(1.0, min(value, 300.0))
