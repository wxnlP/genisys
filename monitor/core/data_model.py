"""
Time-series storage for joint-state data.

Each data series is identified by a ``{joint_name}/{field}`` key
(e.g. ``shoulder_joint/position``).  Data accumulates without limit —
use the "Clear All" button in the plot toolbar to reset when needed.
"""

from __future__ import annotations

import time as _time
from collections import deque
from typing import Dict, List, Optional

from PyQt5 import QtCore

from monitor.core.record_client import RecordPlaybackClient

_MAX_SAMPLES_UNLIMITED = None  # deque maxlen=None → no cap
_MAX_SAMPLES_PER_SERIES = 200000  # safety cap to avoid unbounded memory growth


class JointStateSeries:
    """Per-series ring buffer."""

    __slots__ = ("_times", "_values")

    def __init__(self) -> None:
        # Per-series safety cap to avoid unbounded growth when topics are noisy.
        self._times: deque[float] = deque(maxlen=_MAX_SAMPLES_PER_SERIES)
        self._values: deque[float] = deque(maxlen=_MAX_SAMPLES_PER_SERIES)

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

    *ingest()* appends samples immediately on message arrival.
    Timestamp source priority:
    1) message ``time_stamp`` (from protocol header)
    2) local arrival monotonic clock fallback.
    """

    # Emitted after a batch of samples is ingested.
    # payload: {"new_keys": list[str], "latest": dict[str, float]}
    data_updated = QtCore.pyqtSignal(dict)

    # Emitted when plugin-backed recording starts/stops/fails.
    record_started = QtCore.pyqtSignal(str)   # output dir
    record_stopped = QtCore.pyqtSignal(str)   # output dir
    record_failed = QtCore.pyqtSignal(str)    # error message

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._series: Dict[str, JointStateSeries] = {}
        self._latest_buffer: Dict[str, float] = {}   # transient — high-frequency overwrite
        self._latest_full: dict = {}                  # last raw message
        # Incoming-message queue (fast append from signal handler), processed
        # in batches by the UI timer to avoid per-message overhead.
        self._ingest_queue: deque[dict] = deque(maxlen=20000)
        self._start_time: Optional[float] = None      # wall-clock anchor
        self._source_time_origin_ns: Optional[int] = None   # first message time_stamp(ns)
        self._arrival_time_origin_ns: Optional[int] = None  # fallback origin (perf_counter_ns)
        self._last_series_t: float = 0.0                    # enforce non-decreasing X
        self._sample_count: int = 0
        self._paused: bool = True                     # start paused — time frozen at 0
        self._record_client = RecordPlaybackClient()

    # ------------------------------------------------------------------
    # public
    # ------------------------------------------------------------------

    def ingest(self, msg: dict) -> None:
        """Backwards-compatible single-message ingest (kept for tests).

        Prefer `ingest_batch()` when processing many messages.
        """
        self.ingest_batch([msg])

    def enqueue(self, msg: dict) -> None:
        """Quickly enqueue a decoded message for later batch processing.

        This method should be used by GUI slots that receive high-rate
        messages so they return quickly and avoid long per-message work.
        """
        try:
            self._ingest_queue.append(msg)
        except Exception:
            # Defensive: ignore queue failures to avoid crashing the UI
            pass

    def process_queue(self, max_items: int = 200) -> int:
        """Drain up to *max_items* messages from the ingest queue and
        process them via a single batch ingest. Returns number processed.
        """
        if not self._ingest_queue:
            return 0
        items = []
        for _ in range(min(max_items, len(self._ingest_queue))):
            try:
                items.append(self._ingest_queue.popleft())
            except IndexError:
                break
        if items:
            self.ingest_batch(items)
        return len(items)

    def ingest_batch(self, msgs: List[dict]) -> None:
        """Process a batch of decoded messages and emit a single update.

        This reduces signal churn and amortizes per-message overhead.
        """
        new_keys: List[str] = []
        for msg in msgs:
            source = msg.get("source", "unknown")
            t = self._resolve_series_time(msg)

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
                    if not self._paused:
                        self._series[key].append(t, value)

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
                if not self._paused:
                    self._series[key].append(t, msg[field])

            self._latest_full = msg
            if not self._paused:
                self._sample_count += 1

        if new_keys:
            # Emit once with all newly created keys and the latest snapshot
            self.data_updated.emit(
                {"new_keys": new_keys, "latest": dict(self._latest_buffer)}
            )

    def _resolve_series_time(self, msg: dict) -> float:
        """Resolve non-decreasing relative time (seconds) for one message."""
        if self._start_time is None:
            self._start_time = _time.time()

        raw_ts = msg.get("time_stamp")
        try:
            ts_ns = int(raw_ts) if raw_ts is not None else 0
        except (TypeError, ValueError):
            ts_ns = 0

        if ts_ns > 0:
            if self._source_time_origin_ns is None:
                self._source_time_origin_ns = ts_ns
            t = (ts_ns - self._source_time_origin_ns) / 1_000_000_000.0
        else:
            now_ns = _time.perf_counter_ns()
            if self._arrival_time_origin_ns is None:
                self._arrival_time_origin_ns = now_ns
            t = (now_ns - self._arrival_time_origin_ns) / 1_000_000_000.0

        if t < self._last_series_t:
            t = self._last_series_t
        else:
            self._last_series_t = t
        return t

    # -- pause / resume ---------------------------------------------------

    def pause(self) -> None:
        """Pause appending incoming samples to the series."""
        self._paused = True

    def resume(self) -> None:
        """Resume appending incoming samples to the series."""
        self._paused = False

    @property
    def is_paused(self) -> bool:
        return self._paused

    # -- session control --------------------------------------------------

    def reset_and_resume(self) -> None:
        """Clear all data, reset timeline to zero, and **auto-resume**
        so new data starts recording immediately.

        Use for: "Reset Data" — I want to start a fresh recording NOW.
        """
        self.clear_all()
        self._paused = False

    def reset_and_pause(self) -> None:
        """Clear all data, reset timeline to zero, and **stay paused**
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

    # ── recording bridge to record_playback plugin ─────────────────────

    def export_recording(
        self,
        action_name: str = "record_joint_state",
        preparation_duration_s: int = 0,
        record_duration_s: int = 0,
    ) -> None:
        """Start a recording via the record_playback plugin and report its output dir."""
        self._run_record_worker(
            action_name,
            mode="start",
            preparation_duration_s=preparation_duration_s,
            record_duration_s=record_duration_s,
        )

    def stop_recording(self, action_name: str = "record_joint_state") -> None:
        """Stop a recording via the record_playback plugin."""
        self._run_record_worker(action_name, mode="stop")

    def set_record_rpc_base_url(self, base_url: str) -> None:
        """Update record_playback RPC endpoint for subsequent requests."""
        self._record_client.set_base_url(base_url)

    def _run_record_worker(
        self,
        action_name: str,
        mode: str,
        preparation_duration_s: int = 0,
        record_duration_s: int = 0,
    ) -> None:
        class Worker(QtCore.QObject):
            done = QtCore.pyqtSignal(str)
            error = QtCore.pyqtSignal(str)

            def __init__(
                self,
                client: RecordPlaybackClient,
                action_name: str,
                mode: str,
                preparation_duration_s: int,
                record_duration_s: int,
            ) -> None:
                super().__init__()
                self._client = client
                self._action_name = action_name
                self._mode = mode
                self._preparation_duration_s = preparation_duration_s
                self._record_duration_s = record_duration_s

            def run(self) -> None:
                try:
                    if self._mode == "start":
                        rsp = self._client.start_record(
                            self._action_name,
                            preparation_duration_s=self._preparation_duration_s,
                            record_duration_s=self._record_duration_s,
                        )
                        output_dir = rsp.get("filefolder") or self._client.default_output_dir
                        self.done.emit(output_dir)
                    elif self._mode == "stop":
                        self._client.stop_record(self._action_name)
                        self.done.emit(self._client.default_output_dir)
                    else:
                        raise ValueError(f"unknown record mode: {self._mode}")
                except Exception as exc:  # pragma: no cover - defensive path
                    self.error.emit(str(exc))

        thread = QtCore.QThread(self)
        worker = Worker(
            self._record_client,
            action_name,
            mode,
            preparation_duration_s,
            record_duration_s,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)

        def _cleanup() -> None:
            thread.quit()
            thread.wait()
            worker.deleteLater()
            thread.deleteLater()

        def _on_done(out: str) -> None:
            if mode == "start":
                self.record_started.emit(out)
            elif mode == "stop":
                self.record_stopped.emit(out)
            _cleanup()

        def _on_error(msg: str) -> None:
            self.record_failed.emit(msg)
            _cleanup()

        worker.done.connect(_on_done)
        worker.error.connect(_on_error)
        thread.start()

    def clear_all(self) -> None:
        """Discard all accumulated data and reset timeline anchors."""
        self._series.clear()
        self._latest_buffer.clear()
        self._latest_full = {}
        self._start_time = None
        self._source_time_origin_ns = None
        self._arrival_time_origin_ns = None
        self._last_series_t = 0.0

    def load_external_data(self, curves: dict) -> float:
        """Load externally recorded trajectory data (e.g. from NPZ import).

        This bypasses the normal ``ingest()`` pipeline
        and writes directly into the time series.

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

        # Keep realtime append monotonic after imported data.
        if max_t > self._last_series_t:
            self._last_series_t = max_t
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

