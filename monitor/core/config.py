"""
Configuration loader — reads the monitor's own settings from ``aimrt_cfg.yaml``.

They live in that file's ``MonitorSubscriberPyModule`` section, which is the same
node AIMRT hands to the worker module. Parsed once and cached as a singleton::

    from monitor.core.config import config

    for source, topic in config.topics.items():
        ...

    interval = config.refresh_interval_ms
"""

from __future__ import annotations

import os
from typing import Optional

import yaml

# Section of aimrt_cfg.yaml holding this program's parameters.
_SECTION = "MonitorSubscriberPyModule"

_TOPICS_FALLBACK = {"joint_state": "/geniarm/joint_state"}


def config_path() -> str:
    """Absolute path of ``monitor/aimrt_cfg.yaml``."""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "aimrt_cfg.yaml")


class _Config:
    """Typed access to the ``MonitorSubscriberPyModule`` section."""

    def __init__(self, data: dict) -> None:
        self._data = data

    @property
    def section(self) -> dict:
        raw = self._data.get(_SECTION)
        return raw if isinstance(raw, dict) else {}

    # -- topics (source_name → topic expression) ---------------------------
    @property
    def topics(self) -> dict[str, str]:
        raw = self.section.get("topics")
        if isinstance(raw, dict) and raw:
            return {str(k): str(v) for k, v in raw.items()}
        return dict(_TOPICS_FALLBACK)

    # -- plot refresh interval (ms) ----------------------------------------
    @property
    def refresh_interval_ms(self) -> int:
        return int(self.section.get("refresh_interval_ms", 33))

    # -- initial X-axis window (seconds) -----------------------------------
    @property
    def initial_x_window_s(self) -> float:
        return float(self.section.get("initial_x_window_s", 10.0))

    # -- Y-axis minimum range ----------------------------------------------
    @property
    def y_range_min(self) -> float:
        return float(self.section.get("y_range_min", 0))

    # -- UI scale factor ---------------------------------------------------
    @property
    def ui_scale(self) -> float:
        return float(self.section.get("ui_scale", 1.0))


# ---------------------------------------------------------------------------
# Singleton
# ---------------------------------------------------------------------------
_config: Optional[_Config] = None


def _load() -> _Config:
    with open(config_path(), encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return _Config(data)


def get_config() -> _Config:
    """Return the cached config singleton (loads on first call)."""
    global _config
    if _config is None:
        _config = _load()
    return _config


config = get_config()
