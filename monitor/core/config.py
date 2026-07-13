"""
Configuration loader — reads ``config.yaml`` on first access and caches the
parsed values as a typed singleton.

Usage::

    from monitor.core.config import config

    # Iterate topic sources
    for source, expr in config.topics.items():
        ...

    interval = config.refresh_interval_ms
"""

from __future__ import annotations

import os
from typing import Optional

import yaml


_TOPICS_FALLBACK = {"joint_state": "/geniarm/joint_state"}


class _Config:
    """Typed access to ``config.yaml`` values."""

    def __init__(self, data: dict) -> None:
        self._data = data

    # -- zenoh topics (source_name → topic_expression) ---------------------
    @property
    def topics(self) -> dict[str, str]:
        raw = self._data.get("topics")
        if isinstance(raw, dict):
            return {str(k): str(v) for k, v in raw.items()}
        # Fallback for flat ``topic: "…"`` format
        fallback = self._data.get("topic")
        if fallback:
            return {"joint_state": str(fallback)}
        return dict(_TOPICS_FALLBACK)

    # -- plot refresh interval (ms) ----------------------------------------
    @property
    def refresh_interval_ms(self) -> int:
        return int(self._data.get("refresh_interval_ms", 33))

    # -- initial X-axis window (seconds) -----------------------------------
    @property
    def initial_x_window_s(self) -> float:
        return float(self._data.get("initial_x_window_s", 10.0))

    # -- Y-axis minimum range -----------------------------------------------
    @property
    def y_range_min(self) -> float:
        return float(self._data.get("y_range_min", 0))

    # -- remote zenoh endpoints --------------------------------------------
    @property
    def zenoh_connect_endpoints(self) -> list[str]:
        raw = self._data.get("zenoh_connect_endpoints", [])
        if isinstance(raw, list):
            return [str(e) for e in raw]
        return []

    # -- UI scale factor ----------------------------------------------------
    @property
    def ui_scale(self) -> float:
        return float(self._data.get("ui_scale", 1.0))


# ---------------------------------------------------------------------------
# Singleton
# ---------------------------------------------------------------------------
_config: Optional[_Config] = None


def _load() -> _Config:
    """Load (or reload) ``config.yaml`` from the monitor package directory."""
    path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config.yaml")
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return _Config(data)


def get_config() -> _Config:
    """Return the cached config singleton (loads on first call)."""
    global _config
    if _config is None:
        _config = _load()
    return _config


# Convenience alias for ``from monitor.config import config``.
config = get_config()
