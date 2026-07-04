"""
Plot tag/layout configuration manager.

Saves/loads the set of plotted curves (keys, labels, visibility) as JSON.
File extension: ``.plottag``

Usage::

    from monitor.core.tag_manager import PlotTagManager

    # Save current plot layout
    PlotTagManager.save("my_layout.plottag", curves_info)

    # Load a previously saved layout
    curves = PlotTagManager.load("my_layout.plottag")
"""

from __future__ import annotations

import json
from typing import Any, Dict, List


class PlotTagManager:
    """Manages serialization of plot curve configurations."""

    @staticmethod
    def save(filepath: str, curves: List[Dict[str, Any]]) -> None:
        """Export plot tag configuration to a JSON file.

        Args:
            filepath: Output path (e.g. ``my_layout.plottag``).
            curves: List of curve dicts with keys ``key``, ``label``, ``visible``.
        """
        data = {
            "version": 1,
            "type": "plot_tag",
            "curves": curves,
        }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    @staticmethod
    def load(filepath: str) -> List[Dict[str, Any]]:
        """Load plot tag configuration from a JSON file.

        Args:
            filepath: Path to a ``.plottag`` file.

        Returns:
            List of curve dicts with keys ``key``, ``label``, ``visible``.

        Raises:
            ValueError: If the file format is invalid.
        """
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict) or data.get("type") != "plot_tag":
            raise ValueError("Invalid plot tag file")
        return data.get("curves", [])
