"""
Trajectory data manager.

Export/import full time-series data using NumPy NPZ (compressed) format.
File extension: ``.npz``

NPZ is a compressed archive of named NumPy arrays.  Each curve is stored
as three arrays: ``{safe_key}_times``, ``{safe_key}_values``, and
``{safe_key}_label``.  A ``_keys`` array records the original keys so the
curves can be reconstructed during import.

Usage::

    from monitor.core.trajectory_manager import TrajectoryManager

    # Export
    TrajectoryManager.export("recording.npz", curves, labels)

    # Import
    result = TrajectoryManager.load("recording.npz")
    curves = result["curves"]     # {key: (times, values)}
    labels = result["labels"]     # {key: label}
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class TrajectoryManager:
    """Handles NPZ-based trajectory export and import."""

    @staticmethod
    def export(
        filepath: str,
        curves: Dict[str, Tuple[List[float], List[float]]],
        labels: Dict[str, str],
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Export trajectory data to a compressed NPZ file.

        .. note::
            The actual file on disk will have a ``.npz`` extension
            (numpy appends it automatically).  The ``.traj`` extension
            is used for display / file-dialog filtering purposes only.

        Args:
            filepath: Output path (e.g. ``recording.npz``).
            curves: ``{key: (times_list, values_list)}``.
            labels: ``{key: display_label}``.
            metadata: Optional extra info (source, sample_rate, duration, etc.).
        """
        arrays: Dict[str, Any] = {}

        for key, (times, values) in curves.items():
            safe_key = key.replace("/", "_").replace(".", "_")
            arrays[f"{safe_key}_times"] = np.array(times, dtype=np.float64)
            arrays[f"{safe_key}_values"] = np.array(values, dtype=np.float64)
            arrays[f"{safe_key}_label"] = np.array([labels.get(key, key)])

        # Store the ordered list of original keys so we can reconstruct
        arrays["_keys"] = np.array(list(curves.keys()))

        # Optional metadata
        if metadata:
            for k, v in metadata.items():
                arrays[f"_meta_{k}"] = np.array([v])

        np.savez_compressed(filepath, **arrays)

    @staticmethod
    def load(filepath: str) -> Dict[str, Any]:
        """Load trajectory data from a compressed NPZ file.

        Args:
            filepath: Path to a ``.npz`` file.

        Returns:
            A dict with keys:

            - ``curves``: ``{key: (times_list, values_list)}``
            - ``labels``: ``{key: label_string}``
            - ``metadata``: ``{meta_key: value}``
        """
        data = np.load(filepath)
        keys: List[str] = list(data["_keys"])

        curves: Dict[str, Tuple[List[float], List[float]]] = {}
        labels: Dict[str, str] = {}

        for key in keys:
            safe_key = key.replace("/", "_").replace(".", "_")
            curves[key] = (
                data[f"{safe_key}_times"].tolist(),
                data[f"{safe_key}_values"].tolist(),
            )
            label_arr = data.get(f"{safe_key}_label")
            labels[key] = str(label_arr[0]) if label_arr is not None else key

        # Collect metadata
        metadata: Dict[str, Any] = {}
        for name in data.files:
            if name.startswith("_meta_"):
                meta_key = name[6:]  # strip "_meta_"
                val = data[name]
                metadata[meta_key] = val.item() if val.size == 1 else val.tolist()

        data.close()
        return {"curves": curves, "labels": labels, "metadata": metadata}
