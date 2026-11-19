"""Packing helpers for the web UI (no ROS imports)."""
from __future__ import annotations

import numpy as np


def rle(values: np.ndarray) -> list[int]:
    """[v0, n0, v1, n1, ...] for a 1-D int array."""
    v = np.asarray(values).ravel()
    if v.size == 0:
        return []
    idx = np.flatnonzero(np.diff(v)) + 1
    starts = np.concatenate(([0], idx))
    counts = np.diff(np.concatenate((starts, [v.size])))
    out = np.empty(2 * starts.size, dtype=np.int64)
    out[0::2], out[1::2] = v[starts], counts
    return out.tolist()
