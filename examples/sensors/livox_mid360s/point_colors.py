"""Range colouring shared by the publisher's viewer overlay and the reader.

Both draw the same cloud, so they use the same palette; a point that reads as
"far" in one window must read as "far" in the other.
"""

from __future__ import annotations

import numpy as np

TURBO = np.array([
    [0.19, 0.07, 0.23], [0.27, 0.35, 0.80], [0.10, 0.65, 0.93], [0.19, 0.87, 0.72],
    [0.56, 0.99, 0.35], [0.87, 0.90, 0.22], [0.99, 0.65, 0.14], [0.92, 0.32, 0.05],
    [0.60, 0.09, 0.02],
])

FULL_SCALE_M = 25.0


def turbo(normalised: np.ndarray) -> np.ndarray:
    """Map values in 0..1 onto the palette. Out-of-range values clamp."""
    t = np.clip(normalised, 0, 1) * (len(TURBO) - 1)
    i = np.clip(t.astype(int), 0, len(TURBO) - 2)
    u = (t - i)[:, None]
    return TURBO[i] * (1 - u) + TURBO[i + 1] * u


def by_range(distances: np.ndarray, full_scale: float = FULL_SCALE_M) -> np.ndarray:
    """Colour each point by how far away it is."""
    return turbo(np.asarray(distances, dtype=np.float64) / full_scale)
