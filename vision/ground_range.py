"""Floor distance from an image row, using the horizon (VP) row.

Flat floor, camera pitched down a little: a floor point at image row y (px,
0 = top) lies d = k / (y - y_vp) ahead of the camera, with k = f * h (focal
length in px times camera height) and y_vp the horizon row (the lane VP row,
above the image on this car, ~-340). Measured from the front axle centre
(the car steers with the rear wheels, so the front axle is the point that
runs a clean arc in a turn):

    D(y) = c + k / (y - y_vp)

k and c are fixed by the camera mount, so they are fitted once from floor
seams of known distance (fit_ground_range). y_vp is read live, so D stays
right when the car pitches. With a strong pitch the 1/(y - y_vp) model
drifts; the fit residual shows it.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GroundRange:
    k_px_cm: float = 0.0  # f * h; 0 = not calibrated
    c_cm: float = 0.0  # front axle centre to the camera's floor point

    @property
    def enabled(self) -> bool:
        return self.k_px_cm > 0.0

    def distance_cm(self, y_px: float, vp_px: float) -> float | None:
        """Front axle centre to the floor point at row y_px."""
        dy = float(y_px) - float(vp_px)
        if not self.enabled or dy <= 1.0:
            return None
        return self.c_cm + self.k_px_cm / dy

    def row_px(self, d_cm: float, vp_px: float) -> float | None:
        """Image row of the floor point d_cm ahead of the front axle centre."""
        r = float(d_cm) - self.c_cm
        if not self.enabled or r <= 1.0:
            return None
        return float(vp_px) + self.k_px_cm / r


@dataclass(frozen=True)
class GroundFit:
    k_px_cm: float
    c_cm: float
    rms_cm: float
    residuals_cm: list[float]

    def as_dict(self) -> dict[str, object]:
        return {
            "k_px_cm": round(self.k_px_cm, 1),
            "c_cm": round(self.c_cm, 2),
            "rms_cm": round(self.rms_cm, 2),
            "residuals_cm": [round(r, 2) for r in self.residuals_cm],
        }


def fit_ground_range(points: list[tuple[float, float, float]]) -> GroundFit:
    """Least squares D = c + k * u, u = 1 / (y - y_vp).

    points: (distance_cm, y_px, vp_px) per floor point; needs two at
    different rows. Raises ValueError otherwise."""
    rows = [(float(d), 1.0 / (float(y) - float(vp))) for d, y, vp in points if float(y) - float(vp) > 1.0]
    if len(rows) < 2:
        raise ValueError("need at least two floor points below the horizon")
    n = len(rows)
    mu = sum(u for _, u in rows) / n
    md = sum(d for d, _ in rows) / n
    suu = sum((u - mu) ** 2 for _, u in rows)
    if suu <= 1e-12:
        raise ValueError("floor points must be at different image rows")
    k = sum((u - mu) * (d - md) for d, u in rows) / suu
    if k <= 0:
        raise ValueError("fit gave k <= 0: rows and distances do not match (nearer seams must sit lower)")
    c = md - k * mu
    res = [d - (c + k * u) for d, u in rows]
    rms = (sum(r * r for r in res) / n) ** 0.5
    return GroundFit(k_px_cm=k, c_cm=c, rms_cm=rms, residuals_cm=res)


def seam_points(
    lines_px: list[float], vp_px: float, nearest_cm: float, tile_cm: float
) -> list[tuple[float, float, float]]:
    """Fit points from one standstill frame: the lowest seam is nearest_cm
    ahead of the front axle, each next one up is tile_cm further."""
    ys = sorted((float(y) for y in lines_px), reverse=True)
    return [(nearest_cm + i * tile_cm, y, vp_px) for i, y in enumerate(ys)]
