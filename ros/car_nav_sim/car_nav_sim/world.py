"""One world description -> LiDAR ray casting, occupancy map, Gazebo SDF.

A world file (worlds/*.yaml) lists boxes and cylinders on a flat floor:

    name: test_floor
    bounds: [x_min, y_min, x_max, y_max]   # map extent (m)
    boxes:      [{center: [x, y], size: [sx, sy], yaw: 0.0, height: 1.0}, ...]
    cylinders:  [{center: [x, y], radius: r, height: 1.0}, ...]
    start: [x, y, yaw]                     # car base_link (car centre) start

The world frame is the map frame: the map Nav2 loads is rasterised from
the same shapes, so localisation error = AMCL pose - true pose directly.
No ROS imports (numpy + yaml only), so pytest covers it.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Iterable, Sequence

import numpy as np
import yaml


@dataclass(frozen=True)
class Box:
    cx: float
    cy: float
    sx: float
    sy: float
    yaw: float = 0.0
    height: float = 1.0

    def corners(self) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self.sx / 2.0, self.sy / 2.0
        local = np.array([[hx, hy], [-hx, hy], [-hx, -hy], [hx, -hy]])
        rot = np.array([[c, -s], [s, c]])
        return local @ rot.T + np.array([self.cx, self.cy])


@dataclass(frozen=True)
class Cylinder:
    cx: float
    cy: float
    r: float
    height: float = 1.0


@dataclass
class World:
    name: str
    bounds: tuple[float, float, float, float]
    boxes: list[Box] = field(default_factory=list)
    cylinders: list[Cylinder] = field(default_factory=list)
    start: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ #
    @classmethod
    def load(cls, path: str) -> "World":
        with open(path, "r", encoding="utf-8") as fh:
            d = yaml.safe_load(fh) or {}
        return cls.from_dict(d, default_name=os.path.splitext(os.path.basename(path))[0])

    @classmethod
    def from_dict(cls, d: dict, default_name: str = "world") -> "World":
        boxes = [
            Box(float(b["center"][0]), float(b["center"][1]), float(b["size"][0]), float(b["size"][1]),
                float(b.get("yaw", 0.0)), float(b.get("height", 1.0)))
            for b in d.get("boxes") or []
        ]
        cyl = [
            Cylinder(float(c["center"][0]), float(c["center"][1]), float(c["radius"]), float(c.get("height", 1.0)))
            for c in d.get("cylinders") or []
        ]
        start = tuple(float(v) for v in (d.get("start") or (0.0, 0.0, 0.0)))
        bounds = tuple(float(v) for v in d["bounds"])
        return cls(str(d.get("name", default_name)), bounds, boxes, cyl, start)  # type: ignore[arg-type]

    # ------------------------------------------------------------------ #
    def add_extras(self, mission: dict | None) -> int:
        """Add a mission's `extra_obstacles` (same boxes / cylinders keys as a world file) to this
        World. The map is rasterised from the world FILE, so these are NOT in the map Nav2 localises
        on: a box someone put down, or a low object the LiDAR cannot see (give it a `height`)."""
        extra = World.from_dict({"bounds": self.bounds, **((mission or {}).get("extra_obstacles") or {})})
        self.boxes.extend(extra.boxes)
        self.cylinders.extend(extra.cylinders)
        for attr in ("_segs_cache", "_box_cache", "_ray_segs"):
            if hasattr(self, attr):
                delattr(self, attr)
        return len(extra.boxes) + len(extra.cylinders)

    def segments(self, min_height: float | None = None) -> np.ndarray:
        """Box edges as an (M, 4) array of x0, y0, x1, y1. With min_height only
        boxes taller than that (a LiDAR at height z does not see anything lower)."""
        segs = []
        for b in self.boxes:
            if min_height is not None and b.height <= min_height:
                continue
            c = b.corners()
            for i in range(4):
                segs.append([*c[i], *c[(i + 1) % 4]])
        return np.array(segs, dtype=float).reshape(-1, 4)

    def raycast(self, ox: float, oy: float, angles: np.ndarray, max_range: float,
                z: float | None = None) -> np.ndarray:
        """Distance along each world-frame direction to the first obstacle (inf if none).

        z = height of the (level) scan plane: only obstacles taller than z are hit
        (boxes and cylinders have a height). None = every obstacle."""
        dx, dy = np.cos(angles), np.sin(angles)
        best = np.full(angles.shape, np.inf)
        cache = getattr(self, "_ray_segs", None)
        if cache is None:
            cache = self._ray_segs = {}
        if z not in cache:
            cache[z] = self.segments(z)
        segs = cache[z]
        if len(segs):
            x0, y0, x1, y1 = (segs[:, i][None, :] for i in range(4))
            ex, ey = x1 - x0, y1 - y0
            denom = dx[:, None] * ey - dy[:, None] * ex
            wx, wy = x0 - ox, y0 - oy
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (wx * ey - wy * ex) / denom
                u = (wx * dy[:, None] - wy * dx[:, None]) / denom
            ok = (np.abs(denom) > 1e-12) & (t > 1e-6) & (u >= 0.0) & (u <= 1.0)
            t = np.where(ok, t, np.inf)
            best = np.minimum(best, t.min(axis=1))
        for c in self.cylinders:
            if z is not None and c.height <= z:
                continue
            fx, fy = ox - c.cx, oy - c.cy
            b = fx * dx + fy * dy
            disc = b * b - (fx * fx + fy * fy - c.r * c.r)
            with np.errstate(invalid="ignore"):
                sq = np.sqrt(np.where(disc >= 0, disc, np.nan))
            t0, t1 = -b - sq, -b + sq
            t = np.where(t0 > 1e-6, t0, np.where(t1 > 1e-6, t1, np.inf))
            t = np.where(np.isnan(t), np.inf, t)
            best = np.minimum(best, t)
        best[best > max_range] = np.inf
        return best

    # ------------------------------------------------------------------ #
    def occupancy(self, resolution: float = 0.05, min_height: float | None = None) -> np.ndarray:
        """Boolean grid, row 0 = y_min (flip vertically for a PGM). min_height: only
        obstacles taller than this (what a LiDAR at that height maps)."""
        x0, y0, x1, y1 = self.bounds
        w = int(math.ceil((x1 - x0) / resolution))
        h = int(math.ceil((y1 - y0) / resolution))
        xs = x0 + (np.arange(w) + 0.5) * resolution
        ys = y0 + (np.arange(h) + 0.5) * resolution
        gx, gy = np.meshgrid(xs, ys)
        occ = np.zeros((h, w), dtype=bool)
        half = resolution / 2.0
        for b in self.boxes:
            if min_height is not None and b.height <= min_height:
                continue
            c, s = math.cos(b.yaw), math.sin(b.yaw)
            lx = (gx - b.cx) * c + (gy - b.cy) * s
            ly = -(gx - b.cx) * s + (gy - b.cy) * c
            occ |= (np.abs(lx) <= b.sx / 2.0 + half) & (np.abs(ly) <= b.sy / 2.0 + half)
        for cy in self.cylinders:
            if min_height is not None and cy.height <= min_height:
                continue
            occ |= (gx - cy.cx) ** 2 + (gy - cy.cy) ** 2 <= (cy.r + half) ** 2
        return occ

    def write_map(self, out_dir: str, resolution: float = 0.05, min_height: float | None = None) -> str:
        """Write <name>.pgm + <name>.yaml for nav2 map_server; returns the yaml path."""
        os.makedirs(out_dir, exist_ok=True)
        occ = self.occupancy(resolution, min_height)
        img = np.where(occ, 0, 254).astype(np.uint8)[::-1, :]
        pgm = os.path.join(out_dir, f"{self.name}.pgm")
        with open(pgm, "wb") as fh:
            fh.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode("ascii"))
            fh.write(img.tobytes())
        meta = {
            "image": os.path.basename(pgm),
            "mode": "trinary",
            "resolution": resolution,
            "origin": [self.bounds[0], self.bounds[1], 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
        }
        path = os.path.join(out_dir, f"{self.name}.yaml")
        with open(path, "w", encoding="utf-8") as fh:
            yaml.safe_dump(meta, fh, sort_keys=False)
        return path

    # ------------------------------------------------------------------ #
    def clearance(self, polygon: np.ndarray) -> float:
        """Smallest distance from a closed polygon (N, 2) to any obstacle; 0 if touching."""
        poly_segs = np.hstack([polygon, np.roll(polygon, -1, axis=0)])
        best = math.inf
        # Segments nearest first by a lower bound (gap between bounding boxes); stop once
        # no remaining segment can be closer. Same result as checking all of them, but
        # ~1 ms instead of ~25 ms on the campus map (the mission scorer calls this for every
        # 100 Hz truth message and fell seconds behind, 2026-11-16).
        segs = getattr(self, "_segs_cache", None)
        if segs is None:
            segs = self._segs_cache = self.segments()
        if len(segs):
            px0, py0 = polygon[:, 0].min(), polygon[:, 1].min()
            px1, py1 = polygon[:, 0].max(), polygon[:, 1].max()
            sx0, sx1 = np.minimum(segs[:, 0], segs[:, 2]), np.maximum(segs[:, 0], segs[:, 2])
            sy0, sy1 = np.minimum(segs[:, 1], segs[:, 3]), np.maximum(segs[:, 1], segs[:, 3])
            gx = np.maximum(0.0, np.maximum(sx0 - px1, px0 - sx1))
            gy = np.maximum(0.0, np.maximum(sy0 - py1, py0 - sy1))
            lb = np.hypot(gx, gy)
            for i in np.argsort(lb, kind="stable"):
                if lb[i] > best:
                    break
                for ps in poly_segs:
                    best = min(best, _seg_seg_distance(ps, segs[i]))
                    if best == 0.0:
                        return 0.0
        for c in self.cylinders:
            if _point_in_polygon(c.cx, c.cy, polygon):
                return 0.0
            for ps in poly_segs:
                best = min(best, max(0.0, _point_seg_distance(c.cx, c.cy, ps) - c.r))
        bb = getattr(self, "_box_cache", None)
        if bb is None:
            corners = [b.corners() for b in self.boxes]
            bb = self._box_cache = (corners, np.array([[c[:, 0].min(), c[:, 1].min(), c[:, 0].max(), c[:, 1].max()]
                                                       for c in corners]).reshape(-1, 4))
        corners, boxbb = bb
        qx, qy = polygon[0]
        px0, py0 = polygon[:, 0].min(), polygon[:, 1].min()
        px1, py1 = polygon[:, 0].max(), polygon[:, 1].max()
        for i, b in enumerate(self.boxes):  # car fully inside a box, or a small box fully inside the car
            # bounding-box prefilter (exact: a point outside the bounding box is outside the polygon)
            c_in = px0 <= b.cx <= px1 and py0 <= b.cy <= py1
            q_in = boxbb[i, 0] <= qx <= boxbb[i, 2] and boxbb[i, 1] <= qy <= boxbb[i, 3]
            if (c_in and _point_in_polygon(b.cx, b.cy, polygon)) or (q_in and _point_in_polygon(qx, qy, corners[i])):
                return 0.0
        return best


def footprint_polygon(x: float, y: float, theta: float, x_front: float, x_rear: float, half_width: float) -> np.ndarray:
    local = np.array([[x_front, half_width], [x_rear, half_width], [x_rear, -half_width], [x_front, -half_width]])
    c, s = math.cos(theta), math.sin(theta)
    return local @ np.array([[c, -s], [s, c]]).T + np.array([x, y])


def _point_seg_distance(px: float, py: float, seg: Sequence[float]) -> float:
    x0, y0, x1, y1 = seg
    ex, ey = x1 - x0, y1 - y0
    L2 = ex * ex + ey * ey
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * ex + (py - y0) * ey) / L2))
    return math.hypot(px - (x0 + t * ex), py - (y0 + t * ey))


def _segments_intersect(a: Sequence[float], b: Sequence[float]) -> bool:
    def orient(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    d1 = orient(b[0], b[1], b[2], b[3], a[0], a[1])
    d2 = orient(b[0], b[1], b[2], b[3], a[2], a[3])
    d3 = orient(a[0], a[1], a[2], a[3], b[0], b[1])
    d4 = orient(a[0], a[1], a[2], a[3], b[2], b[3])
    return (d1 * d2 <= 0) and (d3 * d4 <= 0)


def _seg_seg_distance(a: Sequence[float], b: Sequence[float]) -> float:
    if _segments_intersect(a, b):
        return 0.0
    return min(
        _point_seg_distance(a[0], a[1], b), _point_seg_distance(a[2], a[3], b),
        _point_seg_distance(b[0], b[1], a), _point_seg_distance(b[2], b[3], a),
    )


def _point_in_polygon(px: float, py: float, poly: Iterable[Sequence[float]]) -> bool:
    pts = list(poly)
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside
