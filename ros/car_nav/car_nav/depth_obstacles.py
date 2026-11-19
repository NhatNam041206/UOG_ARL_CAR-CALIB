"""Depth camera -> 2D obstacle points in base_link (pure math, no ROS imports).

The LiDAR plane is at 0.42 m: boxes, steps and anything lying on the floor
pass under it, table tops pass over it. The depth camera (Astra Pro, 1.035 m
high, pitched 15 deg down) sees both. This module turns one depth frame into
the same kind of data the safety guard already uses: 2D points (x, y) in
base_link.

    points in the camera frame        (N, 3), optical convention by default:
                                      x right, y down, z forward (the ROS
                                      `camera_depth_optical_frame`)
      -> base_link                    camera pose from car.yaml (x, y, z, roll,
                                      pitch, yaw; pitch > 0 = looking down)
      -> keep the height band         floor_limit(range) < z <= max_height_m
      -> keep range <= max_range_m    horizontal distance from base_link
      -> grid (voxel) downsample      one point per voxel_m cell, needs
                                      min_cell_points raw points in the cell
      -> (optional) drop map walls    dynamic_obstacles.dynamic_points, so a
                                      wall seen at 0.3 m high is not a "person"

Blind zone: the camera looks down at 15 deg with a 45 deg vertical field of
view, so the lowest ray hits the floor 1.35 m in front of the camera and a
0.2 m high box leaves the view at about 1.1 m. `blind_distance()` gives that
number for the guard's depth stop distance.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields

import numpy as np

from .geometry import Pose3D

# optical (x right, y down, z forward) -> camera_link (x forward, y left, z up)
OPTICAL_TO_LINK = np.array([[0.0, 0.0, 1.0],
                            [-1.0, 0.0, 0.0],
                            [0.0, -1.0, 0.0]])


@dataclass
class DepthParams:
    min_height_m: float = 0.05        # at or below this: floor
    max_height_m: float = 0.8         # above this: the car passes under it
    max_range_m: float = 3.0          # horizontal distance from base_link
    min_range_m: float = 0.1
    floor_margin_per_m: float = 0.03  # extra floor margin per metre of range (pitch / floor slope error)
    voxel_m: float = 0.05
    min_cell_points: int = 2          # fewer raw points in a cell = noise (flying pixels at edges)

    def update(self, d: dict) -> list[str]:
        """Set the fields given in d (unknown keys ignored); returns the names changed."""
        changed = []
        for f in fields(self):
            if f.name in d and d[f.name] is not None:
                v = int(d[f.name]) if f.type in ("int", int) else float(d[f.name])
                if v != getattr(self, f.name):
                    setattr(self, f.name, v)
                    changed.append(f.name)
        self.clamp()
        return changed

    def clamp(self) -> None:
        self.min_height_m = min(max(self.min_height_m, 0.0), 1.0)
        self.max_height_m = min(max(self.max_height_m, self.min_height_m + 0.02), 2.0)
        self.max_range_m = min(max(self.max_range_m, 0.3), 6.0)
        self.min_range_m = min(max(self.min_range_m, 0.0), self.max_range_m)
        self.floor_margin_per_m = min(max(self.floor_margin_per_m, 0.0), 0.3)
        self.voxel_m = min(max(self.voxel_m, 0.01), 0.5)
        self.min_cell_points = max(int(self.min_cell_points), 1)

    def as_dict(self) -> dict:
        return asdict(self)


def cloud_to_base(points: np.ndarray, camera: Pose3D, optical: bool = True) -> np.ndarray:
    """(N, 3) camera-frame points -> base_link, with the camera pose from car.yaml."""
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    if optical:
        p = p @ OPTICAL_TO_LINK.T
    rot = np.array(camera.rotation())
    return p @ rot.T + np.array([camera.x, camera.y, camera.z])


def finite_points(points: np.ndarray) -> np.ndarray:
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    return p[np.isfinite(p).all(axis=1)]


def height_range_filter(pts_base: np.ndarray, p: DepthParams) -> np.ndarray:
    """Keep points above the floor (growing margin with range) and below max_height_m,
    within [min_range_m, max_range_m] horizontal distance."""
    pts = np.asarray(pts_base, dtype=float).reshape(-1, 3)
    if pts.shape[0] == 0:
        return pts
    rng = np.hypot(pts[:, 0], pts[:, 1])
    floor = p.min_height_m + p.floor_margin_per_m * rng
    keep = (pts[:, 2] > floor) & (pts[:, 2] <= p.max_height_m) & (rng <= p.max_range_m) & (rng >= p.min_range_m)
    return pts[keep]


def voxel_downsample(xy: np.ndarray, voxel_m: float, min_cell_points: int = 1) -> np.ndarray:
    """One point (the cell's mean) per voxel_m x voxel_m cell with at least min_cell_points points."""
    pts = np.asarray(xy, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return pts
    ij = np.floor(pts / voxel_m).astype(np.int64)
    _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    sums = np.zeros((cnt.shape[0], 2))
    np.add.at(sums, inv, pts)
    means = sums / cnt[:, None]
    return means[cnt >= min_cell_points]


def voxel_downsample_h(xyz: np.ndarray, voxel_m: float, min_cell_points: int = 1) -> np.ndarray:
    """Like voxel_downsample for (N, 3) points: (M, 3) = cell mean x, y and the HIGHEST z in the cell."""
    pts = np.asarray(xyz, dtype=float).reshape(-1, 3)
    if pts.shape[0] == 0:
        return pts
    ij = np.floor(pts[:, :2] / voxel_m).astype(np.int64)
    _, inv, cnt = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    sums = np.zeros((cnt.shape[0], 2))
    np.add.at(sums, inv, pts[:, :2])
    top = np.full(cnt.shape[0], -np.inf)
    np.maximum.at(top, inv, pts[:, 2])
    out = np.column_stack([sums / cnt[:, None], top])
    return out[cnt >= min_cell_points]


def obstacle_points3(cloud_cam: np.ndarray, camera: Pose3D, p: DepthParams, optical: bool = True) -> np.ndarray:
    """Depth points (camera frame) -> obstacle cells (M, 3) in base_link: x, y, highest z in the cell."""
    base = cloud_to_base(finite_points(cloud_cam), camera, optical)
    return voxel_downsample_h(height_range_filter(base, p), p.voxel_m, p.min_cell_points)


def obstacle_points(cloud_cam: np.ndarray, camera: Pose3D, p: DepthParams, optical: bool = True) -> np.ndarray:
    """Depth points (camera frame) -> 2D obstacle points (M, 2) in base_link."""
    return obstacle_points3(cloud_cam, camera, p, optical)[:, :2]


def blind_distance(camera: Pose3D, vfov_deg: float, height_m: float = 0.05) -> float:
    """x in base_link (m) closer than which an object's top at height_m is below the
    lowest ray of the camera (the camera cannot see it any more)."""
    down = camera.pitch + math.radians(vfov_deg) / 2.0
    if down <= 1e-3:
        return 0.0
    if down >= math.pi / 2:
        return camera.x
    return camera.x + max(camera.z - height_m, 0.0) / math.tan(down)


def points_to_polar(xy: np.ndarray, max_points: int = 400) -> tuple[np.ndarray, np.ndarray]:
    """(N, 2) points around the origin -> (ranges, angles); evenly thinned to max_points."""
    pts = np.asarray(xy, dtype=float).reshape(-1, 2)
    if pts.shape[0] > max_points:
        pts = pts[np.linspace(0, pts.shape[0] - 1, max_points).astype(int)]
    return np.hypot(pts[:, 0], pts[:, 1]), np.arctan2(pts[:, 1], pts[:, 0])


def parse_cloud(data: bytes, point_step: int, offsets: tuple[int, int, int], n_points: int,
                big_endian: bool = False, stride: int = 1) -> np.ndarray:
    """x, y, z (float32 fields at `offsets`) of a PointCloud2 buffer -> (N, 3) float; every
    stride-th point. NaN / inf points (no depth) stay in; finite_points() drops them."""
    raw = np.frombuffer(data, dtype=np.uint8, count=n_points * point_step).reshape(n_points, point_step)
    if stride > 1:
        raw = raw[::stride]
    dt = ">f4" if big_endian else "<f4"
    cols = [np.ascontiguousarray(raw[:, o:o + 4]).view(dt).reshape(-1) for o in offsets]
    return np.stack(cols, axis=1).astype(float)


def encode_xyz(pts: np.ndarray) -> bytes:
    """(N, 3) -> little-endian float32 x, y, z, 12 bytes per point (PointCloud2 data)."""
    return np.asarray(pts, dtype="<f4").reshape(-1, 3).tobytes()
