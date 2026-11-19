"""Safety guard decision for people in FRONT of the car (pure math, no ROS imports).

Input: the points of /scan_dynamic (not on the map) in base_link (x forward,
y left). Only points in the front sector |bearing| <= fov_deg / 2, measured
from the nose, count: something mounted behind the LiDAR (or a person
walking behind the car) never stops it.

    stop   a point ahead of the nose closer than danger_range_m, within
           +-half_width_m of the centre line  (or, while turning on the
           spot, within turn_clear_m of the nose / front corners)
    slow   a point within slow_range_m, +-slow_half_width_m  -> speed x slow_ratio
    clear  otherwise

The car drives on by itself once the zone has been clear for resume_delay_s.

Second source (depth camera, depth_obstacles.py): when depth_enabled, the 2D
obstacle points of the depth camera (low boxes under the LiDAR plane, table
tops over it) are judged with the SAME zones, except that the stop distance
is depth_danger_range_m: the camera cannot see things closer than ~1.1 m
(blind zone under its field of view), so it has to stop the car while the
obstacle is still visible. decide_merged() takes the worse of the two.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields, replace

import numpy as np


@dataclass
class GuardParams:
    enabled: bool = True
    danger_range_m: float = 0.6       # stop: this far past the nose
    half_width_m: float = 0.35        # stop zone half width (car 0.135)
    slow_range_m: float = 1.5         # slow down: this far past the nose
    slow_half_width_m: float = 0.5
    slow_ratio: float = 0.4           # speed share while something is in the slow zone
    fov_deg: float = 120.0            # only points within +-fov/2 of straight ahead (from the nose)
    turn_clear_m: float = 0.25        # turning on the spot: stop for anything this close to the front
    resume_delay_s: float = 0.5       # zone clear this long before driving on
    min_points: int = 3               # fewer points than this = noise
    # ---- depth camera as a second source (off by default; nav.launch.py depth:=true switches it on)
    depth_enabled: bool = False
    depth_danger_range_m: float = 1.1  # stop distance past the nose for depth points (camera blind zone)
    depth_timeout_s: float = 1.0       # depth data older than this = stale
    depth_required: bool = False       # stale depth: True = stop the car, False = go on with the LiDAR only
    depth_min_height_m: float = 0.05   # depth_obstacles: floor (at or below this) is not an obstacle
    depth_max_height_m: float = 0.8    # ... above this the car passes under it
    depth_max_range_m: float = 3.0     # ... only points this close (horizontal, from base_link)

    def update(self, d: dict) -> list[str]:
        """Set the fields given in d (unknown keys ignored); returns the names changed."""
        changed = []
        for f in fields(self):
            if f.name in d:
                v = d[f.name]
                v = bool(v) if f.type in ("bool", bool) else (int(v) if f.type in ("int", int) else float(v))
                if v != getattr(self, f.name):
                    setattr(self, f.name, v)
                    changed.append(f.name)
        self.clamp()
        return changed

    def clamp(self) -> None:
        self.danger_range_m = min(max(self.danger_range_m, 0.1), 3.0)
        self.slow_range_m = min(max(self.slow_range_m, self.danger_range_m), 5.0)
        self.half_width_m = min(max(self.half_width_m, 0.14), 1.5)
        self.slow_half_width_m = min(max(self.slow_half_width_m, self.half_width_m), 2.0)
        self.slow_ratio = min(max(self.slow_ratio, 0.0), 1.0)
        self.fov_deg = min(max(self.fov_deg, 20.0), 360.0)
        self.turn_clear_m = min(max(self.turn_clear_m, 0.0), 1.0)
        self.resume_delay_s = min(max(self.resume_delay_s, 0.0), 10.0)
        self.min_points = max(int(self.min_points), 1)
        self.depth_danger_range_m = min(max(self.depth_danger_range_m, 0.1), 3.0)
        self.depth_timeout_s = min(max(self.depth_timeout_s, 0.1), 30.0)
        self.depth_min_height_m = min(max(self.depth_min_height_m, 0.0), 1.0)
        self.depth_max_height_m = min(max(self.depth_max_height_m, self.depth_min_height_m + 0.02), 2.0)
        self.depth_max_range_m = min(max(self.depth_max_range_m, 0.3), 6.0)

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass
class Decision:
    state: str            # "stop" | "slow" | "clear" | "off"
    nearest_m: float      # nearest front dynamic point past the nose (inf if none)
    n_stop: int
    n_slow: int


def front_points(points: np.ndarray, x_front: float, fov_deg: float) -> np.ndarray:
    """Points (N, 2) in base_link that lie ahead of the nose within the front sector."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if pts.shape[0] == 0:
        return pts
    dx = pts[:, 0] - x_front
    bearing = np.degrees(np.arctan2(pts[:, 1], np.maximum(dx, 1e-9)))
    ahead = dx > -0.05
    return pts[ahead & (np.abs(bearing) <= fov_deg / 2.0)]


def decide(points: np.ndarray, p: GuardParams, x_front: float, half_body: float,
           turning_on_spot: bool = False) -> Decision:
    if not p.enabled:
        return Decision("off", math.inf, 0, 0)
    pts = front_points(points, x_front, p.fov_deg)
    if pts.shape[0] == 0:
        return Decision("clear", math.inf, 0, 0)
    dx = np.maximum(pts[:, 0] - x_front, 0.0)
    ay = np.abs(pts[:, 1])
    nearest = float(np.hypot(dx, np.maximum(ay - half_body, 0.0)).min())
    in_stop = (dx <= p.danger_range_m) & (ay <= p.half_width_m)
    if turning_on_spot:
        # turning on the spot sweeps the front corners: anything close in front stops it
        in_stop |= (dx <= p.turn_clear_m) & (ay <= half_body + p.turn_clear_m)
    in_slow = (dx <= p.slow_range_m) & (ay <= p.slow_half_width_m)
    n_stop, n_slow = int(in_stop.sum()), int(in_slow.sum())
    if n_stop >= p.min_points:
        return Decision("stop", nearest, n_stop, n_slow)
    if n_slow >= p.min_points:
        return Decision("slow", nearest, n_stop, n_slow)
    return Decision("clear", nearest, n_stop, n_slow)


def depth_params(p: GuardParams) -> GuardParams:
    """The zones as the depth points see them: stop distance depth_danger_range_m."""
    q = replace(p, danger_range_m=p.depth_danger_range_m)
    q.slow_range_m = max(p.slow_range_m, p.depth_danger_range_m)
    q.clamp()
    return q


def decide_merged(lidar_points: np.ndarray, depth_points: np.ndarray | None, p: GuardParams, x_front: float,
                  half_body: float, turning_on_spot: bool = False) -> Decision:
    """LiDAR dynamic points + depth obstacle points, the worse zone wins (stop > slow > clear).

    depth_points None (or depth_enabled false) = the LiDAR decision unchanged."""
    dec = decide(lidar_points, p, x_front, half_body, turning_on_spot)
    if depth_points is None or not p.depth_enabled or not p.enabled:
        return dec
    dd = decide(depth_points, depth_params(p), x_front, half_body, turning_on_spot)
    rank = {"stop": 2, "slow": 1, "clear": 0, "off": 0}
    state = dd.state if rank[dd.state] > rank[dec.state] else dec.state
    return Decision(state, min(dec.nearest_m, dd.nearest_m), dec.n_stop + dd.n_stop, dec.n_slow + dd.n_slow)
