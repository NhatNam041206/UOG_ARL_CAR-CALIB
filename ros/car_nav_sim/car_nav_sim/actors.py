"""Moving people for the simulator (no ROS imports).

A mission file may list actors: cylinders (a person seen by a LiDAR at
0.42 m height is a ~0.4 m wide blob) that walk along a polyline at a fixed
speed. They are NOT in the map Nav2 localises on, exactly like a person on
the real floor.

    actors:
      - name: crossing
        radius: 0.2
        path: [[4.0, 0.2], [4.0, 2.0]]      # walk from the first point along the polyline
        speed_mps: 1.0
        trigger: {x: 3.0, y: 1.1, dist: 1.6} # start walking when the car's base_link is this close
        # or start_s: 12.0                   # start this many sim seconds after the first update
        wait_s: 0.0                          # pause at every path point (e.g. stands in the way)
        loop: false                          # back and forth forever instead of stopping at the end
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from .world import Cylinder


@dataclass
class Actor:
    name: str
    radius: float
    path: list[tuple[float, float]]
    speed_mps: float = 1.0
    trigger: tuple[float, float, float] | None = None   # x, y, dist
    start_s: float | None = None
    wait_s: float = 0.0
    loop: bool = False
    started_t: float | None = field(default=None, init=False)
    _t0: float | None = field(default=None, init=False)

    @classmethod
    def from_dict(cls, d: dict, i: int = 0) -> "Actor":
        trig = d.get("trigger")
        return cls(
            name=str(d.get("name", f"actor{i}")),
            radius=float(d.get("radius", 0.2)),
            path=[(float(p[0]), float(p[1])) for p in d["path"]],
            speed_mps=float(d.get("speed_mps", 1.0)),
            trigger=None if not trig else (float(trig["x"]), float(trig["y"]), float(trig["dist"])),
            start_s=None if d.get("start_s") is None else float(d["start_s"]),
            wait_s=float(d.get("wait_s", 0.0)),
            loop=bool(d.get("loop", False)),
        )

    def _legs(self) -> list[tuple[tuple[float, float], tuple[float, float], float]]:
        pts = self.path if not self.loop else self.path + self.path[-2::-1]
        return [(a, b, math.dist(a, b)) for a, b in zip(pts, pts[1:])]

    def update(self, t: float, car_xy: tuple[float, float] | None) -> None:
        if self._t0 is None:
            self._t0 = t
        if self.started_t is not None:
            return
        if self.trigger is not None:
            if car_xy is not None and math.dist(car_xy, self.trigger[:2]) <= self.trigger[2]:
                self.started_t = t
        elif self.start_s is not None:
            if t - self._t0 >= self.start_s:
                self.started_t = t
        else:
            self.started_t = t

    def position(self, t: float) -> tuple[float, float]:
        if self.started_t is None or len(self.path) < 2:
            return self.path[0]
        legs = self._legs()
        dt = max(t - self.started_t, 0.0)
        cycle = sum(L / self.speed_mps + self.wait_s for _, _, L in legs)
        if self.loop and cycle > 0:
            dt %= cycle
        for a, b, L in legs:
            walk = L / self.speed_mps if self.speed_mps > 0 else math.inf
            if dt <= walk:
                f = dt / walk if walk > 0 else 1.0
                return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
            dt -= walk
            if dt <= self.wait_s:
                return b
            dt -= self.wait_s
        return legs[-1][1]

    def cylinder(self, t: float) -> Cylinder:
        x, y = self.position(t)
        return Cylinder(x, y, self.radius, 1.8)


def load_actors(mission: dict | None) -> list[Actor]:
    return [Actor.from_dict(d, i) for i, d in enumerate((mission or {}).get("actors") or [])]
