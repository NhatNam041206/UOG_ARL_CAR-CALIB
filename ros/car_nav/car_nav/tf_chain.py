"""Planar two-link TF lookup without the tf2 buffer (pure math, no ROS imports).

pd_path_follower only needs map -> base_link = (map -> odom) * (odom -> base_link).
The tf2 Python buffer cost a lot of CPU on a small computer (every /tf message is
converted and inserted; sim 2026-11-16: pd_path_follower ~55 % of a core, its
control math ~2.5 %). This class keeps a short history of the two links from
/tf and answers the same question as tf2's lookup_transform(map, base_link, Time()):

  * t = latest common time = the older of the two links' newest stamps,
  * each link at t: the sample at t, or linear interpolation of the translation and
    shortest-arc interpolation of the yaw between the two samples around t
    (tf2: lerp + quaternion slerp; for a rotation about z these are the same),
  * the composition of the two.

Like tf2's default cache it keeps cache_s seconds of history per link. Stamps are
integer nanoseconds (as tf2 does) so ratios match it.
"""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class _Link:
    t: list[int] = field(default_factory=list)          # stamps, ns, sorted
    v: list[tuple[float, float, float]] = field(default_factory=list)  # x, y, yaw

    def add(self, t_ns: int, pose: tuple[float, float, float], cache_ns: int) -> None:
        i = bisect.bisect_left(self.t, t_ns)
        if i < len(self.t) and self.t[i] == t_ns:
            self.v[i] = pose                               # same stamp: replace (tf2 keeps the newer)
        else:
            self.t.insert(i, t_ns)
            self.v.insert(i, pose)
        cut = bisect.bisect_left(self.t, self.t[-1] - cache_ns)
        if cut:
            del self.t[:cut], self.v[:cut]

    def at(self, t_ns: int):
        if not self.t or t_ns < self.t[0] or t_ns > self.t[-1]:
            return None
        i = bisect.bisect_left(self.t, t_ns)
        if self.t[i] == t_ns:
            return self.v[i]
        t0, t1 = self.t[i - 1], self.t[i]
        (x0, y0, a0), (x1, y1, a1) = self.v[i - 1], self.v[i]
        r = float(t_ns - t0) / float(t1 - t0)
        return x0 + (x1 - x0) * r, y0 + (y1 - y0) * r, _wrap(a0 + _wrap(a1 - a0) * r)


class PlanarTfChain:
    """parent -> mid -> child, e.g. map -> odom -> base_link."""

    def __init__(self, parent: str = "map", mid: str = "odom", child: str = "base_link",
                 cache_s: float = 10.0) -> None:
        self.parent, self.mid, self.child = parent, mid, child
        self._a = _Link()   # parent -> mid
        self._b = _Link()   # mid -> child
        self._cache_ns = int(cache_s * 1e9)

    def add(self, parent: str, child: str, t_ns: int, x: float, y: float, yaw: float) -> bool:
        """One transform from /tf; returns True if it belongs to the chain."""
        parent, child = parent.lstrip("/"), child.lstrip("/")
        if (parent, child) == (self.parent, self.mid):
            self._a.add(t_ns, (x, y, yaw), self._cache_ns)
        elif (parent, child) == (self.mid, self.child):
            self._b.add(t_ns, (x, y, yaw), self._cache_ns)
        else:
            return False
        return True

    def latest(self):
        """(t_ns, x, y, yaw) of child in parent at the latest common time, or None."""
        if not self._a.t or not self._b.t:
            return None
        t = min(self._a.t[-1], self._b.t[-1])
        a, b = self._a.at(t), self._b.at(t)
        if a is None or b is None:
            return None
        c, s = math.cos(a[2]), math.sin(a[2])
        return t, a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2])
