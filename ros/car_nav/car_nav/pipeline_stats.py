"""Rate / age / last value of each stage of the data pipeline (no ROS imports).

The Logs tab of the web UI shows, for every topic on the way from the sensors to the motors,
how often it arrives (Hz), how old the last message is and what it said. ui_bridge feeds
`Stage.hit()` from its subscriptions (they exist only while the tab is open).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

# (group, topic, kind): kind picks how a message is summarised (ui_bridge._summary)
STAGES: list[tuple[str, str, str]] = [
    ("LiDAR", "/scan", "scan"),
    ("LiDAR", "/scan_filtered", "scan"),
    ("LiDAR", "/scan_nav", "scan"),
    ("LiDAR", "/scan_dynamic", "scan"),
    ("Depth camera", "/camera/depth/points", "raw"),
    ("Depth camera", "/depth_obstacles", "cloud"),
    ("IMU", "/imu/data_raw", "imu"),
    ("Odometry", "/odom_rf2o", "odom"),
    ("Odometry", "/odom", "odom"),
    ("Localisation", "/amcl_pose", "pose"),
    ("Localisation", "/localized_pose", "pose"),
    ("Planning", "/plan", "path"),
    ("Commands", "/cmd_vel_teleop", "twist"),
    ("Commands", "/cmd_vel_nav", "twist"),
    ("Commands", "/cmd_vel_mux", "twist"),
    ("Commands", "/cmd_vel_guarded", "twist"),
    ("Commands", "/cmd_vel", "twist"),
    ("Commands", "/car/wheels/cmd", "string"),
    ("Commands", "/car/wheels/applied", "string"),
    ("Safety", "/safety_guard/state", "guard"),
    ("Safety", "/teleop_mux/state", "mux"),
]

WINDOW_S = 5.0


@dataclass
class Stage:
    topic: str
    group: str = ""
    kind: str = "string"
    count: int = 0
    last_t: float | None = None
    last: str = ""
    times: deque = field(default_factory=lambda: deque(maxlen=400))

    def hit(self, now: float, summary: str = "") -> None:
        self.count += 1
        self.last_t = now
        self.times.append(now)
        if summary:
            self.last = summary

    def rate(self, now: float) -> float:
        """Messages per second over the last WINDOW_S seconds (0 when none)."""
        while self.times and now - self.times[0] > WINDOW_S:
            self.times.popleft()
        if len(self.times) < 2:
            return 0.0
        span = max(self.times[-1] - self.times[0], 1e-6)
        return (len(self.times) - 1) / span

    def age(self, now: float) -> float | None:
        return None if self.last_t is None else max(0.0, now - self.last_t)

    def row(self, now: float, publishers: int) -> dict:
        age = self.age(now)
        return {"topic": self.topic, "group": self.group, "pubs": publishers,
                "hz": round(self.rate(now), 1), "age_s": None if age is None else round(age, 2),
                "n": self.count, "last": self.last}


class Pipeline:
    def __init__(self, stages=STAGES) -> None:
        self.stages = {t: Stage(t, g, k) for g, t, k in stages}

    def hit(self, topic: str, now: float, summary: str = "") -> None:
        st = self.stages.get(topic)
        if st is not None:
            st.hit(now, summary)

    def rows(self, now: float, publishers) -> list[dict]:
        """publishers: topic -> number of publishers (a callable or a dict)."""
        get = publishers if callable(publishers) else (lambda t: publishers.get(t, 0))
        return [s.row(now, int(get(s.topic))) for s in self.stages.values()]

    def reset(self) -> None:
        for s in self.stages.values():
            s.count, s.last_t, s.last = 0, None, ""
            s.times.clear()


def status(row: dict, stale_s: float = 2.0) -> str:
    """'ok' | 'stale' | 'silent' (publisher but no message yet) | 'absent' (no publisher)."""
    if row["age_s"] is None:
        return "silent" if row["pubs"] else "absent"
    return "stale" if row["age_s"] > stale_s else "ok"
