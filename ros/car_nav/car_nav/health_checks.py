"""Rules of the web UI's Health check (no ROS imports, unit tested).

The owner's rule: a check that nobody confirmed counts as FAILED. So every check ends as "pass" or
"fail" and an overall run is "pass" only when EVERY check that was run passed. A check that cannot run
(sensor missing, the simulator has no IMU) is "fail" with the reason, never skipped silently.

A rule is (name, ok, value, rule text); a check passes when all its rules pass. The numbers (PASS rules):

  LiDAR    scan rate 5.5 .. 10.5 Hz; >= 25 % of the beams valid (finite, inside range_min..range_max);
           valid ranges inside 0.05 m .. range_max; at most 2 valid beams inside the footprint + margin
           outside the self_mask sectors (the car's own body / mast is only allowed where it is masked)
  Depth    cloud rate >= 5 Hz; >= 300 valid points per frame; (organised cloud) >= 20 % of the pixels
           valid; a floor plane (RANSAC, >= 150 points) that, with the camera pose of car.yaml, lies
           within 0.06 m of z = 0 and tilts less than 3 deg along x and y (height / pitch consistency)
  Colour   image rate >= 5 Hz AND the observer clicks "looks right" on the snapshot
  IMU      rate 7 .. 13 Hz; mean |acceleration| 9.81 +- 0.8 m/s^2; |mean gyro| <= 0.08 rad/s and every
           gyro axis std <= 0.05 rad/s (the car stands still)
  Wheels   per step: observer "yes" and the ESP32 echoed the command (/car/wheels/applied) within
           2 units of what ugv_link sends; all six steps must pass
  Speed    moved 50 .. 160 % of the commanded distance, heading change <= 6 deg, not stopped by the guard,
           (tape distance given) tape within 20 % of the LiDAR distance, observer "yes"
"""
from __future__ import annotations

import math
import os
import re
import time
from dataclasses import dataclass, field

import numpy as np

G = 9.80665

THRESH = {
    "lidar_rate": (5.5, 10.5), "lidar_valid_share": 0.25, "lidar_min_range": 0.05, "lidar_inside_max": 2,
    "depth_rate_min": 5.0, "depth_points_min": 300, "depth_valid_share": 0.20, "floor_points_min": 150,
    "floor_height_m": 0.06, "floor_tilt_deg": 3.0,
    "color_rate_min": 5.0,
    "imu_rate": (7.0, 13.0), "imu_g_tol": 0.8, "imu_gyro_mean": 0.08, "imu_gyro_std": 0.05,
    "echo_units_tol": 2,
    "speed_ratio": (0.5, 1.6), "speed_heading_deg": 6.0, "speed_tape_tol": 0.20,
}


@dataclass
class Rule:
    name: str
    ok: bool
    value: str
    rule: str

    def as_dict(self) -> dict:
        return {"name": self.name, "ok": bool(self.ok), "value": self.value, "rule": self.rule}


def verdict(rules: list[Rule], extra: dict | None = None) -> dict:
    bad = [r for r in rules if not r.ok]
    out = {"status": "fail" if bad else "pass",
           "detail": "; ".join(f"{r.name}: {r.value} (need {r.rule})" for r in bad) or "all rules passed",
           "rules": [r.as_dict() for r in rules]}
    out.update(extra or {})
    return out


def failed(detail: str, **extra) -> dict:
    return {"status": "fail", "detail": detail, "rules": [], **extra}


def overall(results: dict[str, dict]) -> str:
    """"pass" only when at least one check ran and every one passed."""
    return "pass" if results and all(r.get("status") == "pass" for r in results.values()) else "fail"


# ------------------------------------------------------------------ rates
def rate_hz(stamps: list[float]) -> float | None:
    """Messages per second from arrival times (monotonic s)."""
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return None
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def max_gap(stamps: list[float]) -> float | None:
    return max((b - a for a, b in zip(stamps, stamps[1:])), default=None)


def _f(v, nd=2) -> str:
    return "n/a" if v is None else f"{v:.{nd}f}"


# ------------------------------------------------------------------ LiDAR
def lidar_frame_stats(ranges, angle_min: float, inc: float, range_min: float, range_max: float,
                      laser, footprint, margin: float, sectors) -> dict:
    """One scan: beam counts, valid share, min / max valid range, valid beams inside the footprint
    (+margin) that are not in a masked sector."""
    from .geometry import _in_sector, bearing_deg

    r = np.asarray(ranges, dtype=float)
    n = int(r.size)
    valid = np.isfinite(r) & (r > 0.0) & (r >= range_min) & (r <= range_max)
    nv = int(valid.sum())
    inside = 0
    if nv:
        idx = np.nonzero(valid)[0]
        ang = angle_min + idx * inc
        rot = laser.rotation()
        px, py = r[idx] * np.cos(ang), r[idx] * np.sin(ang)
        bx = rot[0][0] * px + rot[0][1] * py + laser.x
        by = rot[1][0] * px + rot[1][1] * py + laser.y
        ins = ((bx >= footprint.x_rear - margin) & (bx <= footprint.x_front + margin)
               & (np.abs(by) <= footprint.half_width + margin))
        for k in np.nonzero(ins)[0]:
            b = bearing_deg(float(bx[k]), float(by[k]))
            if not any(_in_sector(b, s) for s in sectors):
                inside += 1
    return {"beams": n, "valid": nv, "valid_share": nv / n if n else 0.0,
            "min_range": float(r[valid].min()) if nv else None,
            "max_range": float(r[valid].max()) if nv else None, "inside_footprint": inside}


def eval_lidar(stamps: list[float], frames: list[dict], range_max: float) -> dict:
    t = THRESH
    if len(frames) < 3:
        return failed(f"only {len(frames)} scans arrived: no LiDAR data on /scan", rate_hz=rate_hz(stamps))
    hz = rate_hz(stamps)
    share = float(np.median([f["valid_share"] for f in frames]))
    mins = [f["min_range"] for f in frames if f["min_range"] is not None]
    maxs = [f["max_range"] for f in frames if f["max_range"] is not None]
    inside = max(f["inside_footprint"] for f in frames)
    lo, hi = t["lidar_rate"]
    rules = [
        Rule("scan rate", hz is not None and lo <= hz <= hi, f"{_f(hz, 1)} Hz", f"{lo} .. {hi} Hz"),
        Rule("valid beams", share >= t["lidar_valid_share"], f"{100 * share:.0f} %",
             f">= {100 * t['lidar_valid_share']:.0f} %"),
        Rule("range sanity", bool(mins) and min(mins) >= t["lidar_min_range"] and max(maxs) <= range_max * 1.01,
             f"{_f(min(mins) if mins else None, 2)} .. {_f(max(maxs) if maxs else None, 2)} m",
             f"{t['lidar_min_range']} .. {range_max} m"),
        Rule("inside the footprint", inside <= t["lidar_inside_max"], f"{inside} beams",
             f"<= {t['lidar_inside_max']} outside the masked sectors"),
    ]
    return verdict(rules, {"rate_hz": hz, "valid_share": share, "scans": len(frames),
                           "min_range_m": min(mins) if mins else None, "max_range_m": max(maxs) if maxs else None,
                           "inside_footprint": inside})


# ------------------------------------------------------------------ depth camera
def fit_floor(pts_base: np.ndarray, band: float = 0.6, thr: float = 0.03, iters: int = 150, seed: int = 1) -> dict | None:
    """Dominant near-horizontal plane z = a x + b y + c in base_link points (RANSAC, then least squares
    on the inliers). Only points with |z| < band and 0.3 m < range < 4.5 m are candidates, so a wall or
    a box cannot win. Returns None when there are too few points."""
    p = np.asarray(pts_base, dtype=float).reshape(-1, 3)
    rng = np.hypot(p[:, 0], p[:, 1])
    p = p[(np.abs(p[:, 2]) < band) & (rng > 0.3) & (rng < 4.5)]
    if p.shape[0] < 30:
        return None
    gen = np.random.default_rng(seed)
    if p.shape[0] > 4000:
        p = p[gen.choice(p.shape[0], 4000, replace=False)]
    best, best_n = None, 0
    for _ in range(iters):
        s = p[gen.choice(p.shape[0], 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        if abs(n[2]) < 1e-9:
            continue
        n = n / np.linalg.norm(n)
        if n[2] < 0:
            n = -n
        if n[2] < math.cos(math.radians(35.0)):
            continue                                   # not floor-like
        cnt = int((np.abs((p - s[0]) @ n) < thr).sum())
        if cnt > best_n:
            best, best_n = (n, s[0]), cnt
    if best is None:
        return None
    n, p0 = best
    inl = p[np.abs((p - p0) @ n) < thr]
    for _ in range(2):
        a_mat = np.column_stack([inl[:, 0], inl[:, 1], np.ones(inl.shape[0])])
        (a, b, c), *_ = np.linalg.lstsq(a_mat, inl[:, 2], rcond=None)
        res = np.abs(a * p[:, 0] + b * p[:, 1] + c - p[:, 2])
        inl = p[res < thr]
        if inl.shape[0] < 10:
            return None
    cx, cy = float(inl[:, 0].mean()), float(inl[:, 1].mean())
    return {"a": float(a), "b": float(b), "c": float(c), "n_inliers": int(inl.shape[0]),
            "z_at_centroid": float(a * cx + b * cy + c), "centroid": [cx, cy],
            "slope_x_deg": math.degrees(math.atan(a)), "slope_y_deg": math.degrees(math.atan(b))}


def eval_depth(stamps: list[float], frames: list[dict], camera, floor: dict | None) -> dict:
    """frames: [{"n_valid", "n_total" (width * height), "organized"}]; floor from fit_floor() on a frame."""
    t = THRESH
    if len(frames) < 3:
        return failed(f"only {len(frames)} point clouds arrived: no depth data on /camera/depth/points",
                      rate_hz=rate_hz(stamps))
    hz = rate_hz(stamps)
    npts = int(np.median([f["n_valid"] for f in frames]))
    org = [f for f in frames if f["organized"] and f["n_total"]]
    share = float(np.median([f["n_valid"] / f["n_total"] for f in org])) if org else None
    rules = [
        Rule("cloud rate", hz is not None and hz >= t["depth_rate_min"], f"{_f(hz, 1)} Hz", f">= {t['depth_rate_min']} Hz"),
        Rule("valid points", npts >= t["depth_points_min"], f"{npts} per frame", f">= {t['depth_points_min']}"),
    ]
    if share is not None:
        rules.append(Rule("valid pixels", share >= t["depth_valid_share"], f"{100 * share:.0f} %",
                          f">= {100 * t['depth_valid_share']:.0f} %"))
    extra: dict = {"rate_hz": hz, "points": npts, "valid_share": share, "clouds": len(frames)}
    if floor is None or floor["n_inliers"] < t["floor_points_min"]:
        n = 0 if floor is None else floor["n_inliers"]
        rules.append(Rule("floor plane", False, f"{n} floor points", f">= {t['floor_points_min']} within +-0.6 m of z = 0"))
    else:
        cam_z = camera.z
        rules += [
            Rule("floor height", abs(floor["z_at_centroid"]) <= t["floor_height_m"],
                 f"{100 * floor['z_at_centroid']:+.1f} cm", f"within +-{100 * t['floor_height_m']:.0f} cm of z = 0"),
            Rule("floor tilt x (pitch)", abs(floor["slope_x_deg"]) <= t["floor_tilt_deg"],
                 f"{floor['slope_x_deg']:+.1f} deg", f"within +-{t['floor_tilt_deg']} deg"),
            Rule("floor tilt y (roll)", abs(floor["slope_y_deg"]) <= t["floor_tilt_deg"],
                 f"{floor['slope_y_deg']:+.1f} deg", f"within +-{t['floor_tilt_deg']} deg"),
        ]
        extra["floor"] = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in floor.items() if k != "centroid"}
        extra["hint"] = {"camera_z_m": round(cam_z - floor["c"], 3),
                         "camera_pitch_deg": round(math.degrees(camera.pitch) + floor["slope_x_deg"], 1),
                         "note": "what camera.z / camera.pitch_deg would make the floor level (hint, measure to confirm)"}
    return verdict(rules, extra)


def _base_points(ranges, angle_min, inc, range_min, range_max, laser):
    r = np.asarray(ranges, dtype=float)
    idx = np.nonzero(np.isfinite(r) & (r > 0.0) & (r >= range_min) & (r <= range_max))[0]
    if idx.size == 0:
        return np.zeros(0), np.zeros(0)
    ang = angle_min + idx * inc
    rot = laser.rotation()
    px, py = r[idx] * np.cos(ang), r[idx] * np.sin(ang)
    return (rot[0][0] * px + rot[0][1] * py + laser.x), (rot[1][0] * px + rot[1][1] * py + laser.y)


def clearance_outside(ranges, angle_min, inc, range_min, range_max, laser, footprint) -> float | None:
    """Distance (m) from the footprint's edge to the nearest valid beam end OUTSIDE the footprint (the car's own
    body is not an obstacle); None when the scan sees nothing."""
    bx, by = _base_points(ranges, angle_min, inc, range_min, range_max, laser)
    if bx.size == 0:
        return None
    dx = np.maximum(np.maximum(footprint.x_rear - bx, bx - footprint.x_front), 0.0)
    dy = np.maximum(np.abs(by) - footprint.half_width, 0.0)
    d = np.hypot(dx, dy)
    d = d[d > 0.0]
    return float(d.min()) if d.size else None


def min_radius_outside(ranges, angle_min, inc, range_min, range_max, laser, footprint) -> float | None:
    """Nearest valid beam end outside the footprint, as distance from base_link (to compare with the spin radius)."""
    bx, by = _base_points(ranges, angle_min, inc, range_min, range_max, laser)
    if bx.size == 0:
        return None
    out = ~((bx >= footprint.x_rear) & (bx <= footprint.x_front) & (np.abs(by) <= footprint.half_width))
    return float(np.hypot(bx[out], by[out]).min()) if out.any() else None


def passed_today(last: dict | None, now: float | None = None) -> bool:
    """True when the latest stored run passed and was made today (local time)."""
    if not last or last.get("overall") != "pass":
        return False
    ts = last.get("started_ts")
    if ts is None:
        return False
    now = time.time() if now is None else now
    return time.strftime("%Y-%m-%d", time.localtime(ts)) == time.strftime("%Y-%m-%d", time.localtime(now))


# ------------------------------------------------------------------ colour camera
def eval_color(stamps: list[float], size: tuple[int, int] | None, observer: str | None) -> dict:
    """observer: "yes" | "no" | None (no answer in time)."""
    hz = rate_hz(stamps)
    rules = [
        Rule("image rate", hz is not None and hz >= THRESH["color_rate_min"], f"{_f(hz, 1)} Hz",
             f">= {THRESH['color_rate_min']} Hz"),
        Rule("observer", observer == "yes", {"yes": "looks right", "no": "does NOT look right"}.get(observer or "", "no answer"),
             "clicks 'looks right'"),
    ]
    return verdict(rules, {"rate_hz": hz, "size": list(size) if size else None})


# ------------------------------------------------------------------ IMU
def eval_imu(samples: list[tuple]) -> dict:
    """samples: (t, ax, ay, az, gx, gy, gz) in m/s^2 and rad/s."""
    t = THRESH
    if len(samples) < 10:
        return failed(f"only {len(samples)} IMU samples arrived on /imu/data_raw", samples=len(samples))
    a = np.asarray(samples, dtype=float)
    hz = rate_hz(list(a[:, 0]))
    gmag = float(np.linalg.norm(a[:, 1:4], axis=1).mean())
    gyro_mean = a[:, 4:7].mean(axis=0)
    gyro_std = a[:, 4:7].std(axis=0)
    lo, hi = t["imu_rate"]
    rules = [
        Rule("rate", hz is not None and lo <= hz <= hi, f"{_f(hz, 1)} Hz", f"{lo} .. {hi} Hz"),
        Rule("gravity", abs(gmag - G) <= t["imu_g_tol"], f"{gmag:.2f} m/s^2", f"{G:.2f} +- {t['imu_g_tol']}"),
        Rule("gyro bias (still)", float(np.linalg.norm(gyro_mean)) <= t["imu_gyro_mean"],
             f"{float(np.linalg.norm(gyro_mean)):.3f} rad/s", f"<= {t['imu_gyro_mean']}"),
        Rule("gyro noise (still)", float(gyro_std.max()) <= t["imu_gyro_std"], f"{float(gyro_std.max()):.3f} rad/s",
             f"<= {t['imu_gyro_std']}"),
    ]
    return verdict(rules, {"rate_hz": hz, "gravity_mps2": gmag, "gyro_mean": [round(float(x), 4) for x in gyro_mean],
                           "gyro_std": [round(float(x), 4) for x in gyro_std], "samples": len(samples)})


# ------------------------------------------------------------------ wheels test
def wheel_steps(speed_mps: float) -> list[dict]:
    """The six pulses: name, (left, right) m/s, what the observer should see."""
    s = float(speed_mps)
    return [
        {"id": "left_fwd", "name": "Left wheel forward", "left": s, "right": 0.0,
         "expect": "ONLY the LEFT wheel turns, forward (the car would curve to the right)"},
        {"id": "left_back", "name": "Left wheel backward", "left": -s, "right": 0.0,
         "expect": "ONLY the LEFT wheel turns, backward"},
        {"id": "right_fwd", "name": "Right wheel forward", "left": 0.0, "right": s,
         "expect": "ONLY the RIGHT wheel turns, forward (the car would curve to the left)"},
        {"id": "right_back", "name": "Right wheel backward", "left": 0.0, "right": -s,
         "expect": "ONLY the RIGHT wheel turns, backward"},
        {"id": "both_fwd", "name": "Both wheels forward", "left": s, "right": s,
         "expect": "BOTH wheels turn forward at the same speed (the car would drive straight ahead)"},
        {"id": "spin_left", "name": "Spin left", "left": -s, "right": s,
         "expect": "left wheel BACKWARD, right wheel FORWARD (the car would turn left, counter-clockwise, on the spot)"},
    ]


def expected_units(left_mps: float, right_mps: float, ugv_cfg) -> tuple[int, int]:
    """What ugv_link sends the ESP32 for this wheel command, as integers."""
    a, b = ugv_cfg.command(left_mps, right_mps).split()
    return int(a), int(b)


def parse_applied(text: str) -> tuple[float, float] | None:
    try:
        a, b = text.strip().split(",")
        return float(a), float(b)
    except (ValueError, AttributeError):
        return None


def eval_echo(expected: tuple[int, int], echoes: list[tuple[float, float]]) -> Rule:
    tol = THRESH["echo_units_tol"]
    ok = any(abs(l - expected[0]) <= tol and abs(r - expected[1]) <= tol for l, r in echoes)
    seen = f"{len(echoes)} echoes" + (f", last {echoes[-1][0]:g},{echoes[-1][1]:g}" if echoes else ", none")
    return Rule("ESP32 echo", ok, seen, f"{expected[0]},{expected[1]} units (+-{tol}) on /car/wheels/applied")


def eval_wheel_step(step: dict, echo: Rule, observer: str | None) -> dict:
    rules = [echo, Rule("observer", observer == "yes",
                        {"yes": "yes", "no": "NO"}.get(observer or "", "no answer in time"), "yes")]
    return {"id": step["id"], "name": step["name"], **verdict(rules)}


# ------------------------------------------------------------------ speed test
def _interp(series: list[tuple], t: float, col: int = 1):
    """Linear interpolation of series [(t, v, ...)] at t (None outside)."""
    if not series or t < series[0][0] or t > series[-1][0]:
        return None
    for (t0, *r0), (t1, *r1) in zip(series, series[1:]):
        if t0 <= t <= t1:
            k = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return r0[col - 1] + k * (r1[col - 1] - r0[col - 1])
    return series[-1][col]


def window_distance(odom: list[tuple], t0: float, t1: float) -> float | None:
    """Straight distance between the (t, x, y) positions at t0 and t1."""
    ax, ay, bx, by = (_interp(odom, t0, 1), _interp(odom, t0, 2), _interp(odom, t1, 1), _interp(odom, t1, 2))
    if None in (ax, ay, bx, by):
        return None
    return math.hypot(bx - ax, by - ay)


def window_yaw(yaws: list[tuple], t0: float, t1: float) -> float | None:
    """Unwrapped yaw change (rad) between t0 and t1 from [(t, yaw)]."""
    if len(yaws) < 2:
        return None
    unwrapped, off, prev = [], 0.0, None
    for t, y in yaws:
        if prev is not None:
            d = y - prev
            if d > math.pi:
                off -= 2 * math.pi
            elif d < -math.pi:
                off += 2 * math.pi
        prev = y
        unwrapped.append((t, y + off))
    a, b = _interp(unwrapped, t0), _interp(unwrapped, t1)
    return None if a is None or b is None else b - a


def window_wall(front: list[tuple], t0: float, t1: float) -> float | None:
    """How much nearer the wall ahead got between t0 and t1 from [(t, distance ahead)]."""
    a, b = _interp(front, t0), _interp(front, t1)
    return None if a is None or b is None else a - b


def speed_result(cmd_v: float, t_cmd: float, t_window: tuple[float, float], odom: list[tuple], front: list[tuple],
                 yaws: list[tuple], total_path_m: float | None, units_per_mps: float, tape_m: float | None,
                 guard_stopped: bool, observer: str | None) -> dict:
    """Forward leg: real speed from the LiDAR (wall ahead when it was seen at both ends, else rf2o) and the
    proposed ugv.units_per_mps. odom = [(t, x, y)] (rf2o), front = [(t, x of the wall ahead)], yaws = [(t, yaw)]."""
    t = THRESH
    t0, t1 = t_window
    span = t1 - t0
    d_rf = window_distance(odom, t0, t1)
    d_wall = window_wall(front, t0, t1)
    if d_wall is not None and d_wall <= 0.0:
        d_wall = None
    use = "wall ahead" if d_wall is not None else "rf2o odometry"
    d = d_wall if d_wall is not None else d_rf
    heading = window_yaw(yaws, t0, t1)
    out: dict = {"commanded_mps": cmd_v, "window_s": round(span, 2), "distance_rf2o_m": d_rf, "distance_wall_m": d_wall,
                 "source": use, "units_per_mps_now": units_per_mps}
    if d is None or span <= 0:
        return {**failed("the LiDAR / rf2o gave no distance (no odometry samples in the drive window)", **out)}
    v_meas = d / span
    scale = 1.0
    if tape_m is not None and total_path_m and total_path_m > 0.05:
        scale = tape_m / total_path_m          # the tape corrects the LiDAR's total distance
        v_meas *= scale
    ratio = v_meas / cmd_v
    lo, hi = t["speed_ratio"]
    rules = [
        Rule("moved", lo <= ratio <= hi, f"{100 * ratio:.0f} % of commanded ({v_meas:.3f} of {cmd_v:.3f} m/s)",
             f"{100 * lo:.0f} .. {100 * hi:.0f} %"),
        Rule("straight", heading is None or abs(math.degrees(heading)) <= t["speed_heading_deg"],
             "n/a" if heading is None else f"{math.degrees(heading):+.1f} deg", f"within +-{t['speed_heading_deg']} deg"),
        Rule("guard", not guard_stopped, "stopped the car" if guard_stopped else "never stopped it", "no stop"),
    ]
    if tape_m is not None and total_path_m:
        err = abs(tape_m - total_path_m) / tape_m if tape_m > 0 else 1.0
        rules.append(Rule("tape vs LiDAR", err <= t["speed_tape_tol"], f"{100 * err:.0f} % apart "
                          f"(tape {tape_m:.2f} m, LiDAR {total_path_m:.2f} m)", f"<= {100 * t['speed_tape_tol']:.0f} %"))
    rules.append(Rule("observer", observer == "yes", {"yes": "yes", "no": "NO"}.get(observer or "", "no answer in time"), "yes"))
    out.update(real_mps=round(v_meas, 4), ratio=round(ratio, 4), tape_m=tape_m, heading_deg=None if heading is None else round(math.degrees(heading), 2))
    if 0.2 <= ratio <= 3.0:
        out["units_per_mps_proposed"] = round(units_per_mps / ratio, 1)
    return verdict(rules, out)


def spin_result(cmd_w: float, t_window: tuple[float, float], yaws: list[tuple], track_m: float) -> dict:
    """Spin on the spot: real yaw rate, yaw_efficiency = real / commanded (the ideal turn of the wheels)."""
    t0, t1 = t_window
    dyaw = window_yaw(yaws, t0, t1)
    if dyaw is None or t1 <= t0:
        return {"status": "fail", "detail": "no yaw samples in the spin window", "rules": []}
    w = dyaw / (t1 - t0)
    eff = w / cmd_w if cmd_w else None
    ok = eff is not None and 0.2 <= eff <= 1.4
    return {"status": "pass" if ok else "fail", "commanded_rps": cmd_w, "real_rps": round(w, 4),
            "yaw_efficiency": None if eff is None else round(eff, 3),
            "detail": f"real {w:.3f} rad/s of commanded {cmd_w:.3f} -> yaw_efficiency {eff:.2f}" + ("" if ok else " (need 0.2 .. 1.4)")
            if eff is not None else "no commanded rate"}


# ------------------------------------------------------------------ store
_ID = re.compile(r"[^A-Za-z0-9_.\-]+")


class HealthStore:
    """Runs as JSON files under <data_dir>/health/<id>.json (like ui_store's test records)."""

    def __init__(self, data_dir: str) -> None:
        self.dir = os.path.join(data_dir, "health")
        os.makedirs(self.dir, exist_ok=True)

    def _path(self, rid: str) -> str:
        rid = _ID.sub("", rid)
        if not rid:
            raise ValueError("empty id")
        return os.path.join(self.dir, rid + ".json")

    def save(self, run: dict) -> str:
        import json

        rid = time.strftime("%Y%m%d-%H%M%S", time.localtime(run.get("started_ts", time.time())))
        path = self._path(rid)
        n = 1
        while os.path.exists(path):
            n += 1
            path = self._path(f"{rid}-{n}")
        run = {**run, "id": os.path.basename(path)[:-5]}
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(run, fh, indent=1)
        os.replace(tmp, path)
        return run["id"]

    def get(self, rid: str) -> dict | None:
        import json

        try:
            with open(self._path(rid), encoding="utf-8") as fh:
                return json.load(fh)
        except (OSError, ValueError):
            return None

    def list(self) -> list[dict]:
        out = []
        for f in sorted(os.listdir(self.dir), reverse=True):
            if f.endswith(".json"):
                d = self.get(f[:-5])
                if d:
                    out.append({"id": d.get("id", f[:-5]), "started": d.get("started"), "started_ts": d.get("started_ts"), "overall": d.get("overall"),
                                "checks": {k: v.get("status") for k, v in (d.get("results") or {}).items()}})
        return out

    def delete(self, rid: str) -> bool:
        try:
            os.remove(self._path(rid))
            return True
        except OSError:
            return False

    def last(self) -> dict | None:
        runs = self.list()
        return runs[0] if runs else None
