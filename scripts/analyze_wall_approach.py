#!/usr/bin/env python3
"""Baseline speed + stopping distance from a wall-approach CSV.

The car has no speed feedback and no speed command, so NAV2 mode starts
from a measured baseline (car.yaml speed.baseline_speed_mps). Record a
run with `ros2 run car_nav front_range_logger`, drive straight at a flat
wall, send STOP, then:

    python scripts/analyze_wall_approach.py run1.csv [run2.csv ...] \
        [--stop-time <epoch seconds of the STOP command, one per file>]

Without --stop-time only the steady speed and the final distance are
reported. Pure standard library; runs on the Windows laptop too.
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
from dataclasses import dataclass


@dataclass
class RunResult:
    name: str
    speed_mps: float | None
    final_x_m: float | None
    stop_x_m: float | None = None
    stopping_distance_m: float | None = None
    settle_time_s: float | None = None


def read_csv(path: str) -> list[tuple[float, float]]:
    rows: list[tuple[float, float]] = []
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            if row.get("front_x_m"):
                rows.append((float(row["wall_time_s"]), float(row["front_x_m"])))
    return rows


def _slope(points: list[tuple[float, float]]) -> float:
    n = len(points)
    mt = sum(t for t, _ in points) / n
    mx = sum(x for _, x in points) / n
    den = sum((t - mt) ** 2 for t, _ in points)
    return sum((t - mt) * (x - mx) for t, x in points) / den if den else 0.0


def steady_speed(rows: list[tuple[float, float]], window_s: float = 1.0,
                 min_speed: float = 0.05) -> float | None:
    """Median closing speed over sliding windows where the car is moving."""
    speeds: list[float] = []
    j = 0
    for i in range(len(rows)):
        while rows[i][0] - rows[j][0] > window_s:
            j += 1
        win = rows[j:i + 1]
        if len(win) >= 4 and win[-1][0] - win[0][0] >= 0.6 * window_s:
            v = -_slope(win)
            if v > min_speed:
                speeds.append(v)
    if not speeds:
        return None
    # The steady part dominates; median ignores the ramp-up and ramp-down.
    return statistics.median(speeds)


def x_at(rows: list[tuple[float, float]], t: float) -> float | None:
    for (t0, x0), (t1, x1) in zip(rows, rows[1:]):
        if t0 <= t <= t1:
            return x0 + (x1 - x0) * (t - t0) / (t1 - t0) if t1 > t0 else x0
    return None


def analyse(name: str, rows: list[tuple[float, float]], stop_time: float | None,
            still_mps: float = 0.03) -> RunResult:
    res = RunResult(name, steady_speed(rows), rows[-1][1] if rows else None)
    if stop_time is not None and rows:
        res.stop_x_m = x_at(rows, stop_time)
        tail = [r for r in rows if r[0] >= stop_time]
        for k in range(len(tail)):
            win = [r for r in tail[k:] if r[0] - tail[k][0] <= 0.5]
            if len(win) >= 3 and abs(_slope(win)) < still_mps:
                res.settle_time_s = tail[k][0] - stop_time
                rest = statistics.median(x for _, x in win)
                if res.stop_x_m is not None:
                    res.stopping_distance_m = res.stop_x_m - rest
                break
    return res


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--stop-time", type=float, nargs="*", default=[])
    args = ap.parse_args(argv)
    if args.stop_time and len(args.stop_time) != len(args.csv):
        ap.error("give one --stop-time per CSV file")
    results = []
    for i, path in enumerate(args.csv):
        st = args.stop_time[i] if args.stop_time else None
        results.append(analyse(path, read_csv(path), st))
    for r in results:
        print(f"{r.name}: speed={r.speed_mps} m/s final_x={r.final_x_m} m "
              f"stopping_distance={r.stopping_distance_m} m settle={r.settle_time_s} s")
    speeds = [r.speed_mps for r in results if r.speed_mps is not None]
    if speeds:
        spread = statistics.pstdev(speeds) if len(speeds) > 1 else 0.0
        print(f"SPEED mean={statistics.mean(speeds):.3f} m/s spread(sd)={spread:.3f} n={len(speeds)}")
    stops = [r.stopping_distance_m for r in results if r.stopping_distance_m is not None]
    if stops:
        spread = statistics.pstdev(stops) if len(stops) > 1 else 0.0
        print(f"STOPPING mean={statistics.mean(stops):.3f} m spread(sd)={spread:.3f} n={len(stops)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
