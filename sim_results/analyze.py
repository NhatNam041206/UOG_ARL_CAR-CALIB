"""Summarise one sim run (report JSON + log) for sim_results notes.

    python sim_results/analyze.py <run_dir>            # prints a markdown block
    python sim_results/analyze.py --table <exp_dir>... # one line per run, for SUMMARY.md

A run dir holds report-*.json and *.log (copied by run_batch.sh).
"""
from __future__ import annotations

import glob
import json
import math
import os
import re
import sys

PATTERNS = {
    "wall_avoid": r"wall avoid: PD arc .* hits a wall",
    "avoid_release": r"wall avoid: PD arc clear again",
    "blind_reseed": r"blind corridor",
    "no_valid_path": r"no valid path found",
    "start_occupied": r"Start occupied",
    "invalid_source": r"invalid source",
    "pose_old": r"pose is .* old",
    "no_progress": r"no progress",
    "cm_stop": r"Robot to stop due to",
}


def load(run_dir: str):
    reps = sorted(glob.glob(os.path.join(run_dir, "report-*.json")))
    logs = sorted(glob.glob(os.path.join(run_dir, "*.log")))
    rep = json.load(open(reps[-1])) if reps else None
    log = open(logs[-1], encoding="utf-8", errors="replace").read() if logs else ""
    return rep, log


def counts(log: str) -> dict:
    return {k: len(re.findall(p, log)) for k, p in PATTERNS.items()}


def lag_stats(goal: dict):
    """Along-track / total error between est and true in the 0.5 s trace (est lags in the mission node)."""
    worst_along, worst = 0.0, 0.0
    for row in goal.get("trace") or []:
        e, t = row.get("est"), row.get("true")
        if not e or not t:
            continue
        dx, dy = t[0] - e[0], t[1] - e[1]
        th = math.radians(t[2])
        along = dx * math.cos(th) + dy * math.sin(th)  # + = truth ahead of estimate
        if abs(along) > abs(worst_along):
            worst_along = along
        worst = max(worst, math.hypot(dx, dy))
    return worst_along, worst


def goal_line(i: int, g: dict) -> str:
    al, tot = lag_stats(g)
    err = f" nav2: {g['nav2_error']}" if g.get("nav2_error") else ""
    return (f"| {i} | {g['result']}{' COLLISION' if g['collision'] else ''} | {g['time_s']:.0f} | "
            f"{g['min_clearance_m']:.2f} | {g['final_xy_error_m']:.2f} | {g['max_localization_error_m']:.2f} | "
            f"{al:+.2f} | {g['max_odom_drift_m']:.2f} |{err} |")


def passed_goals(rep) -> int:
    n = 0
    for g in rep["goals"]:
        if g["result"] == "SUCCEEDED" and not g["collision"]:
            n += 1
        else:
            break
    return n


def markdown(run_dir: str) -> str:
    rep, log = load(run_dir)
    out = [f"### {os.path.basename(run_dir.rstrip('/'))}"]
    if rep is None:
        out.append("NO REPORT (see log)")
    else:
        out.append(f"{'PASS' if rep['passed'] else 'FAIL'} - odom {rep.get('odom_source')}, sim truth {rep.get('truth')}")
        out.append("")
        out.append("| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |")
        out.append("|---|---|---|---|---|---|---|---|---|")
        for i, g in enumerate(rep["goals"], 1):
            out.append(goal_line(i, g))
    c = counts(log)
    out.append("")
    out.append("log counts: " + ", ".join(f"{k} {v}" for k, v in c.items()))
    return "\n".join(out)


def table_row(run_dir: str) -> str:
    rep, log = load(run_dir)
    c = counts(log)
    if rep is None:
        return f"{run_dir}: no report"
    res = " ".join(("ok" if g["result"] == "SUCCEEDED" and not g["collision"] else
                    ("COL" if g["collision"] else g["result"][:4])) for g in rep["goals"])
    mc = min(g["min_clearance_m"] for g in rep["goals"])
    return (f"{os.path.basename(run_dir)}: {'PASS' if rep['passed'] else 'FAIL'} [{res}] min_clear {mc:.2f} "
            f"loc {max(g['max_localization_error_m'] for g in rep['goals']):.2f} "
            f"avoid {c['wall_avoid']} reseed {c['blind_reseed']} novalid {c['no_valid_path']} "
            f"startocc {c['start_occupied']} poseold {c['pose_old']} noprog {c['no_progress']}")


if __name__ == "__main__":
    if sys.argv[1] == "--table":
        for d in sys.argv[2:]:
            for r in sorted(glob.glob(os.path.join(d, "run*"))):
                print(table_row(r))
    else:
        print(markdown(sys.argv[1]))
