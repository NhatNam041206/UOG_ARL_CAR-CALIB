"""Combine the car nodes' "timing <name>: ..." log lines (car_nav/timing.py) of runs.

    python sim_results/tools/timing_summary.py <exp dir> [<exp dir> ...]      # per experiment
    python sim_results/tools/timing_summary.py --runs <exp dir>              # per run

Per name: median of the 10 s window p50s, max of the window p95s, max of the maxima,
total samples; counters are summed.
"""
import glob
import os
import re
import statistics
import sys
from collections import defaultdict

LINE = re.compile(r"timing (\S+): n (\d+) p50 ([\d.]+) p95 ([\d.]+) max ([\d.]+) (ms|s)")
COUNT = re.compile(r"timing (\S+): count (\d+)")
ORDER = ["mask.scan_cb", "upright.scan_cb", "odom.scan_cb", "odom.rf2o_cb", "odom.amcl_cb", "gate.amcl_cb",
         "gate.tf_timer", "pd.scan_cb", "pd.arc_check", "pd.tick", "bridge.cmd_cb",
         "odom.scan_age_s", "odom.rf2o_age_s", "odom.rf2o_period_s", "gate.amcl_age_s", "pd.pose_age_s",
         "pd.scan_to_cmd_s", "pd.cmd_period_s"]


def parse(logs):
    win = defaultdict(list)
    cnt = defaultdict(int)
    unit = {}
    for f in logs:
        for line in open(f, encoding="utf-8", errors="replace"):
            m = LINE.search(line)
            if m:
                k = m.group(1)
                win[k].append((int(m.group(2)), float(m.group(3)), float(m.group(4)), float(m.group(5))))
                unit[k] = m.group(6)
                continue
            m = COUNT.search(line)
            if m:
                cnt[m.group(1)] += int(m.group(2))
    return win, cnt, unit


def table(win, cnt, unit):
    rows = []
    for k in sorted(win, key=lambda k: (ORDER.index(k) if k in ORDER else 99, k)):
        w = win[k]
        rows.append(f"| {k} | {unit[k]} | {sum(x[0] for x in w)} | {statistics.median(x[1] for x in w):.3g} | "
                    f"{max(x[2] for x in w):.3g} | {max(x[3] for x in w):.3g} |")
    out = ["| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |", "|---|---|---|---|---|---|"]
    out += rows
    if cnt:
        out.append("")
        out.append("counters: " + ", ".join(f"{k} {v}" for k, v in sorted(cnt.items())))
    return "\n".join(out)


def main(argv):
    per_run = argv and argv[0] == "--runs"
    dirs = argv[1:] if per_run else argv
    for d in dirs:
        if per_run:
            for r in sorted(glob.glob(os.path.join(d, "run*"))):
                print(f"### {r}")
                print(table(*parse(glob.glob(os.path.join(r, "*.log")))))
        else:
            print(f"### {d}")
            print(table(*parse(glob.glob(os.path.join(d, "run*", "*.log")))))
        print()


if __name__ == "__main__":
    main(sys.argv[1:])
