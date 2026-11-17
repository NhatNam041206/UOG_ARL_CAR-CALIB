"""Summarise probe.csv (latency chain, localisation error) and cpu.txt (per-process CPU)
of profiled runs.
    python sim_results/tools/profile_summary.py sim_results/07_base_cpus2 [more exp dirs...]
"""
import csv
import glob
import math
import os
import sys
from collections import defaultdict


def pct(v, q):
    v = sorted(x for x in v if not math.isnan(x))
    if not v:
        return math.nan
    return v[min(len(v) - 1, int(q * (len(v) - 1) + 0.5))]


def probe(path):
    rows = list(csv.DictReader(open(path)))
    moving = [r for r in rows if float(r["v_true"]) > 0.2]
    out = {}
    for col in ("scan_nav_age", "rf2o_age", "odom_age", "tf_age", "cmd_dt"):
        vals = [float(r[col]) for r in moving]
        out[col] = (pct(vals, 0.5), pct(vals, 0.95), max(vals) if vals else math.nan)
    for col in ("err_now_along", "err_stamp_along", "err_now_lat"):
        vals = [abs(float(r[col])) for r in moving]
        out[col] = (pct(vals, 0.5), pct(vals, 0.95), max(vals) if vals else math.nan)
    return out, len(moving)


def cpu(path):
    tot, n = defaultdict(float), 0
    for line in open(path):
        if line.startswith("##"):
            n += 1
            continue
        parts = line.split()
        if len(parts) == 2:
            try:
                tot[parts[1]] += float(parts[0])
            except ValueError:
                pass
    return {k: v / max(n, 1) for k, v in sorted(tot.items(), key=lambda kv: -kv[1])}, n


def main(dirs):
    for d in dirs:
        for run in sorted(glob.glob(os.path.join(d, "run*"))):
            print(f"## {run}")
            p = os.path.join(run, "probe.csv")
            if os.path.exists(p):
                stats, n = probe(p)
                print(f"probe ({n} rows while moving): median / p95 / max")
                for k, (a, b, c) in stats.items():
                    print(f"  {k:16s} {a:7.3f} {b:7.3f} {c:7.3f}")
            c = os.path.join(run, "cpu.txt")
            if os.path.exists(c):
                avg, n = cpu(c)
                print(f"cpu (mean % of one core over {n} samples): " +
                      ", ".join(f"{k} {v:.0f}" for k, v in list(avg.items())[:14]) +
                      f"  | total {sum(avg.values()):.0f}")


if __name__ == "__main__":
    main(sys.argv[1:])
