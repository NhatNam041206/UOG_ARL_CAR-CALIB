#!/usr/bin/env python3
"""Closed-loop kinematic sim of SteeringController on a straight lane.

Idealised model (no vision noise, 2-frame actuation delay) to compare
controller settings, e.g. lateral gain 0 vs 60. Usage:
    python scripts/simulate_steering.py [plot.png]   # plot needs matplotlib
"""
import sys, math
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from control.steering_controller import SteeringController
from models.robot_state import PIDConstants
from unified_calibration_components import GeometryCalculator

W, D, F, LANE_PX = 0.6, 0.35, 686.0, 560.0   # lane m, lookahead m, focal px (HFOV ~50 deg), lane px at bottom
V, WB, DT, LIM, DELAY = 0.6, 0.26, 0.05, 45.0, 2

def run(lat_gain, y0, psi0_deg, kp=0.5, kd=0.2, T=10.0, hfov=0.0, inner=3.0, outer=5.0):
    c = SteeringController(PIDConstants(kp=kp, ki=0, kd=kd), danger_margin=93, nudge_deg=8, inner_thresh=inner, outer_thresh=outer,
                           center_angle=90, max_offset=90, danger_offset_ratio=0.2, vp_center_deg=90, lateral_gain=lat_gain)
    y, psi = y0 * W, math.radians(psi0_deg)  # y>0: right of centre; psi>0: pointing right
    queue = [90.0] * DELAY; log = []
    for k in range(int(T / DT)):
        vp_x = 320 - F * math.tan(psi)
        vp_angle = GeometryCalculator.map_vp_to_angle(vp_x, 640, hfov)
        off = -(y + D * math.tan(psi)) / W
        cx = 320 + off * LANE_PX
        cmd, state = c.compute_steering(vp_angle, int(cx - LANE_PX / 2), int(cx + LANE_PX / 2), 640)
        queue.append(cmd); applied = queue.pop(0)
        delta = math.radians(max(-LIM, min(LIM, -(applied - 90))))
        psi += V / WB * math.tan(delta) * DT
        y += V * math.sin(psi) * DT
        log.append((k * DT, y / W, math.degrees(psi), cmd, state, c.last_command_source))
    return log

def summary(log):
    cmds = [r[3] - 90 for r in log]
    flips = sum(1 for a, b in zip(cmds, cmds[1:]) if a * b < 0 and abs(a) > 3 and abs(b) > 3)
    danger = sum(1 for r in log if r[4].startswith("DANGER"))
    sat = sum(1 for c in cmds if abs(c) >= LIM)
    tail = [abs(r[1]) for r in log[-40:]]
    return dict(steer_flips=flips, danger_frames=danger, saturated_frames=sat, max_abs_offset=round(max(abs(r[1]) for r in log), 2),
                final_offset=round(log[-1][1], 3), final_heading=round(log[-1][2], 1), last2s_max_offset=round(max(tail), 3))

if __name__ == "__main__":
    import json
    cases = [(0.25, 0.0), (0.25, 10.0), (-0.3, -5.0), (0.1, 20.0)]
    res = {}
    for y0, p0 in cases:
        for g in (0.0, 60.0):
            res[f"y0={y0} psi0={p0} lat={g}"] = summary(run(g, y0, p0))
    for k, v in res.items(): print(k, v)
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
        for col, (y0, p0) in enumerate([(0.25, 0.0), (0.25, 10.0)]):
            for g, color in ((0.0, "#d62728"), (60.0, "#1f77b4")):
                log = run(g, y0, p0)
                t = [r[0] for r in log]
                axes[0][col].plot(t, [r[1] for r in log], color=color, label=f"lateral gain {g:.0f}")
                axes[1][col].plot(t, [r[3] - 90 for r in log], color=color)
            axes[0][col].axhspan(-0.2, 0.2, color="green", alpha=0.08)
            axes[0][col].set_title(f"start {y0:+.2f} lane right, heading {p0:+.0f} deg")
            axes[0][col].set_ylabel("car offset (lane widths, + = right)"); axes[1][col].set_ylabel("steer cmd - 90 (+ = left)")
            axes[1][col].set_xlabel("time (s)"); axes[0][col].legend()
        fig.tight_layout(); fig.savefig(sys.argv[1] if len(sys.argv) > 1 else "sim.png", dpi=90)
    except ImportError:
        print("no matplotlib")
