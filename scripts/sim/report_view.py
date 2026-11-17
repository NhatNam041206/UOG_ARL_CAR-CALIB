#!/usr/bin/env python3
"""Turn NAV2 sim mission reports into HTML pages you open in a browser.

    scripts/sim/run_sim.sh view            # every report under run_logs/sim
    python3 scripts/sim/report_view.py run_logs/sim/report-loop_left-*.json

Each page draws the world (walls, shelf, clutter), the goals with their
tolerance circle, the TRUE path of the car (solid, one colour per goal) and
the AMCL estimate (dashed), and a slider / play button that moves the car
outline along the run while showing the command, the collision-monitor
state and the clearance. An index.html lists every report.

Standard library + PyYAML only, so it runs in the sim image or on any
Python with PyYAML. Reports written before 2026-11-11 have no trace.
"""
from __future__ import annotations

import glob
import html
import json
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
WORLD_DIRS = [
    "/opt/car_nav_ws/share/car_nav_sim/worlds",
    os.path.join(REPO, "ros", "car_nav_sim", "worlds"),
]
CAR_YAMLS = [
    "/opt/car_nav_ws/share/car_nav/config/car.yaml",
    os.path.join(REPO, "ros", "car_nav", "config", "car.yaml"),
]
COLORS = ["#2563eb", "#16a34a", "#d97706", "#9333ea", "#0891b2", "#db2777", "#65a30d", "#dc2626"]
MAX_SCALE = 80   # px per metre (small rooms)
MAX_SIDE_PX = 1800  # big maps (the 56 m school map) are scaled down to this


def _first(paths):
    return next((p for p in paths if os.path.exists(p)), None)


def load_world(name: str) -> dict:
    for d in WORLD_DIRS:
        p = os.path.join(d, f"{name}.yaml")
        if os.path.exists(p):
            with open(p, encoding="utf-8") as fh:
                return yaml.safe_load(fh) or {}
    raise FileNotFoundError(f"world {name!r} not found in {WORLD_DIRS}")


def load_footprint() -> dict:
    p = _first(CAR_YAMLS)
    if not p:
        return {"x_front": 0.16, "x_rear": -0.37, "half_width": 0.135}
    with open(p, encoding="utf-8") as fh:
        return (yaml.safe_load(fh) or {}).get("footprint") or {}


def box_points(b: dict) -> list[tuple[float, float]]:
    cx, cy = b["center"]
    sx, sy = b["size"]
    yaw = float(b.get("yaw", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for hx, hy in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2)):
        pts.append((cx + c * hx - s * hy, cy + s * hx + c * hy))
    return pts


def render(report_path: str, out_path: str) -> dict:
    with open(report_path, encoding="utf-8") as fh:
        rep = json.load(fh)
    world = load_world(rep.get("world", "test_floor"))
    fp = load_footprint()
    x0, y0, x1, y1 = (float(v) for v in world["bounds"])
    SCALE = min(MAX_SCALE, MAX_SIDE_PX / max(x1 - x0, y1 - y0))
    W, H = (x1 - x0) * SCALE, (y1 - y0) * SCALE

    def X(x):
        return round((x - x0) * SCALE, 1)

    def Y(y):
        return round((y1 - y) * SCALE, 1)

    def poly(pts):
        return " ".join(f"{X(x)},{Y(y)}" for x, y in pts)

    svg = [f'<rect x="0" y="0" width="{W}" height="{H}" fill="var(--floor)"/>']
    step = 1 if max(x1 - x0, y1 - y0) < 20 else 5   # grid every 1 m (room) / 5 m (school map)
    for gx in range(math.ceil(x0), math.floor(x1) + 1, step):
        svg.append(f'<line x1="{X(gx)}" y1="0" x2="{X(gx)}" y2="{H}" class="grid"/>')
        svg.append(f'<text x="{X(gx) + 2}" y="{H - 3}" class="tick">{gx}</text>')
    for gy in range(math.ceil(y0), math.floor(y1) + 1, step):
        svg.append(f'<line x1="0" y1="{Y(gy)}" x2="{W}" y2="{Y(gy)}" class="grid"/>')
        svg.append(f'<text x="2" y="{Y(gy) - 2}" class="tick">{gy}</text>')
    for b in world.get("boxes") or []:
        svg.append(f'<polygon points="{poly(box_points(b))}" class="obst"/>')
    for c in world.get("cylinders") or []:
        svg.append(f'<circle cx="{X(c["center"][0])}" cy="{Y(c["center"][1])}" r="{c["radius"] * SCALE}" class="obst"/>')

    goals = rep.get("goals") or []
    frames = []
    rows = []
    for i, g in enumerate(goals):
        col = COLORS[i % len(COLORS)]
        gx, gy, gyaw = g["goal"]
        ok = g["result"] == "SUCCEEDED" and not g["collision"]
        svg.append(f'<circle cx="{X(gx)}" cy="{Y(gy)}" r="{0.5 * SCALE}" fill="{col}" fill-opacity="0.12" '
                   f'stroke="{col}" stroke-dasharray="4 3"/>')
        ax, ay = gx + 0.3 * math.cos(gyaw), gy + 0.3 * math.sin(gyaw)
        svg.append(f'<line x1="{X(gx)}" y1="{Y(gy)}" x2="{X(ax)}" y2="{Y(ay)}" stroke="{col}" stroke-width="2"/>')
        svg.append(f'<text x="{X(gx) + 6}" y="{Y(gy) - 6}" class="glabel" fill="{col}">G{i + 1}</text>')
        trace = g.get("trace") or []
        true_pts = [(r["true"][0], r["true"][1]) for r in trace]
        est_pts = [(r["est"][0], r["est"][1]) for r in trace if r.get("est")]
        if len(true_pts) > 1:
            svg.append(f'<polyline points="{poly(true_pts)}" fill="none" stroke="{col}" stroke-width="2.5"/>')
        if len(est_pts) > 1:
            svg.append(f'<polyline points="{poly(est_pts)}" fill="none" stroke="{col}" stroke-width="1.2" '
                       f'stroke-dasharray="5 4" opacity="0.8"/>')
        for r in trace:
            if r.get("clear", 1) <= 0.0:
                svg.append(f'<circle cx="{X(r["true"][0])}" cy="{Y(r["true"][1])}" r="6" class="hit"/>')
        for r in trace:
            frames.append({"g": i + 1, "t": r["t"], "x": r["true"][0], "y": r["true"][1], "th": r["true"][2],
                           "est": r.get("est"), "cmd": r.get("cmd"), "cm": r.get("cm") or "", "clear": r.get("clear")})
        rows.append(
            f'<tr><td><span class="dot" style="background:{col}"></span>G{i + 1}</td>'
            f'<td class="{"ok" if ok else "bad"}">{html.escape(g["result"])}{" · COLLISION" if g["collision"] else ""}</td>'
            f'<td>{g["time_s"]}</td><td>{g["distance_m"]}</td><td>{g["min_clearance_m"]}</td>'
            f'<td>{g["final_xy_error_m"]}</td><td>{g.get("max_localization_error_m")}</td>'
            f'<td>{g.get("steer_saturated_share")}</td><td>{html.escape(str(g.get("nav2_error") or ""))}</td></tr>')

    passed = bool(rep.get("passed"))
    title = f'{os.path.splitext(rep.get("mission", "mission"))[0]} · {os.path.basename(report_path)}'
    note = "" if frames else ('<p class="warn">No trace in this report, so Play has nothing to show: the '
                              'sim image was built before the trace was added. Run '
                              '<code>scripts/sim/run_sim.sh build</code>, then the mission again.</p>')
    car = {"xf": fp.get("x_front", 0.16), "xr": fp.get("x_rear", -0.37), "hw": fp.get("half_width", 0.135)}
    truth = rep.get("truth") or {}
    page = TEMPLATE.format(
        title=html.escape(title), W=W, H=H, svg="\n".join(svg), rows="\n".join(rows), note=note,
        verdict="PASS" if passed else "FAIL", vclass="ok" if passed else "bad",
        meta=html.escape(f'odometry {rep.get("odom_source") or "?"} · localizer {rep.get("localizer")} · speed {truth.get("speed_mps")} m/s · '
                         f'latency {truth.get("latency_s")} s · understeer {truth.get("understeer_factor")}'),
        frames=json.dumps(frames), car=json.dumps(car), x0=x0, y1=y1, scale=SCALE,
    )
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write(page)
    return {"title": title, "passed": passed, "out": out_path, "frames": len(frames)}


TEMPLATE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title>
<style>
:root {{ --bg:#f8fafc; --fg:#0f172a; --muted:#64748b; --floor:#ffffff; --obst:#94a3b8; --grid:#e2e8f0; --card:#ffffff; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#0b1120; --fg:#e2e8f0; --muted:#94a3b8; --floor:#111827; --obst:#475569; --grid:#1f2937; --card:#111827; }} }}
body {{ margin:0; padding:16px; background:var(--bg); color:var(--fg); font:14px/1.4 system-ui,sans-serif; }}
h1 {{ font-size:18px; margin:0 0 4px; }} .muted {{ color:var(--muted); }}
.ok {{ color:#16a34a; font-weight:600; }} .bad {{ color:#dc2626; font-weight:600; }} .warn {{ color:#d97706; }}
.card {{ background:var(--card); border:1px solid var(--grid); border-radius:8px; padding:8px; margin-top:12px; overflow-x:auto; }}
svg {{ width:100%; height:auto; max-width:{W}px; display:block; }}
.grid {{ stroke:var(--grid); stroke-width:1; }} .tick {{ fill:var(--muted); font-size:10px; }}
.obst {{ fill:var(--obst); }} .glabel {{ font-weight:700; font-size:13px; }}
.hit {{ fill:none; stroke:#dc2626; stroke-width:2.5; }}
#car {{ fill:#f59e0b; fill-opacity:.85; stroke:#78350f; stroke-width:1.5; }} #est {{ fill:none; stroke:#78350f; stroke-dasharray:3 2; }}
table {{ border-collapse:collapse; width:100%; }} th,td {{ text-align:left; padding:4px 8px; border-bottom:1px solid var(--grid); white-space:nowrap; }}
.dot {{ display:inline-block; width:10px; height:10px; border-radius:50%; margin-right:6px; }}
.ctl {{ display:flex; gap:8px; align-items:center; flex-wrap:wrap; }} input[type=range] {{ flex:1; min-width:160px; }}
button {{ padding:4px 12px; }} #info {{ font-family:ui-monospace,monospace; font-size:13px; }}
</style></head><body>
<h1>{title} <span class="{vclass}">{verdict}</span></h1>
<div class="muted">{meta}. Solid line = true path, dashed = AMCL estimate, circle = goal tolerance, red ring = collision.</div>
{note}
<div class="card"><svg viewBox="0 0 {W} {H}" id="map">{svg}
<polygon id="est" points=""/><polygon id="car" points=""/></svg></div>
<div class="card ctl"><button id="play">Play</button>
<select id="speed"><option value="1">1×</option><option value="2" selected>2×</option><option value="5">5×</option></select>
<input type="range" id="slider" min="0" max="0" value="0"><span id="info"></span></div>
<div class="card"><table><tr><th>Goal</th><th>Result</th><th>Time s</th><th>Dist m</th><th>Min clear m</th>
<th>End err m</th><th>Max loc err m</th><th>Steer sat</th><th>Nav2 error</th></tr>{rows}</table></div>
<script>
const F = {frames}, CAR = {car}, X0 = {x0}, Y1 = {y1}, S = {scale};
const sx = x => (x - X0) * S, sy = y => (Y1 - y) * S;
const outline = (x, y, thDeg) => {{
  const t = thDeg * Math.PI / 180, c = Math.cos(t), s = Math.sin(t);
  return [[CAR.xf, CAR.hw], [CAR.xf, -CAR.hw], [CAR.xr, -CAR.hw], [CAR.xr, CAR.hw]].map(([a, b]) => sx(x + c * a - s * b) + ',' + sy(y + s * a + c * b)).join(' ');
}};
const slider = document.getElementById('slider'), info = document.getElementById('info');
slider.max = Math.max(F.length - 1, 0);
if (!F.length) {{ document.getElementById('play').disabled = true; slider.disabled = true; }}
function show(i) {{
  const f = F[i]; if (!f) {{ info.textContent = 'no trace'; return; }}
  document.getElementById('car').setAttribute('points', outline(f.x, f.y, f.th));
  document.getElementById('est').setAttribute('points', f.est ? outline(f.est[0], f.est[1], f.est[2]) : '');
  info.textContent = `G${{f.g}} t=${{f.t.toFixed(1)}}s  true (${{f.x.toFixed(2)}}, ${{f.y.toFixed(2)}}, ${{f.th}}°)  cmd ${{f.cmd ?? '-'}}  ` +
    `monitor ${{f.cm || '-'}}  clear ${{f.clear}} m`;
}}
slider.oninput = () => show(+slider.value);
let timer = null;
document.getElementById('play').onclick = function () {{
  if (timer) {{ clearInterval(timer); timer = null; this.textContent = 'Play'; return; }}
  if (+slider.value >= F.length - 1) slider.value = 0;
  this.textContent = 'Pause';
  timer = setInterval(() => {{
    if (+slider.value >= F.length - 1) {{ clearInterval(timer); timer = null; this.textContent = 'Play'; return; }}
    slider.value = +slider.value + 1; show(+slider.value);
  }}, 500 / +document.getElementById('speed').value);
}};
show(0);
</script></body></html>
"""


def main(argv: list[str]) -> int:
    root = os.path.join(REPO, "run_logs", "sim") if not os.path.isdir("/data/sim") else "/data/sim"
    reports = argv or sorted(glob.glob(os.path.join(root, "**", "report-*.json"), recursive=True))
    if not reports:
        print(f"no reports under {root}: run scripts/sim/run_sim.sh mission loop_left first")
        return 1
    out_dir = os.path.join(root, "view")
    os.makedirs(out_dir, exist_ok=True)
    done = []
    for rp in reports:
        out = os.path.join(out_dir, os.path.splitext(os.path.basename(rp))[0] + ".html")
        try:
            done.append(render(rp, out))
        except Exception as exc:  # one bad report should not stop the rest
            print(f"skip {rp}: {exc}")
    items = "\n".join(
        f'<li><a href="{html.escape(os.path.basename(d["out"]))}">{html.escape(d["title"])}</a> '
        f'<b style="color:{"#16a34a" if d["passed"] else "#dc2626"}">{"PASS" if d["passed"] else "FAIL"}</b>'
        f'{"" if d["frames"] else " (no trace)"}</li>' for d in sorted(done, key=lambda d: d["out"], reverse=True))
    with open(os.path.join(out_dir, "index.html"), "w", encoding="utf-8") as fh:
        fh.write('<!doctype html><meta charset="utf-8"><title>NAV2 sim reports</title>'
                 '<body style="font:14px system-ui;padding:16px"><h1>NAV2 sim reports</h1><ul>'
                 f"{items}</ul></body>")
    print(f"{len(done)} page(s) in run_logs/sim/view/  ->  open run_logs/sim/view/index.html")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
