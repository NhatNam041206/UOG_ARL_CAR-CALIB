#!/usr/bin/env bash
# NAV2 simulator helper (laptop). Usage:
#   scripts/sim/run_sim.sh build                     # build the image (needs internet, ~15 min first time)
#   scripts/sim/run_sim.sh mission loop_left [args]  # headless 2D run, prints PASS/FAIL + report
#   scripts/sim/run_sim.sh mission loop_right odom_source:=truth
#   scripts/sim/run_sim.sh gui [args]                # Gazebo + RViz, drive with "2D Goal Pose"
#   scripts/sim/run_sim.sh rviz [args]               # 2D backend + RViz (lighter than Gazebo)
#   scripts/sim/run_sim.sh all                       # every mission, both odometry sources
#   scripts/sim/run_sim.sh shell                     # bash inside the image
# Extra args are ros2 launch arguments of car_nav_sim sim.launch.py.
set -euo pipefail
cd "$(dirname "$0")/../.."
COMPOSE=(docker compose -f docker-compose.sim.yml)
mkdir -p run_logs/sim

run() { "${COMPOSE[@]}" run --rm sim "$@"; }

latest_report() { ls -t run_logs/sim/report-*.json 2>/dev/null | head -1; }

mission() {
  local name="$1"; shift
  local before; before=$(latest_report || true)
  run ros2 launch car_nav_sim sim.launch.py backend:=lite mission:="$name" "$@" || true
  local rep; rep=$(latest_report || true)
  if [[ -z "$rep" || "$rep" == "$before" ]]; then
    echo "FAIL $name: no report written (see the log above)"; return 1
  fi
  python3 - "$rep" <<'PY' || return 1
import json, sys
r = json.load(open(sys.argv[1]))
for i, g in enumerate(r["goals"], 1):
    print(f"  goal {i}: {g['result']:9s} t={g['time_s']:6.1f}s clear={g['min_clearance_m']:.2f}m "
          f"end_err={g['final_xy_error_m']:.2f}m loc_err={g['max_localization_error_m']} sat={g['steer_saturated_share']}")
print(("PASS " if r["passed"] else "FAIL ") + r["mission"] + "  report: " + sys.argv[1])
sys.exit(0 if r["passed"] else 1)
PY
}

cmd="${1:-help}"; shift || true
case "$cmd" in
  build) "${COMPOSE[@]}" build ;;
  mission) mission "${1:?mission name}" "${@:2}" ;;
  all)
    rc=0
    for m in loop_left loop_right; do
      for o in truth rf2o; do echo "== $m odom=$o"; mission "$m" odom_source:="$o" || rc=1; done
    done
    exit $rc ;;
  gui) run ros2 launch car_nav_sim sim.launch.py backend:=gazebo gui:=true rviz:=true "$@" ;;
  rviz) run ros2 launch car_nav_sim sim.launch.py backend:=lite rviz:=true "$@" ;;
  shell) run bash ;;
  *) sed -n '2,11p' "$0"; exit 2 ;;
esac
