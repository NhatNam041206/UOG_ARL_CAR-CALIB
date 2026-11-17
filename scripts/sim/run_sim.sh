#!/usr/bin/env bash
# NAV2 simulator helper (laptop). Usage:
#   scripts/sim/run_sim.sh build                     # build the image (needs internet, ~15 min first time)
#   scripts/sim/run_sim.sh mission loop_left [args]  # headless 2D run, prints PASS/FAIL + report
#   scripts/sim/run_sim.sh mission loop_right odom_source:=truth
#   scripts/sim/run_sim.sh gui [args]                # Gazebo + RViz, drive with "2D Goal Pose"
#   scripts/sim/run_sim.sh rviz [args]               # 2D backend + RViz (lighter than Gazebo)
#   scripts/sim/run_sim.sh campus [anchor] [args]    # school map in RViz, car at an anchor (AAD-1)
#   scripts/sim/run_sim.sh campus-mission [args]     # school map mission (campus_block_left), with RViz
#   SIM_STRESS=speed_drift scripts/sim/run_sim.sh ... # car worse than car.yaml (sweeps/stress/*.yaml)
#   scripts/sim/run_sim.sh all                       # every mission, both odometry sources
#   scripts/sim/run_sim.sh view                      # reports -> run_logs/sim/view/index.html (browser)
#   scripts/sim/run_sim.sh shell                     # bash inside the image
# RViz: a window from WSL Ubuntu / Linux; from Git Bash in the browser at http://localhost:6080 (SIM_VIEW=browser|x11).
# Extra args are ros2 launch arguments of car_nav_sim sim.launch.py.
set -euo pipefail
cd "$(dirname "$0")/../.."
COMPOSE=(docker compose -f docker-compose.sim.yml)
mkdir -p run_logs/sim

# SIM_CPUS=<n>: limit the sim container to n CPUs (emulates a slower computer such as
# the Pi 5 or an overloaded laptop). Off by default; only adds a CPU quota.
if [[ -n "${SIM_CPUS:-}" ]]; then
  printf 'services:\n  sim:\n    cpus: "%s"\n' "$SIM_CPUS" > run_logs/sim/cpus-override.yml
  COMPOSE+=(-f run_logs/sim/cpus-override.yml)
  echo "cpu limit: $SIM_CPUS"
fi

if ! docker info >/dev/null 2>&1; then
  echo "Docker is not running. Start Docker Desktop, wait for 'Engine running', then retry." >&2
  echo "(check with: docker version  -> it must show a 'Server:' section)" >&2
  exit 3
fi

RUN_EXTRA=()
run() { "${COMPOSE[@]}" run --rm ${RUN_EXTRA[@]+"${RUN_EXTRA[@]}"} sim "$@"; }

# A window (RViz / Gazebo) needs a display. Two ways:
#   x11     (default when DISPLAY is set: Linux desktop, WSL Ubuntu via WSLg):
#            a normal RViz window, /tmp/.X11-unix mounted into the container.
#   browser (default otherwise, e.g. Git Bash): RViz on a virtual screen in the
#            container, open http://localhost:6080 (noVNC). Needs nothing.
# Force one with SIM_VIEW=browser or SIM_VIEW=x11.
need_display() {
  local view="${SIM_VIEW:-}"
  if [[ -z "$view" ]]; then
    if [[ "$(uname -s)" == Linux && -n "${DISPLAY:-}" ]]; then view=x11; else view=browser; fi
  fi
  if [[ "$view" == browser ]]; then
    # A previous sim run that did not exit (window closed, Ctrl+C in Git Bash)
    # still holds port 6080: remove only our own sim containers that publish it.
    local old
    old=$(docker ps -q --filter "name=sim-run" --filter "publish=6080" 2>/dev/null || true)
    if [[ -n "$old" ]]; then
      echo "stopping the previous sim run that still holds port 6080"
      docker rm -f $old >/dev/null
    fi
    # 127.0.0.1 only: the page is not reachable from the LAN
    RUN_EXTRA+=(-e SIM_VIEW=browser -p 127.0.0.1:6080:6080)
    echo "view: browser -> open http://localhost:6080/vnc.html?autoconnect=1&resize=scale (wait ~15 s)"
    return
  fi
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is empty: no X display. Use SIM_VIEW=browser, or on Linux run from a desktop session (xhost +local:)." >&2
    exit 4
  fi
  RUN_EXTRA+=(-v /tmp/.X11-unix:/tmp/.X11-unix -e QT_QPA_PLATFORM=xcb -e WAYLAND_DISPLAY= -e LIBGL_ALWAYS_SOFTWARE=1)
  echo "view: x11 DISPLAY=$DISPLAY. RViz errors show as [rviz2-...] lines."
}

# SIM_STRESS=<name>: the simulated car behaves as ros/car_nav_sim/sweeps/stress/<name>.yaml
# says (e.g. SIM_STRESS=speed_drift: real speed 0.42 m/s +-15 % while car.yaml says 0.5).
apply_stress() {
  [[ -z "${SIM_STRESS:-}" ]] && return
  local f="ros/car_nav_sim/sweeps/stress/${SIM_STRESS}.yaml"
  [[ -f "$f" ]] || { echo "no stress file $f" >&2; exit 2; }
  run bash -c "python3 /opt/car_nav_sim_tools/apply_overrides.py /opt/car_nav_ws/share/car_nav_sim/config/sim.yaml \
    /ws/src/car_nav_sim/sweeps/stress/${SIM_STRESS}.yaml /data/sim/stress-sim.yaml sim"
  # Git Bash would rewrite the container path into a Windows one (D:/Git/data/...)
  export MSYS2_ARG_CONV_EXCL="CAR_NAV_SIM_CONFIG=${MSYS2_ARG_CONV_EXCL:+;$MSYS2_ARG_CONV_EXCL}"
  RUN_EXTRA+=(-e CAR_NAV_SIM_CONFIG=/data/sim/stress-sim.yaml)
  echo "stress: $SIM_STRESS"
}
apply_stress

# SIM_NAV2_PARAMS=<file>: run with another nav2_params.yaml (parameter variants), copied
# into run_logs/sim so the container sees it. The default is ros/car_nav/config/nav2_params.yaml.
if [[ -n "${SIM_NAV2_PARAMS:-}" ]]; then
  [[ -f "$SIM_NAV2_PARAMS" ]] || { echo "no params file $SIM_NAV2_PARAMS" >&2; exit 2; }
  cp "$SIM_NAV2_PARAMS" run_logs/sim/nav2-params-variant.yaml
  export MSYS2_ARG_CONV_EXCL="CAR_NAV_NAV2_PARAMS=${MSYS2_ARG_CONV_EXCL:+;$MSYS2_ARG_CONV_EXCL}"
  RUN_EXTRA+=(-e CAR_NAV_NAV2_PARAMS=/data/sim/nav2-params-variant.yaml)
  echo "nav2 params: $SIM_NAV2_PARAMS"
fi

latest_report() { ls -t run_logs/sim/report-*.json 2>/dev/null | head -1; }

mission() {
  local name="$1"; shift
  local before; before=$(latest_report || true)
  local log; log="run_logs/sim/${name}-$(date +%Y%m%d-%H%M%S).log"
  run ros2 launch car_nav_sim sim.launch.py backend:=lite mission:="$name" "$@" 2>&1 | tee "$log" || true
  echo "full log: $log"
  local rep; rep=$(latest_report || true)
  if [[ -z "$rep" || "$rep" == "$before" ]]; then
    echo "FAIL $name: no report written (see the log above)"; return 1
  fi
  python3 - "$rep" <<'PY' || return 1
import json, sys
r = json.load(open(sys.argv[1]))
for i, g in enumerate(r["goals"], 1):
    print(f"  goal {i}: {g['result']:9s} t={g['time_s']:6.1f}s clear={g['min_clearance_m']:.2f}m "
          f"end_err={g['final_xy_error_m']:.2f}m loc_err={g['max_localization_error_m']} sat={g['wheel_saturated_share']}"
          + (f"  nav2: {g['nav2_error']}" if g.get("nav2_error") else ""))
if not any(g.get("trace") for g in r["goals"]):
    print("  (no trace in this report: the image is older than the code -> scripts/sim/run_sim.sh build)")
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
  gui) need_display; run ros2 launch car_nav_sim sim.launch.py backend:=gazebo gui:=true rviz:=true "$@" ;;
  rviz) need_display; run ros2 launch car_nav_sim sim.launch.py backend:=lite rviz:=true "$@" ;;
  campus)
    need_display
    start="${1:-AAD-1}"; shift || true
    run ros2 launch car_nav_sim sim.launch.py backend:=lite rviz:=true world:=campus_path start:="$start" "$@" ;;
  campus-mission) need_display; mission campus_block_left rviz:=true "$@" ;;
  view)
    # bash -c keeps Git Bash from rewriting the container path into a Windows one
    run bash -c "python3 /opt/car_nav_sim_tools/report_view.py"
    echo "Open it:  start run_logs/sim/view/index.html   (Git Bash / Windows)" ;;
  shell) run bash ;;
  *) sed -n '2,16p' "$0"; exit 2 ;;
esac
