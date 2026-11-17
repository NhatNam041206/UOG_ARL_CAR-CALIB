#!/usr/bin/env bash
# Started by run_batch.sh when PROFILE=1: waits for the sim container, starts the
# latency probe inside it and samples per-process CPU every 3 s until it exits.
#   profile_watch.sh <out_dir>
out="$1"; mkdir -p "$out"
for i in $(seq 1 120); do c=$(docker ps -q --filter name=sim-run | head -1); [[ -n "$c" ]] && break; sleep 1; done
[[ -z "$c" ]] && exit 0
sleep 8   # let the nodes start
MSYS_NO_PATHCONV=1 docker cp sim_results/tools/latency_probe.py "$c":/tmp/latency_probe.py
MSYS_NO_PATHCONV=1 docker exec -d "$c" bash -c 'source /opt/ros/jazzy/setup.bash; source /opt/car_nav_ws/setup.bash; export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}; python3 /tmp/latency_probe.py /data/sim/probe-current.csv'
: > "$out/cpu.txt"
while docker ps -q --no-trunc | grep -q "$(docker inspect -f '{{.Id}}' "$c" 2>/dev/null || echo none)"; do
  { echo "## $(date +%s)"; MSYS_NO_PATHCONV=1 docker exec "$c" top -b -n1 -w 200 2>/dev/null | sed -n '8,40p' | awk '{print $9, $12}'; } >> "$out/cpu.txt" 2>/dev/null
  sleep 3
done
