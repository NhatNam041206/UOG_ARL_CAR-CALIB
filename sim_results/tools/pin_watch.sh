#!/usr/bin/env bash
# Car-only CPU limit (started by run_batch.sh when CAR_CORES is set):
# pins every process in the sim container to CAR_CORES, except the simulator's
# own tooling (lite_sim, the mission scorer, the latency probe, ros2 launch,
# truth_odom_tf, this pinning loop) which go to SIM_CORES. So the limit models
# the car computer, not the simulator. Re-pins every 2 s (nodes start late).
#   CAR_CORES=0-1 SIM_CORES=6-7 pin_watch.sh <out_dir>
out="$1"; mkdir -p "$out"
car="${CAR_CORES:?}"; sim="${SIM_CORES:-6-7}"
for i in $(seq 1 120); do c=$(docker ps -q --filter name=sim-run | head -1); [[ -n "$c" ]] && break; sleep 1; done
[[ -z "$c" ]] && exit 0
script='
car="$1"; sim="$2"
while true; do
  for d in /proc/[0-9]*; do
    p=${d#/proc/}
    cmd=$(tr "\0" " " < $d/cmdline 2>/dev/null) || continue
    [[ -z "$cmd" ]] && continue
    case "$cmd" in
      *lite_sim*|*car_nav_sim/mission*|*latency_probe*|*"ros2 launch"*|*truth_odom_tf*|*"gz sim"*|*pin_loop*|*top\ -b*) cores=$sim ;;
      *) cores=$car ;;
    esac
    taskset -a -cp "$cores" "$p" >/dev/null 2>&1
  done
  sleep 2
done'
MSYS_NO_PATHCONV=1 docker exec -d "$c" bash -c "exec -a pin_loop bash -c '$script' pin_loop $car $sim"
sleep 20
# record the pinning once the stack is up
MSYS_NO_PATHCONV=1 docker exec "$c" bash -c 'for d in /proc/[0-9]*; do p=${d#/proc/}; n=$(tr "\0" " " < $d/cmdline 2>/dev/null | cut -c1-90); [[ -n "$n" ]] && echo "$(taskset -cp $p 2>/dev/null | sed "s/.*: //") | $n"; done' > "$out/pinning.txt" 2>&1
