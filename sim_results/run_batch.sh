#!/usr/bin/env bash
# Run campus_block_left N times headless and keep every log + report.
#   sim_results/run_batch.sh <exp_dir_name> <n_runs> [first_seed] [launch args...]
# e.g. sim_results/run_batch.sh 01_baseline 3 1
#      sim_results/run_batch.sh 02_truth_odom 1 1 odom_source:=truth
# CAR_CORES=<cpu list>: pin the car stack to these cores, the simulator to SIM_CORES (tools/pin_watch.sh).
# PROFILE=1: also latency probe (probe.csv) + per-process CPU (cpu.txt). SIM_CPUS=n: CPU limit.
# Run i uses LiDAR noise seed first_seed+i-1. Results: sim_results/<exp>/runNN/.
set -uo pipefail
cd "$(dirname "$0")/.."
exp="$1"; n="$2"; seed0="${3:-1}"; shift 3 || shift $#
out="${RESULTS_DIR:-sim_results}"   # RESULTS_DIR: put the runs elsewhere (e.g. from a second worktree)
mkdir -p "$out/$exp"
for i in $(seq 1 "$n"); do
  seed=$((seed0 + i - 1))
  d=$(printf "%s/%s/run%02d" "$out" "$exp" "$seed")
  mkdir -p "$d"
  before_log=$(ls -t run_logs/sim/campus_block_left-*.log 2>/dev/null | head -1)
  before_rep=$(ls -t run_logs/sim/report-*.json 2>/dev/null | head -1)
  echo "== $exp run seed=$seed args: $*"
  if [[ "${PROFILE:-0}" == 1 ]]; then rm -f run_logs/sim/probe-current.csv; sim_results/tools/profile_watch.sh "$d" & wpid=$!; fi
  # CAR_CORES=0-1: car-only CPU limit (sim tooling pinned to SIM_CORES, default 6-7)
  if [[ -n "${CAR_CORES:-}" ]]; then sim_results/tools/pin_watch.sh "$d" & ppid=$!; fi
  scripts/sim/run_sim.sh mission campus_block_left seed:="$seed" "$@" > "$d/console.txt" 2>&1 &
  rpid=$!; t0=$(date +%s); hung=0
  # watchdog: a run that is not done after RUN_TIMEOUT_S wall seconds (default 25 min) is hung
  # (seen: Nav2 lifecycle waiting for controller_server forever at 1 CPU): keep its log, stop it.
  while kill -0 $rpid 2>/dev/null; do
    if (( $(date +%s) - t0 > ${RUN_TIMEOUT_S:-1500} )); then
      hung=1
      for c in $(docker ps -q --filter name=sim-run); do docker logs "$c" > "$d/hung_run.log" 2>&1; docker rm -f "$c" >/dev/null; done
      kill $rpid 2>/dev/null; break
    fi
    sleep 10
  done
  wait $rpid 2>/dev/null
  if [[ "${PROFILE:-0}" == 1 ]]; then wait $wpid; [[ -f run_logs/sim/probe-current.csv ]] && mv run_logs/sim/probe-current.csv "$d/probe.csv"; fi
  log=$(ls -t run_logs/sim/campus_block_left-*.log 2>/dev/null | head -1)
  rep=$(ls -t run_logs/sim/report-*.json 2>/dev/null | head -1)
  [[ -n "$log" && "$log" != "$before_log" ]] && cp "$log" "$d/"
  [[ -n "$rep" && "$rep" != "$before_rep" ]] && cp "$rep" "$d/"
  rm -f "$d/console.txt"   # same content as the log
  echo "args: $* SIM_CPUS=${SIM_CPUS:-} SIM_STRESS=${SIM_STRESS:-} CAR_CORES=${CAR_CORES:-} wall_s=$(( $(date +%s) - t0 ))$([[ $hung == 1 ]] && echo ' HUNG: stopped by the watchdog')" > "$d/args.txt"
  python sim_results/analyze.py --table "$out/$exp" | grep "run$(printf %02d "$seed")"
done
