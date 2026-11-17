# campus_block_left simulation campaign - SUMMARY

Machine: Escapepod, nam's laptop (i7-11800H, 8 cores / 16 threads, 15.7 GB, Docker Desktop VM with 8 CPUs;
see 00_environment.md). Branch `claude/project-thread-t304ze`. Updated 2026-11-17.

Mission `campus_block_left`: 4 goals, all left turns. Goal 2 = left turn from the 1.8 m corridor into the
1.2 m corridor, and the turn must start within about +-0.4 m of the ideal point (01_method_review.md).
PASS = all 4 goals SUCCEEDED, no collision. Every experiment folder has a `notes.md` (per-goal table,
log counts, timing budget). Each `runNN/` keeps the log, the report JSON, args.txt and, for profiled
runs, probe.csv / cpu.txt / pinning.txt. These are on Escapepod, and the big files are git-ignored.

Code versions:
- **orig** = 6b4b3d4 (branch tip at the start) plus measurement-only changes: mission TF thread, exact
  ~70x faster clearance in the scorer, SIM_CPUS / SIM_STRESS script fixes, timing logs. It runs from the
  worktree `../repo_base`.
- **numpy** = orig + numpy scan_observability / arc sweep / points_in_base (a68b3ab, identical outputs).

CPU limits:
- **quota** = `SIM_CPUS=n`. The whole container, simulator included, gets n CPUs.
- **pinned** = `CAR_CORES=...`. Only the car stack (Nav2, AMCL, rf2o, our nodes) runs on those cores. The
  simulator, scorer, probe and launch are pinned to cores 6-7 (sim_results/tools/pin_watch.sh), so the limit
  models the car computer. "6 cores" = unconstrained reference.

## 1. All experiments

| id | code | CPU | condition | runs passed | goals per run (ok / COL = collision / TIME / FAIL) | key numbers |
|---|---|---|---|---|---|---|
| 01_baseline | orig (old scorer) | full | none | 3/3 | ok ok ok ok x3 | min clear 0.23-0.25 m |
| 02_baseline_tf_fix | orig | full | mission TF thread | 5/5 | ok ok ok ok x5 | min clear 0.18-0.27 m; goal-2 start pose 0.03-0.1 m from truth |
| 03_odom_truth | orig | full | odom_source:=truth | 1/2 | startup race (AMCL never took the initial pose) / ok x4 | |
| 04_no_pose_gate | orig | full | pose_gate:=false | 1/1 | ok x4 | min clear 0.24 m |
| 05_stress_speed_drift | orig | full | speed_drift (0.42 m/s +-15 %) | 3/3 | ok x4 x3 | min clear 0.14-0.29 m |
| 06_base_profile_fullcpu | orig | full | first (heavy) probe | 1/1 | ok x4 | |
| 07a_base_cpus2_old_scorer | orig | 2 quota | slow scorer + heavy probe | 0/1 | ok ok TIME TIME | overshot goal 3, stuck at the end wall |
| 07_base_cpus2 | orig | 2 quota | | 0/3 | ok COL COL COL / ok ok ok TIME / ok COL COL COL | |
| 08_base_cpus1 | orig | 1 quota | | 0/2 + 1 hang | COL FAIL FAIL FAIL / HUNG at Nav2 startup / ok COL COL COL | |
| 11_numpy_fullcpu | numpy | full | | 3/3 | ok x4 x3 | min clear 0.14-0.20 m |
| 12_numpy_stress_speed_drift | numpy | full | speed_drift | 2/3 | ok x4 / ok ok ok TIME / ok x4 | |
| 13_numpy_cpus2 | numpy | 2 quota | | 0/2 + 1 hang | ok TIME TIME TIME / ok ok FAIL TIME / HUNG after the initial pose | |
| 14_numpy_cpus1 | numpy | 1 quota | | 0/3 | ok FAIL FAIL FAIL / COL x4 / ok COL COL COL | |
| 20_pintest | numpy | 2 pinned | first pinned run | 0/1 | ok ok ok TIME | |
| 26_pin_orig_car6core | orig | 6 pinned | reference | 1/2 + 1 hang | ok ok FAIL FAIL / ok x4 / HUNG | |
| 36_pin_numpy_car6core | numpy | 6 pinned | reference | **3/3** | ok x4 x3 | min clear 0.07-0.26 m |
| 22_pin_orig_car2core | orig | 2 pinned | | **0/3** | ok ok FAIL FAIL / ok COL COL COL / ok FAIL FAIL FAIL | |
| 32_pin_numpy_car2core | numpy | 2 pinned | | **1/3** | ok ok TIME TIME / ok x4 / ok COL COL COL | |
| 21_pin_orig_car1core | orig | 1 pinned | | 0/2 + 1 hang | FAIL x4 / FAIL x4 / HUNG | AMCL pose up to 11.6 s late |
| 31_pin_numpy_car1core | numpy | 1 pinned | | 0/3 | TIME x4 / ok TIME TIME TIME / FAIL COL COL COL | pose age up to 7.8 s |

Pass rates in short:

| car CPU | orig | numpy |
|---|---|---|
| full / 6 pinned cores | 19/22 runs with a real start (+2 startup hangs) | 6/6 (+ speed_drift 2/3) |
| 2 (quota) | 0/3 | 0/2 (+1 hang) |
| 2 pinned cores | 0/3 | 1/3 |
| 1 (quota or pinned) | 0/4 (+2 hangs) | 0/6 |

## 2. Timing budget (pinned runs; car nodes' own logs, p50 / worst-window p95)

| measurement | 6 cores orig | 6 cores numpy | 2 cores orig | 2 cores numpy | 1 core orig | 1 core numpy |
|---|---|---|---|---|---|---|
| car stack CPU used (% of one core) | 220 | 242 | 171 (of 200) | 169 (of 200) | 91 (of 100) | 94 (of 100) |
| **AMCL pose age on arrival at the gate (s)** | 0.19 / 1.21 | 0.22 / 1.02 | **0.96 / 3.61** | **0.82 / 3.56** | 5.3 / 11.6 | 2.8 / 18.3 |
| adaptive_odom scan callback (ms) | 20 / 104 | 4.5 / 25 | 24 / 112 | 14 / 97 | 31 / 253 | 42 / 214 |
| adaptive_odom: scan age when processed (s) | 0.03 / 0.69 | 0.01 / 0.21 | 0.03 / 0.57 | 0.03 / 0.46 | 0.04 / 0.90 | 0.05 / 0.84 |
| rf2o period (s; scans every 0.14) | 0.14 / 0.29 | 0.14 / 0.15 | 0.14 / 0.28 | 0.14 / 0.29 | 0.14 / 0.29 | 0.14 / 0.29 |
| pd tick, whole control step (ms) | 1.9 / 63 | 2.6 / 95 | 4.8 / 307 | 13.5 / 137 | 2.2 / 785 | 68 / 1140 |
| pd arc check (ms) | 1.1 / 57 | 1.5 / 95 | 1.3 / 130 | 10 / 125 | 0.8 / 663 | 36 / 399 |
| **pose age at use in the PD (s)** | 0.13 / 0.24 | 0.13 / 0.49 | 0.18 / 0.41 | 0.17 / 0.42 | 3.0 / 6.5 | 0.66 / 7.6 |
| scan -> command (s) | 0.08 / 0.14 | 0.09 / 0.17 | 0.10 / 0.21 | 0.10 / 0.23 | 0.12 / 0.84 | 0.13 / 0.84 |
| command period (s; nominal 0.1), max | 3.5 | 2.3 | 2.3 | 3.5 | 30 | 30 |
| probe: estimate error along the corridor at its own stamp, p95 per run (m) | 0.33-0.74 | 0.30-0.60 | **0.40-2.20** | **0.44-1.11** | 0.74 | 1.3-5.2 |

(Callback times are wall time, so under contention they include waiting for the CPU. On an idle
machine the numpy scan check is 1.8-2.2 ms and the arc check 0.1-2.7 ms (bench_hotspots_escapepod.md).)

Per-node CPU with the car stack unconstrained (6 pinned cores, % of one i7 core):

| node | orig | numpy |
|---|---|---|
| pd_path_follower | 64 | 55 |
| amcl | 20 | 30 |
| adaptive_odom | 18 | 21 |
| pose_gate | 17 | 18 |
| lifecycle managers | 11 | 17 |
| bt_navigator | 13 | 14 |
| scan_self_mask / scan_upright | 11 / 10 | 13 / 11 |
| cmd_vel_to_control | 10 | 12 |
| controller_server / planner_server | 9 / 9 | 9 / 9 |
| **total car stack** | **~220** | **~240** |

## 3. What the measurements say

1. **The car stack needs about 2.2-2.4 i7 cores.** With 2 cores it runs at 85 % load and with 1 core it
   is saturated. A Pi 5 core is slower than an i7 core, so the Pi 5's 4 cores are about this budget or
   less.
2. **With 2 cores the failures come from localization accuracy, not a stale pose.** The pose the PD uses
   is only 0.17-0.18 s old (p95 0.41-0.42 s, i.e. 0.1-0.2 m). But the estimate is wrong at its own
   timestamp by p95 0.4-2.2 m. That is more than the +-0.4 m turn window, so the car turns late, collides
   or gets a "no valid path" / "Start occupied" from the planner, and then cannot reverse.
3. **AMCL is the bottleneck of that chain.** Its pose arrives at the pose gate 0.8-1.0 s late (p50) and
   3.6 s late (p95) with 2 cores, against 0.2 / 1.0-1.2 s with 6 cores. AMCL runs 720 beams x 500-2000
   particles every 5 cm (nav2_params.yaml). In the blind corridors the estimate then rides on the
   adaptive odometry for longer, and its error grows.
4. **The numpy speed-ups help, but not enough.** The adaptive_odom scan callback went from 20 to 4.5 ms
   p50 unconstrained, and the 2-core pass rate from 0/3 to 1/3. pd_path_follower still uses about 55 % of
   a core, while the arc check itself is about 2.5 % at 10 Hz, so the rest is rclpy overhead: the
   4-thread MultiThreadedExecutor, the Python TF listener on every /tf message, and deserialising 720-beam
   scans.
5. **At 1 core even the pose age explodes** (p95 6.5-7.6 s, commands stop for up to 30 s). There, pose
   extrapolation could not help either.
6. **Startup is fragile under load.** 5 of about 60 runs hung at startup (Nav2 lifecycle waiting for
   controller_server, or the mission waiting for AMCL's first pose). The 25-min watchdog stops them; they
   are recorded as HUNG.

## 4. Proposed fixes (not implemented; for nam to pick)

Pure speed-ups (no change in what the car decides):
- **P1** pd_path_follower: SingleThreadedExecutor or 2 threads instead of 4, and read map->odom /
  odom->base_link from direct /tf subscriptions instead of the tf2 buffer. Expected: most of its
  ~55 % CPU.
- **P2** pose_gate publish_rate_hz 20 -> 10. map->odom is future-dated by 0.5 s anyway, and every TF
  listener (AMCL, planner, controller, collision monitor, bt_navigator, PD) deserialises each /tf message.
- **P3** One Python process for scan_self_mask + scan_upright (one scan deserialisation instead of
  two). Or move them, and the observability check, to C++.
- **P4** Nav2 lifecycle bond heartbeat: lifecycle managers use 11-17 %. Disabling the bonds
  (bond_timeout 0) saves that but loses crash detection; risky, only if needed.

Accuracy trade-offs, one at a time, each checked against this baseline at 6 and 2 cores:
- **A1** AMCL max_beams 720 -> 180 (or 240) and max_particles 2000 -> 1000. This is the bottleneck
  in item 3. The comment in nav2_params says 720 was chosen so a far end wall gets enough beams, so check
  the goal-2 accuracy.
- **A2** AMCL update_min_d 0.05 -> 0.1 (half as many updates).

Control-method changes (need nam's choice):
- **C1** Pose extrapolation by age in the PD follower (written and unit-tested as
  pd_follower.extrapolate_pose, not wired). The data shows it covers only 0.1-0.2 m at 2 cores, so it
  would not fix the 2-core failures on its own.
- **C2** Use AMCL's age in the pose gate: weight or skip AMCL corrections older than X s, or reseed
  less aggressively when they are late.
- **C3** Make startup robust: retry the initial pose until AMCL answers, and a lifecycle timeout with
  restart.

Suggested order: P1 + P2 (no behaviour change), then A1, re-measured at 2 pinned cores (target >= 4/5),
then decide on C1/C2.

## 5. Other findings

- At full CPU the arc-check wall avoid works in the full stack (fires 2-15 times per run in the left
  turns, never fought a planned turn).
- The adaptive odometry stands still for the first ~2 s after the start (about 1 m lag that is
  recovered by the end of goal 1).
- A forward-only car stuck at a wall never recovers. All goals after the first failure are knock-on
  failures.
- The earlier "1.5-2.5 m localization error" at full CPU was mostly the mission scorer lagging (slow
  clearance + TF listener on a busy thread). Both are fixed.

## Files

- 00_environment.md: machine facts.
- 01_method_review.md: review of the proposed methods and the goal-2 geometry.
- bench_hotspots_escapepod.md: per-call cost, old vs numpy, idle machine.
- `NN_*/notes.md`: per experiment, with per-goal tables and the timing budget.
- tools/: run_batch.sh helpers (latency probe, CPU sampler, core pinning, summaries, benchmark).
