# 21_pin_orig_car1core

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0 wall_s=1517 HUNG: stopped by the watchdog
### run01
NO REPORT (see log)

log counts: wall_avoid 0, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0 wall_s=223
### run02
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | FAILED | 39 | 0.67 | 11.10 | 50.61 | -26.27 | 0.00 | |
| 2 | FAILED | 36 | 0.67 | 15.84 | 50.61 | -26.27 | 0.00 | |
| 3 | FAILED | 30 | 0.67 | 12.22 | 50.61 | -26.27 | 0.00 | |
| 4 | FAILED | 33 | 0.67 | 4.88 | 50.61 | -26.27 | 0.00 | |

log counts: wall_avoid 0, avoid_release 0, blind_reseed 0, no_valid_path 25, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0 wall_s=263
### run03
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | FAILED | 110 | 0.38 | 2.94 | 11.15 | +3.44 | 1.60 | |
| 2 | FAILED | 31 | 0.49 | 8.93 | 11.15 | +3.44 | 1.58 | |
| 3 | FAILED | 33 | 0.49 | 11.32 | 11.15 | +3.44 | 1.58 | |
| 4 | FAILED | 36 | 0.49 | 10.17 | 11.15 | +3.44 | 1.58 | |

log counts: wall_avoid 1, avoid_release 1, blind_reseed 0, no_valid_path 26, start_occupied 0, invalid_source 0, pose_old 68, no_progress 0, cm_stop 0

## timing budget (car nodes)
| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |
|---|---|---|---|---|---|
| mask.scan_cb | ms | 24668 | 2.29 | 142 | 1.05e+03 |
| upright.scan_cb | ms | 24766 | 0.72 | 44.3 | 198 |
| odom.scan_cb | ms | 24373 | 31 | 253 | 349 |
| odom.rf2o_cb | ms | 23292 | 9.64 | 107 | 461 |
| odom.amcl_cb | ms | 34 | 0.12 | 78.4 | 78.4 |
| gate.amcl_cb | ms | 34 | 69.4 | 194 | 194 |
| gate.tf_timer | ms | 66051 | 7.2 | 65.3 | 260 |
| pd.scan_cb | ms | 24369 | 0.29 | 94.7 | 226 |
| pd.arc_check | ms | 82 | 0.82 | 663 | 663 |
| pd.tick | ms | 86 | 2.18 | 785 | 1.41e+03 |
| bridge.cmd_cb | ms | 107 | 1.66 | 26.6 | 34.1 |
| odom.scan_age_s | s | 24374 | 0.04 | 0.9 | 1.64 |
| odom.rf2o_age_s | s | 23292 | 0.11 | 2.42 | 2.56 |
| odom.rf2o_period_s | s | 23288 | 0.14 | 0.29 | 1.29 |
| gate.amcl_age_s | s | 34 | 5.32 | 11.6 | 11.6 |
| pd.pose_age_s | s | 373 | 2.99 | 6.5 | 6.54 |
| pd.scan_to_cmd_s | s | 92 | 0.12 | 0.84 | 1.2 |
| pd.cmd_period_s | s | 89 | 0.14 | 30 | 30 |

counters: odom.rf2o_gap_skipped 5, pd.tick_no_pose 289

