# 26_pin_orig_car6core

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-5 wall_s=112
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.66 | 0.05 | 0.16 | +0.13 | 0.12 | |
| 2 | SUCCEEDED | 20 | 0.13 | 0.23 | 0.70 | +0.68 | 0.63 | |
| 3 | FAILED | 25 | 0.04 | 7.50 | 0.55 | +0.51 | 0.44 | |
| 4 | FAILED | 13 | 0.04 | 14.72 | 0.08 | +0.08 | 0.44 | |

log counts: wall_avoid 15, avoid_release 3, blind_reseed 0, no_valid_path 18, start_occupied 1, invalid_source 0, pose_old 0, no_progress 0, cm_stop 2

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-5 wall_s=122
### run02
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.66 | 0.05 | 0.20 | +0.20 | 0.13 | |
| 2 | SUCCEEDED | 20 | 0.07 | 0.16 | 0.85 | +0.81 | 0.79 | |
| 3 | SUCCEEDED | 28 | 0.29 | 0.71 | 0.92 | +0.91 | 1.23 | |
| 4 | SUCCEEDED | 24 | 0.29 | 0.12 | 1.00 | +0.95 | 1.55 | |

log counts: wall_avoid 12, avoid_release 4, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 1, no_progress 0, cm_stop 1

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-5 wall_s=1512 HUNG: stopped by the watchdog
### run03
NO REPORT (see log)

log counts: wall_avoid 0, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

## timing budget (car nodes)
| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |
|---|---|---|---|---|---|
| mask.scan_cb | ms | 23206 | 6.38 | 48.3 | 210 |
| upright.scan_cb | ms | 23199 | 0.58 | 29.8 | 78.6 |
| odom.scan_cb | ms | 23109 | 20.3 | 104 | 480 |
| odom.rf2o_cb | ms | 22130 | 3.97 | 57.4 | 387 |
| odom.amcl_cb | ms | 1002 | 0.135 | 0.56 | 31.5 |
| gate.amcl_cb | ms | 1002 | 1.17 | 15.8 | 106 |
| gate.tf_timer | ms | 62797 | 1.51 | 31.6 | 104 |
| pd.scan_cb | ms | 23028 | 0.48 | 26 | 116 |
| pd.arc_check | ms | 1358 | 1.12 | 57.2 | 96.8 |
| pd.tick | ms | 1360 | 1.85 | 63.3 | 251 |
| bridge.cmd_cb | ms | 1362 | 0.31 | 1.04 | 12.4 |
| odom.scan_age_s | s | 23113 | 0.03 | 0.69 | 0.99 |
| odom.rf2o_age_s | s | 22130 | 0.09 | 0.41 | 0.95 |
| odom.rf2o_period_s | s | 22126 | 0.14 | 0.29 | 1.28 |
| gate.amcl_age_s | s | 1002 | 0.185 | 1.21 | 1.5 |
| pd.pose_age_s | s | 1372 | 0.13 | 0.24 | 1.51 |
| pd.scan_to_cmd_s | s | 1367 | 0.08 | 0.14 | 0.19 |
| pd.cmd_period_s | s | 1362 | 0.1 | 0.16 | 3.49 |

counters: odom.rf2o_gap_skipped 12, pd.tick_no_pose 1

