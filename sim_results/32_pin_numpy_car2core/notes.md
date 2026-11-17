# 32_pin_numpy_car2core

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-1 wall_s=315
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.67 | 0.07 | 0.54 | +0.54 | 0.11 | |
| 2 | SUCCEEDED | 19 | 0.15 | 0.13 | 0.30 | +0.27 | 0.13 | |
| 3 | TIMEOUT | 120 | 0.12 | 8.80 | 0.83 | +0.64 | 0.45 | |
| 4 | TIMEOUT | 120 | 0.12 | 16.20 | 0.42 | +0.22 | 0.16 | |

log counts: wall_avoid 64, avoid_release 3, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 1, no_progress 0, cm_stop 1

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-1 wall_s=133
### run02
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 26 | 0.66 | 0.62 | 0.76 | +0.78 | 0.66 | |
| 2 | SUCCEEDED | 19 | 0.14 | 0.20 | 0.64 | +0.61 | 0.71 | |
| 3 | SUCCEEDED | 27 | 0.29 | 0.18 | 0.87 | +0.68 | 0.73 | |
| 4 | SUCCEEDED | 27 | 0.14 | 0.13 | 1.00 | +0.95 | 0.60 | |

log counts: wall_avoid 8, avoid_release 4, blind_reseed 2, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 1, no_progress 0, cm_stop 2

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-1 wall_s=133
### run03
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 26 | 0.63 | 1.10 | 1.24 | +1.24 | 0.44 | |
| 2 | FAILED COLLISION | 40 | 0.00 | 10.60 | 1.38 | +1.35 | 0.34 | |
| 3 | FAILED COLLISION | 13 | 0.00 | 20.68 | 0.51 | +0.49 | 0.22 | |
| 4 | FAILED COLLISION | 13 | 0.00 | 23.64 | 0.51 | +0.49 | 0.22 | |

log counts: wall_avoid 15, avoid_release 3, blind_reseed 2, no_valid_path 30, start_occupied 1, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

## timing budget (car nodes)
| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |
|---|---|---|---|---|---|
| mask.scan_cb | ms | 3679 | 3.76 | 61.2 | 664 |
| upright.scan_cb | ms | 3668 | 1.72 | 23.1 | 45.5 |
| odom.scan_cb | ms | 3642 | 14 | 97.1 | 385 |
| odom.rf2o_cb | ms | 3471 | 5.31 | 42.6 | 200 |
| odom.amcl_cb | ms | 929 | 0.135 | 26.6 | 34 |
| gate.amcl_cb | ms | 928 | 5.7 | 36.3 | 154 |
| gate.tf_timer | ms | 9873 | 3.21 | 24.1 | 99.8 |
| pd.scan_cb | ms | 3594 | 0.3 | 21.5 | 113 |
| pd.arc_check | ms | 3421 | 10.3 | 125 | 220 |
| pd.tick | ms | 3426 | 13.5 | 137 | 643 |
| bridge.cmd_cb | ms | 1541 | 0.94 | 17.7 | 70.8 |
| odom.scan_age_s | s | 3643 | 0.03 | 0.46 | 0.71 |
| odom.rf2o_age_s | s | 3471 | 0.09 | 0.27 | 1.37 |
| odom.rf2o_period_s | s | 3468 | 0.14 | 0.29 | 1.43 |
| gate.amcl_age_s | s | 928 | 0.815 | 3.56 | 3.56 |
| pd.pose_age_s | s | 3454 | 0.17 | 0.42 | 1.65 |
| pd.scan_to_cmd_s | s | 3450 | 0.1 | 0.23 | 0.62 |
| pd.cmd_period_s | s | 3439 | 0.12 | 0.25 | 3.5 |

counters: odom.rf2o_gap_skipped 5, pd.tick_no_pose 4

