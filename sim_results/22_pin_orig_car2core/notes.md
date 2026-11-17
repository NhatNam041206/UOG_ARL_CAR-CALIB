# 22_pin_orig_car2core

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-1 wall_s=122
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.66 | 0.10 | 0.69 | +0.70 | 0.31 | |
| 2 | SUCCEEDED | 22 | 0.16 | 1.65 | 1.52 | +1.52 | 0.32 | |
| 3 | FAILED | 23 | 0.05 | 9.29 | 2.46 | +2.44 | 0.24 | |
| 4 | FAILED | 13 | 0.05 | 16.67 | 0.19 | +0.19 | 0.16 | |

log counts: wall_avoid 19, avoid_release 2, blind_reseed 0, no_valid_path 20, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 2

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-1 wall_s=112
### run02
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.65 | 0.26 | 0.58 | +0.58 | 0.31 | |
| 2 | FAILED COLLISION | 21 | 0.00 | 5.72 | 0.56 | +0.55 | 0.19 | |
| 3 | FAILED COLLISION | 13 | 0.00 | 14.06 | 0.13 | +0.08 | 0.12 | |
| 4 | FAILED COLLISION | 13 | 0.00 | 16.15 | 0.13 | +0.08 | 0.12 | |

log counts: wall_avoid 13, avoid_release 1, blind_reseed 0, no_valid_path 0, start_occupied 30, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-1 wall_s=132
### run03
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 31 | 0.42 | 1.91 | 2.31 | +2.07 | 0.46 | |
| 2 | FAILED | 36 | 0.10 | 11.93 | 2.04 | +2.04 | 0.80 | |
| 3 | FAILED | 12 | 0.10 | 22.21 | 0.49 | +0.18 | 0.61 | |
| 4 | FAILED | 13 | 0.10 | 25.68 | 0.49 | +0.18 | 0.61 | |

log counts: wall_avoid 8, avoid_release 2, blind_reseed 3, no_valid_path 28, start_occupied 1, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

## timing budget (car nodes)
| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |
|---|---|---|---|---|---|
| mask.scan_cb | ms | 2045 | 5 | 44.9 | 480 |
| upright.scan_cb | ms | 1983 | 1.73 | 22 | 54.9 |
| odom.scan_cb | ms | 2024 | 24.3 | 112 | 267 |
| odom.rf2o_cb | ms | 1925 | 5.28 | 32.9 | 149 |
| odom.amcl_cb | ms | 568 | 0.11 | 11.3 | 57.7 |
| gate.amcl_cb | ms | 568 | 6.54 | 39.2 | 112 |
| gate.tf_timer | ms | 5503 | 3.75 | 23.3 | 90.1 |
| pd.scan_cb | ms | 2010 | 0.43 | 33.4 | 91 |
| pd.arc_check | ms | 1133 | 1.31 | 130 | 255 |
| pd.tick | ms | 1135 | 4.81 | 307 | 333 |
| bridge.cmd_cb | ms | 1012 | 0.87 | 10.6 | 27.5 |
| odom.scan_age_s | s | 2024 | 0.03 | 0.57 | 0.84 |
| odom.rf2o_age_s | s | 1927 | 0.09 | 0.28 | 1.32 |
| odom.rf2o_period_s | s | 1922 | 0.14 | 0.28 | 1.28 |
| gate.amcl_age_s | s | 568 | 0.96 | 3.61 | 3.68 |
| pd.pose_age_s | s | 1146 | 0.18 | 0.41 | 1.18 |
| pd.scan_to_cmd_s | s | 1142 | 0.1 | 0.21 | 0.37 |
| pd.cmd_period_s | s | 1138 | 0.11 | 0.26 | 2.33 |

counters: odom.rf2o_gap_skipped 1

