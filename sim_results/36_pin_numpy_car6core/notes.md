# 36_pin_numpy_car6core

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-5 wall_s=133
### run01
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 26 | 0.66 | 0.53 | 0.72 | +0.70 | 0.77 | |
| 2 | SUCCEEDED | 19 | 0.25 | 0.30 | 0.67 | +0.64 | 0.77 | |
| 3 | SUCCEEDED | 28 | 0.20 | 0.20 | 0.64 | +0.61 | 0.60 | |
| 4 | SUCCEEDED | 25 | 0.36 | 0.02 | 1.06 | +1.00 | 0.39 | |

log counts: wall_avoid 5, avoid_release 3, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-5 wall_s=123
### run02
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.65 | 0.03 | 0.43 | +0.27 | 0.11 | |
| 2 | SUCCEEDED | 19 | 0.26 | 0.30 | 0.71 | +0.71 | 0.35 | |
| 3 | SUCCEEDED | 27 | 0.29 | 0.37 | 0.51 | +0.45 | 0.43 | |
| 4 | SUCCEEDED | 24 | 0.36 | 0.12 | 0.47 | +0.44 | 0.50 | |

log counts: wall_avoid 5, avoid_release 3, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-5 wall_s=122
### run03
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.67 | 0.04 | 0.42 | +0.30 | 0.18 | |
| 2 | SUCCEEDED | 20 | 0.07 | 0.31 | 0.73 | +0.52 | 1.00 | |
| 3 | SUCCEEDED | 27 | 0.30 | 0.14 | 0.60 | +0.60 | 2.46 | |
| 4 | SUCCEEDED | 24 | 0.34 | 0.02 | 0.59 | +0.54 | 3.29 | |

log counts: wall_avoid 11, avoid_release 9, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

## timing budget (car nodes)
| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |
|---|---|---|---|---|---|
| mask.scan_cb | ms | 2159 | 2.29 | 13.2 | 285 |
| upright.scan_cb | ms | 2165 | 0.66 | 10.4 | 25.1 |
| odom.scan_cb | ms | 2153 | 4.52 | 25.4 | 397 |
| odom.rf2o_cb | ms | 2079 | 1.07 | 15.7 | 132 |
| odom.amcl_cb | ms | 1782 | 0.16 | 1.45 | 51 |
| gate.amcl_cb | ms | 1779 | 1.36 | 29.5 | 168 |
| gate.tf_timer | ms | 5889 | 0.65 | 8.24 | 31.4 |
| pd.scan_cb | ms | 2205 | 1.64 | 40.1 | 74 |
| pd.arc_check | ms | 2527 | 1.53 | 94.7 | 147 |
| pd.tick | ms | 2530 | 2.63 | 95.4 | 507 |
| bridge.cmd_cb | ms | 2394 | 0.33 | 2.25 | 11.2 |
| odom.scan_age_s | s | 2153 | 0.01 | 0.21 | 0.8 |
| odom.rf2o_age_s | s | 2079 | 0.06 | 0.13 | 1.09 |
| odom.rf2o_period_s | s | 2076 | 0.14 | 0.15 | 1.29 |
| gate.amcl_age_s | s | 1780 | 0.22 | 1.02 | 1.56 |
| pd.pose_age_s | s | 2552 | 0.13 | 0.49 | 1.44 |
| pd.scan_to_cmd_s | s | 2538 | 0.09 | 0.17 | 0.59 |
| pd.cmd_period_s | s | 2536 | 0.11 | 0.22 | 2.28 |

counters: odom.rf2o_gap_skipped 2

