# 31_pin_numpy_car1core

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0 wall_s=518
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | TIMEOUT | 120 | 0.04 | 1.78 | 2.05 | +1.97 | 0.72 | |
| 2 | TIMEOUT | 120 | 0.04 | 8.81 | 0.73 | +0.67 | 0.13 | |
| 3 | TIMEOUT | 120 | 0.04 | 12.57 | 0.73 | +0.67 | 0.13 | |
| 4 | TIMEOUT | 120 | 0.04 | 11.72 | 0.73 | +0.67 | 0.13 | |

log counts: wall_avoid 22, avoid_release 0, blind_reseed 4, no_valid_path 0, start_occupied 0, invalid_source 2, pose_old 187, no_progress 0, cm_stop 2

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0 wall_s=446
### run02
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 49 | 0.01 | 2.60 | 3.18 | +3.17 | 0.63 | |
| 2 | TIMEOUT | 120 | 0.01 | 6.04 | 3.25 | +3.21 | 0.02 | |
| 3 | TIMEOUT | 120 | 0.01 | 13.52 | 2.42 | +2.28 | 0.02 | |
| 4 | TIMEOUT | 120 | 0.01 | 15.12 | 2.42 | +2.28 | 0.02 | |

log counts: wall_avoid 18, avoid_release 1, blind_reseed 4, no_valid_path 1, start_occupied 0, invalid_source 2, pose_old 43, no_progress 0, cm_stop 2

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0 wall_s=243
### run03
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | FAILED | 105 | 0.05 | 4.67 | 5.28 | +5.15 | 0.47 | |
| 2 | FAILED COLLISION | 62 | 0.00 | 8.56 | 5.69 | +5.43 | 0.46 | |
| 3 | FAILED COLLISION | 14 | 0.00 | 18.68 | 5.39 | +3.69 | 0.38 | |
| 4 | FAILED COLLISION | 16 | 0.00 | 21.94 | 5.39 | +3.69 | 0.38 | |

log counts: wall_avoid 15, avoid_release 3, blind_reseed 7, no_valid_path 4, start_occupied 25, invalid_source 2, pose_old 84, no_progress 0, cm_stop 2

## timing budget (car nodes)
| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |
|---|---|---|---|---|---|
| mask.scan_cb | ms | 8177 | 3.1 | 117 | 1.02e+03 |
| upright.scan_cb | ms | 8177 | 0.47 | 64.3 | 194 |
| odom.scan_cb | ms | 8052 | 42.2 | 214 | 417 |
| odom.rf2o_cb | ms | 7607 | 13.5 | 103 | 260 |
| odom.amcl_cb | ms | 151 | 0.125 | 55.4 | 75 |
| gate.amcl_cb | ms | 151 | 37.3 | 431 | 431 |
| gate.tf_timer | ms | 20888 | 9.54 | 72.8 | 249 |
| pd.scan_cb | ms | 7588 | 0.23 | 130 | 327 |
| pd.arc_check | ms | 3276 | 35.9 | 399 | 1.28e+03 |
| pd.tick | ms | 3291 | 68.3 | 1.14e+03 | 2.03e+03 |
| bridge.cmd_cb | ms | 651 | 0.825 | 42.8 | 52.3 |
| odom.scan_age_s | s | 8054 | 0.05 | 0.84 | 1.69 |
| odom.rf2o_age_s | s | 7608 | 0.12 | 1.76 | 2.12 |
| odom.rf2o_period_s | s | 7604 | 0.14 | 0.29 | 1 |
| gate.amcl_age_s | s | 151 | 2.83 | 18.3 | 18.3 |
| pd.pose_age_s | s | 4592 | 0.66 | 7.61 | 7.84 |
| pd.scan_to_cmd_s | s | 3366 | 0.13 | 0.84 | 1.38 |
| pd.cmd_period_s | s | 3334 | 0.18 | 23.1 | 30 |

counters: pd.tick_no_pose 1271

