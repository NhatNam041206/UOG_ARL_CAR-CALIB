# 20_pintest

args: args:  SIM_CPUS= SIM_STRESS= CAR_CORES=0-1 wall_s=254
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.67 | 0.13 | 0.29 | +0.28 | 0.16 | |
| 2 | SUCCEEDED | 55 | 0.07 | 0.11 | 0.94 | +0.87 | 3.14 | |
| 3 | SUCCEEDED | 27 | 0.25 | 0.18 | 0.62 | +0.49 | 6.55 | |
| 4 | TIMEOUT | 120 | 0.13 | 5.91 | 1.23 | +1.19 | 8.53 | |

log counts: wall_avoid 234, avoid_release 11, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 3

## timing budget (car nodes)
| name | unit | samples | p50 (median of windows) | p95 (worst window) | max |
|---|---|---|---|---|---|
| mask.scan_cb | ms | 1617 | 2.2 | 63.6 | 203 |
| upright.scan_cb | ms | 1682 | 1.25 | 12.1 | 62.2 |
| odom.scan_cb | ms | 1675 | 9.29 | 54.6 | 348 |
| odom.rf2o_cb | ms | 1604 | 3.05 | 25.4 | 147 |
| odom.amcl_cb | ms | 661 | 0.12 | 17.9 | 25.4 |
| gate.amcl_cb | ms | 661 | 3.48 | 23.4 | 42 |
| gate.tf_timer | ms | 4599 | 1.96 | 17.1 | 53.8 |
| pd.scan_cb | ms | 1660 | 0.27 | 13.1 | 44.3 |
| pd.arc_check | ms | 1902 | 0.645 | 71.7 | 151 |
| pd.tick | ms | 1904 | 3.92 | 76.7 | 298 |
| bridge.cmd_cb | ms | 1062 | 0.45 | 10.4 | 43.4 |
| odom.scan_age_s | s | 1675 | 0.02 | 0.56 | 0.82 |
| odom.rf2o_age_s | s | 1605 | 0.07 | 0.3 | 0.7 |
| odom.rf2o_period_s | s | 1603 | 0.14 | 0.43 | 1.15 |
| gate.amcl_age_s | s | 661 | 0.25 | 2.1 | 2.17 |
| pd.pose_age_s | s | 1913 | 0.155 | 0.32 | 1.47 |
| pd.scan_to_cmd_s | s | 1914 | 0.1 | 0.23 | 0.54 |
| pd.cmd_period_s | s | 1912 | 0.11 | 0.2 | 2.67 |

counters: odom.rf2o_gap_skipped 3

