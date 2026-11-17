# 12_numpy_stress_speed_drift

args: args:  SIM_CPUS= SIM_STRESS=speed_drift wall_s=142
### run01
PASS - odom adaptive, sim truth {'speed_mps': 0.42, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.15, 'speed_period_s': 30.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 31 | 0.67 | 0.75 | 0.89 | +0.93 | 0.80 | |
| 2 | SUCCEEDED | 23 | 0.20 | 0.50 | 0.83 | +0.80 | 0.92 | |
| 3 | SUCCEEDED | 32 | 0.30 | 0.44 | 0.75 | +0.74 | 1.32 | |
| 4 | SUCCEEDED | 28 | 0.36 | 0.16 | 0.77 | +0.77 | 0.55 | |

log counts: wall_avoid 5, avoid_release 4, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args:  SIM_CPUS= SIM_STRESS=speed_drift wall_s=263
### run02
FAIL - odom adaptive, sim truth {'speed_mps': 0.42, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.15, 'speed_period_s': 30.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 31 | 0.67 | 0.70 | 0.79 | +0.76 | 0.62 | |
| 2 | SUCCEEDED | 61 | 0.10 | 0.29 | 0.90 | +0.91 | 1.00 | |
| 3 | SUCCEEDED | 36 | 0.24 | 2.06 | 2.28 | +2.25 | 1.27 | |
| 4 | TIMEOUT | 120 | 0.06 | 6.26 | 2.51 | +2.47 | 1.30 | |

log counts: wall_avoid 27, avoid_release 11, blind_reseed 0, no_valid_path 1, start_occupied 0, invalid_source 0, pose_old 3, no_progress 0, cm_stop 2

args: args:  SIM_CPUS= SIM_STRESS=speed_drift wall_s=183
### run03
PASS - odom adaptive, sim truth {'speed_mps': 0.42, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.15, 'speed_period_s': 30.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 31 | 0.66 | 1.03 | 1.19 | +1.19 | 1.03 | |
| 2 | SUCCEEDED | 62 | 0.10 | 0.13 | 1.34 | +1.32 | 1.85 | |
| 3 | SUCCEEDED | 35 | 0.20 | 1.17 | 1.46 | +1.46 | 1.99 | |
| 4 | SUCCEEDED | 27 | 0.18 | 0.17 | 1.68 | +1.66 | 1.62 | |

log counts: wall_avoid 15, avoid_release 11, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 1, no_progress 0, cm_stop 2

