# 07_base_cpus2

args: args: 
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.66 | 0.53 | 0.36 | +0.36 | 0.28 | |
| 2 | FAILED COLLISION | 23 | 0.00 | 5.55 | 0.34 | +0.34 | 0.34 | |
| 3 | FAILED COLLISION | 13 | 0.00 | 13.95 | 0.11 | +0.11 | 0.17 | |
| 4 | FAILED COLLISION | 13 | 0.00 | 16.12 | 0.11 | +0.11 | 0.17 | |

log counts: wall_avoid 5, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 29, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

args: args: 
### run02
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.67 | 0.15 | 0.28 | +0.23 | 0.10 | |
| 2 | SUCCEEDED | 23 | 0.06 | 0.15 | 0.47 | +0.42 | 0.22 | |
| 3 | SUCCEEDED | 28 | 0.21 | 0.09 | 0.51 | +0.43 | 0.31 | |
| 4 | TIMEOUT | 120 | 0.10 | 5.87 | 1.07 | +1.03 | 0.27 | |

log counts: wall_avoid 285, avoid_release 3, blind_reseed 0, no_valid_path 0, start_occupied 2, invalid_source 0, pose_old 0, no_progress 0, cm_stop 2

args: args: 
### run03
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 26 | 0.65 | 0.48 | 0.45 | +0.44 | 0.19 | |
| 2 | TIMEOUT COLLISION | 120 | 0.00 | 5.91 | 0.84 | +0.79 | 0.15 | |
| 3 | TIMEOUT COLLISION | 120 | 0.00 | 14.40 | 0.84 | +0.79 | 0.13 | |
| 4 | TIMEOUT COLLISION | 120 | 0.00 | 16.50 | 0.84 | +0.79 | 0.13 | |

log counts: wall_avoid 19, avoid_release 0, blind_reseed 1, no_valid_path 0, start_occupied 4, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

