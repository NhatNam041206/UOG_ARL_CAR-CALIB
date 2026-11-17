# 01_baseline

args: args: 
### run01
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.66 | 0.02 | 1.67 | +1.56 | 0.20 | |
| 2 | SUCCEEDED | 19 | 0.25 | 0.07 | 1.87 | +1.86 | 0.77 | |
| 3 | SUCCEEDED | 28 | 0.29 | 0.21 | 2.58 | +2.51 | 0.51 | |
| 4 | SUCCEEDED | 24 | 0.37 | 0.03 | 2.29 | +1.91 | 1.47 | |

log counts: wall_avoid 9, avoid_release 4, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args: 
### run02
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.66 | 0.10 | 1.54 | +1.51 | 0.14 | |
| 2 | SUCCEEDED | 19 | 0.25 | 0.18 | 1.19 | +1.18 | 0.12 | |
| 3 | SUCCEEDED | 27 | 0.25 | 0.10 | 0.56 | +0.53 | 0.20 | |
| 4 | SUCCEEDED | 24 | 0.37 | 0.09 | 0.23 | +0.22 | 0.26 | |

log counts: wall_avoid 6, avoid_release 2, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args: 
### run03
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.67 | 0.08 | 0.98 | +0.97 | 0.10 | |
| 2 | SUCCEEDED | 19 | 0.23 | 0.13 | 0.20 | +0.18 | 0.14 | |
| 3 | SUCCEEDED | 27 | 0.25 | 0.20 | 0.35 | +0.33 | 0.23 | |
| 4 | SUCCEEDED | 24 | 0.37 | 0.05 | 0.46 | +0.41 | 0.33 | |

log counts: wall_avoid 2, avoid_release 2, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

