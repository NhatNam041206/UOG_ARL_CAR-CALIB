# 02_baseline_tf_fix

args: args: 
### run01
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.66 | 0.06 | 1.31 | +1.30 | 0.16 | |
| 2 | SUCCEEDED | 19 | 0.23 | 0.12 | 0.21 | +0.21 | 0.22 | |
| 3 | SUCCEEDED | 27 | 0.29 | 0.17 | 0.26 | +0.25 | 0.30 | |
| 4 | SUCCEEDED | 24 | 0.34 | 0.04 | 0.40 | +0.36 | 0.53 | |

log counts: wall_avoid 5, avoid_release 3, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args: 
### run02
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.66 | 0.08 | 1.58 | +1.48 | 0.12 | |
| 2 | SUCCEEDED | 19 | 0.22 | 0.14 | 2.03 | +1.92 | 0.83 | |
| 3 | SUCCEEDED | 27 | 0.30 | 0.15 | 1.92 | +1.86 | 0.26 | |
| 4 | SUCCEEDED | 24 | 0.37 | 0.07 | 1.83 | +1.79 | 0.22 | |

log counts: wall_avoid 4, avoid_release 2, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args: 
### run03
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.67 | 0.23 | 1.91 | +1.79 | 0.92 | |
| 2 | SUCCEEDED | 19 | 0.18 | 0.07 | 2.07 | +2.06 | 1.39 | |
| 3 | SUCCEEDED | 28 | 0.24 | 0.23 | 2.78 | +2.71 | 1.06 | |
| 4 | SUCCEEDED | 24 | 0.35 | 0.06 | 2.24 | +2.16 | 0.72 | |

log counts: wall_avoid 5, avoid_release 3, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args: 
### run04
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.67 | 0.09 | 1.52 | +1.50 | 0.10 | |
| 2 | SUCCEEDED | 19 | 0.27 | 0.13 | 1.91 | +1.83 | 0.33 | |
| 3 | SUCCEEDED | 27 | 0.28 | 0.28 | 2.23 | +2.23 | 0.97 | |
| 4 | SUCCEEDED | 24 | 0.33 | 0.07 | 2.31 | +2.34 | 0.57 | |

log counts: wall_avoid 5, avoid_release 2, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args: 
### run05
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.66 | 0.01 | 1.89 | +1.67 | 1.33 | |
| 2 | SUCCEEDED | 19 | 0.21 | 0.15 | 1.45 | +1.43 | 0.12 | |
| 3 | SUCCEEDED | 27 | 0.26 | 0.05 | 1.82 | +1.80 | 0.19 | |
| 4 | SUCCEEDED | 24 | 0.36 | 0.06 | 1.69 | +1.72 | 0.11 | |

log counts: wall_avoid 7, avoid_release 4, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

