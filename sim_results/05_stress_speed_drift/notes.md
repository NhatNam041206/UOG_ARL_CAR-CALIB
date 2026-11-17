# 05_stress_speed_drift

args: args: 
### run01
PASS - odom adaptive, sim truth {'speed_mps': 0.42, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.15, 'speed_period_s': 30.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 30 | 0.66 | 0.38 | 1.53 | +1.50 | 0.46 | |
| 2 | SUCCEEDED | 23 | 0.16 | 0.05 | 1.37 | +1.36 | 0.54 | |
| 3 | SUCCEEDED | 30 | 0.20 | 0.56 | 1.74 | +1.72 | 1.18 | |
| 4 | SUCCEEDED | 30 | 0.32 | 0.12 | 1.52 | +1.50 | 0.95 | |

log counts: wall_avoid 3, avoid_release 2, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

args: args: 
### run02
PASS - odom adaptive, sim truth {'speed_mps': 0.42, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.15, 'speed_period_s': 30.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 30 | 0.67 | 0.53 | 1.82 | +1.79 | 0.70 | |
| 2 | SUCCEEDED | 22 | 0.29 | 0.10 | 1.86 | +1.80 | 0.73 | |
| 3 | SUCCEEDED | 30 | 0.30 | 0.84 | 1.85 | +1.69 | 1.46 | |
| 4 | SUCCEEDED | 31 | 0.32 | 0.13 | 1.55 | +1.43 | 1.28 | |

log counts: wall_avoid 7, avoid_release 4, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args:  SIM_CPUS= SIM_STRESS=speed_drift wall_s=154
### run03
PASS - odom adaptive, sim truth {'speed_mps': 0.42, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.15, 'speed_period_s': 30.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 29 | 0.67 | 0.09 | 0.45 | +0.31 | 0.23 | |
| 2 | SUCCEEDED | 22 | 0.14 | 0.12 | 0.26 | +0.24 | 0.29 | |
| 3 | SUCCEEDED | 30 | 0.23 | 1.35 | 1.31 | -1.26 | 1.50 | |
| 4 | SUCCEEDED | 32 | 0.30 | 0.07 | 1.27 | -1.25 | 1.46 | |

log counts: wall_avoid 5, avoid_release 3, blind_reseed 2, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

