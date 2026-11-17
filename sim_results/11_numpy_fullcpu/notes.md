# 11_numpy_fullcpu

args: args:  SIM_CPUS= SIM_STRESS= wall_s=122
### run01
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 26 | 0.67 | 0.59 | 0.70 | +0.53 | 0.23 | |
| 2 | SUCCEEDED | 18 | 0.22 | 0.16 | 0.33 | +0.27 | 0.23 | |
| 3 | SUCCEEDED | 28 | 0.19 | 0.11 | 0.49 | +0.38 | 0.29 | |
| 4 | SUCCEEDED | 24 | 0.37 | 0.05 | 0.46 | +0.38 | 0.13 | |

log counts: wall_avoid 5, avoid_release 2, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args:  SIM_CPUS= SIM_STRESS= wall_s=145
### run02
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.67 | 0.02 | 0.59 | +0.60 | 0.12 | |
| 2 | SUCCEEDED | 19 | 0.14 | 0.12 | 0.32 | +0.29 | 0.09 | |
| 3 | SUCCEEDED | 28 | 0.23 | 0.14 | 0.33 | +0.31 | 0.22 | |
| 4 | SUCCEEDED | 25 | 0.36 | 0.03 | 0.33 | +0.32 | 0.22 | |

log counts: wall_avoid 8, avoid_release 4, blind_reseed 1, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args:  SIM_CPUS= SIM_STRESS= wall_s=134
### run03
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 26 | 0.66 | 0.62 | 0.80 | +0.81 | 0.67 | |
| 2 | SUCCEEDED | 18 | 0.24 | 0.18 | 0.64 | +0.61 | 0.64 | |
| 3 | SUCCEEDED | 28 | 0.20 | 0.23 | 0.42 | +0.41 | 0.53 | |
| 4 | SUCCEEDED | 24 | 0.36 | 0.32 | 0.39 | +0.38 | 0.34 | |

log counts: wall_avoid 8, avoid_release 5, blind_reseed 3, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

