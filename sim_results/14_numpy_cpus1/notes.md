# 14_numpy_cpus1

args: args:  SIM_CPUS=1 SIM_STRESS= wall_s=145
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 32 | 0.61 | 0.47 | 0.42 | +0.42 | 0.34 | |
| 2 | FAILED | 26 | 0.06 | 4.62 | 0.42 | +0.38 | 0.35 | |
| 3 | FAILED | 13 | 0.06 | 12.77 | 0.20 | +0.20 | 0.21 | |
| 4 | FAILED | 13 | 0.06 | 15.22 | 0.20 | +0.20 | 0.21 | |

log counts: wall_avoid 6, avoid_release 1, blind_reseed 0, no_valid_path 30, start_occupied 0, invalid_source 0, pose_old 5, no_progress 0, cm_stop 2

args: args:  SIM_CPUS=1 SIM_STRESS= wall_s=253
### run02
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | FAILED COLLISION | 62 | 0.00 | 0.92 | 6.37 | +6.35 | 1.02 | |
| 2 | FAILED COLLISION | 45 | 0.00 | 8.30 | 6.17 | +6.11 | 1.02 | |
| 3 | FAILED COLLISION | 44 | 0.00 | 13.19 | 6.17 | +6.11 | 1.02 | |
| 4 | FAILED COLLISION | 42 | 0.00 | 12.90 | 6.17 | +6.11 | 1.02 | |

log counts: wall_avoid 8, avoid_release 0, blind_reseed 2, no_valid_path 36, start_occupied 0, invalid_source 0, pose_old 8, no_progress 0, cm_stop 1

args: args:  SIM_CPUS=1 SIM_STRESS= wall_s=142
### run03
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 26 | 0.67 | 0.58 | 0.63 | +0.60 | 0.23 | |
| 2 | FAILED COLLISION | 41 | 0.00 | 9.60 | 1.36 | +1.29 | 0.88 | |
| 3 | FAILED COLLISION | 13 | 0.00 | 19.49 | 0.04 | +0.04 | 0.80 | |
| 4 | FAILED COLLISION | 13 | 0.00 | 22.19 | 0.04 | +0.04 | 0.80 | |

log counts: wall_avoid 15, avoid_release 4, blind_reseed 1, no_valid_path 1, start_occupied 29, invalid_source 0, pose_old 0, no_progress 0, cm_stop 3

