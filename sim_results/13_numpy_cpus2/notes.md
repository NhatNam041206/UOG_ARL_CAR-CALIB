# 13_numpy_cpus2

args: args:  SIM_CPUS=2 SIM_STRESS= wall_s=451
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.49 | 0.08 | 0.27 | +0.24 | 0.15 | |
| 2 | TIMEOUT | 120 | 0.01 | 5.82 | 0.51 | +0.47 | 0.17 | |
| 3 | TIMEOUT | 120 | 0.01 | 14.13 | 0.11 | +0.11 | 0.04 | |
| 4 | TIMEOUT | 121 | 0.01 | 16.17 | 0.11 | +0.11 | 0.04 | |

log counts: wall_avoid 14, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 6, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

args: args:  SIM_CPUS=2 SIM_STRESS= wall_s=234
### run02
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.64 | 0.04 | 0.35 | +0.28 | 0.14 | |
| 2 | SUCCEEDED | 21 | 0.08 | 0.14 | 0.37 | +0.36 | 0.34 | |
| 3 | FAILED | 30 | 0.07 | 9.22 | 0.55 | +0.54 | 0.48 | |
| 4 | TIMEOUT | 120 | 0.07 | 16.60 | 0.53 | +0.51 | 0.19 | |

log counts: wall_avoid 20, avoid_release 3, blind_reseed 0, no_valid_path 2, start_occupied 0, invalid_source 0, pose_old 1, no_progress 0, cm_stop 2

args: args:  SIM_CPUS=2 SIM_STRESS= wall_s=1511 HUNG: stopped by the watchdog
### run03
NO REPORT (see log)

log counts: wall_avoid 0, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

