# 08_base_cpus1

args: args: 
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | FAILED COLLISION | 35 | 0.00 | 0.98 | 4.84 | +3.59 | 3.32 | |
| 2 | FAILED | 16 | 0.52 | 7.88 | 4.84 | +3.11 | 3.32 | |
| 3 | FAILED | 16 | 0.52 | 12.19 | 4.84 | +3.11 | 3.32 | |
| 4 | FAILED | 15 | 0.52 | 12.02 | 4.84 | +3.11 | 3.32 | |

log counts: wall_avoid 10, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 36, invalid_source 0, pose_old 0, no_progress 0, cm_stop 4

args: args: 
### run02
NO REPORT (see log)

log counts: wall_avoid 0, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

args: args: 
### run03
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 29 | 0.60 | 1.51 | 1.42 | +1.41 | 0.51 | |
| 2 | FAILED COLLISION | 23 | 0.00 | 7.00 | 1.44 | +1.42 | 0.51 | |
| 3 | FAILED COLLISION | 12 | 0.00 | 15.68 | 0.70 | +0.68 | 0.44 | |
| 4 | FAILED COLLISION | 13 | 0.00 | 17.57 | 0.70 | +0.68 | 0.44 | |

log counts: wall_avoid 6, avoid_release 0, blind_reseed 3, no_valid_path 0, start_occupied 30, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

