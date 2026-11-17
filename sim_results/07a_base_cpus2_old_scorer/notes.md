# 07a_base_cpus2_old_scorer

args: args: 
### run01
FAIL - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.67 | 0.14 | 2.62 | +2.49 | 1.31 | |
| 2 | SUCCEEDED | 20 | 0.20 | 0.35 | 2.43 | +2.44 | 1.34 | |
| 3 | TIMEOUT | 120 | 0.09 | 8.81 | 2.53 | +2.31 | 1.42 | |
| 4 | TIMEOUT | 120 | 0.09 | 16.22 | 0.32 | +0.17 | 0.12 | |

log counts: wall_avoid 79, avoid_release 1, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 1

