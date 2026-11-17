# 06_base_profile_fullcpu

args: args: 
### run01
PASS - odom adaptive, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 25 | 0.66 | 0.10 | 1.77 | +1.64 | 0.59 | |
| 2 | SUCCEEDED | 19 | 0.28 | 0.19 | 1.88 | +1.86 | 0.17 | |
| 3 | SUCCEEDED | 27 | 0.29 | 0.19 | 1.98 | +1.91 | 0.23 | |
| 4 | SUCCEEDED | 24 | 0.34 | 0.11 | 2.08 | +1.97 | 0.24 | |

log counts: wall_avoid 3, avoid_release 3, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

