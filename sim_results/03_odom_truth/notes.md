# 03_odom_truth

args: args: odom_source:=truth
### run01
FAIL - odom truth, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | FAILED | 31 | 0.67 | 11.10 | 0.00 | +0.00 | 0.00 | |
| 2 | FAILED | 1 | 0.67 | 15.84 | 0.00 | +0.00 | 0.00 | |
| 3 | FAILED | 0 | 0.67 | 12.22 | 0.00 | +0.00 | 0.00 | |
| 4 | FAILED | 1 | 0.67 | 4.88 | 0.00 | +0.00 | 0.00 | |

log counts: wall_avoid 0, avoid_release 0, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 5, no_progress 0, cm_stop 0

args: args: odom_source:=truth (seed 1, manual rerun)
### run02
PASS - odom truth, sim truth {'speed_mps': 0.5, 'latency_s': 0.65, 'accel_mps2': 1.0, 'decel_mps2': 2.0, 'servo_rate_dps': 300.0, 'understeer_factor': 1.0, 'speed_variation': 0.0, 'speed_period_s': 40.0, 'failsafe_timeout_s': None}

| goal | result | t s | min clear m | end err m | max loc err m | trace along-lag m (+ = truth ahead) | odom drift m | nav2 |
|---|---|---|---|---|---|---|---|---|
| 1 | SUCCEEDED | 24 | 0.63 | 0.04 | 0.53 | +0.45 | 0.04 | |
| 2 | SUCCEEDED | 19 | 0.23 | 0.09 | 0.50 | +0.49 | 0.04 | |
| 3 | SUCCEEDED | 27 | 0.25 | 0.29 | 0.71 | +0.63 | 0.04 | |
| 4 | SUCCEEDED | 24 | 0.35 | 0.06 | 0.65 | +0.62 | 0.04 | |

log counts: wall_avoid 5, avoid_release 4, blind_reseed 0, no_valid_path 0, start_occupied 0, invalid_source 0, pose_old 0, no_progress 0, cm_stop 0

