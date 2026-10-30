# Closed-loop turns and settle steps

Open-loop turns (`left` for T seconds) over-rotated in the 2026-10-28 runs:
T = 2.5 s gave a median of 99° and T = 2.8 s gave 106°, with ±6–8° spread
between turns. The yaw rate at full lock is about 38–40°/s. A command takes
about 0.65 s to show in the camera image, both when the turn starts and when
it stops. After each turn the heading was off by 8–20° and the car was more
than half a lane off centre. Turn steps can now stop on what the camera sees,
and a `settle` step can straighten the car before the next tile step.

## Heading from the tile grid (`vision/grid_heading.py`)

`GridHeadingEstimator` finds the grout lines of both seam families
(black-hat + HoughLinesP on a half-size frame) and takes a vanishing point
for each by RANSAC. The two families are perpendicular on the floor, so
`x1 * x2 = -g²` (x measured from the centre column). `g` is learned from
frames that show both families. Until then `GRID_HEADING_G_PX` (675) is used.
The heading is `psi = atan(x / g)` for the vanishing point nearest the centre
column. It needs no focal length or pitch, and it is known only modulo 90°.
`psi > 0` means the car points left of the seams.

`TurnYawTracker` unwraps psi from frame to frame into the yaw since the last
reset, with left turns positive. When the grid is not visible it uses a
phase correlation of the upper image band. The pixels-per-degree factor for
that is learned from frames where both measures exist. With neither, it
coasts on the last yaw rate for `GRID_HEADING_COAST_MAX_S` (0.5 s).

A vanishing point must lie above row `GRID_HEADING_MAX_VP_Y` (-60, Tune key
`grid_max_vp_y`); a point inside the frame is a tile corner where two seams
cross. The horizon is learned from two-VP frames only. It is not seeded from
the lane tune's `vp_horizon_y`: in the 10-29 run that row was -120 while the
grid horizon was about -340, so every VP was rejected, the heading was never
valid and the 90° turn finished on time at about 60°.

The estimator runs while a script runs (`GRID_HEADING_MODE=script`, or
`always` / `off`). On the cloud test machine it costs about 2 ms per frame,
plus about 1 ms for phase correlation, which runs only during degree steps.
The route CSV gets `grid_psi`, `grid_vps`, `grid_lines`, `grid_g_px`,
`grid_ms`, `turn_yaw`, `turn_omega`, `turn_yaw_src`, `turn_px_per_deg`, and
the per-stage times `capture_ms`, `calib_ms` and `heading_ms`.

## Left and right commands

| Action | Base | Servo |
|--------|------|-------|
| `left` | `FORWARD` (0,1,0) | centre + `TURN_LEFT_STEER_DEG` (Ackermann arc) |
| `right` | `FORWARD` (0,1,0) | centre − `TURN_RIGHT_STEER_DEG` |
| `turn_left` | `TURN_LEFT` (1,0,0) | not set |
| `turn_right` | `TURN_RIGHT` (0,1,1) | not set |

Centre is `90 + SERVO_CENTER_ANGLE`. The script runner sends these servo
values directly, so the Servo group's trim, gains and kicks (vision path
only) do not apply. Both offsets default to `MAX_STEERING_OFFSET` and can be
changed in the Tune tab group **Turning**. The servo value is clamped to
0..180: with `SERVO_CENTER_ANGLE=-28` (centre 62) any right offset above 62
gives servo 0, so the 10-28 runs with offset 80 steered 80° left but only 62°
right.

## Degree steps

`{"action": "left", "degrees": 90}` also works with `right`, `turn_left` and
`turn_right`. The dashboard sets it with the seconds/degrees selector next
to the amount. The step resets the yaw tracker and starts the turn. It stops
when the yaw seen reaches

    degrees − omega × TURN_LATENCY_S

`omega` is the measured yaw rate, capped at 2 × `TURN_NOMINAL_OMEGA_DPS`.
The car keeps turning for about the latency after the frame was taken, so
with 40°/s and 0.65 s the stop goes out when the camera shows about 64°.

- No heading for `TURN_HEADING_LOST_S` (0.6 s, after the first
  `TURN_LATENCY_S`): with `TURN_ON_HEADING_LOST=time` the turn finishes on
  `degrees / omega` seconds from its start (the two latencies cancel). With
  `stop`, the script stops with `last_error`.
- The safety timeout is `TURN_TIMEOUT_FACTOR` (2) × degrees / nominal omega
  + latency. Hitting it stops the script. The same time counts toward
  `ROUTE_MAX_TOTAL_DURATION_S`.
- The runner status reports `step_degrees_done` and `last_turn`: angle seen
  at stop, omega, lead, elapsed time, source, and whether the time fallback
  was used.

## Settle step

`{"action": "settle", "duration_s": 1.5}` drives forward with the servo set by

    centre − clamp(SETTLE_K_PSI × psi + SETTLE_K_OFFSET × lane_offset_ratio, ±SETTLE_MAX_STEER)

The servo slew rate is limited to `SETTLE_RATE_DPS`, and the vision PID and
its danger nudge do not publish during the step. The step ends early once
`|psi| ≤ SETTLE_EXIT_PSI_DEG` and `|offset| ≤ SETTLE_EXIT_OFFSET` hold for
`SETTLE_EXIT_HOLD_S` (0 turns the early exit off). Defaults: k_psi 1.0,
k_offset 20, max 25°, 60°/s, exit at 3° and 0.1 held for 0.3 s. These gains
have not been tuned on the car yet.

## A* scripts

`TILE_TURN_DEGREES=true` makes the map planner emit degree turns.
`TILE_TURN_ACTION=steer` uses `left`/`right` (Ackermann arc, as in the
10-28 runs) instead of `turn_left`/`turn_right`. `TILE_SETTLE_S` > 0 adds a
settle step after every turn.

## Turn approach to the next cross line

`{"action": "left", "degrees": 90, "anchor": "next_line"}` (dashboard unit
"degrees, start at V+1"; A* with `TILE_TURN_ANCHOR=true`) does not start the
turn where the previous tile step ended. It locks onto the nearest cross line
above the counter line (V+1; V has just crossed the counter line), drives
forward with the vision PID until that line is `TURN_ANCHOR_SLOW_MARGIN` (0.08)
rows above y* (`TURN_ANCHOR_Y_LEFT` / `_RIGHT`, row ratio, 0 = top), stops,
and then nudges: wait `TURN_ANCHOR_WAIT_S` (0.3 s) at standstill so the line
is sharp, measure, and pulse FORWARD for (y* − y) / k seconds (0.05..0.3 s),
where k is the rows per second learned from the earlier pulses (first pulse
`TURN_ANCHOR_PULSE_S`, 0.15 s). No speed is needed. The turn starts from
standstill once the line is within `TURN_ANCHOR_TOL` (0.015) of y*, or past
it. A line lost for `TURN_ANCHOR_LOST_S`, more than `TURN_ANCHOR_MAX_PULSES`
(8) nudges or `TURN_ANCHOR_TIMEOUT_S` (8 s) stops the script.

Set y* once per side: put the car by hand where the turn should start (the
rear-axle point d_V = a − L/2 before V) and read the row of the next line on
the HUD.

### Turn errors and the suggestion

The first frames of a `settle` step after a turn give `e_psi` (degrees the
turn fell short; negative = too far) and `e_y_cm` (cm the new line runs past
the middle of the tile row V..V+L; negative = short of it), from the grid
heading and `lane_offset_ratio × TURN_TILE_CM`. Each turn adds an entry to
`turn_log` in the runner status and in `route_summary.json`. From them the
runner suggests

    latency  -= alpha × e_psi / omega          (max 0.1 s per turn)
    y*(side) -= alpha × e_y × tile_dy / L      (max 0.03 rows per turn)

with `tile_dy` the image rows between V+1 and the line above it at the stop
and alpha `TURN_ADAPT_ALPHA` (0.3). Turns that finished on time are skipped.
The Tune tab shows the suggestion and an **Apply turn suggestion** button
(Apply only; Save is still a separate click). All approach parameters are in
the Tune group **Turn approach**.

## Map position through turns

While a `left`, `right`, `turn_left` or `turn_right` step runs, seams counted
by the cross-line counter do not move the car on the map. When a turn step
ends, the runner reports its rotation (the degrees asked for on a degree step,
duration × `TURN_NOMINAL_OMEGA_DPS` on a time step). The map rounds it to
quarter turns. In heading mode (start + heading) it rotates the heading and
counts on from the current cell; `position.heading` shows the new heading. In
path mode the A* path already holds the corner, so only the freeze applies.
The car is assumed to end the turn on the cell where it started it: the
movement along the arc needs `R_eff` and `L` (see below).

## Lane shifts (`vision/lane_shift.py`)

When the car crosses a lane line, the lane tracker locks onto the next pair
of seams and `lane_offset_ratio` jumps by about one lane width (+0.5 to -0.5
when the car crosses its left line). A shift counts when the offset was held
past `LANE_SHIFT_EDGE_RATIO` (0.25) for `LANE_SHIFT_HOLD_FRAMES` (3) frames,
jumps by `LANE_SHIFT_JUMP_RATIO` (0.6), is held on the other side for as many
frames, and the grid heading changes less than `LANE_SHIFT_MAX_PSI_CHANGE_DEG`
(12°) across it. A tracker that flickers between the two lines of one lane
does not count.

- Active: a turn step is running or ended less than `LANE_SHIFT_ACTIVE_HOLD_S`
  (2 s) ago. The tracker is re-locking after the turn; the map is handled by
  the turn.
- Passive: any other shift while the base drives. The map moves one row to
  the side of travel (as an anchor, so path mode re-plans from there).

### Lane recovery

A passive shift during a `forward` step pauses the step and steers back:
`FORWARD` with the servo `LANE_RECOVER_STEER_DEG` (15°) from centre toward the
old lane, vision PID off. When the detector sees the shift back, the map moves
back too and the step resumes from centre with the vision PID (the recovery
time does not count as step time). No shift back within
`LANE_RECOVER_TIMEOUT_S` (3 s, paused time excluded) stops the script with
`lane recovery failed`. `LANE_RECOVER=false` turns this off. All three are in
the Tune group **Lane shift**; the runner status shows `lane_recovering`
(+1 drifted left, -1 right, 0 none).

The route CSV gets `lane_shift_side`, `lane_shift_kind`, `lane_shift_t` and
`lane_shift_passive_count`. Replaying the eleven 10-28 runs finds two shifts,
both in the stop right after a left turn (active), and no passive ones.

## Dashboard Tune tab

The turn, settle and grid-heading parameters are also in the Tune tab, in the
groups **Turning**, **Settle** and **Grid heading**. Their env values above
are the defaults. Apply and Save work as for the vision parameters, and saved
values go to `calib_tune.json`. The group selector above the fields shows a
single group or ALL. `GRID_HEADING_MODE`, the camera settings and the A*
settings are env-only: they take effect at start-up or when a plan is made.

## Camera latency

`CAMERA_BUFFER_SIZE` (1) sets `CAP_PROP_BUFFERSIZE`. `CAMERA_LATEST_FRAME=true`
wraps the camera in `runtime/latest_frame.py`, a thread that keeps only the
newest frame, so a slow loop never processes a queued, older frame. After
changing either setting, measure `TURN_LATENCY_S` again: run a degree turn
and compare `last_turn.seen_at_stop_deg` with the final `grid_psi` once the
car has stopped.

## Not done yet

The turn does not start at a distance to the cross line yet. The plan is to
turn at `d_V = R_eff − L/2` so the car ends half a tile from that line.
`R_eff` and the tile size `L` still need to be measured with a tape.
