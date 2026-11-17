# Review of the proposed methods (before running the sim) - 2026-11-16

I wrote this from the code on `claude/project-thread-t304ze` (6b4b3d4), the campus tile map, and a
geometric check of the goal-2 corner. Sim runs will confirm or reject each point. Their results go in
the numbered folders and in SUMMARY.md.

## The goal-2 corner, measured

From `maps/tile_maps/campus-path.json`, the car goes south in the 3-tile corridor (cols 3-5, x 42.6-44.4)
and turns left (east) into the 2-tile corridor (rows -5/-4, y 11.4-12.6). It is **not an L corner,
it is a junction**: a 2-tile corridor (cols 4-5, x 43.2-44.4) carries straight on south, and col 3
ends just before the turn. So driving straight on is free. Nothing ends in front of the car.

I swept the car footprint (x_front 0.16, x_rear -0.37, half width 0.135) along an ideal
straight - arc - straight path through this corner on the real cells, moving the turn start
earlier or later:

| planned radius | turn start error that still fits (late = +) | with 5 cm margin |
|---|---|---|
| 0.75 m | -0.45 .. +0.45 m | -0.40 .. +0.40 m |
| 0.66 m | -0.45 .. +0.45 m | -0.40 .. +0.40 m |
| 0.55 m | -0.45 .. +0.45 m | -0.40 .. +0.40 m |

So the car has about **0.4 m of along-track error budget**, and the measured pose lag is
**0.6-1.1 m**. With that lag it cannot get round, however good the controller is. Note the window
does not depend on the radius: a turn that starts d late ends d too far south in a 1.2 m exit
corridor, whatever the radius.

## Why the previous attempts failed

1. **Along-corridor pose lag larger than the 0.4 m budget.** In the 3-tile corridor the scan sees
   two parallel walls, so neither rf2o nor AMCL can measure forward motion. Adaptive odometry
   fills the gap with a learned speed, and the pose gate (by design) holds back AMCL's along
   correction there. The pose becomes observable again only when the junction itself is in view.
   By then the car has to start its turn (0.75 m radius + 0.33 m driven during the 0.65 s latency
   is about 1.1 m before the corner). Any error in the learned speed integrates over the blind
   stretch: 5 % of a 10 m corridor is already 0.5 m.
2. **Distance-based wall avoid fought the turn.** At the inner corner of a left turn the inner
   wall comes closest, so a "steer away from the nearest wall" rule steered right, against the
   planned turn. Being close to a wall is not a danger when the arc you drive clears it. This is
   why it was replaced by the arc check.
3. **No reverse makes every mistake final.** Once the car is against a wall, or the collision
   monitor stops it with every forward arc blocked within 0.54 m, it can only wait for the 10 s
   progress check to abort. Goals 3-4 then fail as a knock-on effect. Their results say nothing
   about goals 3-4 themselves until goal 2 passes.
4. **Planner on a costmap marked with the lagging pose** (fixed in b18554f): scans marked at the
   wrong place closed the 1.2 m corridor ("no valid path found"). The fix (static map only) means
   the global planner no longer sees real obstacles. That is fine on this map, but not for a
   person or a box on the real campus.
5. **Diagnosis on misleading numbers.** The mission node is single-threaded and busy-loops on
   `isTaskComplete()`, so its `est` column and `max_localization_error_m` include its own delay.
   Some of the "0.6-1.1 m lag" may be measurement artefact and some may be real. They need
   different fixes, so the planner log start poses have to be compared against truth first.
6. **Laptop load.** The launch comment says odometry ran about 1.1 s behind on the laptop
   (2026-11-12), and the sim allows a 1.5 s pose age (the car allows 1.0). A pose that is 1.1 s old
   at 0.5 m/s is 0.55 m behind on its own, more than the whole budget.

## The four proposed methods: what each fixes, and what can go wrong

### (1) Arc-check wall avoid (DWA-like), already in the code
Fixes: it no longer fights a planned turn whose arc is clear. It needs no pose, so it is the right
kind of safety net for a wrong pose.

What can go wrong:
- **It cannot fix a late turn at this corner.** The straight arc is clear (the corridor continues
  south), so the layer stays silent while the car overshoots. It only wakes up when the late turn's
  arc hits the far wall, which is already outside the 0.4 m window.
- **It can choose the wrong corridor.** Its ranking is "furthest free distance first, then closest
  to the PD's arc". At this junction a straight or right arc down the south 2-tile corridor is
  often clear for the full 1.3 m, while a left arc into the 1.2 m corridor is not. The layer then
  steers into the south corridor. Without reverse, and with no room for a U-turn (a 1.2 m corridor
  needs more than 1.5 m), that is a dead end.
- **It ignores the command latency.** The sweep assumes the new curvature acts at once, but the
  wheels keep the old curvature for 0.65 s (0.33 m). So it rates escape arcs as more open than they
  are. The first 0.33 m of every candidate should be swept on the command still in flight.
- **Hysteresis can keep it in charge.** It hands back only when the PD arc is clear for 1.69 m. In a
  1.2 m corridor just before or after a turn that may never happen, and meanwhile it drives its own
  arc with no idea where the path is.
- **It conflicts with the collision monitor.** The monitor's approach zone (0.54 m) can STOP the
  car on the avoider's own arc, and once stopped the car never restarts if every arc stays blocked.
- **Coarse candidates.** 13 arcs give steps of about 0.2 1/m, so near a full lock the choice is
  rough.

### (2) Reduce pose lag: TF timestamps plus extrapolation in the PD follower
Fixes: only the **stale** part of the lag (pose age). The PD reads map->base_link at the latest TF
time. If odometry is 0.2 s old that is 0.1 m, and if it is 1.1 s old that is 0.55 m. Extrapolating
by v x age along the heading (and the curvature) removes it.

What can go wrong:
- It does **nothing for a wrong estimate** (learned speed wrong while blind). That error is fresh
  but wrong, and extrapolation keeps it.
- Extrapolating with the commanded 0.5 m/s while the car is really stopped (collision-monitor STOP,
  or STOP still in the latency pipe) or slower (speed_drift 0.42 m/s) overshoots the other way. It
  has to use the learned speed, and the drive state as it will be after the latency.
- If the lag is mostly a mission-node artefact, this changes nothing on the car. **Measure first**:
  log the PD's pose age, and the planner start pose vs. truth.

### (3) Planner minimum turning radius 0.75 -> 0.66
What can go wrong:
- **It does not widen the timing window at this corner** (table above: ±0.45 m for 0.75, 0.66 and
  0.55). It only changes the path shape a little, so it is unlikely to fix goal 2 alone.
- 0.66 is exactly the limit the unit test allows (1.2 x the weak-side radius 0.55). That leaves no
  steering reserve for the PD: with understeer or the latency it saturates (watch
  `steer_saturated_share`).
- The curvature table is a **placeholder** (1:1 linkage, unmeasured). On the real car 0.66 m may be
  tighter than the car can drive. A sim win here may not transfer.

### (4) LiDAR corner turn (full lock toward the opening when an end wall is ahead and the path turns)
What can go wrong:
- **Its trigger does not exist at this corner**: there is no end wall ahead, because the corridor
  continues south. As written it would never fire for goal 2. It would need "opening on the side
  the path turns to" as the trigger instead.
- It needs the path to know that a turn is coming, and the path is placed using the same lagging
  pose. So it inherits the error it is meant to fix, unless it turns purely on the LiDAR opening.
- Openings that are not the right one (side gaps, pillars, the col-3 pocket here) can trigger it.
- Full lock (about 0.49 m left radius) swings the rear out to about 0.73 m from the turn centre.
  In a 1.2 m corridor that is tight, and it hits the corner if it starts early.
- After the turn the pose estimate is still behind, so the PD fights the car back. This is the
  same "two controllers disagree" pattern as the old wall-avoid layer.
- It is a new controller mode with its own state machine. You wanted to pick control methods
  yourself, so I will not implement it beyond what you approve.

## What I expect actually matters (to be confirmed by the runs)

1. **Fix the along-track pose error near the junction**, because that is what the 0.4 m budget is
   about. Data first: baseline vs `odom_source:=truth` vs `pose_gate:=false`. If truth odometry
   passes, the controller and arc check are good enough and the work is localization.
2. Cheap localization levers to test, one at a time: let AMCL's along correction in earlier as the
   junction comes into view (`pose_gate.obs_full`, observability thresholds), and the learned
   speed's process noise (`adaptive_odom.*`).
3. Arc check: add the in-flight command to the sweep, and prefer candidates that follow the path's
   turn direction over "furthest free distance" when both are clear for most of the horizon. These
   are bug-level fixes to the existing method, not a new method.
