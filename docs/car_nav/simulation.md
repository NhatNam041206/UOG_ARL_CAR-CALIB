# NAV2 simulation (laptop)

Test the NAV2 mode before any real run. The simulator replaces only the
**car and the LiDAR**; everything from `/scan_filtered` to the
`"<angle>,<drive_code>"` command is the same code and the same config files
the car uses.

```
           simulator (car_nav_sim)                 same as the car (car_nav)
  ┌──────────────────────────────────┐   ┌───────────────────────────────────────────┐
  │ lite_sim (2D)  or  Gazebo        │   │ scan_self_mask → scan_upright → /scan_nav  │
  │  • LiDAR upside down, frame laser├──►│ rf2o (odom) · AMCL or slam_toolbox (map)   │
  │  • actuator model: latency,      │   │ Smac Hybrid-A* (DUBIN, forward only)       │
  │    fixed speed, understeer       │◄──┤ PD  → collision_monitor → cmd_vel_to_control│
  └──────────────────────────────────┘   └───────────── "/car/raspi5/control" ────────┘
         ▲ truth pose → mission scorer (clearance, end error, localisation error)
```

## Run it

Needs Docker (Docker Desktop on Windows) and internet for the first build.

```bash
scripts/sim/run_sim.sh build                     # ~15 min the first time
scripts/sim/run_sim.sh mission loop_left         # left-turn loop, headless, PASS/FAIL
scripts/sim/run_sim.sh mission loop_right        # right-turn loop (the weak side)
scripts/sim/run_sim.sh mission loop_left odom_source:=truth   # perfect odometry, isolates rf2o
scripts/sim/run_sim.sh all                       # both loops x both odometry sources
scripts/sim/run_sim.sh view                      # reports -> HTML pages, open in a browser
scripts/sim/run_sim.sh rviz                      # 2D sim + RViz, click "Nav2 Goal"
scripts/sim/run_sim.sh gui                       # Gazebo Harmonic + RViz
```

GUI on Windows 11: run the script from a **WSL** shell (WSLg gives the
container a display). On Linux run `xhost +local:` once. Headless runs need
no display and no GPU.

Reports go to `run_logs/sim/report-<mission>-<time>.json`. Per goal: result,
time, distance, **smallest clearance** between the car outline and any
obstacle (0 = collision), **end position/heading error** after the car has
coasted to a stop, **worst localisation error** (AMCL vs. truth), and the
share of commands at a servo limit, plus a 0.5 s trace (true pose, AMCL
pose, command, collision-monitor state, clearance) that CI prints for any
failed goal.

GitHub Actions (`nav2 simulation` workflow) builds the image and runs all
four missions on every push that touches `ros/` or `docker/sim/`; download
`nav-sim-reports-<sha>` for the reports and logs.

## See it

**After a run (any shell, no display needed):**

```bash
scripts/sim/run_sim.sh view          # every report -> run_logs/sim/view/*.html
start run_logs/sim/view/index.html   # Git Bash on Windows (Linux: xdg-open)
```

Each page draws the room, the goals (circle = 0.5 m tolerance), the true
path (solid) and the AMCL estimate (dashed). Play / the slider move the car
outline through the run and show the command, the collision-monitor state
and the clearance; a red ring marks a collision.

**Live (RViz):** `scripts/sim/run_sim.sh rviz`, `campus`, `campus-mission`
or `gui`. From a **WSL Ubuntu** terminal (Docker Desktop -> Settings ->
Resources -> WSL integration on for that distro; not the `docker-desktop`
distro, it has no bash) or a Linux desktop (`xhost +local:` once) RViz opens
as a normal window: the script mounts `/tmp/.X11-unix` (WSLg) into the
container. From Git Bash, or with `SIM_VIEW=browser`, RViz runs on a virtual
screen inside the container instead: open
`http://localhost:6080/vnc.html?autoconnect=1&resize=scale` about 15 s after
the start (bound to 127.0.0.1 only). Force a window with `SIM_VIEW=x11`.
AMCL gets the true start pose automatically; the red arrow "True pose" shows
where the car really is. In RViz use **Nav2 Goal** to send goals by hand:
click on a straight aisle at least ~1.5 m ahead and drag in the driving
direction (the car only drives forward and needs ~0.75 m turning radius).

**When to rebuild.** `docker-compose.sim.yml` mounts `ros/car_nav` and
`ros/car_nav_sim` into the container and the image is built with
`--symlink-install`, so edits to Python code, launch files, configs, worlds and
missions take effect on the next run with no build. Rebuild
(`run_sim.sh build`) only after a Dockerfile change, a new executable in
`setup.py`, or a new file in `launch/` or `rviz/`. A rebuild after a code
change redoes only the last layer (seconds); rf2o (C++, ~2 min) stays cached.

## Backends

| | `lite` (default) | `gazebo` |
|---|---|---|
| Needs | nothing (CPU only) | a GPU or software OpenGL for the LiDAR |
| LiDAR | ray cast on the world shapes, noise + dropouts | `gpu_lidar` sensor, rolled by π like the real A1 |
| Car motion | actuator model, integrated at the front axle | actuator model → Gazebo `VelocityControl` at the front axle |
| Use for | missions, CI, quick parameter sweeps | seeing it in 3D |

Both read the same world file (`ros/car_nav_sim/worlds/*.yaml`, boxes and
cylinders). The map AMCL uses is rasterised from it, so map frame = world frame.

## What is shared with the car, and what is sim-only

| File | Used by | Notes |
|---|---|---|
| `ros/car_nav/config/car.yaml` | car + sim | geometry, LiDAR pose, baseline speed, **`steering:`** (new) |
| `ros/car_nav/config/nav2_params.yaml` | car + sim | the whole Nav2 stack |
| `ros/car_nav/behavior_trees/navigate_forward_only.xml` | car + sim | no Spin/BackUp |
| `ros/car_nav/launch/nav.launch.py` | car + sim | the nav stack launch |
| `ros/car_nav/car_nav/actuation.py` | car + sim | `cmd_vel` → `"<angle>,<drive_code>"` |
| `ros/car_nav_sim/config/sim.yaml` | sim only | the **true** car: latency, speed, understeer… |

Make `sim.yaml` worse than `car.yaml` to find the margin, e.g.
`understeer_factor: 0.8`, `latency_s: 0.9`, `speed_mps: 0.6`.

The compose file mounts these folders, so edits apply on the next run
without rebuilding.

## Car model

* `base_link` = floor point under the **front (fixed) axle**. The car steers
  with the **rear** wheels, so the front axle never slips sideways: the car
  moves like a point that drives along its heading and yaws about it
  (`x' = v cosθ, y' = v sinθ, θ' = v·κ`), and the tail swings out in turns.
* Commands are what Pi4/ESP32 accept: absolute servo angle 35..140
  (62 = straight) and drive code 1 = FORWARD / 0 = STOP. NAV2 never sends 2.
* Servo → curvature comes from `steering.curvature_table` in `car.yaml`.
  Until it is measured, a placeholder assumes rear wheel angle = servo offset
  (1:1), capped at 30°, wheelbase 0.28 m: right max ≈ 27° → radius ≈ 0.55 m,
  left ≈ 0.49 m. The planner uses 0.75 m.
* `steering.servo_up_turns: left` assumes angles above 62 turn the car left
  (as the VISION `left` presets do). **Confirm on the floor.**
* Speed is fixed (no speed command, no feedback): `sim.yaml truth.speed_mps`
  vs. `car.yaml speed.baseline_speed_mps` (both 0.5, unmeasured).
* Latency 0.65 s: at 0.5 m/s the car drives 0.33 m before a command acts.
  This is why lookahead is 1.1 m and the goal tolerance is 0.5 m
  (position only: the car cannot fix its heading without driving on).
* Odometry (`odom_source:=adaptive`, default): between two long parallel
  walls the LiDAR cannot see forward motion, so rf2o alone reports the car
  standing still and AMCL fell up to 5 m behind on the school map.
  `car_nav/adaptive_odom.py` keeps rf2o for rotation and sideways motion and
  takes forward motion from rf2o only where the scan shows it (points facing
  the driving direction: end walls, corners, pillars); in blind stretches it
  uses a learned speed (Kalman filter, `speed_filter.py`) that rf2o and
  AMCL's corrections keep updating, starting from car.yaml's
  `baseline_speed_mps`. STOP commands (delayed by the command latency) mean
  speed 0. A speed change in the middle of a blind corridor is only fixed at
  the next observable place. Try it against a car that is slower than
  car.yaml says: `SIM_STRESS=speed_drift scripts/sim/run_sim.sh campus-mission`
  (0.42 m/s, +-15 % drift). `odom_source:=rf2o` is the old behaviour.
* Steering controller (`controller:=pd`, default): a classical PD on the
  planned path, like the VISION lane PID, in `car_nav/pd_follower.py`:
  curvature = path curvature − kp·sideways offset − kd·sin(heading error).
  The heading error is the rate of change of the offset along the path, so
  kd is the D term that stops the rocking Pure Pursuit (a pure P: gain =
  1/lookahead) showed after turns. Gains `pd_path_follower.kp/kd` in
  nav2_params.yaml (0.7 / 1.2), tunable without a rebuild. Too much kd reacts
  to AMCL heading noise and, with the 0.65 s delay, rocks again; too little
  kp leaves the car off the path in turns when it understeers.
  `controller:=rpp` brings back Nav2's Regulated Pure Pursuit.
* Wall avoidance (reactive layer, `car_nav/wall_avoid.py`, inside the PD
  follower), arc check like DWA: before each command the car's footprint
  (rear swing included) is swept 1.3 m along the PD's arc on the latest
  scan. Clear -> the PD drives, however close a wall is, so the inner corner
  of a planned turn is not "avoided" (the earlier distance-based layer forced
  a right turn there, sim 2026-11-14). Hits -> of 13 arcs from full right to
  full left (steering table limits) it drives the one that goes furthest,
  closest to the PD's among the clear ones, until the PD's arc is clear over
  1.3 x the horizon ("wall avoid: PD arc ... hits a wall" / "back to the
  path" in the log). Needs no pose. In tests it gets a car whose pose lags
  0.8-1.2 m into a 1.2 m side corridor that it otherwise hits. Params
  `pd_path_follower.avoid_*`; `avoid_enabled: false` switches it off.
* Reading the path curvature further ahead (`ff_ahead_m` 0.65 instead of
  0.3) was checked 2026-11-13 and NOT adopted: with a true pose and the
  0.65 s delay the PD follows a 1 m-radius corner within ~2 cm at 0.3 m,
  while 0.65 m cuts it by ~20 cm. A turn that starts late in the sim means
  the pose estimate lags along the corridor.
* Global costmap = static map only (2026-11-15): the planner no longer
  marks scans into its map. With a pose that lags in blind corridors, scans
  marked at the wrong place closed the 1.2 m corridor ("no valid path
  found"). Live obstacles: local costmap, collision monitor, wall avoidance.
* Pose gate (`car_nav/pose_gate.py`, `pose_gate:=true` default): AMCL no
  longer publishes map -> odom itself. The gate takes AMCL's sideways and
  heading corrections, but its correction along the heading only as far as
  the scan can see forward motion (`/scan_observability` from
  adaptive_odom). In a blind corridor AMCL's particles slide along it (log
  2026-11-12: AMCL 4.2 m off, odometry alone 1.35 m); when the cloud is more
  than 0.3 m away along the corridor the gate reseeds AMCL at the gated pose.
  `pose_gate:=false` is the old behaviour. The sim stops for a stale pose
  only above 1.5 s (`max_pose_age_s`, 1.0 on the car).
* One speed means the collision monitor cannot slow down. Its "approach"
  zone follows the current curvature and the bridge turns any speed it
  scales below 0.3 m/s into STOP, so the car stops when the path ahead is
  blocked within ~0.54 m. A narrow bumper zone reaching 0.64 m ahead of
  the front axle (0.48 m past the nose = scan period + latency + braking)
  is the backstop when that prediction is wrong, because the wheels follow
  a new curvature only ~0.65 s later. A shorter 0.30 m bumper fired too
  late and the car hit a shelf; a wider 0.75 m box froze the car before
  turns next to pillars.

## Upside-down LiDAR: why `/scan_nav`

rf2o laser odometry only uses the **yaw** of the laser → base transform, so
with the A1 upside down (roll = π) it would see every turn mirrored.
`scan_upright` re-publishes the masked scan in a virtual frame
`laser_upright` (same position and yaw, no roll): ranges reversed, angles
mirrored. Every Nav2 consumer reads `/scan_nav`. The raw `/scan` and the
`laser` frame stay exactly as on the car.

## Safety: the simulator cannot drive the car

* Nothing in `car_nav` or `car_nav_sim` imports an MQTT client.
* The sim container runs on its own Docker network (no host network), ROS
  domain 77 and DDS discovery limited to localhost, so its
  `/car/raspi5/control` topic is not visible to the car's `ros-nav` container
  even on the same Wi-Fi.

## Measuring the curvature table (replaces the placeholder)

On an open floor, wheels on the ground, E-stop in hand, VISION dashboard
joystick or script with a fixed servo angle:

1. For each servo angle 35, 45, 52, 72, 85, 100, 120, 140: drive FORWARD
   one full circle.
2. Mark the floor under the **front-axle centre** at the points farthest
   apart; that distance is the diameter D. Curvature = 2/D (+ for a left
   turn, − for right).
3. Put the pairs into `car.yaml`:
   `curvature_table: [[35, -1.7], [45, -1.1], …, [62, 0.0], …, [140, 2.0]]`
4. Re-run `scripts/sim/run_sim.sh all`; the planner radius test
   (`tests/test_car_nav_nav2_params.py`) fails if
   `minimum_turning_radius` is now below 1.2 × the weak side's radius.

## School map (campus tile map)

`ros/car_nav_sim/worlds/campus_path.yaml` is generated from the tile map
`maps/tile_maps/campus-path.json` (0.6 m tiles, 1462 drivable cells, about
56 x 38 m) by `scripts/sim/tile_map_to_world.py`:

* every drivable cell is floor; every edge to a non-drivable cell is a thin
  wall on the non-drivable side, so corridors keep their full width
  (2 tiles = 1.2 m, 3 tiles = 1.8 m);
* **assumption:** the LiDAR sees something at every map edge. Where the real
  place is open (grass, a hall with no wall) the sim localises better than
  the car will;
* frame: x = east, y = north, cell (row, col) centre at
  x = (col + 68.5) * 0.6, y = (15.5 - row) * 0.6;
* the tile map anchors (AAD-1, gym-01, F.218, F.203, 1, 2) are named start
  poses: the pose near the anchor where the car footprint has most room.

After editing the tile map, regenerate (Python 3 with numpy + PyYAML,
`pip install numpy pyyaml`), from the repo folder:

    python3 scripts/sim/tile_map_to_world.py

Drive it in RViz (browser view, see "See it"):

    scripts/sim/run_sim.sh build                 # once: the image changed
    scripts/sim/run_sim.sh campus                # car at AAD-1
    scripts/sim/run_sim.sh campus gym-01         # car at another anchor
    scripts/sim/run_sim.sh campus-mission        # campus_block_left with RViz, then a report

In RViz click **Nav2 Goal** and drag the arrow on a corridor. The car only
drives forward and turns no tighter than ~0.5-0.75 m radius, so a goal
behind the car or round a tight corner of a 1.2 m corridor may fail; the
mission `campus_block_left` goes round the block east of AAD-1 with left
turns only (the car's stronger side).

The simulation no longer runs on GitHub on every push (only the unit
tests). To run it there anyway: Actions -> "nav2 simulation" -> Run workflow.
