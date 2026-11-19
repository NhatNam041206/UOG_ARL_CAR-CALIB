# NAV2 simulation (laptop)

Test the NAV2 mode before any real run. The simulator replaces only the
**car and the LiDAR**; everything from `/scan_filtered` to the
`"<left m/s>,<right m/s>"` wheel command is the same code and the same config files
the car uses.

```
           simulator (car_nav_sim)                 same as the car (car_nav)
  ┌──────────────────────────────────┐   ┌───────────────────────────────────────────┐
  │ lite_sim (2D)  or  Gazebo        │   │ scan_self_mask → scan_upright → /scan_nav  │
  │  • LiDAR upright, frame laser    ├──►│ rf2o (odom) · AMCL or slam_toolbox (map)   │
  │  • skid-steer model: latency,    │   │ Smac 2D A* (turns on the spot)             │
  │    wheel limits, yaw slip        │◄──┤ PD  → collision_monitor → cmd_vel_to_control│
  └──────────────────────────────────┘   └────────────── "/car/wheels/cmd" ───────────┘
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
share of commands at the wheel speed limit and turning on the spot, plus a 0.5 s trace (true pose, AMCL
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
| `ros/car_nav/config/car.yaml` | car + sim | geometry, LiDAR pose, **`drive:`** (diff drive) |
| `ros/car_nav/config/nav2_params.yaml` | car + sim | the whole Nav2 stack |
| `ros/car_nav/behavior_trees/navigate_forward_only.xml` | car + sim | spin recovery, no BackUp |
| `ros/car_nav/launch/nav.launch.py` | car + sim | the nav stack launch |
| `ros/car_nav/car_nav/actuation.py` | car + sim | `cmd_vel` → `"<left m/s>,<right m/s>"` |
| `ros/car_nav_sim/config/sim.yaml` | sim only | the **true** car: latency, speed scale, yaw efficiency… |

Make `sim.yaml` worse than `car.yaml` to find the margin, e.g.
`yaw_efficiency: 0.6`, `latency_s: 0.5`, `speed_scale: 0.8` (or `SIM_STRESS=hard`).

The compose file mounts these folders, so edits apply on the next run
without rebuilding.

## Car model (skid-steer)

This branch is for the **skid-steer (differential drive) car**: same
0.37 x 0.27 m body as the old rear-steer car, LiDAR mounted the normal way
up. The rear-steer version stays on `claude/project-thread-t304ze`.

* `base_link` = floor point at the **centre of the car**, midway between the
  left and right wheels: the point a skid-steer car turns about. The car can
  turn on the spot; `x' = v cosθ, y' = v sinθ, θ' = ω`.
* Command: `"<left m/s>,<right m/s>"` on the ROS topic `/car/wheels/cmd`
  (`car_nav/actuation.py`): `left = v − ω·b/2`, `right = v + ω·b/2`, b =
  `drive.track_width_m`. A wheel over `drive.max_wheel_speed_mps` scales both
  wheels down together, so the curvature is kept and only the speed drops.
  Reverse is never sent (`drive.allow_reverse: false`). On the car an
  adapter turns this into the ugv-controller's own command.
* `drive.track_width_m` is **not measured yet**: 0.22 m is assumed
  (`assumed_track_width_m`), the launch warns about it.
* Truth in `sim.yaml`: latency 0.25 s, wheel accel/decel limits,
  `speed_scale` (real/commanded forward speed), `left_gain`/`right_gain`
  (one side weaker), **`yaw_efficiency` 0.8**: a skid-steer car turns slower
  than `(right − left)/b` because the wheels slide sideways, and
  `min_spin_wheel_mps` (too-slow wheels do not get it to turn on the spot).
  `SIM_STRESS=hard` = latency 0.5 s, speed 0.85x, yaw efficiency 0.6, right
  side 8 % weaker, stiction 0.12 m/s.
* The lite backend also publishes `/odom_wheel`: what wheel encoders would
  report (no slip). It is only used with `adaptive_odom wheel_odom_topic`.
* Odometry (`odom_source:=adaptive`, default): rf2o for rotation and
  sideways motion (it measures the real, slipping turn), forward motion from
  rf2o where the scan shows it and otherwise from the commanded forward
  speed `(left + right)/2` (delayed, ramped) times a learned ratio
  real/commanded (Kalman filter, starts at 1.0). With encoders on the car,
  set `adaptive_odom.wheel_odom_topic` and the encoder speed replaces the
  command; the ratio then absorbs slip.
* Planner: **Smac 2D** (grid A*, no turning radius); `cost_travel_multiplier`
  keeps paths in the middle of corridors.
* Controller (`controller:=pd`, default), `car_nav/pd_follower.py`: the same
  PD on the path (curvature = ff·path curvature − kp·offset − kd·sin(heading
  error), kp 1.5 / kd 2.0, `ff_ahead_m` 0.1 = distance driven in the
  latency), then `diff_drive_command()`: speed 0.4 m/s, lower in tight
  curves (yaw ≤ 1.5 rad/s) and over the last 0.6 m; above 52° heading error
  it **turns on the spot** until below 14°. At the goal it turns on the spot
  to the goal heading (`yaw_goal_tolerance` 0.25 rad; xy tolerance 0.2 m).
  `controller:=rpp` = Regulated Pure Pursuit with rotate-to-heading.
* Wall avoidance (`car_nav/wall_avoid.py`) unchanged in idea: the footprint
  is swept along the PD's arc on the latest scan, candidate arcs span
  ±`avoid_max_curvature` (2.5 1/m), horizon 0.9 m. Not used while turning
  on the spot.
* Collision monitor: `VelocityStop` (velocity polygon) is a bumper reaching
  0.22 m past the nose when driving forward, and only the footprint + 3 cm
  when turning on the spot (a box ahead would block the turn away from a
  wall). `FootprintApproach` now really slows the car (variable speed).
* Behaviour tree recovery: clear costmaps / spin 90° / wait (round robin);
  no BackUp.
* Pose gate, static-map-only global costmap, AMCL settings: unchanged from
  the rear-steer branch.

## LiDAR mounted upright: `/scan_nav`

The A1 is now mounted the normal way up (`laser.roll: 0`). `scan_upright`
stays in the chain as a pass-through: it re-publishes `/scan_filtered` as
`/scan_nav` (frame `laser_upright`, identical to `laser` now) and
`/scan_odom` (invalid beams set to 0 for rf2o). If the LiDAR is ever
mounted upside down again, `roll: pi` in car.yaml is all it takes.

## Depth camera and low obstacles

`sim.launch.py depth:=true` (also `run_sim.sh mission low_box depth:=true`) publishes a simulated Astra on
`/camera/depth/points` (`config/sim.yaml depth:`: 80 x 60 rays, 58 x 45 deg, 10 Hz, 0.6-4 m, noise, dropout; 3D ray
cast from the `car.yaml` camera pose against the floor, boxes / cylinders with their `height`, actors 1.8 m) and turns
on the depth source of the safety guard. The simulated LiDAR now respects heights: it does not see shapes lower than its
mounting height. A mission may list `extra_obstacles:` (`boxes:` / `cylinders:` like a world file): they exist in the
simulation and in the collision score but NOT in the map. `missions/low_box.yaml`: a 0.15 m high box in the aisle;
`timeout_ok: true` makes "stopped in front of it, goal not reached" a pass as long as nothing is hit.
`depth:=false` -> the car drives into it (FAIL, collision).

## Safety: the simulator cannot drive the car

* Nothing in `car_nav` or `car_nav_sim` imports an MQTT client or opens a
  serial port.
* The sim container runs on its own Docker network (no host network), ROS
  domain 77 (or `SIM_ROS_DOMAIN_ID`) and DDS discovery limited to
  localhost, so its `/car/wheels/cmd` topic is not visible to the car.

## Measuring the car (replaces the assumed values)

1. **Track width**: tape, centre of the left wheel (or track) to centre of
   the right one -> `drive.track_width_m`.
2. **Effective track / yaw efficiency**: on the real floor, command a turn
   on the spot, e.g. `left = -0.2, right = +0.2` m/s, for 10 s and measure
   the angle turned (LiDAR: `ros2 topic echo /odom_rf2o`, or tape on the
   floor). Ideal rate = 0.4 / b rad/s; real / ideal = yaw efficiency. Put it
   in `sim.yaml truth.yaw_efficiency` and re-run the sim (Nav2 does not need
   it: rf2o measures the real turn).
3. **Latency**: from command to wheels moving (video at 60 fps, or the
   controller's feedback) -> `speed.command_latency_s`.
4. **Speed scale**: drive 3 m straight at 0.3 m/s; real / commanded ->
   `sim.yaml truth.speed_scale` (adaptive_odom learns it online anyway).
5. Re-run `scripts/sim/run_sim.sh all` and `SIM_STRESS=hard ...`.

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
