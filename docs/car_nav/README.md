# NAV2 mode (car_nav)

The car gets a second, selectable driving mode next to the existing one:

| Mode | What drives | Container |
|---|---|---|
| **VISION** | lane vision, tile counting, route scripts (unchanged) | `car-calib-pi5` |
| **NAV2** | ROS 2 Jazzy + Nav2 with an RPLIDAR A1 and an Orbbec Astra Pro, no lane vision | `car-calib-ros-nav` |

Only the selected mode publishes `car/raspi5/control`; the other stays silent.
There is no mode arbiter and no local broker (decided 2026-11-10).

## Work orders

| WO | Scope | State |
|---|---|---|
| WO-01 | Lane camera pinned by ID; `ros-nav` with sensors only (LiDAR, Astra, TF, self-mask), baseline speed tool. **Sends nothing to the car.** | this PR |
| WO-02 | `car/drive_mode` flag + dashboard mode selector, nav bridge (`cmd_vel` → `"<angle>,<drive_code>"` through `map_calibrated_servo()`), teleop, servo → curvature table, Last Will `62,0` in NAV2 only | next |
| WO-03 | rf2o, slam_toolbox, AMCL, Nav2 (Smac Hybrid forward only + Regulated Pure Pursuit), collision monitor, behavior tree without spin/back-up | later |
| WO-04 | Dashboard NAV2 tab (map, goals, status), event log | later |
| SIM | Laptop simulator (Gazebo Harmonic + 2D backend), shared `nav2_params.yaml`, `nav.launch.py`, `actuation.py` (cmd_vel → payload), `cmd_vel_to_control` (ROS topic only). See [simulation.md](simulation.md) | PR stacked on WO-01 |

## Hardware on the Pi5 (checked 2026-11-10)

| Device | USB ID | Path used |
|---|---|---|
| Lane camera (rapoo) | `0c45:6365` | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._rapoo_camera_SN0001-video-index0` → `/dev/video0` in `car-calib-pi5` |
| RPLIDAR A1 (CP2102) | `10c4:ea60` | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` → `/dev/rplidar` in `ros-nav` |
| Astra Pro colour | `2bc5:0501` | libuvc over `/dev/bus/usb` (no V4L2 node in `ros-nav`) |
| Astra Pro depth | `2bc5:0403` | OpenNI2 (bundled with `ros2_astra_camera`) over `/dev/bus/usb` |

If a device is replaced, set `LANE_CAMERA_DEVICE` / `LIDAR_DEVICE` in `.env`
(find the path with `ls -l /dev/v4l/by-id/ /dev/serial/by-id/`).

**Important:** `car-calib-pi5` refuses to start when the lane camera is
unplugged. That is intended: before this change, VISION could silently open
the Astra colour camera as its lane camera.

## Skid-steer car (this branch)

This branch targets the **skid-steer (differential drive) car**: same
size as the rear-steer car, LiDAR mounted the normal way up. Nav2 sends
`"<left m/s>,<right m/s>"` on the ROS topic `/car/wheels/cmd`; an adapter
for the car's `ugv-controller` turns that into its own command (see
[simulation.md](simulation.md), "Car model"). The rear-steer version
(`"<angle>,<drive_code>"` on `/car/raspi5/control`) stays on
`claude/project-thread-t304ze`.

## Running it on the skid-steer car

Motor controller: `ugv-controller` (ESP32-C3 Super Mini, USB CDC, 303a:1001).
It takes `"<left> <right>"` integers -500..500 (ESC pulse 1500 ± us), has
no wheel encoders, and streams its MPU6050 as `IMU: ...` lines at 10 Hz.
`car_nav/ugv_serial_bridge.py` writes Nav2's `/car/wheels/cmd` to it at
20 Hz (m/s → units with `car.yaml ugv.*`) and publishes the IMU on
`/imu/data_raw`.

**Safety: the firmware has no command timeout.** If the computer hangs or
the USB cable comes out while the car drives, it keeps the last speed. The
bridge sends `0 0` whenever Nav2 stops commanding (0.5 s watchdog) and
`stop` when it exits, but it cannot help if it hangs itself. Keep the motor
power switch within reach. [ugv-controller-failsafe.patch](ugv-controller-failsafe.patch)
adds a 0.5 s timeout to `main/main.c` if the firmware can ever be reflashed.

1. Build the image with Nav2. The new car runs it on a **mini-PC** (x86):
   build it there or on any amd64 laptop with internet,
   `docker build -f docker/ros/Dockerfile -t car-calib-ros-nav:latest .`
   (the GitHub Actions workflow builds arm64 for the Pi 5). On the mini-PC
   raise `ROS_NAV_CPU_LIMIT` (default 1.5 CPUs, sized for the Pi).
2. `.env`: `UGV_DEVICE=/dev/serial/by-id/usb-Espressif_...` (see
   `ls -l /dev/serial/by-id/`), `LIDAR_DEVICE` as before.
3. First run with the wheels off the ground and motors off:
   `ROS_NAV_COMMAND="ros2 launch car_nav car.launch.py slam:=true motors:=false"`.
   Check `/scan` (LEFT/RIGHT with `scan_probe`), `/imu/data_raw`, and that
   `/car/wheels/cmd` changes when you send a goal in RViz.
4. Calibrate on the floor (`car.yaml ugv:` / `drive:`): drive `100 100` for
   5 s and measure the distance → `units_per_mps`; the smallest `n n` that
   moves the car → `deadband_units`; track width → `drive.track_width_m`;
   `100 0` must turn the car RIGHT (else `swap_sides`), `100 100` must go
   forward on both sides (else `invert_*`).
5. Map: `slam:=true`, drive around, save with
   `ros2 run nav2_map_server map_saver_cli -f /data/nav/maps/<name>`; then
   `car.launch.py map:=/data/nav/maps/<name>.yaml` and set the initial pose
   in RViz.

## How the parts talk

Everything runs on the mini-PC, in the `ros-nav` container:

* RPLIDAR (USB) → `sllidar_ros2` → `/scan` → self-mask, `scan_upright` → `/scan_nav`
* Nav2 + PD follower → collision monitor (wall bumper) → `/cmd_vel_guarded` →
  **safety_guard** (people) → `/cmd_vel` → `cmd_vel_to_control` → `/car/wheels/cmd`
  → `ugv_serial_bridge` → USB serial → ESP32 `ugv-controller` → ESCs
* ROS nodes talk over DDS on the mini-PC only (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`).
* A laptop or phone on the same Wi-Fi opens the **web UI** at `http://<mini-PC IP>:8080`;
  the page talks to ROS through rosbridge (WebSocket, port 9090).

## Web UI (like RViz, in a browser)

`nav.launch.py ui:=true` (default) starts rosbridge and `ui_bridge`. The page shows the map,
the LiDAR points (red = not on the map, i.e. what the safety guard treats as a person),
the car, the path and the guard zones. You can:

* **Initial pose**: click where the car is, drag the way it faces (→ `/initialpose`).
* **Nav goal**: click the goal, drag the arrival direction (→ `/goal_pose`).
* Change the guard's stop / slow distances, widths, field of view, slow speed and resume
  delay live (**Apply**). Changes are not saved: copy good values into
  `nav2_params.yaml` (`safety_guard:`).
* **EMERGENCY STOP**: the guard sends zero until you press RELEASE STOP. This is a
  software stop over Wi-Fi: it does not replace the motor power switch.

**Driving** sliders: cruise speed, max turn rate, slow-down before curves (PD follower,
act from the next goal).

**Profiles** (saved on the mini-PC in `/data/nav/ui/profiles/`, survive restarts and image
updates): *Save current* stores what the sliders show (guard + driving) under a name, with a
note; *Load + apply* sends a profile to the car; *Use at start* applies it every time the
stack starts; *Delete*. Also over HTTP: `GET/PUT/DELETE /api/profiles/<name>`.

**Tests** (saved in `/data/nav/ui/tests/`): type a name, *Start recording*, drive / walk in
front of the car, *Stop + save*. A record holds the parameters in use, every event (guard
stop/slow/clear with the distance, goals, initial pose, E-STOP, parameter changes) and a
5 Hz trace (pose, guard state, nearest person, wheel command). The list shows a summary
(time, distance, stops, nearest person, goals); add a verdict and a note, **Download** the
JSON to send for analysis, or delete it.

The ports are open to the whole network the mini-PC is on: use it on the lab's own Wi-Fi.
In the simulator: `scripts/sim/run_sim.sh ui` (or `ui mission:=person_stops`), then open
`http://localhost:8080`.

## Map modes (Load / Create / Unknown-flat)

The web page's **Map** panel (and `POST /api/mode`) switches between three ways of knowing where the car is:

| Mode | Localization | Global costmap | Use |
|---|---|---|---|
| **Load map** | `map_server` + AMCL (+ `pose_gate`) on a stored map; the map's default start pose (if set) is sent as `/initialpose` | static map + inflation | normal runs |
| **Create map** | `slam_toolbox` (online async, `config/slam_toolbox.yaml`); the map grows live in the view; **Save map as...** stores it | static layer on the growing `/map` | mapping a new place |
| **Unknown / flat** | none: `flat_origin` publishes `map -> odom` so that the car's pose at the start of the mode is `(0,0,0)` (x = the way it faces); the pose is odometry only | rolling 20 x 20 m window, LiDAR obstacle layer, no static layer (`config/nav2_flat_overrides.yaml`) | quick tests on an open floor |

**What restarts.** `nav.launch.py` is the persistent part: `scan_upright`, `rf2o` / `adaptive_odom` (odom), `cmd_vel_to_control`,
rosbridge and `ui_bridge` (the sensors and `ugv_serial_bridge` come from `car.launch.py`). The restartable part is
`launch/nav_stack.launch.py` (`map_server`/AMCL/`pose_gate` or `slam_toolbox` or `flat_origin`, the Nav2 servers, PD follower,
collision monitor, `dynamic_scan`, `safety_guard` and their lifecycle managers). `ui_bridge` runs it as a child process
(`ros2 launch car_nav nav_stack.launch.py mode:=... map:=...`), stops it (SIGINT to its process group, then SIGTERM / SIGKILL)
and starts it again on a switch. Meanwhile `ui_bridge` publishes zero `/cmd_vel` (and `cmd_vel_to_control` sends `0.000,0.000`
0.5 s after `/cmd_vel` stops anyway), and `/ui/state` carries `mode: {mode, map, state, detail}` with state
`idle | switching | starting | ready | error` (shown in the page header and the Map panel; the view shows a banner in
Create and flat mode). `starting` ends when `lifecycle_manager_navigation` (and in Load mode `..._localization`) report active.
A switch is refused (HTTP 409) while the wheel command is not 0, unless forced (the page asks). After a switch the guard
parameters and E-STOP last set in the page, and the driving sliders, are sent again.

**Stored maps** are `pgm + yaml + json` (note, default start pose) in `<data_dir>/maps/` (car `/data/nav/ui/maps`,
simulator `run_logs/sim/ui/maps`). A map saved with `map_saver_cli` can simply be copied in. **Save map as...** writes
the occupancy grid `ui_bridge` holds from `/map` itself (no extra service needed; unknown cells grey 205, `free_thresh 0.196`)
with the car's pose at the start of Create as the default start pose. Load: list, load, rename, note, start pose (type it,
"From current pose", or "Pick on map": load the map, drag where the car stands and faces), delete (not the loaded map).

**Safety guard in flat mode.** There is no map, so `dynamic_scan` passes every LiDAR point on (`fallback: all`): anything
in front of the car inside the guard zone, a wall included, counts as a person and stops / slows the car. Fine on an open
floor; do not use it to drive along walls. The page shows a "FLAT MODE" banner. The pose has no correction: it drifts like
the odometry (rf2o + learned speed).

On the car: `car.launch.py` with no map arguments starts sensors, odometry, motors and the page, and waits for you to pick a
mode. Old command lines still work: `map:=<yaml>` (Load), `slam:=true` (Create). New: `mode:=load map_name:=<stored name>`,
`mode:=create`, `mode:=flat`. `ui:=false` starts the stack once, without switching.
In the simulator: `scripts/sim/run_sim.sh ui` starts in Load on the world's map (stored as `<world>` on the first start with
the world's start as its start pose); `ui mode:=flat` / `ui mode:=create` start in the other modes; all three switch in the
page. Missions (`run_sim.sh mission ...`) are unchanged (`ui:=false`, Load).

REST (besides profiles and tests): `GET /api/maps`, `GET|PUT|DELETE /api/maps/<name>` (PUT body `note`, `start_pose`
`{x,y,yaw}` or `null`, `rename`), `POST /api/maps/<name>/load`, `POST /api/maps/save {name, note, overwrite}`,
`GET /api/mode`, `POST /api/mode {mode: load|create|flat|idle, map, force}`.

## Safety guard (people in front)

| Layer | Reads | Does |
|---|---|---|
| `safety_guard` | `/scan_dynamic` | **stop** while a point that is **not on the map** is in front of the car within `danger_range_m` (0.6 m past the nose, ±0.35 m); **slow** to 40 % within 1.5 m (±0.5 m); turning on the spot: stop for anything within 0.25 m in front. Only points past the nose and within ±`fov_deg`/2 (120°) count, so what is mounted behind the LiDAR, or someone behind the car, never stops it. Drives on by itself 0.5 s after the zone is clear. Stops if `/scan_dynamic` is older than 1 s. |
| collision monitor `VelocityStop` + `FootprintApproach` | `/scan_nav` (everything) | bumper 0.22 m past the nose, slow down so the footprint never reaches anything within 1.2 s |

`/scan_dynamic` comes from `car_nav/dynamic_scan.py`: the scan is first slid onto the
map walls (±0.45 m, ±7°) to absorb pose error, then the points more than 0.35 m from any
wall (within 2 m) are "not on the map". Without a map or a fresh pose it passes every
point (the car may then stop near walls: safe side). Something mounted on the car that the
LiDAR sees all the time should also go into `car.yaml self_mask.sectors_deg`.

Tested in the sim with moving people (`missions/person_*.yaml`, `car_nav_sim/actors.py`):
someone crossing at 1 m/s, someone stepping in and standing 5 s, someone running across
at 1.5 m/s when the car is 1 m away.

## Geometry (`ros/car_nav/config/car.yaml`)

`base_link` is the floor point at the **centre** of the car, midway between
the left and right wheels; the car turns about it and can turn on the spot.
The LiDAR is upright (`roll: 0`, the driver's `inverted` stays false).
Values left as `null` are not measured yet; the launch prints a warning and
uses 0 (or the `assumed_*` value). The file is mounted into the container,
so editing it on the Pi and restarting `car-calib-ros-nav` is enough. No
rebuild is needed.

Still to measure: `drive.track_width_m` (0.22 m assumed), `laser.x`,
`laser.yaw` (where the A1's 0° mark points), `camera.x`,
`speed.command_latency_s` (0.25 s assumed).

## Build and deploy (the Pi has no internet)

1. GitHub → Actions → **ros-nav image (arm64)** runs on every push that
   touches `docker/ros/`, `ros/` or the workflow (or start it by hand).
   Download the artifact `ros-nav-<sha>` and unzip it to get `ros-nav-<sha>.tar.gz`.
2. Free space on the Pi if needed (about 3× the .tar.gz size):
   `sudo docker builder prune -f` is safe. **Never remove**
   `car-calib-pi5:base` or `car-calib-pi5:latest`; the offline VISION build needs them.
3. From the school laptop (Git Bash), on this branch:
   ```bash
   ./deploy_pi5_remote.sh --ros-image ~/Downloads/ros-nav-<sha>.tar.gz
   ```
   The image is streamed into `docker load` over ssh; the tar is not stored
   on the SD card. Later deploys start `ros-nav` automatically while the
   image is on the Pi (`--profile nav`). Without the image, deploys are VISION only, as before.

## Checks (WO-01 gate)

Run on the Pi:

```bash
sudo docker exec car-calib-pi5 ls /dev/video*          # only /dev/video0 (rapoo)
sudo docker logs --tail 50 car-calib-ros-nav           # TF + "NOT MEASURED YET" warning, driver start
sudo docker exec -it car-calib-ros-nav bash -c 'source /entrypoint.sh true; \
  ros2 topic hz /scan'                                  # A1: about 5.5-10 Hz
sudo docker exec -it car-calib-ros-nav /entrypoint.sh ros2 topic hz /camera/depth/image_raw
sudo docker exec -it car-calib-ros-nav /entrypoint.sh ros2 topic hz /camera/color/image_raw
sudo docker exec -it car-calib-ros-nav /entrypoint.sh ros2 run car_nav scan_probe
#   put a box 0.5 m to the car's LEFT -> "... -> LEFT"; in front -> "AHEAD".
#   If LEFT/RIGHT or AHEAD/BEHIND are swapped, fix laser.yaw in car.yaml.
sudo docker stats --no-stream; vcgencmd get_throttled  # with VISION driving, wheels off the ground
```

`scan_self_mask` logs the masked bearings every 10 s; report them.

## Speed and stopping distance

The skid-steer car takes a speed per wheel, so Nav2 commands the speed
(cruise 0.4 m/s, `drive.cruise_speed_mps`) and slows down itself. Whether
the car really drives at the commanded speed is learned online by
`adaptive_odom` (ratio real/commanded). The old wall-approach logger
(`front_range_logger`, `scripts/analyze_wall_approach.py`) still works to
measure the stopping distance: drive straight at a wall at 0.4 m/s, send
0,0 at about 1.5 m, and compare with the 0.22 m bumper reach in
`nav2_params.yaml` (`collision_monitor.VelocityStop.forward`).
