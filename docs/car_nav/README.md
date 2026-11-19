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

This branch targets the **skid-steer (differential drive) car**: body
**0.70 m long x 0.60 m wide** (owner's measurement, 2026-11-19), LiDAR
mounted the normal way up. Nav2 sends
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

`nav.launch.py ui:=true` (default) starts rosbridge and `ui_bridge`. Open `http://<mini-PC IP>:8080`. The page is black / white /
grey; colour is only used for safety (guard STOP / ESTOP state, the E-STOP button, the points that count as people / obstacles).
The top bar (connection, map mode, guard state, **E-STOP**) is always visible. The side panel has one open tab at a time:

| Tab | What it holds |
|---|---|
| **Map** | Load / Create / Unknown-flat mode, stored maps, tools: pan / zoom, **initial pose** (click + drag, `/initialpose`), **nav goal** (click + drag, `/goal_pose`), follow car, "Go ahead" in flat mode |
| **Drive** | cruise speed, max turn rate, slow-down before curves (PD follower, act from the next goal); settings **profiles** (guard + driving values, saved in `/data/nav/ui/profiles/`: save, load + apply, use at start, delete; HTTP `GET/PUT/DELETE /api/profiles/<name>`) |
| **Safety guard** | state, nearest person, stop / slow distances, widths, field of view, slow speed, resume delay, depth camera section (**Apply** sends them live; not saved: use a profile or copy them to `nav2_params.yaml safety_guard:`) |
| **Car** | car properties, drawings, car profiles, Apply to the car (below) |
| **Health** | checks every device, one run at a time, saved in `/data/nav/ui/health/` (below); the top bar shows "last health check: PASS/FAIL at <time>" |
| **Tests** | name, *Start recording*, drive / walk in front of the car, *Stop + save* (parameters, every event, 5 Hz trace). Verdict + note, **Download** (JSON), delete. Saved in `/data/nav/ui/tests/` |
| **Manual** | drive by hand: on-screen stick (touch / mouse) + W A S D / arrows, speed and turn-rate limits (default 0.2 m/s, max `drive.cruise_speed_mps`), STOP. Top bar shows **MANUAL** |
| **Runs** | *Record run* / *Stop + save* (rosbag2 of the important topics), list with size, note, **Download (.zip)**, delete, free-disk guard |
| **Logs** | pipeline table (Hz, age, last value of every stage from `/scan` to `/car/wheels/applied`) and a `/rosout` console (level / node filter, pause, last 500 lines, download) |

**E-STOP**: the guard sends zero until you press RELEASE STOP. A software stop over Wi-Fi; it does not replace the motor power switch.
The ports are open to the whole network the mini-PC is on: use it on the lab's own Wi-Fi.
In the simulator: `scripts/sim/run_sim.sh ui` (or `ui mission:=person_stops`), then open `http://localhost:8080`.

### Why the map showed nothing (fixed)

`ui_bridge` looked up every pose and LiDAR / depth point in the **map** frame only. `car.launch.py` without a mode starts in
**idle** (no AMCL / SLAM / flat origin, so no `map -> odom` TF), and Load mode has none either until the initial pose is set: every
lookup failed, so the page got `pose: null` and empty scan / depth lists and drew no car, no points, no path. Also the page never
subscribed to depth when the guard's `depth_enabled` was false (the default on the car), and `depth_obstacles` does not run then.
Now: while `map -> odom` is missing the bridge sends the pose and points in the **odom** frame (`frame: "odom"` in `/ui/pose`,
`/ui/scan`, `/ui/depth`); the page draws them on the blank grid around the car start (0, 0), fades the map and shows
"NOT LOCALISED / NO MAP MODE". When no `/depth_obstacles` publisher exists the bridge shows the camera's own cloud (same height
filter). The plan is drawn only in the map frame.

### Manual drive, runs, logs

* **Manual drive.** The page publishes `geometry_msgs/Twist` on `/cmd_vel_teleop` at 20 Hz through rosbridge. `teleop_mux`
  (`teleop_logic.py`, in the restartable stack) sits in front of the collision monitor: `/cmd_vel_nav` (Nav2 / PD follower) or
  `/cmd_vel_teleop` -> `/cmd_vel_mux` -> collision monitor -> safety guard -> `/cmd_vel`, so people and walls still stop the car.
  No command for 0.3 s -> zero (page closed, Wi-Fi lost, stick released, tab hidden: the page also stops). While manual is in force
  Nav2's output is dropped and the active goal is cancelled; a new goal (RViz, page, mission) ends manual control; E-STOP overrides.
  Speed is clipped to `drive.cruise_speed_mps`, turn to `drive.max_yaw_rate_rps`; backward input is ignored unless
  `drive.allow_reverse`. It needs a running map mode (the guard and monitor are part of it). State: `/teleop_mux/state`.
* **Runs.** `POST /api/runs/start` runs `ros2 bag record` (mcap if installed, else sqlite3) into `<data_dir>/runs/<id>/bag` with
  `/scan_nav /odom /tf /tf_static /amcl_pose /plan /cmd_vel* /car/wheels/* /safety_guard/state /depth_obstacles /imu/data_raw /rosout`
  (+ teleop state, goal, initial pose, map). Refused below 1 GB free disk; stopped when free space falls below that or after 1 h.
  A *test record* is a light JSON summary; a *run* is the full bag; a test started during a run stores the run id (and the run the test id).
  Replay: unzip, `ros2 bag play <id>/bag`. After a power cut the run is marked `interrupted` (`ros2 bag reindex`).
* **Logs.** `/ui/pipe` (2 Hz) and `/ui/logs` (/rosout, 2 Hz) are packed and subscribed only while that tab is open.

### Work only when needed

* **Topics.** `ui_bridge` publishes small topics instead of one big state: `/ui/map` (3 s and when a page subscribes), `/ui/pose`,
  `/ui/scan`, `/ui/depth`, `/ui/guard` (5 Hz), `/ui/plan` (2 Hz), `/ui/mode` (1 Hz: map mode, driving values, test state, guard
  state for the top bar, car apply state). A topic is packed and published **only while it has a subscriber**
  (`get_subscription_count()`); the LiDAR / depth / plan subscriptions of `ui_bridge` itself exist only while a page needs them (or
  while a test records); its hold-stop and test-sampling timers run only when needed. No browser connected = nothing is packed.
* **The page** subscribes only to what the open tab shows: Map: map, pose, scan, depth (if used), guard, plan; Drive: map, pose,
  plan; Safety guard: map, pose, scan, depth, guard; Car and Tests: only `/ui/mode`. A hidden browser tab unsubscribes from everything.
  The map canvas is redrawn only when data, the view or the size changed (no animation loop). The Car tab's drawings are drawn
  only while that tab is open and only when a value changes.
* **Idle pause** (`idle_logic.py`, `idle_watch.py`). The car is *idle* when no goal is active and nothing non-zero has been
  commanded on `/cmd_vel_guarded` / `/cmd_vel` for 10 s (`idle_after_s`). While idle: `dynamic_scan` aligns and classifies one scan
  per 0.4 s instead of every scan; the depth node (if on) processes one cloud per 0.4 s; `ui_bridge` sends scan / depth / plan at
  2 Hz instead of 5 Hz. **Localization is never paused** (AMCL, `pose_gate`, odometry, `map_server`, Nav2 keep running).
  It ends at once on a non-zero command, a goal becoming active (`/navigate_to_pose/_action/status`) or an E-STOP change. The guard
  stays safe across a resume: on a **start from rest** (no non-zero command for 0.6 s) `safety_guard` passes nothing (sends 0,
  `wake_holds` in its state) until it has received a scan (and a depth cloud when depth is in use) that arrived **after** that
  command (`idle_logic.resume_gate`), so a start is never judged on data from before the pause. It costs at most one scan
  period (~0.15 s) per start. Stale data still stops the car as before (`no_scan` after 1 s). Parameters (`dynamic_scan`,
  `depth_obstacles`, `ui_bridge`): `idle_pause` (true), `idle_after_s` (10), `idle_period_s` (0.4 / 0.5).

### Car tab (car properties)

Edit, for the active `car.yaml`: front / rear edge and half width (the body), body height (optional), track width (optional), LiDAR
x / y / z / roll / pitch / yaw (180 deg roll = upside down), camera x / y / z / pitch / yaw, cruise and max wheel speed. Two line
drawings follow the values while you type: **top view** (body, wheels at the track width, LiDAR with its 0 deg direction, camera with
its field of view, the guard's stop / slow zones from the current guard values, the spin circle = half diagonal) and **side view**
(body, LiDAR scan plane height, camera with the 45 deg vertical field of view and its pitch, the floor point of the lowest ray;
the "camera blind distance" in the text is `depth_obstacles.blind_distance()`, asked from the server).

*Car profiles* (`<data_dir>/cars/<name>.json`, `/data/nav/ui/cars/` on the car): load into the form, save as, rename, delete.
REST: `GET /api/car`, `POST /api/car/derived`, `POST /api/car/apply {props | name, force}`, `DELETE /api/car/active`, `GET /api/cars`,
`GET|PUT|DELETE /api/cars/<name>`, `POST /api/cars/<name>/rename {to}`, `POST /api/cars/<name>/apply`.

**Where car.yaml is.** The shipped file stays read-only (`docker-compose.pi5.yml`: `./ros/car_nav/config/car.yaml:/config/car.yaml:ro`,
`CAR_NAV_CONFIG`). The Car tab writes a **writable copy** on the data volume, `CAR_NAV_CONFIG_OVERRIDE=/data/nav/ui/car.yaml` (simulator:
`/data/sim/ui/car.yaml` = `run_logs/sim/ui/car.yaml`, set by `run_sim.sh ui` only, so missions never see it). `config_loader.load()` reads, in
this order: the override if it exists, `CAR_NAV_CONFIG`, the package copy. While the override exists, editing the shipped file (or a
deploy that changes it) has no effect: use **Back to shipped file** (`DELETE /api/car/active`) or delete the override.
The write patches only the VALUES in the text of the file it replaces, so all comments stay; the previous file is kept next to it as
`car.yaml.<YYYYmmdd-HHMMSS>.bak` (10 newest kept); the new file is written atomically.

**Apply to the car** (`POST /api/car/apply`) is refused with **HTTP 409** unless the car is stopped (wheel command zero: no
`/car/wheels/cmd` above 0.03 m/s in the last second, `nav_modes.wheels_moving`) and no goal / mission is active; also while the stack
is already restarting, in Create mode while a map is being built (unless `force`: the map would be lost) and when the file is read-only
(`car_store.apply_blocker`). Then:

1. validate (ranges, front ahead of rear, cruise <= max wheel speed; 400 otherwise), write the file (above);
2. `ui_bridge` publishes `/car_config/reload`; these nodes **re-read car.yaml in place**, without restarting: `car_tf_publisher`
   (static TFs `base_link -> laser`, `laser_upright`, `camera_link`; it replaced the three `static_transform_publisher` nodes of
   `sensors.launch.py`, `nav.launch.py` and `sim.launch.py`), `scan_upright` (upside-down flag), `scan_self_mask` (laser pose, footprint, margin,
   sectors), `cmd_vel_to_control` (track width, wheel / yaw limits), `adaptive_odom` (laser yaw, drive geometry; the learned speed
   ratio and the odometry pose stay), `ui_bridge` itself and, in the simulator, `lite_sim` (simulated LiDAR / camera mount and drive);
3. `ui_bridge` sets the new cruise speed on `pd_path_follower` and **restarts the nav stack** (`nav_stack.launch.py`, the same
   restart as a map-mode switch): `safety_guard` (footprint, laser pose), `pd_path_follower` (footprint, laser pose, track), `dynamic_scan`,
   `depth_obstacles` (camera pose), Nav2, AMCL / `pose_gate` / `map_server` or `slam_toolbox` or `flat_origin`. The wheel command is held
   at 0 meanwhile and the guard parameters and E-STOP are re-sent after the restart, as for a map-mode switch. In **Load** mode the car's
   pose before the restart is sent to AMCL as the initial pose (the car did not move); in **Flat** mode the origin restarts at the
   current pose (goals are relative to it); **Create** mode loses the unsaved map (hence the `force`).

`rf2o` reads the `laser_upright -> base_link` TF once, at its first scan, so when the Apply moved the LiDAR (x, y or yaw changed) `ui_bridge`
stops the `rf2o_laser_odometry_node` process (SIGTERM, found in /proc) and `nav.launch.py` starts it again (`respawn=True`, 1 s);
`adaptive_odom` drops its previous rf2o sample on the reload, and the car is stopped, so the restart does not move the pose.

**Not restarted** (they do not read the Car tab's values): `sllidar_node` / Astra drivers, `ugv_serial_bridge` (reads `ugv.*`,
which the tab does not edit: restart the container after changing those), rosbridge. The Nav2 costmap footprint and inflation values
are in `nav2_params.yaml` (not generated from `car.yaml`): after changing the body size also change `footprint` there
(`tests/test_car_nav_nav2_params.py` checks the two agree). The simulator's truth model (`sim.yaml truth:`) and the Gazebo world
are not changed by Apply.

### Health tab (device checks)

Rule of the owner: **a check the observer has not confirmed counts as FAILED**. A run is PASS only when every check that ran passed;
a sensor that is missing (the simulator has no colour camera and no IMU) is a FAIL with the reason. Code: rules and thresholds
`ros/car_nav/car_nav/health_checks.py` (unit tested), measurements `health_runner.py` (inside `ui_bridge`), page `web/health.js`.

| Check | Measures | PASS rule |
|---|---|---|
| **LiDAR** | `/scan` for 4.5 s | rate 5.5 .. 10.5 Hz; >= 25 % beams valid; valid ranges 0.05 m .. range_max; <= 2 valid beams inside the footprint + margin outside `self_mask.sectors_deg` (the car's own body is only allowed where it is masked) |
| **Depth camera** | `/camera/depth/points` 4.5 s, RANSAC floor plane in base_link with the `car.yaml` camera pose | rate >= 5 Hz; >= 300 valid points per frame (organised cloud: >= 20 % valid pixels); floor plane (>= 150 points) within 6 cm of z = 0 and tilted < 3 deg along x and y. The result also shows which camera z / pitch would make the floor level (hint) |
| **Colour camera** | image topic rate (`health_image_topic`, default `/camera/color/image_raw`) + a JPEG snapshot shown in the page | rate >= 5 Hz **and** the observer clicks "Looks right" (no answer in 60 s = FAIL) |
| **IMU** | `/imu/data_raw` 4.5 s, car standing still | rate 7 .. 13 Hz; mean |acceleration| 9.81 +- 0.8 m/s^2; |mean gyro| <= 0.08 rad/s; each gyro axis std <= 0.05 rad/s |
| **Wheels test** (moves the wheels) | six pulses of 0.15 m/s for 1.5 s: left forward, left back, right forward, right back, both forward, spin left. Each step is announced 3 s before (what to expect), then asks "Did it? yes / no" | per step: observer "yes" **and** `/car/wheels/applied` echoed the command within 2 units of what `ugv_link` sends (proves the ESP32 got it). "No" or no answer in 30 s = FAIL and the test stops there |
| **Speed test** (drives) | forward at 0.2 m/s for 4.5 s (clamped 0.05 .. 0.3 m/s, 2.5 .. 6 s), then a spin at 0.5 rad/s for 3 s. Real speed from the LiDAR (the wall ahead when seen at both ends, else `/odom_rf2o`) over the steady part (after 1.5 s); optional tape distance in the page scales it | moved 50 .. 160 % of commanded; heading change <= 6 deg; the guard never stopped the car; tape within 20 % of the LiDAR total when typed; spin has room and `yaw_efficiency` 0.2 .. 1.4; observer "yes" |

The speed test proposes `ugv.units_per_mps = now / (real speed / commanded speed)` and shows `yaw_efficiency` (real yaw rate / commanded;
information only). **Apply to car.yaml** (only for a PASSED test, only while the car is stopped: no goal, wheel command 0, no run)
writes `ugv.units_per_mps` to the writable car.yaml (timestamped backup, `car_store.ActiveCar.write_updates`) and announces it on
`/car_config/reload`; `ugv_serial_bridge` re-reads it at once, nothing is restarted.

**Safety of the actuator tests.** Refused (HTTP 409) while a navigation goal is active, the E-STOP is on, the wheels are being driven,
the nav stack is switching, or without a heartbeat. Before Start the operator ticks: E-STOP within reach; for the wheels test "wheels off
the ground" or "clear space" (without "off the ground" the LiDAR must show nothing within 0.5 m of the footprint); for the speed test
"2 m clear ahead" (the LiDAR must show at least max(1.0, v*t + 0.7) m free ahead of the nose, and room for the spin radius + 0.15 m).
While a test runs the page sends a heartbeat (`/ui/health_hb` over rosbridge every 200 ms, only while the Health tab is open and visible;
`POST /api/health/heartbeat` does the same); **no heartbeat for 0.5 s, E-STOP, a goal sent, Abort, closing the page or the WebSocket ->
the wheels are zeroed and the run ends as FAILED**. A run always ends with zero wheels (5 stop messages).

| Test | Path to the wheels | Guards |
|---|---|---|
| Wheels | `/health/wheels_override` -> `cmd_vel_to_control` -> `/car/wheels/cmd` | **bypassed on purpose**: one wheel alone or backwards cannot be a `/cmd_vel` (`allow_reverse` is off). Limits: 0.15 m/s per wheel (`cmd_vel_to_control` clamps every override to 0.25 m/s), 1.5 s pulses, the override expires 0.3 s after the last message, ignored while the E-STOP is on or a non-zero `/cmd_vel` (navigation) is fresh |
| Speed | `/cmd_vel_nav` -> collision monitor -> safety guard -> `/cmd_vel` -> `cmd_vel_to_control` | both apply (same path as a Nav2 goal). Needs a running map mode (flat / load / create, state ready); forward only |

REST (module docstring of `ui_bridge.py`): `GET /api/health`, `POST /api/health/start|answer|heartbeat|abort|apply_speed`,
`GET /api/health/snapshot`, `GET|DELETE /api/health/runs/<id>[?download=1]`. Optional: `car.launch.py require_health:=true`
(ui_bridge parameter `require_health`, default false) cancels NAV goals (and sends zero) until a run that covered EVERY check PASSED today (a partial run shows "(partial)" in the banner and does not count).
Simulator: lite sim publishes a stand-in for the ESP32 echo on `/car/wheels/applied`; it has no colour camera and no IMU, so those two checks FAIL there.

## Map modes (Load / Create / Unknown-flat)

The web page's **Map** tab (and `POST /api/mode`) switches between three ways of knowing where the car is:

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
0.5 s after `/cmd_vel` stops anyway), and `/ui/mode` carries `mode: {mode, map, state, detail}` with state
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
| `safety_guard` | `/scan_dynamic` | **stop** while a point that is **not on the map** is in front of the car within `danger_range_m` (0.6 m past the nose, ±0.45 m); **slow** to 40 % within 1.5 m (±0.65 m); turning on the spot: stop for anything within 0.25 m in front. Only points past the nose and within ±`fov_deg`/2 (120°) count, so what is mounted behind the LiDAR, or someone behind the car, never stops it. Drives on by itself 0.5 s after the zone is clear. Stops if `/scan_dynamic` is older than 1 s. |
| collision monitor `VelocityStop` + `FootprintApproach` | `/scan_nav` (everything) | bumper 0.22 m past the nose (x 0.57 from the centre), slow down so the footprint never reaches anything within 1.2 s |

`/scan_dynamic` comes from `car_nav/dynamic_scan.py`: the scan is first slid onto the
map walls (±0.45 m, ±7°) to absorb pose error, then the points more than 0.35 m from any
wall (within 2 m) are "not on the map". Without a map or a fresh pose it passes every
point (the car may then stop near walls: safe side). Something mounted on the car that the
LiDAR sees all the time should also go into `car.yaml self_mask.sectors_deg`.

### Depth camera as a second source (low boxes, table tops)

The LiDAR plane is low, so objects under it (boxes, steps) or hanging over it (table tops) are
invisible to the guard and the bumper. With `depth:=true` (`car.launch.py` also starts the Astra,
`nav.launch.py`, `sim.launch.py`; default **false** until tested on the car) the guard also uses the depth camera:

* `depth_guard_source` (`car_nav/depth_guard_source.py`, pure math in `depth_obstacles.py`):
  `/camera/depth/points` (PointCloud2, optical frame) -> camera pose from `car.yaml camera:` ->
  `base_link` -> keep `depth_min_height_m` (floor, 0.05) < z <= `depth_max_height_m` (0.8) and range
  <= `depth_max_range_m` (3.0) -> 5 cm grid downsample -> drop points on map walls (`map_filter`)
  -> **`/depth_obstacles`** (PointCloud2 in `base_link`, x/y = position, z = 0, one message per frame,
  empty when clear) and a JSON status on `/depth_obstacles/state`. No TF waiting; a tilt margin of
  3 cm per metre keeps the far floor out (`floor_margin_per_m`).
* `safety_guard` judges those points with the same front zones; the stop distance is
  `depth_danger_range_m` (1.1 m past the nose) because the camera has a **blind zone**: it looks 15 deg
  down with a 45 deg field of view, so a low box leaves its view ~1.1-1.4 m from the camera
  (`depth_obstacles.blind_distance()`); the car has to stop while it is still visible. Re-check this
  number after changing the camera mount in `car.yaml`.
* Stale depth (older than `depth_timeout_s`, or not started yet) never stops the car by itself: the
  guard goes on with the LiDAR only, state `depth_stale` (when otherwise clear) and `depth_status: stale` in
  `/safety_guard/state`, plus a log warning. `depth_required: true` stops the car instead (`no_depth`).
* Depth off (`depth_enabled: false`) = exactly the LiDAR guard as before.
* All `depth_*` values are in `nav2_params.yaml safety_guard:` and live in the web UI (own section: on/off,
  required, stop distance, floor / ceiling height, range, stale time; red outlined squares = depth obstacles on the
  map); they are part of the saved profiles. `depth_obstacles:` holds the processing details.
* Not done: the obstacle is not fed to Nav2's costmaps, so the car stops in front of it and waits.

Tested in the sim with moving people (`missions/person_*.yaml`, `car_nav_sim/actors.py`):
someone crossing at 1 m/s, someone stepping in and standing 5 s, someone running across
at 1.5 m/s when the car is 1 m away.

## Geometry (`ros/car_nav/config/car.yaml`)

`base_link` is the floor point at the **centre** of the car, midway between
the left and right wheels; the car turns about it and can turn on the spot.
The LiDAR is upright (`roll: 0`, the driver's `inverted` stays false).

| Measured by the owner (2026-11-19) | Value |
|---|---|
| Body | 0.70 m long x 0.60 m wide (`footprint`: x_front 0.35, x_rear -0.35, half_width 0.30) |
| LiDAR height | 0.24 m (`laser.z`); it only sees obstacles taller than this |
| Camera | at the car centre (x = y = 0), 1.20 m high (`camera.z`) |

Assumed, not measured: LiDAR along the car (`laser.x: 0`, centre), camera
pitch (15° down), track width (`assumed_track_width_m` 0.52 m).

Turning on the spot the corners sweep a circle of radius
sqrt(0.35² + 0.30²) = **0.46 m**; the PD follower only turns on the spot
while no scan point is within 0.46 + 0.04 m of the centre (it creeps on at
the minimum speed or waits otherwise). A car centred in a 1.2 m corridor
has 0.30 m to each wall and 0.14 m to spare for the turn.
Values left as `null` are not measured yet; the launch prints a warning and
uses 0 (or the `assumed_*` value). The file is mounted into the container,
so editing it on the Pi and restarting `car-calib-ros-nav` is enough. No
rebuild is needed (nor for any other file of `ros/car_nav`, see
"Dev loop without rebuilding the image").

Still to measure: `drive.track_width_m` (0.52 m assumed), `laser.x`
(0 assumed), `laser.yaw` (where the A1's 0° mark points), camera pitch (15°
assumed), `speed.command_latency_s` (0.25 s assumed), and the body height
(does anything of the car reach the LiDAR plane at 0.24 m?).

## Dev loop without rebuilding the image

The image holds only the heavy, rarely changing parts (ROS, Nav2, drivers,
rf2o). `car_nav` itself comes from the repo: `docker-compose.pi5.yml` mounts
`./ros/car_nav` at `/ws/src/car_nav` (read-only), and at container start the
entrypoint (`docker/common/car_nav_overlay.sh`) runs a fast
`colcon build` of that package (pure Python, a few seconds, no internet) into
`/data/nav/.overlay/builds/<hash>` and puts it in front of the copy baked into
the image. So **editing any file in `ros/car_nav` and restarting the container
is enough**, including a new console script (`setup.py`), launch file, config,
web page or behaviour tree.

* Rebuilt only when the content of `ros/car_nav` changed (hash); an unchanged
  restart prints `[overlay] up to date` and costs nothing.
* A failed build (syntax error in `setup.py`, ...) never takes the car down:
  the log is `/data/nav/.overlay/last_build_error.log` (and the container log),
  and the last good build keeps running.
* No mounted source (folder missing) -> the code baked into the image runs.
  `CAR_NAV_OVERLAY=0` in `.env` forces the baked code.
* The simulator does the same for `ros/car_nav` and `ros/car_nav_sim`
  (overlay in `run_logs/sim/.overlay`).

| You changed | Needed |
|---|---|
| Python node, launch file, config, web page, behaviour tree in `ros/car_nav` | restart the container |
| New console script, launch file or data file (`setup.py`, `package.xml` file list) | restart the container |
| `ros/car_nav/config/car.yaml` | restart (it is also mounted directly) |
| `docker/ros/Dockerfile`, apt packages, a new Python/ROS dependency, drivers (rf2o, sllidar, astra), a new package other than `car_nav` | rebuild the image (needs internet: laptop or GitHub Actions) and `--ros-image` it |
| `docker/ros/entrypoint.sh`, `docker/common/car_nav_overlay.sh` | rebuild the image (they are copied into it) |
| `docker-compose.pi5.yml` (devices, volumes, env) | `up -d` again (recreate), no image rebuild |

On the car (mini-PC), after editing files in the repo checkout it runs from:

```bash
sudo docker compose --profile nav -f docker-compose.pi5.yml restart ros-nav
sudo docker logs --tail 20 car-calib-ros-nav        # shows "[overlay] build ok" or the build error
```

Pushing code from the laptop: `./deploy_pi5_remote.sh --no-build` uploads the
repo as a new release and recreates the containers; `ros-nav` is not built
from source on the car (it only uses the loaded image), so no image rebuild
happens. Only a fast one-folder sync of `ros/car_nav` into the running release
is also possible:

```bash
tar -C ros -cz car_nav | ssh <car> 'sudo tar -xz -C /opt/car-calib-pi5/current/ros'
ssh <car> 'cd /opt/car-calib-pi5/current && sudo docker compose --profile nav -f docker-compose.pi5.yml restart ros-nav'
```

(`restart` keeps the container's mount, which points at that release folder;
after a deploy the container is recreated and points at the new release.)

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
