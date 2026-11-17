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

1. Build the image with Nav2 (GitHub Actions **ros-nav image (arm64)**, or
   `docker build -f docker/ros/Dockerfile -t car-calib-ros-nav:latest .`).
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
