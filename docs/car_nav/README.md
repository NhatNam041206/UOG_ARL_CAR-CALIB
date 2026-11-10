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

## Geometry (`ros/car_nav/config/car.yaml`)

`base_link` is the floor point under the centre of the **front** (fixed)
axle; the car steers with its rear wheels. The LiDAR is upside down: this is
handled only by `roll = pi` in its transform, and the driver's `inverted` stays false.
Values left as `null` are not measured yet; the launch prints a warning and
uses 0. The file is mounted into the container, so editing it on the Pi
(`/opt/car-calib-pi5/current/ros/car_nav/config/car.yaml`) and restarting
`car-calib-ros-nav` is enough. No rebuild is needed.

Still to measure with a tape: `laser.x`, `laser.yaw` (where the A1's 0° mark
points), `camera.x`.

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

## Baseline speed and stopping distance

The car reports no speed and accepts no speed value (drive_code is only
STOP / FORWARD). NAV2 therefore starts from a **measured baseline** in
`car.yaml` (`speed.baseline_speed_mps`, conservative 0.5 m/s until measured),
and an adaptive estimate from LiDAR odometry comes later.

Measure it with the LiDAR (power switch within reach, nobody near the car):

1. Put the car about 4 m from a flat wall, pointing at it.
2. Start the logger:
   ```bash
   sudo docker exec -it car-calib-ros-nav /entrypoint.sh \
     ros2 run car_nav front_range_logger --ros-args -p run_name:=wall1
   ```
3. Drive straight at the wall in VISION (dashboard joystick or a `forward`
   script), send STOP at about 1.5 m. Note the STOP time if you can
   (VISION's run log has the command timestamps; same clock).
4. Ctrl-C the logger. Repeat for `wall2` … `wall5`.
5. Copy `/opt/car-calib-pi5/data/nav/wall_runs/*.csv` to the laptop, then:
   ```bash
   python scripts/analyze_wall_approach.py wall*.csv [--stop-time t1 t2 t3 t4 t5]
   ```
6. Put the mean into `car.yaml`: `baseline_speed_mps`, `stopping_distance_m`,
   `measured: true`.
