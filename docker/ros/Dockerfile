# ros-nav image: ROS 2 Jazzy + sensor drivers for NAV2 mode (RPLIDAR A1 +
# Orbbec Astra Pro) + Nav2, slam_toolbox, rf2o and car_nav, for the
# skid-steer car. NO MQTT. The only thing that can move the car is
# car_nav's ugv_serial_bridge (car.launch.py), which writes the
# ugv-controller's "<left> <right>" lines to /dev/ugv; the default CMD
# (sensors.launch.py) does not start it.
#
# The Pi5 has no internet, so this image is built off the Pi (GitHub
# Actions arm64 runner, .github/workflows/ros-nav-image.yml) and streamed
# onto the Pi with `deploy_pi5_remote.sh --ros-image <file.tar.gz>`.
#
# Local build (any arch with internet):
#   docker build -f docker/ros/Dockerfile -t car-calib-ros-nav:latest .

ARG ROS_DISTRO=jazzy

# --------------------------------------------------------------------- #
# Builder: drivers from source (pinned) + our car_nav package
# --------------------------------------------------------------------- #
FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO
ARG LIBUVC_REF=v0.0.7
ARG SLLIDAR_REF=34300099fadfc772965962dec837bf436706188f
ARG RF2O_REF=b38c68e46387b98845ecbfeb6660292f967a00d3
ARG ASTRA_REF=f7e71d9ce806e788cb48d8580aac2c778fba4214
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake pkg-config libusb-1.0-0-dev libjpeg-dev \
        libgflags-dev libgoogle-glog-dev libeigen3-dev nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# libuvc (Astra Pro colour stream is read over libuvc by astra_camera).
# astra_camera calls uvc_mjpeg2rgb, which libuvc only builds when it finds
# libjpeg, so fail here if JPEG support did not get compiled in.
RUN git clone https://github.com/libuvc/libuvc.git /tmp/libuvc \
    && cd /tmp/libuvc && git checkout ${LIBUVC_REF} \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build build -j"$(nproc)" && cmake --install build \
    && grep -q "^#define LIBUVC_HAS_JPEG" /usr/local/include/libuvc/libuvc_config.h \
    && rm -rf /tmp/libuvc

WORKDIR /ws/src
RUN git clone https://github.com/Slamtec/sllidar_ros2.git \
    && git -C sllidar_ros2 checkout ${SLLIDAR_REF} \
    && git clone https://github.com/orbbec/ros2_astra_camera.git \
    && git -C ros2_astra_camera checkout ${ASTRA_REF} \
    && git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git \
    && git -C rf2o_laser_odometry checkout ${RF2O_REF} \
    && grep -rl 'tf2_geometry_msgs/tf2_geometry_msgs.h"' rf2o_laser_odometry \
       | xargs -r sed -i 's#tf2_geometry_msgs/tf2_geometry_msgs.h"#tf2_geometry_msgs/tf2_geometry_msgs.hpp"#'
# rf2o's package.xml names the ROS 1 key cmake_modules (unused): skipped below.
COPY ros/car_nav /ws/src/car_nav

RUN apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /ws/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
         --skip-keys "cmake_modules" \
    && apt-get install -y --no-install-recommends libboost-dev \
    && rm -rf /var/lib/apt/lists/*

COPY docker/ros/patch_astra_for_jazzy.sh /tmp/patch_astra_for_jazzy.sh
RUN bash /tmp/patch_astra_for_jazzy.sh /ws/src/ros2_astra_camera

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws && colcon build --merge-install --install-base /opt/car_nav_ws \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# --------------------------------------------------------------------- #
# Runtime
# --------------------------------------------------------------------- #
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

COPY --from=builder /usr/local/ /usr/local/
COPY --from=builder /ws/src /tmp/src
RUN apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src --ignore-src -y -t exec --rosdistro ${ROS_DISTRO} \
         --skip-keys "cmake_modules" \
    && apt-get install -y --no-install-recommends libusb-1.0-0 libjpeg8 libgoogle-glog0v6t64 libgflags2.2 \
        ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-slam-toolbox python3-serial \
        ros-${ROS_DISTRO}-rosbridge-server \
        ros-${ROS_DISTRO}-rosbag2 ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/* /tmp/src /root/.ros/rosdep \
    && ldconfig

COPY --from=builder /opt/car_nav_ws /opt/car_nav_ws
COPY docker/ros/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /data/nav

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "car_nav", "sensors.launch.py"]
