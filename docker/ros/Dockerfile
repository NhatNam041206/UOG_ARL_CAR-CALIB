# ros-nav image: ROS 2 Jazzy + sensor drivers for NAV2 mode (RPLIDAR A1 +
# Orbbec Astra Pro). WO-01 scope: sensors only, no Nav2 yet, and NO MQTT:
# nothing in this image can send a command to the car.
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
ARG ASTRA_REF=f7e71d9ce806e788cb48d8580aac2c778fba4214
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential cmake pkg-config libusb-1.0-0-dev \
        libgflags-dev libgoogle-glog-dev libeigen3-dev nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# libuvc (Astra Pro colour stream is read over libuvc by astra_camera)
RUN git clone https://github.com/libuvc/libuvc.git /tmp/libuvc \
    && cd /tmp/libuvc && git checkout ${LIBUVC_REF} \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build build -j"$(nproc)" && cmake --install build \
    && rm -rf /tmp/libuvc

WORKDIR /ws/src
RUN git clone https://github.com/Slamtec/sllidar_ros2.git \
    && git -C sllidar_ros2 checkout ${SLLIDAR_REF} \
    && git clone https://github.com/orbbec/ros2_astra_camera.git \
    && git -C ros2_astra_camera checkout ${ASTRA_REF}
COPY ros/car_nav /ws/src/car_nav

RUN apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /ws/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

COPY docker/ros/patch_jazzy_headers.sh /tmp/patch_jazzy_headers.sh
RUN bash /tmp/patch_jazzy_headers.sh /ws/src/ros2_astra_camera

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
    && apt-get install -y --no-install-recommends libusb-1.0-0 libgoogle-glog0v6t64 libgflags2.2 \
    && rm -rf /var/lib/apt/lists/* /tmp/src /root/.ros/rosdep \
    && ldconfig

COPY --from=builder /opt/car_nav_ws /opt/car_nav_ws
COPY docker/ros/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /data/nav

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "car_nav", "sensors.launch.py"]
