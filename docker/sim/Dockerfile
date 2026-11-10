# NAV2 simulator image for a LAPTOP (amd64 or arm64), not for the Pi.
# ROS 2 Jazzy + Gazebo Harmonic + Nav2 + slam_toolbox + rf2o + car_nav +
# car_nav_sim. Same car_nav configs as the car (ros/car_nav/config/*).
#
#   docker compose -f docker-compose.sim.yml build
#   ./scripts/sim/run_sim.sh mission loop_left        # headless, prints PASS/FAIL
#
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop
ARG ROS_DISTRO
ARG RF2O_REF=b38c68e46387b98845ecbfeb6660292f967a00d3
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-simple-commander \
        ros-${ROS_DISTRO}-nav2-rviz-plugins \
        ros-${ROS_DISTRO}-slam-toolbox \
        ros-${ROS_DISTRO}-ros-gz \
        python3-numpy python3-yaml git build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src
# rf2o laser odometry (same pinned commit WO-03 will use on the car).
# Jazzy ships tf2_geometry_msgs.hpp; the .h shim is deprecated.
RUN git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git \
    && git -C rf2o_laser_odometry checkout ${RF2O_REF} \
    && grep -rl 'tf2_geometry_msgs/tf2_geometry_msgs.h"' rf2o_laser_odometry \
       | xargs -r sed -i 's#tf2_geometry_msgs/tf2_geometry_msgs.h"#tf2_geometry_msgs/tf2_geometry_msgs.hpp"#'
COPY ros/car_nav /ws/src/car_nav
COPY ros/car_nav_sim /ws/src/car_nav_sim

# car_nav lists the car's sensor drivers; the simulator does not need them.
RUN apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /ws/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
         --skip-keys "sllidar_ros2 astra_camera" \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd /ws && colcon build --merge-install --install-base /opt/car_nav_ws \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf /ws/build /ws/log

COPY docker/sim/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /data/sim
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "car_nav_sim", "sim.launch.py", "mission:=loop_left"]
