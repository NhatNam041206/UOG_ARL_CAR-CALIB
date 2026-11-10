#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/car_nav_ws/setup.bash
exec "$@"
