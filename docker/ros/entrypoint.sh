#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/car_nav_ws/setup.bash
# Rebuild-free dev loop: if the repo's ros/car_nav is mounted at /ws/src/car_nav, build it into an
# overlay (seconds, only when it changed) and use it instead of the baked copy. See docs/car_nav/README.md.
source /opt/car_nav_overlay.sh
car_nav_overlay || echo "[overlay] failed unexpectedly: using the code baked into the image" >&2
exec "$@"
