#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source /opt/car_nav_ws/setup.bash
if [[ "${SIM_VIEW:-}" == "browser" ]]; then
  # Virtual screen + VNC + noVNC: open http://localhost:6080/vnc.html on the host.
  export DISPLAY=:99 QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1
  unset WAYLAND_DISPLAY
  Xvfb :99 -screen 0 1600x900x24 +extension GLX -nolisten tcp >/tmp/xvfb.log 2>&1 &
  for _ in $(seq 50); do [[ -e /tmp/.X11-unix/X99 ]] && break; sleep 0.1; done
  x11vnc -display :99 -forever -shared -nopw -localhost -quiet >/tmp/x11vnc.log 2>&1 &
  websockify --web /usr/share/novnc 6080 localhost:5900 >/tmp/novnc.log 2>&1 &
  echo "[view] RViz in the browser: http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
fi
exec "$@"
