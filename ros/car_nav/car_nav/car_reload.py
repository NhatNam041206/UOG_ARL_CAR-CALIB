"""Re-read car.yaml while running: `/car_config/reload`.

The web UI's Car tab writes a new car.yaml and publishes one message on /car_config/reload
(std_msgs/String, the data is the path of the file). Nodes that keep geometry from car.yaml
in memory subscribe with watch() and re-read it in place: static TFs (car_tf_publisher),
scan_upright, scan_self_mask, cmd_vel_to_control, adaptive_odom (laser yaw, drive limits) and
the simulator (lite_sim). Nodes that live in the restartable stack (safety_guard,
pd_path_follower, dynamic_scan, depth_guard_source, Nav2) are restarted by ui_bridge instead.
"""
from __future__ import annotations

from typing import Callable

from std_msgs.msg import String

RELOAD_TOPIC = "/car_config/reload"


def watch(node, callback: Callable[[dict], None]) -> None:
    """Call callback(cfg) with the freshly loaded car.yaml whenever a reload is announced."""
    from . import config_loader

    def on_msg(_msg: String) -> None:
        try:
            cfg = config_loader.load()
            callback(cfg)
            node.get_logger().info(f"car.yaml reloaded from {cfg.get('_path')}")
        except Exception as exc:  # noqa: BLE001 - a bad file must not kill the node
            node.get_logger().error(f"car.yaml reload failed, keeping the old values: {exc!r}")

    node.create_subscription(String, RELOAD_TOPIC, on_msg, 10)
