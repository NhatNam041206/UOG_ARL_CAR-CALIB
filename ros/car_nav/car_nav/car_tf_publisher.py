"""Static TFs base_link -> laser / laser_upright / camera_link from car.yaml, updated on reload.

Replaces tf2_ros static_transform_publisher for the car's sensor mounts: a static_transform_publisher
cannot change its pose, this node re-publishes the changed transform when the web UI's Car tab
applies a new car.yaml (/car_config/reload). tf2 keeps the newest static transform of a frame.

    ros2 run car_nav car_tf_publisher --ros-args -p frames:="[laser, camera_link]"
frames: laser (laser pose, roll/pitch/yaw as in car.yaml), laser_upright (laser x, y, z, yaw only:
the frame /scan_nav is in), camera_link (camera pose).
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from . import car_reload, config_loader


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def frame_poses(cfg: dict, frames) -> dict[str, tuple]:
    """child frame -> (x, y, z, roll, pitch, yaw) of base_link -> child for the frames asked for."""
    laser = config_loader.laser_pose(cfg)
    cam = config_loader.camera_pose(cfg)
    all_ = {
        "laser": (laser.x, laser.y, laser.z, laser.roll, laser.pitch, laser.yaw),
        "laser_upright": (laser.x, laser.y, laser.z, 0.0, 0.0, laser.yaw),
        "camera_link": (cam.x, cam.y, cam.z, cam.roll, cam.pitch, cam.yaw),
    }
    unknown = [f for f in frames if f not in all_]
    if unknown:
        raise ValueError(f"unknown frames {unknown}: use laser, laser_upright, camera_link")
    return {f: all_[f] for f in frames}


class CarTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("car_tf_publisher")
        self.declare_parameter("frames", ["laser"])
        self._frames = [str(f) for f in self.get_parameter("frames").value]
        self._bc = None
        self._publish(config_loader.load())
        car_reload.watch(self, self._publish)

    def _publish(self, cfg: dict) -> None:
        out = []
        for child, (x, y, z, r, p, yaw) in frame_poses(cfg, self._frames).items():
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = "base_link", child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
            q = rpy_to_quat(r, p, yaw)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
            out.append(t)
        # rclpy's StaticTransformBroadcaster ignores a transform of a child frame it has sent before, and a
        # transient-local publisher keeps its old message for late joiners: replace the whole publisher.
        if self._bc is not None:
            self.destroy_publisher(self._bc.pub_tf)
        self._bc = StaticTransformBroadcaster(self)
        self._bc.sendTransform(out)
        self.get_logger().info("static TF base_link -> " + ", ".join(
            f"{t.child_frame_id} ({t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}, "
            f"{t.transform.translation.z:.3f})" for t in out))


def main() -> None:
    rclpy.init()
    node = CarTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
